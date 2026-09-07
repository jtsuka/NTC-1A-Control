/*
  ESP32-S3 TC Bridge RTOS — Final Integrated Build
  =================================================
  Directory: ESP32_TC_Bridge_RTOS_Final_Silent_BootMagic_20260907

  STATUS: NOT FROZEN. Code-review pending (per 最終ESP統合仕様書 v0.3, GO判定).
  Do not treat this file as production-final until the freeze checklist
  in the spec (回帰試験 + freeze条件) has been executed on real hardware.

  ------------------------------------------------------------
  Provenance (per 最終ESP統合仕様書 v0.3)
  ------------------------------------------------------------
  Base: ESP32_TC_Bridge_RTOS_Phase3_Skeleton_Stage5_fault_recovery
    Kept unchanged (semantics):
      - TcMainTx (golden asset, Step3A実機PASS済み, byte-for-byte identical)
      - qPiToTc Pi command parser (WAIT_CMD/COLLECTING state machine,
        buf[0] direct read — NOT tc::Packet::cmd()'s &0x07 mask)
      - SAFE (localSafeEnabled)
      - fault latch (localFaultLatched), origin A / origin B
      - recovery RESET / abort processing
      - FLAG_RECOVERY_RESET (tc_message.hpp)
      - Core0->Core1 atomic fault/recovery counters

  Ported from: ESP32_TC_Bridge_RTOS_Phase3_Skeleton_Step4D_BOOTMAGIC_20260902
    (Direction-B official RX + Silent + Boot Magic; formerly gated behind
     TC_TEST_MODE==5/6/7/8, now unconditional production code):
      - C1 GPIO4 edge-capture ISR (ring buffer version)
      - c1PopEdge / C1 ring buffer
      - C2 byte decode (c2ProcessEdge / c2HandleBoundary / c2DecodeCurrentByte)
      - C3 6-byte frame assembly, 0x7F footer validation, resync
        (c3ConsumeByte / c3InvalidateFrame)
      - c4ForwardFrameToPi() -> qTcToPi (now unconditional, not TC_TEST_MODE==8 only)
      - Silent mechanism (C4_USB_DEBUG macro gating all USB CDC Serial output)
      - Boot Magic (ESP_BOOT_MAGIC, sent once from taskPiUart at boot)

  Deleted (legacy / test-only, per spec v0.3 §18):
      - TC_TEST_MODE and all mode-switch branches (1/2/4/5/6/7/8)
      - Stage5 legacy fixed-array RX: p11TryReceiveFrame(), p11EdgeTimes/
        p11EdgeLevels/p11EdgeCount
      - Legacy Phase1.2 TX: p11SendByte17Slot(), p11SendFrame17Slot(),
        p11SetBusLogical(), p11WaitUntilMicros()
      - Step4D-only 9-bit legacy TX: tcWriteLogicalBit(), tcSend9BitByte(),
        legacy tcSendFrame() (unrelated protocol, dead code, duplicated
        TcMainTx's role)
      - Phase1.3.5-A time-window state machine (P135_* constants/logic)
      - Step4D's taskPiUart() Pi->ESP parser (tc::PacketFactory::tryParse(),
        the {12,8,6} sliding-window ring parser). This parser has a known
        defect (0x11 SAFE OFF collides with 0x01 RESET via &0x07 mask, and
        an early-match defect across candidate lengths) that Stage5 already
        fixed by replacing it with the WAIT_CMD/COLLECTING state machine.
        It is NOT ported into this build under any circumstance.

  ------------------------------------------------------------
  Communication protocol — origin-verified FORMAL PASS (2026-09-07)
  against TC-FW/zhukongban.asm (Main Controller original) and
  TC-FW/FW_TC106-1_v0.0_250201-UTF8.c (TC106 original).
  NOT MODIFIED by this integration:
    Main->TC RESET : 12 byte, 13 slot/byte, tension<<1,
                      checksum = (len0+len1+len2+(tension<<1)) & 0x7F,
                      confirm bytes 0xF1 -> 0xF9
    Main->TC SEND  : (ch1Tens<<1), 0xF3, (ch3Tens<<1), 0xFB, 13 slot/byte
    Main->TC SENS.ADJ : 0xF2 (BREAK 0xF4), 11 slot/byte
    common: LSB first, slot = 3300us
    TC->Main/ESP   : 17 slot/byte, 7bit LSB first, 6 byte
                      payload = lastlen[0..2], txtens[0..1], 0x7F(footer)
                      Original250201 has no tx_enable; Direction-B is
                      continuous transmission.
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include <atomic>
#include "tc_packet_phase3.hpp"
#include "tc_message.hpp"

// ======================================================
// Silent build switch
// ======================================================
// Product default is Silent (USB CDC Serial fully disabled).
// Set to 1 only for bench/regression debug builds.
// Turning this off must not affect: SerialPi, qPiToTc, qTcToPi,
// C1/C2/C3/C4, TcMainTx, fault/recovery, Boot Magic, timing, payload,
// queue depth, task priority, core assignment.
#define C4_USB_DEBUG 0

// ======================================================
// Core0 poll-interval diagnostic switch (independent of C4_USB_DEBUG)
// ======================================================
// Enables ONLY the micros()-based measurement in taskTcBus() of the
// interval between successive tcMainTx.poll() calls (last poll time,
// max interval, count of intervals > 1000us), kept as plain in-memory
// counters. It performs no Serial output of any kind on Core0, and is
// intentionally decoupled from C4_USB_DEBUG so poll-interval measurement
// can be exercised (e.g. for overrun/backlog regression testing) without
// also enabling USB CDC Serial output (which would itself perturb the
// timing being measured). Must be 0 for production freeze.
#define TC_POLL_DIAG_ENABLED 0

// ======================================================
// Pin assignment (unchanged, identical in both source branches)
// ======================================================
static constexpr int PIN_PI_RX = D1;  // Pi_Tx_MCU -> ESP32 RX  (= GPIO2)
static constexpr int PIN_PI_TX = D0;  // ESP32 TX -> Pi_Rx_MCU  (= GPIO1)

static constexpr int PIN_TC_RX      = D3;   // TC_MCU_RX <- JP7 <- clamp <- divider (= GPIO4)
static constexpr int PIN_TC_TX_TRIG = D10;  // MCU_TX_TRIG -> JP5 -> R12 -> Q2 gate   (= GPIO9)

static constexpr int PIN_STATUS_LED = LED_BUILTIN;

// ======================================================
// Communication parameters
// ======================================================
static constexpr uint32_t PI_BAUD = 9600;

// ======================================================
// 17-slot protocol constants (used by the C2/C3 RX decode path below).
// Intentionally kept independent from TcMainTx's own SLOT_US symbol
// (defined near the TcMainTx class) — see comment there.
// ======================================================
static constexpr uint32_t P11_SLOT_US    = 3300;
static constexpr uint8_t  P11_DATA_SLOTS = 7;
static constexpr uint8_t  P11_SLOTS_PER_BYTE = 17; // documents the fixed 17-slot/byte spec

// ======================================================
// Step4C/Step4D C1 ring buffer + C2 byte-local edge history
// (ported unconditionally — this is the production Direction-B RX,
//  no longer gated behind TC_TEST_MODE)
// ======================================================

// Diagnostic-only provisional threshold, inherited from Step4D.
// Original250201 ESP/Main-view: data-only HIGH run <= 7 slots ~= 23100us,
// guard HIGH >= 9 slots ~= 29700us. 26000us sits deliberately between them.
static constexpr uint32_t C1_GUARD_MIN_HIGH_US = 26000UL;

// Power-of-two ring size. Effective capacity = size - 1.
static constexpr uint16_t C1_EDGE_RING_SIZE = 1024;
static constexpr uint16_t C1_EDGE_RING_MASK = C1_EDGE_RING_SIZE - 1;

static_assert((C1_EDGE_RING_SIZE & (C1_EDGE_RING_SIZE - 1)) == 0,
              "C1_EDGE_RING_SIZE must be power of two");

static uint32_t c1EdgeTimes[C1_EDGE_RING_SIZE];
static uint8_t  c1EdgeLevels[C1_EDGE_RING_SIZE];

static volatile uint16_t c1WriteIndex = 0; // ISR producer
static volatile uint16_t c1ReadIndex  = 0; // Core0 consumer

static volatile uint32_t c1IsrEdgeCount      = 0;
static volatile uint32_t c1EdgeOverflowCount = 0;

static portMUX_TYPE c1RingMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t c1ConsumedEdgeCount = 0;
static uint32_t c1GuardCount        = 0;

static bool     c1HavePrevEdge = false;
static uint32_t c1PrevEdgeUs   = 0;
static uint8_t  c1PrevLevel    = LOW;

static bool     c1HavePrevGuard = false;
static uint32_t c1PrevGuardUs   = 0;

#if C4_USB_DEBUG
static uint32_t c1LastSummaryMs = 0;
#endif

// Stage C2 byte-local edge history (also used by C3).
// A 17-slot byte can only produce a small number of transitions.
static constexpr uint8_t C2_BYTE_EDGE_MAX = 32;

static bool     c2HaveByteStart = false;
static uint32_t c2ByteStartUs   = 0;

static uint32_t c2ByteEdgeTimes[C2_BYTE_EDGE_MAX];
static uint8_t  c2ByteEdgeLevels[C2_BYTE_EDGE_MAX];
static uint8_t  c2ByteEdgeCount = 0;

static uint32_t c2DecodedBytes      = 0;
static uint32_t c2DecodeErrors      = 0;
static uint32_t c2TimingErrors      = 0;
static uint32_t c2LocalEdgeOverflow = 0;

// Diagnostic only. Production tolerance is not separately frozen here;
// it is inherited from Step4C Stage C1 measurement (~55.97..56.04ms vs
// nominal 56.1ms). Kept deliberately generous to catch gross anomalies.
static constexpr uint32_t C2_BYTE_DT_MIN_US = 54000UL;
static constexpr uint32_t C2_BYTE_DT_MAX_US = 58000UL;

static uint32_t c2LastSeenRingOverflow = 0;

// ------------------------------------------------------
// Stage C3 frame assembly / resynchronization
// ------------------------------------------------------
static constexpr uint8_t C3_FRAME_BYTES = 6;
static constexpr uint8_t C3_FOOTER = 0x7F;

static uint8_t  c3Frame[C3_FRAME_BYTES]{};
static uint8_t  c3DataCount = 0;
static bool     c3Aligned = false;
static bool     c3DiscardUntilFooter = false;
static bool     c3BadSpan = false;

static uint32_t c3ValidFrames = 0;
static uint32_t c3FrameErrors = 0;
static uint32_t c3SyncMisses = 0;
static uint32_t c3Resyncs = 0;

static uint32_t c4QueuedFrames = 0;
static uint32_t c4QueueDrops   = 0;

// ======================================================
// GPIO4 CHANGE ISR (Step4D ring buffer version)
// ======================================================
// ISR remains deliberately minimal. No Serial / decode / frame logic here.
static void IRAM_ATTR p11TcRxIsr() {
  const uint32_t t = micros();
  const uint8_t lvl = (uint8_t)digitalRead(PIN_TC_RX);

  portENTER_CRITICAL_ISR(&c1RingMux);

  const uint16_t w = c1WriteIndex;
  const uint16_t next = (uint16_t)((w + 1u) & C1_EDGE_RING_MASK);

  if (next == c1ReadIndex) {
    c1EdgeOverflowCount++;
    portEXIT_CRITICAL_ISR(&c1RingMux);
    return;
  }

  c1EdgeTimes[w]  = t;
  c1EdgeLevels[w] = lvl;
  c1WriteIndex    = next;
  c1IsrEdgeCount++;

  portEXIT_CRITICAL_ISR(&c1RingMux);
}

// ======================================================
// Queue / RTOS objects
// ======================================================
static constexpr uint8_t QUEUE_DEPTH_PI_TO_TC = 8;
static constexpr uint8_t QUEUE_DEPTH_TC_TO_PI = 8;

// Reserved (not currently referenced by any TC response-wait logic;
// kept for parity with both source branches).
static constexpr uint32_t TC_RESPONSE_TIMEOUT_MS = 150;

HardwareSerial SerialPi(1);

static QueueHandle_t qPiToTc = nullptr;
static QueueHandle_t qTcToPi = nullptr;

// TC bus send mutex. Not currently contended (only taskTcBus drives the
// bus), kept for parity with Stage5 / future expansion.
static SemaphoreHandle_t tcBusMutex = nullptr;

// ============================================================
// Stage5: fault / recovery RESET notification, Core0->Core1 one-way.
// (Step3C_fault_recovery設計_最終版.md §1 準拠)
// writer=Core0(taskTcBus), reader=Core1(taskPiUart) only.
// Core1 never clears these counters (detects via diff against last-seen).
// ============================================================
static std::atomic<uint32_t> gNonRecoveryAbortCount{0};   // origin A
static std::atomic<uint32_t> gRecoveryResetDoneCount{0};  // recovery RESET success
static std::atomic<uint32_t> gRecoveryResetAbortCount{0}; // recovery RESET failure

// ======================================================
// C4: forward a validated 6-byte Direction-B frame to Pi (unconditional)
// ======================================================
static bool c4ForwardFrameToPi(const uint8_t* frame) {
  if (!qTcToPi || !frame) {
    return false;
  }

  tc::TcMessage m;
  m.timestamp_us = micros();
  m.source = tc::MsgSource::Tc;
  m.type = tc::MsgType::Response;
  m.len = C3_FRAME_BYTES;
  memcpy(m.data, frame, C3_FRAME_BYTES);

  // Direction-B 0x7F is a footer, not a checksum. Do not claim FLAG_CHECKSUM_OK.
  m.flags = tc::FLAG_NONE;

  // Continuous TC receive must never be stalled by a slow/disconnected Pi.
  return xQueueSend(qTcToPi, &m, 0) == pdTRUE;
}

// ======================================================
// Utilities
// ======================================================
static void debugDumpMessage(const char* tag, const tc::TcMessage& m) {
#if C4_USB_DEBUG
  Serial.print(tag);
  Serial.print(" len=");
  Serial.print(m.len);
  Serial.print(" flags=0x");
  Serial.print(m.flags, HEX);
  Serial.print(" data=");
  m.dumpTo(Serial);
  Serial.println();
#else
  (void)tag;
  (void)m;
#endif
}

static void setStatusLed(bool on) {
#ifdef LED_BUILTIN
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
#endif
}

// ======================================================
// TcMainTx: verbatim from Stage5_fault_recovery (golden asset).
// NOT MODIFIED — byte-for-byte identical to Stage5 / Standalone
// (Step3A実機PASS済み). See separate diff report.
// ======================================================
//
// Source: ESP32S3_TcMainTx_Standalone.ino (Step3A実機PASS).
// Class body has not changed by a single line (golden asset).
// GPIO9->Q2->R7->JP3->Nano D11 path, same polarity as TC_OC_ACTIVE_LOW=true.
//
// ASM-confirmed facts:
//   - RESET/SEND tension byte is shifted left by 1 bit before transmission.
//     API argument `tens` is defined as the Main-internal value (pre-shift);
//     internally txTens = tens<<1 is transmitted.
//   - RESET checksum is (len0+len1+len2+txTens)&0x7F.

// Constants referenced by TcMainTx (Standalone). Same name/value as
// Standalone body. Deliberately independent from this file's own
// P11_SLOT_US (=3300, used by the RX/C2 decode path) so the TcMainTx
// class body can be integrated without a single line of change.
static const uint32_t SLOT_US = 3300;

static const uint32_t TX_SLOT_OVERRUN_US_TCMAINTX = 1000; // same rationale as RX side: 1 slot (3300) is not adopted

class TcMainTx {
public:
    void begin(uint8_t pin) {
        _pin = pin;
        pinMode(_pin, OUTPUT);
        writeLogical(true); // idle = logical HIGH
        _state = TXTX_IDLE;
        _doneFlag = false;
        _aborted = false;
    }

    bool busy() const { return _state != TXTX_IDLE; }

    // RESET: tens is the Main-internal value (pre-shift). Internally
    // txTens=tens<<1 is transmitted, checksum=(len0+len1+len2+txTens)&0x7F
    // is computed, and the same content is sent twice consecutively
    // (only the confirm byte differs: 0xF1 -> 0xF9).
    bool sendReset(uint8_t len0, uint8_t len1, uint8_t len2, uint8_t tens) {
        if (busy()) return false;
        const uint8_t txTens = (uint8_t)(tens << 1);
        const uint8_t checksum = (uint8_t)((len0 + len1 + len2 + txTens) & 0x7F);

        _frame.byte[0]  = { len0,     false };
        _frame.byte[1]  = { len1,     false };
        _frame.byte[2]  = { len2,     false };
        _frame.byte[3]  = { txTens,   false };
        _frame.byte[4]  = { checksum, false };
        _frame.byte[5]  = { 0xF1,     true  };
        _frame.byte[6]  = { len0,     false };
        _frame.byte[7]  = { len1,     false };
        _frame.byte[8]  = { len2,     false };
        _frame.byte[9]  = { txTens,   false };
        _frame.byte[10] = { checksum, false };
        _frame.byte[11] = { 0xF9,     true  };
        _frame.byteCount    = 12;
        _frame.slotsPerByte = 13;
        startFrame();
        return true;
    }

    // SEND: both ch1Tens/ch3Tens are sent as tens<<1. Re-confirmed via ASM
    // that both send_send_2_1 (byte0=tens_ch1) and send_send_2_2
    // (byte2=tens_ch3) perform bcf STATUS,C; rlf rreg_1 (left shift).
    // For CH1-only tests, ch3Tens=0 (default) is fine.
    bool sendTension(uint8_t ch1Tens, uint8_t ch3Tens = 0) {
        if (busy()) return false;
        const uint8_t txTens1 = (uint8_t)(ch1Tens << 1);
        const uint8_t txTens3 = (uint8_t)(ch3Tens << 1);

        _frame.byte[0] = { txTens1, false };
        _frame.byte[1] = { 0xF3,    true  };
        _frame.byte[2] = { txTens3, false };
        _frame.byte[3] = { 0xFB,    true  };
        _frame.byteCount    = 4;
        _frame.slotsPerByte = 13;
        startFrame();
        return true;
    }

    // SENS.ADJ (isBreak=false -> 0xF2, true -> 0xF4/BREAK)
    bool sendSensAdj(bool isBreak = false) {
        if (busy()) return false;
        _frame.byte[0] = { (uint8_t)(isBreak ? 0xF4 : 0xF2), true };
        _frame.byteCount    = 1;
        _frame.slotsPerByte = 11;
        startFrame();
        return true;
    }

    // Call every loop(). At most 1 step per poll, no catch-up. No Serial
    // output is performed here (the caller detects completion via
    // consumeDoneFlag()).
    void poll() {
        if (_state == TXTX_IDLE) return;

        const uint32_t now = micros();
        const int32_t lateBy = (int32_t)(now - _nextDueUs);
        if (lateBy < 0) return; // not yet

        if (lateBy > (int32_t)TX_SLOT_OVERRUN_US_TCMAINTX) {
            _overrunCount++;
            writeLogical(true); // force back to idle (pairs with V2.6 RX's RESYNC_WAIT_HIGH)
            _state = TXTX_IDLE;
            _doneFlag = true;
            _aborted = true;
            return;
        }

        if (_slotIndex >= 1 && _slotIndex <= 8) {
            const bool bit = ((_frame.byte[_byteIndex].value >> (_slotIndex - 1)) & 0x01) != 0;
            writeLogical(bit); // LSB first: bit=1->HIGH, bit=0->LOW
            _nextDueUs += SLOT_US;
            _slotIndex++;
            return;
        }

        if (_slotIndex == 9) {
            writeLogical(_frame.byte[_byteIndex].isCommand); // marker: confirm byte = HIGH
            _nextDueUs += SLOT_US;
            _slotIndex++;
            return; // byte boundary is not decided here (see below)
        }

        if (_slotIndex >= 10 && _slotIndex < _frame.slotsPerByte) {
            // guard: go HIGH only on the first pass (slotIndex==10); after
            // that, hold level and only consume timing.
            if (_slotIndex == 10) {
                writeLogical(true);
            }
            _nextDueUs += SLOT_US;
            _slotIndex++;
            return; // byte boundary is not decided here (see below)
        }

        // slotIndex == slotsPerByte: treat the byte boundary as an
        // independent event (same idea as the Step1 slot16/17 boundary
        // fix). This ensures the previous guard's final slot (slot12 for
        // 13-step, slot10 for 11-step) is held for the full SLOT_US before
        // transitioning to the next byte's START (or frame completion).
        advanceByte();
    }

    // Notifies frame completion (normal or overrun-aborted) exactly once.
    // outAborted reports whether it was an overrun-abort.
    bool consumeDoneFlag(bool* outAborted = nullptr) {
        if (!_doneFlag) return false;
        _doneFlag = false;
        if (outAborted) *outAborted = _aborted;
        _aborted = false;
        return true;
    }

    uint32_t overrunCount() const { return _overrunCount; }

private:
    struct TxByteSpec { uint8_t value; bool isCommand; };
    struct TxFrame {
        TxByteSpec byte[12]; // RESET(12byte) is the maximum
        uint8_t    byteCount;
        uint8_t    slotsPerByte;
    };
    enum TxTxState : uint8_t { TXTX_IDLE, TXTX_SENDING };

    void startFrame() {
        _byteIndex = 0;
        writeLogical(false); // slot0 START = logical LOW, output immediately
        _slotIndex = 1;
        _nextDueUs = micros() + SLOT_US;
        _state = TXTX_SENDING;
    }

    void advanceByte() {
        _byteIndex++;
        if (_byteIndex >= _frame.byteCount) {
            writeLogical(true); // all bytes sent, back to idle
            _state = TXTX_IDLE;
            _doneFlag = true;
            return;
        }
        // Output the next byte's START (logical LOW) immediately (slot0 is
        // emitted without waiting, same as any other byte). This poll()
        // call itself occurs at "now == previous byte's boundary time", so
        // the time to wait for slot1 is "now + SLOT_US". The boundary-
        // detection side of poll() has not advanced _nextDueUs before
        // reaching here (that increment was moved here when boundary
        // handling became an independent event), so it is added explicitly.
        writeLogical(false);
        _slotIndex = 1;
        _nextDueUs += SLOT_US;
    }

    // *** This is the ONLY change from the Nano version (platform adaptation) ***
    //
    // Nano version: digitalWrite(_pin, high ? HIGH : LOW);
    //   (direct, no inversion — the Nano itself output TC106's logical
    //   value as-is)
    //
    // ESP32-S3 version: absorbs the logical inversion introduced by the
    // existing board's Q2 (NMOS). Same polarity as the original
    // ESP32_TC_Bridge_RTOS_Phase3_Skeleton.ino's tcWriteLogicalBit()
    // (TC_OC_ACTIVE_LOW=true):
    //     logical HIGH -> GPIO LOW  -> Q2 OFF -> D11 HIGH (Nano internal pull-up)
    //     logical LOW  -> GPIO HIGH -> Q2 ON  -> D11 LOW  (via R7 to GND)
    //   The state machine, frame construction, and timing logic are
    //   untouched. Only this electrical-interface adaptation exists for
    //   ESP32-S3.
    void writeLogical(bool high) {
        digitalWrite(_pin, high ? LOW : HIGH);
    }

    uint8_t   _pin = 0;
    TxTxState _state = TXTX_IDLE;
    TxFrame   _frame;
    uint8_t   _byteIndex = 0;
    uint8_t   _slotIndex = 0;
    uint32_t  _nextDueUs = 0;
    bool      _doneFlag = false;
    bool      _aborted = false;
    uint32_t  _overrunCount = 0;
};

// ======================================================
// Direction-B RX pipeline (ported from Step4D, now unconditional)
// C1 (ring) -> C2 (byte decode) -> C3 (frame assembly / footer validation)
// ======================================================

static bool c1PopEdge(uint32_t& t, uint8_t& lvl) {
  bool ok = false;

  portENTER_CRITICAL(&c1RingMux);

  const uint16_t r = c1ReadIndex;

  if (r != c1WriteIndex) {
    t = c1EdgeTimes[r];
    lvl = c1EdgeLevels[r];
    c1ReadIndex = (uint16_t)((r + 1u) & C1_EDGE_RING_MASK);
    ok = true;
  }

  portEXIT_CRITICAL(&c1RingMux);
  return ok;
}

// True if the ISR ring still holds at least one unconsumed edge.
// Used only to decide whether Core0 may go idle (vTaskDelay(1)) this loop.
static bool c1RingHasPending() {
  bool has;
  portENTER_CRITICAL(&c1RingMux);
  has = (c1ReadIndex != c1WriteIndex);
  portEXIT_CRITICAL(&c1RingMux);
  return has;
}

static void c3InvalidateFrame(const char* reason) {
  if (c3Aligned || c3DataCount != 0 || c3BadSpan) {
    c3FrameErrors++;
  }
  c3Aligned = false;
  c3DataCount = 0;
  c3BadSpan = false;
  c3DiscardUntilFooter = true;

#if C4_USB_DEBUG
  Serial.print("[C3 INVALID] ");
  Serial.println(reason);
#else
  (void)reason;
#endif
}

static void c3PrintFrame() {
#if C4_USB_DEBUG
  Serial.print("[C3 FRAME] #");
  Serial.print(c3ValidFrames);
  Serial.print(" ");
  for (uint8_t i = 0; i < C3_FRAME_BYTES; i++) {
    if (c3Frame[i] < 0x10) Serial.print('0');
    Serial.print(c3Frame[i], HEX);
    if (i + 1u < C3_FRAME_BYTES) Serial.print(' ');
  }
  Serial.println();
#endif
}

static void c3ConsumeByte(uint8_t value) {
  // After any lower-layer loss, ignore data until a footer gives us a
  // trustworthy frame boundary again.
  if (c3DiscardUntilFooter) {
    if (value == C3_FOOTER) {
      c3DiscardUntilFooter = false;
      c3Aligned = true;
      c3DataCount = 0;
      c3BadSpan = false;
      c3SyncMisses++;
      c3Resyncs++;
#if C4_USB_DEBUG
      Serial.println("[C3 RESYNC] footer found after lower-layer invalidation");
#endif
    }
    return;
  }

  if (value == C3_FOOTER) {
    if (c3DataCount == 5 && !c3BadSpan) {
      c3Frame[5] = C3_FOOTER;
      c3ValidFrames++;
      c3Aligned = true;
      c3PrintFrame();

      // Unconditional in the final build (was TC_TEST_MODE==8 only in Step4D).
      if (c4ForwardFrameToPi(c3Frame)) {
        c4QueuedFrames++;
      } else {
        c4QueueDrops++;
      }
    } else {
      // Cold start may legitimately enter in the middle of a frame. This
      // footer establishes the phase for the NEXT frame.
      c3SyncMisses++;
      if (c3Aligned || c3BadSpan) {
        c3FrameErrors++;
      }
      c3Resyncs++;
#if C4_USB_DEBUG
      Serial.print("[C3 RESYNC] footer at data_count=");
      Serial.println(c3DataCount);
#endif
      c3Aligned = true;
    }

    c3DataCount = 0;
    c3BadSpan = false;
    return;
  }

  if (c3DataCount < 5) {
    c3Frame[c3DataCount++] = value;
  } else {
    // More than five non-footer bytes means frame phase is broken. Do not
    // slide a 6-byte window; wait for the unique footer instead.
    c3BadSpan = true;
    if (c3DataCount < 0xFF) c3DataCount++;
  }
}

static void c2ResetByteState() {
  c2HaveByteStart = false;
  c2ByteStartUs   = 0;
  c2ByteEdgeCount = 0;
}

// Decode the byte that started at c2ByteStartUs.
// At that boundary the post-edge level is LOW (slot0).
static bool c2DecodeCurrentByte(uint8_t& value) {
  if (!c2HaveByteStart) {
    return false;
  }

  value = 0;

  for (uint8_t bit = 0; bit < P11_DATA_SLOTS; bit++) {
    const uint32_t sampleUs =
      c2ByteStartUs
      + (uint32_t)(bit + 1u) * P11_SLOT_US
      + (P11_SLOT_US / 2u);

    uint8_t level = LOW; // slot0 begins LOW at the byte boundary.

    for (uint8_t i = 0; i < c2ByteEdgeCount; i++) {
      // Signed subtraction is safe for these short (<60ms) intervals and
      // remains correct across a micros() wrap.
      if ((int32_t)(c2ByteEdgeTimes[i] - sampleUs) <= 0) {
        level = c2ByteEdgeLevels[i];
      } else {
        break;
      }
    }

    if (level == HIGH) {
      value |= (uint8_t)(1u << bit);
    }
  }

  return true;
}

static void c2StartNewByte(uint32_t boundaryUs) {
  c2HaveByteStart = true;
  c2ByteStartUs   = boundaryUs;
  c2ByteEdgeCount = 0;
}

static void c2AppendByteEdge(uint32_t t, uint8_t lvl) {
  if (!c2HaveByteStart) {
    return;
  }

  if (c2ByteEdgeCount < C2_BYTE_EDGE_MAX) {
    c2ByteEdgeTimes[c2ByteEdgeCount]  = t;
    c2ByteEdgeLevels[c2ByteEdgeCount] = lvl;
    c2ByteEdgeCount++;
  } else {
    c2LocalEdgeOverflow++;
    c2DecodeErrors++;
    c3InvalidateFrame("byte-local edge overflow");
    c2ResetByteState();
  }
}

static void c2HandleBoundary(uint32_t boundaryUs, uint32_t highUs) {
  (void)highUs;

  if (!c2HaveByteStart) {
    // Cold-start: this boundary only establishes the first byte start.
    c2StartNewByte(boundaryUs);
    c1GuardCount++;
    c1PrevGuardUs   = boundaryUs;
    c1HavePrevGuard = true;
    return;
  }

  const uint32_t boundaryDt = boundaryUs - c2ByteStartUs;

  if (boundaryDt < C2_BYTE_DT_MIN_US ||
      boundaryDt > C2_BYTE_DT_MAX_US) {
    c2TimingErrors++;
    c3InvalidateFrame("byte boundary timing error");
  }

  uint8_t value = 0;
  if (c2DecodeCurrentByte(value)) {
    c2DecodedBytes++;
    c3ConsumeByte(value);
  } else {
    c2DecodeErrors++;
    c3InvalidateFrame("byte decode failure");
  }

  c1GuardCount++;
  c1PrevGuardUs   = boundaryUs;
  c1HavePrevGuard = true;

  // The current falling edge is slot0 of the NEXT byte.
  c2StartNewByte(boundaryUs);
}

static void c2ProcessEdge(uint32_t t, uint8_t lvl) {
  c1ConsumedEdgeCount++;

  // Ring overflow means edge history is no longer trustworthy.
  uint32_t ringOverflow;
  portENTER_CRITICAL(&c1RingMux);
  ringOverflow = c1EdgeOverflowCount;
  portEXIT_CRITICAL(&c1RingMux);

  if (ringOverflow != c2LastSeenRingOverflow) {
    c2LastSeenRingOverflow = ringOverflow;
    c2DecodeErrors++;
    c3InvalidateFrame("ISR ring overflow");
    c2ResetByteState();

    // Re-anchor the edge-to-edge HIGH duration detector at this edge.
    c1HavePrevEdge = true;
    c1PrevEdgeUs   = t;
    c1PrevLevel    = lvl;
    return;
  }

  if (!c1HavePrevEdge) {
    c1HavePrevEdge = true;
    c1PrevEdgeUs   = t;
    c1PrevLevel    = lvl;
    return;
  }

  const uint32_t dt = t - c1PrevEdgeUs;

  const bool isBoundary =
    (c1PrevLevel == HIGH) &&
    (lvl == LOW) &&
    (dt >= C1_GUARD_MIN_HIGH_US);

  // Once a byte start is known, retain every subsequent edge so the
  // completed byte can be reconstructed when the next boundary arrives.
  // The next boundary itself may be included; its timestamp is after all
  // seven sample points and therefore cannot alter the current byte value.
  c2AppendByteEdge(t, lvl);

  if (isBoundary) {
    c2HandleBoundary(t, dt);
  }

  c1PrevEdgeUs = t;
  c1PrevLevel  = lvl;
}

#if C4_USB_DEBUG
// Debug-build-only periodic RX health summary. Never called in a Silent
// (C4_USB_DEBUG=0) production build; compiles out entirely.
static void c2PrintSummaryIfDue() {
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - c1LastSummaryMs) < 1000UL) {
    return;
  }
  c1LastSummaryMs = nowMs;

  uint16_t w, r;
  uint32_t isrEdges, overflow;
  portENTER_CRITICAL(&c1RingMux);
  w = c1WriteIndex;
  r = c1ReadIndex;
  isrEdges = c1IsrEdgeCount;
  overflow = c1EdgeOverflowCount;
  portEXIT_CRITICAL(&c1RingMux);

  const uint16_t pending = (uint16_t)((w - r) & C1_EDGE_RING_MASK);

  Serial.print("[C3] isr_edges=");
  Serial.print(isrEdges);
  Serial.print(" consumed=");
  Serial.print(c1ConsumedEdgeCount);
  Serial.print(" bytes=");
  Serial.print(c2DecodedBytes);
  Serial.print(" valid_frames=");
  Serial.print(c3ValidFrames);
  Serial.print(" frame_errors=");
  Serial.print(c3FrameErrors);
  Serial.print(" sync_misses=");
  Serial.print(c3SyncMisses);
  Serial.print(" resyncs=");
  Serial.print(c3Resyncs);
  Serial.print(" queued=");
  Serial.print(c4QueuedFrames);
  Serial.print(" queue_drops=");
  Serial.print(c4QueueDrops);
  Serial.print(" decode_errors=");
  Serial.print(c2DecodeErrors);
  Serial.print(" timing_errors=");
  Serial.print(c2TimingErrors);
  Serial.print(" local_edge_overflow=");
  Serial.print(c2LocalEdgeOverflow);
  Serial.print(" pending=");
  Serial.print(pending);
  Serial.print(" overflow=");
  Serial.println(overflow);
}
#endif // C4_USB_DEBUG

// ======================================================
// Core0 poll-interval diagnostics (regression-test only)
// ------------------------------------------------------
// Per spec v0.3/v0.4: no diagnostic code is added to the TcMainTx class
// body itself. Instead, taskTcBus() measures the interval between
// successive tcMainTx.poll() calls from the outside. This is NOT part of
// the production communication logic; it exists purely to verify, during
// integration regression testing, that RX processing never delays
// TcMainTx.poll() beyond the 1000us ceiling assumed by TX_SLOT_OVERRUN_US_TCMAINTX.
//
// Gated by TC_POLL_DIAG_ENABLED, NOT C4_USB_DEBUG: this block performs
// micros()-based measurement into plain in-memory counters ONLY. It never
// calls Serial (Core0 never prints), so enabling it does not add USB CDC
// output overhead to the very timing it is measuring, and it can be
// exercised independently of (and without) the C4_USB_DEBUG-gated C1/C2/C3
// statistics output. In a production build (TC_POLL_DIAG_ENABLED=0) this
// entire block compiles out to nothing; the counters are not computed and
// add zero overhead.
// ======================================================
#if TC_POLL_DIAG_ENABLED
static uint32_t gLastPollUs        = 0;
static bool     gHavePrevPoll      = false;
static std::atomic<uint32_t> gMaxPollIntervalUs{0};
static std::atomic<uint32_t> gPollOverThresholdCount{0}; // intervals > 1000us
static constexpr uint32_t POLL_DIAG_THRESHOLD_US = 1000;

static inline void pollDiagBeforePoll() {
  const uint32_t nowUs = micros();
  if (gHavePrevPoll) {
    const uint32_t interval = nowUs - gLastPollUs;
    if (interval > gMaxPollIntervalUs.load()) {
      gMaxPollIntervalUs.store(interval);
    }
    if (interval > POLL_DIAG_THRESHOLD_US) {
      gPollOverThresholdCount.fetch_add(1);
    }
  }
  gLastPollUs = nowUs;
  gHavePrevPoll = true;
}
#endif // TC_POLL_DIAG_ENABLED

// ======================================================
// C1 edge processing time budget (per loop iteration, taskTcBus only)
// ------------------------------------------------------
// Rationale: TcMainTx.poll() must be revisited well under 1000us in the
// worst case, so RESET/SEND/SENS.ADJ slot timing (3300us/slot) can never
// overrun because of RX work. Each c2ProcessEdge() call performs only a
// handful of integer comparisons and array accesses — no I/O, no Serial —
// so on ESP32-S3 (240MHz) a single call is expected to complete in low
// single-digit microseconds. A 200us budget therefore bounds worst-case
// per-loop RX processing to roughly 1/5 of the 1000us ceiling, leaving
// ample margin for TcMainTx.poll() itself, command dispatch, and
// scheduling jitter. This is a conservative starting value; it MUST be
// reconfirmed against real-hardware measurements using the poll-interval
// diagnostics above (see regression test: RX/TX共存, TcMainTx overrunなし,
// c1EdgeOverflowCountなし) before freeze, and tightened or loosened only
// with that evidence.
// ======================================================
static constexpr uint32_t C1_EDGE_BUDGET_US = 200;

// ======================================================
// setup / loop
// ======================================================
void setup() {
#if C4_USB_DEBUG
  Serial.begin(115200);
#endif

  // Preserve original pre-attach startup timing (inherited from Step4D's
  // SILENT isolation test): the delay occurs before attachInterrupt(), so
  // it cannot build a C1 backlog.
  delay(500);

#if C4_USB_DEBUG
  Serial.println();
  Serial.println("=== ESP32-S3 TC Bridge RTOS Final Silent+BootMagic build ===");
  Serial.println("Core0=TC (TcMainTx + Direction-B RX), Core1=Pi UART");
#endif

  pinMode(PIN_STATUS_LED, OUTPUT);
  setStatusLed(false);

  pinMode(PIN_TC_TX_TRIG, OUTPUT);
  digitalWrite(PIN_TC_TX_TRIG, LOW);  // OC release
  pinMode(PIN_TC_RX, INPUT_PULLUP);

  // Direction-B edge-capture interrupt is always enabled (no test-mode gating).
  attachInterrupt(digitalPinToInterrupt(PIN_TC_RX), p11TcRxIsr, CHANGE);

  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);

  qPiToTc = xQueueCreate(QUEUE_DEPTH_PI_TO_TC, sizeof(tc::TcMessage));
  qTcToPi = xQueueCreate(QUEUE_DEPTH_TC_TO_PI, sizeof(tc::TcMessage));
  tcBusMutex = xSemaphoreCreateMutex();

  if (!qPiToTc || !qTcToPi || !tcBusMutex) {
#if C4_USB_DEBUG
    Serial.println("[FATAL] RTOS object creation failed");
#endif
    while (true) {
      setStatusLed(true);
      delay(100);
      setStatusLed(false);
      delay(100);
    }
  }

  // ESP32 Arduino keeps the Arduino loop on Core1, so stack/priority there
  // stay modest. TC side is timing-critical, pinned to Core0.
  xTaskCreatePinnedToCore(taskTcBus,  "TcBus",  4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(taskPiUart, "PiUart", 4096, nullptr, 3, nullptr, 1);

#if C4_USB_DEBUG
  Serial.println("[setup] tasks created");
#endif
}

void loop() {
  // Nothing here; everything runs in the tasks.
  vTaskDelay(portMAX_DELAY);
}

// ======================================================
// Core1: Raspberry Pi side task
// ======================================================
static const uint8_t ESP_BOOT_MAGIC[6] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x7F
};

static void taskPiUart(void* pv) {
  // Stage2: command-origin parser (WAIT_CMD/COLLECTING). The old
  // PacketFactory::tryParse() {12,8,6} sliding-window (ring/head/lastSig)
  // approach is NOT used — command determination always reads buf[0]
  // directly; tc::Packet::cmd() (&0x07 mask) is never used (0x11 SAFE OFF
  // would collide with 0x01 RESET).
  enum PiParserState : uint8_t { WAIT_CMD, COLLECTING };
  PiParserState piState = WAIT_CMD;
  uint8_t piBuf[tc::PI_MAX]{};
  uint8_t piCollected = 0;
  uint8_t piExpectedLen = 0;

  // Stage5: localPending (Core1-local order-preserving buffer, not a
  // FreeRTOS queue). Overflow destination when qPiToTc is full. Both
  // producer and consumer are this task only.
  constexpr uint8_t LOCAL_PENDING_DEPTH = 8;
  tc::TcMessage localPending[LOCAL_PENDING_DEPTH];
  uint8_t pendingHead = 0;
  uint8_t pendingTail = 0;
  uint8_t pendingCount = 0;

  // While pendingCount>0, new packets are never sent via the qPiToTc fast
  // path; they always go to the end of localPending (no overtaking).
  auto tryAccept = [&](tc::TcMessage& msg) -> bool {
    if (pendingCount == 0 && xQueueSend(qPiToTc, &msg, 0) == pdTRUE) {
      return true;
    }
    if (pendingCount < LOCAL_PENDING_DEPTH) {
      localPending[pendingTail] = msg;
      pendingTail = (uint8_t)((pendingTail + 1) % LOCAL_PENDING_DEPTH);
      pendingCount++;
      return true;
    }
    return false;
  };

  // Stage5: fault state. Owned by Core1 only.
  bool faultState = false;
  bool recoveryResetPending = false; // limits recovery RESET to 1 in flight

  // Previously-seen values of the Core0->Core1 (atomic) notifications.
  // Core1 never clears the atomics themselves.
  uint32_t lastNonRecoveryAbort = 0;
  uint32_t lastRecoveryDone     = 0;
  uint32_t lastRecoveryAbort    = 0;

#if TC_POLL_DIAG_ENABLED
  // Test-only retrieval of Core0's poll-interval diagnostics. Read here on
  // Core1 (never on Core0/taskTcBus, and never inside TcMainTx) so fetching
  // these values cannot add any delay to Core0's timing-critical loop.
  // Printing additionally requires C4_USB_DEBUG=1 (so Serial.begin() has
  // actually run); with C4_USB_DEBUG=0 the values are still read from the
  // atomics every second but nothing is written out — use a debugger/JTAG
  // to inspect gMaxPollIntervalUs/gPollOverThresholdCount in that case.
  uint32_t lastPollDiagPrintMs = 0;
#endif

  // Boot Magic: ESP -> Pi UART only, sent exactly once per boot, before
  // anything else this task does. This is HardwareSerial(1) (SerialPi),
  // never USB CDC. It does not go through qTcToPi and is never treated as
  // TC telemetry. Not resent under any condition.
  SerialPi.write(ESP_BOOT_MAGIC, sizeof(ESP_BOOT_MAGIC));
  SerialPi.flush();

#if C4_USB_DEBUG
  Serial.println("[PiTask] start on Core1");
#endif

  while (true) {
    // ------------------------------------------------------------
    // Stage5: check Core0->Core1 fault/recovery notifications. Always
    // done before accepting any new UART packet (Step3C_fault_recovery
    // 設計_最終版.md §6 execution-order compliance).
    // ------------------------------------------------------------
    {
      const uint32_t v1 = gNonRecoveryAbortCount.load();
      if (v1 != lastNonRecoveryAbort) {
        lastNonRecoveryAbort = v1;
        faultState = true;
#if C4_USB_DEBUG
        Serial.println("[PiTask] FAULT detected (origin A: non-recovery ABORTED)");
#endif
      }
      const uint32_t v2 = gRecoveryResetDoneCount.load();
      if (v2 != lastRecoveryDone) {
        lastRecoveryDone = v2;
        faultState = false;
        recoveryResetPending = false;
#if C4_USB_DEBUG
        Serial.println("[PiTask] FAULT cleared (recovery RESET done)");
#endif
      }
      const uint32_t v3 = gRecoveryResetAbortCount.load();
      if (v3 != lastRecoveryAbort) {
        lastRecoveryAbort = v3;
        recoveryResetPending = false;
#if C4_USB_DEBUG
        Serial.println("[PiTask] recovery RESET aborted; pending cleared, FAULT remains");
#endif
      }
    }

    // Drain localPending -> qPiToTc unconditionally (regardless of fault
    // state), every loop.
    while (pendingCount > 0) {
      if (xQueueSend(qPiToTc, &localPending[pendingHead], 0) == pdTRUE) {
        pendingHead = (uint8_t)((pendingHead + 1) % LOCAL_PENDING_DEPTH);
        pendingCount--;
      } else {
        break; // qPiToTc still full; retry next loop.
      }
    }

    // Pi -> ESP32
    while (SerialPi.available()) {
      const uint8_t b = static_cast<uint8_t>(SerialPi.read());

      if (piState == WAIT_CMD) {
        uint8_t expLen = 0;
        switch (b) {
          case 0x01: expLen = tc::PI_LEN_RESET; break; // RESET     12byte
          case 0x02: expLen = tc::PI_LEN_SEND;  break; // SEND       6byte
          case 0x03: expLen = tc::PI_LEN_SENS;  break; // SENS.ADJ   8byte
          case 0x10: expLen = tc::PI_LEN_SEND;  break; // SAFE ON    6byte
          case 0x11: expLen = tc::PI_LEN_SEND;  break; // SAFE OFF   6byte
          default:   expLen = 0;                break; // invalid command
        }
        if (expLen == 0) {
          // Invalid command byte. Discard this single byte only and
          // resynchronize while staying in WAIT_CMD.
          continue;
        }
        piBuf[0] = b;
        piCollected = 1;
        piExpectedLen = expLen;
        piState = COLLECTING;
        continue;
      }

      // COLLECTING: collect one byte at a time up to expectedLen; never
      // evaluate the checksum early.
      piBuf[piCollected] = b;
      piCollected++;
      if (piCollected < piExpectedLen) {
        continue;
      }

      // Only once expectedLen bytes are gathered is the checksum evaluated.
      if (tc::checksum7(piBuf, piExpectedLen - 1) == piBuf[piExpectedLen - 1]) {
        tc::Packet p;
        p.len = piExpectedLen;
        memcpy(p.buf, piBuf, piExpectedLen);

        tc::TcMessage msg = tc::TcMessage::fromPacket(
          p, tc::MsgSource::Pi, tc::MsgType::Command
        );

        if (faultState) {
          // During fault, only RESET is accepted as a recovery candidate
          // (Step3C_fault_recovery設計_最終版.md §3・§4).
          if (msg.data[0] == 0x01 && !recoveryResetPending) {
            msg.flags |= tc::FLAG_RECOVERY_RESET;
            if (tryAccept(msg)) {
              recoveryResetPending = true;
              debugDumpMessage("[PiTask RECOVERY RESET]", msg);
            }
#if C4_USB_DEBUG
            else {
              Serial.println("[PiTask] RESET recovery not ready: localPending full");
            }
#endif
          }
#if C4_USB_DEBUG
          else if (msg.data[0] == 0x01 && recoveryResetPending) {
            Serial.println("[PiTask] FAULT: recovery RESET already pending, drop");
          } else {
            Serial.print("[PiTask] FAULT: drop cmd=0x");
            Serial.println(msg.data[0], HEX);
          }
#endif
        } else {
          // Normal operation: every command goes through localPending to qPiToTc.
          if (tryAccept(msg)) {
            debugDumpMessage("[PiTask RX]", msg);
          } else {
            // localPending also full = first drop -> fault (origin B).
            faultState = true;
#if C4_USB_DEBUG
            Serial.println("[PiTask] localPending full, first drop -> FAULT (origin B)");
#endif
          }
        }
      }
      // Whether checksum was OK or NG, discard the buffer entirely and
      // return to WAIT_CMD (no partial reuse / sliding window).
      piState = WAIT_CMD;
      piCollected = 0;
      piExpectedLen = 0;
    }

    // TC -> Pi
    tc::TcMessage rx;
    while (xQueueReceive(qTcToPi, &rx, 0) == pdTRUE) {
      debugDumpMessage("[PiTask TX]", rx);

      if (rx.len > 0) {
        SerialPi.write(rx.data, rx.len);
        SerialPi.flush();
      }
    }

#if TC_POLL_DIAG_ENABLED
    // Test-only, Core1-side, low-frequency (1Hz) retrieval of Core0's poll
    // diagnostics. Reading two atomics once a second is negligible next to
    // this task's own 9600-baud UART work, and touches nothing on Core0.
    {
      const uint32_t nowMs = millis();
      if ((uint32_t)(nowMs - lastPollDiagPrintMs) >= 1000UL) {
        lastPollDiagPrintMs = nowMs;
#if C4_USB_DEBUG
        Serial.print("[POLL DIAG] max_interval_us=");
        Serial.print(gMaxPollIntervalUs.load());
        Serial.print(" over_1000us_count=");
        Serial.println(gPollOverThresholdCount.load());
#endif
      }
    }
#endif

    vTaskDelay(1);
  }
}

// ======================================================
// Core0: TC106 side task
// ======================================================
static void taskTcBus(void* pv) {
#if C4_USB_DEBUG
  Serial.println("[TcTask] start on Core0");
#endif

  // Release the OC bus to idle.
  digitalWrite(PIN_TC_TX_TRIG, LOW);

  static TcMainTx tcMainTx;
  tcMainTx.begin(PIN_TC_TX_TRIG);

  // SAFE state, held for the lifetime of taskTcBus() (outside the loop).
  // Owned/mutated by Core0 only; never shared with Core1 or any atomic.
  bool localSafeEnabled = false;

  // Stage5: fault latch. Becomes true when origin A (normal command
  // ABORTED) occurs; cannot be cleared by SAFE ON/OFF. Only a successful
  // recovery RESET clears it.
  bool localFaultLatched = false;

  // Stage5: whether the RESET currently dispatched to TcMainTx was a
  // recovery-context RESET, held until consumeDoneFlag() reports
  // completion (at most one in flight at a time).
  bool activeRecoveryReset = false;

  while (true) {
    // --------------------------------------------------------
    // Step 1: TcMainTx.poll() — highest priority, called first every loop.
    // --------------------------------------------------------
#if TC_POLL_DIAG_ENABLED
    pollDiagBeforePoll();
#endif
    tcMainTx.poll();

    // --------------------------------------------------------
    // Step 2/3: consumeDoneFlag() -> fault/recovery/abort completion
    // handling (Stage5 semantics, unchanged).
    // --------------------------------------------------------
    bool doneFlagHandled = false;
    {
      bool aborted = false;
      if (tcMainTx.consumeDoneFlag(&aborted)) {
        doneFlagHandled = true;
        if (activeRecoveryReset) {
          // Completion of a recovery RESET (Step3C_fault_recovery設計_最終版.md §5).
          if (aborted) {
            // Regardless of fault origin, force convergence to the
            // origin-A-equivalent Core0 safe state as a physical-send
            // anomaly of the recovery RESET itself.
            localFaultLatched = true;
            localSafeEnabled  = true;
            gRecoveryResetAbortCount.fetch_add(1);
#if C4_USB_DEBUG
            Serial.println("[TcTask] RECOVERY RESET ABORTED");
#endif
          } else {
            localFaultLatched = false;
            localSafeEnabled  = false;
            gRecoveryResetDoneCount.fetch_add(1);
#if C4_USB_DEBUG
            Serial.println("[TcTask] RECOVERY RESET DONE");
#endif
          }
          activeRecoveryReset = false;
        } else if (aborted) {
          // origin A: normal (non-recovery) command TcMainTx ABORTED.
          localFaultLatched = true;
          localSafeEnabled  = true;
          gNonRecoveryAbortCount.fetch_add(1);
#if C4_USB_DEBUG
          Serial.print("[TcTask] ABORTED (overrun) count=");
          Serial.println(tcMainTx.overrunCount());
#endif
        }
#if C4_USB_DEBUG
        else {
          Serial.println("[TcTask] frame done");
        }
#endif
      }
    }

    // --------------------------------------------------------
    // Step 4-7: C1 -> C2 -> C3 -> C4 Direction-B RX, bounded per loop.
    // Deliberately NOT "drain until empty" — see C1_EDGE_BUDGET_US comment.
    // TcMainTx.poll() must be revisited before this can stall it.
    // --------------------------------------------------------
    {
      const uint32_t budgetStart = micros();
      uint32_t t; uint8_t lvl;
      while (c1PopEdge(t, lvl)) {
        c2ProcessEdge(t, lvl); // internally drives C2 decode -> C3 assembly -> C4 forwarding
        if ((uint32_t)(micros() - budgetStart) >= C1_EDGE_BUDGET_US) {
          break; // leave any remaining edges for the next loop iteration
        }
      }
    }

#if C4_USB_DEBUG
    c2PrintSummaryIfDue();
#endif

    // --------------------------------------------------------
    // Step 8: Main->TC command dispatch, only when TcMainTx is idle.
    // --------------------------------------------------------
    bool piCmdHandled = false;
    if (!tcMainTx.busy()) {
      tc::TcMessage cmd;

      if (xQueueReceive(qPiToTc, &cmd, 0) == pdTRUE) {
        piCmdHandled = true;
        debugDumpMessage("[TcTask CMD]", cmd);

        // Command determination always reads the raw command byte
        // (data[0]) directly. tc::Packet::cmd() (&0x07 mask) is never
        // used (0x11 SAFE OFF would collide with 0x01 RESET).
        const bool isRecoveryReset =
            (cmd.data[0] == 0x01) && ((cmd.flags & tc::FLAG_RECOVERY_RESET) != 0);

        switch (cmd.data[0]) {
          case 0x01: { // RESET
            if (isRecoveryReset) {
              // recovery RESET: always executed regardless of
              // localFaultLatched/localSafeEnabled (SAFE bypass, the only
              // exception during fault).
              const uint32_t value = ((uint32_t)cmd.data[1] << 16)
                                    | ((uint32_t)cmd.data[2] << 8)
                                    |  (uint32_t)cmd.data[3];
              const uint8_t tens = cmd.data[4];
              const uint8_t len0 = (uint8_t)(value / 10000);
              const uint8_t len1 = (uint8_t)((value / 100) % 100);
              const uint8_t len2 = (uint8_t)(value % 100);
              if (tcMainTx.sendReset(len0, len1, len2, tens)) {
                activeRecoveryReset = true;
              } else {
                // Even if dispatch itself fails, the recovery-pending
                // lifecycle must always be completed (prevents a
                // permanent lock on the Core1 side).
                // activeRecoveryReset stays false (send never started).
                localFaultLatched = true;
                localSafeEnabled  = true;
                gRecoveryResetAbortCount.fetch_add(1);
#if C4_USB_DEBUG
                Serial.println("[TcTask] recovery sendReset failed (busy) -> treated as ABORT");
#endif
              }
              break;
            }
            if (localFaultLatched) {
#if C4_USB_DEBUG
              Serial.println("[TcTask] FAULT DROP cmd=0x01");
#endif
              break;
            }
            if (localSafeEnabled) {
#if C4_USB_DEBUG
              Serial.println("[TcTask] SAFE DROP cmd=0x01");
#endif
              break;
            }
            const uint32_t value = ((uint32_t)cmd.data[1] << 16)
                                  | ((uint32_t)cmd.data[2] << 8)
                                  |  (uint32_t)cmd.data[3];
            const uint8_t tens = cmd.data[4];
            const uint8_t len0 = (uint8_t)(value / 10000);
            const uint8_t len1 = (uint8_t)((value / 100) % 100);
            const uint8_t len2 = (uint8_t)(value % 100);
            if (!tcMainTx.sendReset(len0, len1, len2, tens)) {
#if C4_USB_DEBUG
              Serial.println("[TcTask] sendReset failed (busy)");
#endif
            }
            break;
          }
          case 0x02: { // SEND (CH1 only; CH3-equivalent defaults to 0)
            if (localFaultLatched) {
#if C4_USB_DEBUG
              Serial.println("[TcTask] FAULT DROP cmd=0x02");
#endif
              break;
            }
            if (localSafeEnabled) {
#if C4_USB_DEBUG
              Serial.println("[TcTask] SAFE DROP cmd=0x02");
#endif
              break;
            }
            if (!tcMainTx.sendTension(cmd.data[1])) {
#if C4_USB_DEBUG
              Serial.println("[TcTask] sendTension failed (busy)");
#endif
            }
            break;
          }
          case 0x03: { // SENS.ADJ (BREAK out of scope for Phase3, always false)
            if (localFaultLatched) {
#if C4_USB_DEBUG
              Serial.println("[TcTask] FAULT DROP cmd=0x03");
#endif
              break;
            }
            if (localSafeEnabled) {
#if C4_USB_DEBUG
              Serial.println("[TcTask] SAFE DROP cmd=0x03");
#endif
              break;
            }
            if (!tcMainTx.sendSensAdj(false)) {
#if C4_USB_DEBUG
              Serial.println("[TcTask] sendSensAdj failed (busy)");
#endif
            }
            break;
          }
          case 0x10: // SAFE ON
            if (localFaultLatched) {
              // SAFE state is not updated while fault-latched
              // (Step3C_fault_recovery設計_最終版.md §3).
#if C4_USB_DEBUG
              Serial.println("[TcTask] FAULT: SAFE ON ignored (latched)");
#endif
              break;
            }
            localSafeEnabled = true;
#if C4_USB_DEBUG
            Serial.println("[TcTask] SAFE ON");
#endif
            break;
          case 0x11: // SAFE OFF
            if (localFaultLatched) {
              // fault latch is NOT cleared by SAFE OFF (the core fix).
#if C4_USB_DEBUG
              Serial.println("[TcTask] FAULT: SAFE OFF ignored (latched)");
#endif
              break;
            }
            localSafeEnabled = false;
#if C4_USB_DEBUG
            Serial.println("[TcTask] SAFE OFF");
#endif
            break;
          default:
            // Should already be rejected by Core1's parser; ignored
            // defensively.
#if C4_USB_DEBUG
            Serial.print("[TcTask] unrecognized command 0x");
            Serial.println(cmd.data[0], HEX);
#endif
            break;
        }
      }
    }

    // --------------------------------------------------------
    // Step 9/10: idle only if there was truly no work this iteration.
    // TX busy must NEVER see a vTaskDelay() here.
    // --------------------------------------------------------
    const bool idleThisLoop =
        !tcMainTx.busy() &&
        !c1RingHasPending() &&
        !piCmdHandled &&
        !doneFlagHandled;

    if (idleThisLoop) {
      vTaskDelay(1);
    }
    // else: loop immediately back to step 1 (TcMainTx.poll()).
  }
}
