/*
  ESP32-S3 TC Bridge RTOS Phase3 Skeleton
  --------------------------------------
  Step4C Stage C4 integrated build (2026-09-01)

  Base:
    Step4C Stage C3 integrated build (2026-09-01)

  Stage C4 change:
    - TC_TEST_MODE=8 added for Original250201 Direction-B Pi forwarding
    - Stage C1 edge capture, C2 byte decode, and C3 frame assembly are retained
    - each validated 6-byte frame is wrapped in TcMessage and queued to qTcToPi
    - Core1 taskPiUart forwards rx.data[0..len-1] to Raspberry Pi as raw 6 bytes
    - queue-full is counted and reported; Core0 never blocks waiting for Pi
    - no GUI; Pi-side CUI monitor is the validation target

  Existing modes are retained for regression/history.
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet_phase3.hpp"
#include "tc_message.hpp"

// ======================================================
// Test / integration mode
// ======================================================
//   1 = Phase1.1 legacy receive test
//   2 = Phase1.2 legacy send test
//   4 = Phase1.3.5-A legacy time-window simultaneous test
//   5 = Step4C Stage C1: Original250201 continuous edge/guard monitor
//   6 = Step4C Stage C2: Original250201 continuous 1-byte decoder
//   7 = Step4C Stage C3: 6-byte frame assembly + 0x7F validation/resync
//   8 = Step4C Stage C4: validated 6-byte frame -> qTcToPi -> Pi UART  <-- current
//   0 = Phase3 legacy command/response model
#define TC_TEST_MODE 8

// ======================================================
// ピン設定
// ======================================================
static constexpr int PIN_PI_RX = D1;  // GPIO2
static constexpr int PIN_PI_TX = D0;  // GPIO1

static constexpr int PIN_TC_RX      = D3;   // GPIO4
static constexpr int PIN_TC_TX_TRIG = D10;  // GPIO9

static constexpr int PIN_STATUS_LED = LED_BUILTIN;

// ======================================================
// 通信パラメータ
// ======================================================
static constexpr uint32_t PI_BAUD = 9600;
static constexpr uint32_t TC_BAUD = 300;
static constexpr uint32_t BIT_US  = 1000000UL / TC_BAUD;

static constexpr bool TC_OC_ACTIVE_LOW = true;

// ======================================================
// 17-slot protocol constants
// ======================================================
static constexpr uint32_t P11_SLOT_US         = 3300;
static constexpr uint8_t  P11_DATA_SLOTS      = 7;
static constexpr uint8_t  P11_SLOTS_PER_BYTE  = 17;
static constexpr uint8_t  P11_FRAME_BYTES     = 6;
static constexpr uint32_t P11_BYTE_US =
  P11_SLOT_US * P11_SLOTS_PER_BYTE; // 56100us

// Legacy Phase1.1 threshold. Kept for old modes 1/4 only.
static constexpr uint32_t P11_FOOTER_MIN_US = 26000;

// Legacy fixed capture buffer. Kept for old modes 1/4 only.
static constexpr uint16_t P11_EDGE_BUF_SIZE = 300;

static volatile uint32_t p11EdgeTimes[P11_EDGE_BUF_SIZE];
static volatile uint8_t  p11EdgeLevels[P11_EDGE_BUF_SIZE];
static volatile uint16_t p11EdgeCount = 0;

// ======================================================
// Step4C Stage C1/C2 continuous ring buffer
// ======================================================
#if TC_TEST_MODE == 5 || TC_TEST_MODE == 6 || TC_TEST_MODE == 7 || TC_TEST_MODE == 8

// Diagnostic-only provisional threshold.
// Original250201 ESP/Main-view:
//   data-only HIGH run <= 7 slots ~= 23100us
//   guard HIGH        >= 9 slots ~= 29700us
// 26000us is deliberately BETWEEN them.
// This is NOT a frozen protocol constant yet.
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

static uint32_t c1LastSummaryMs = 0;

#if TC_TEST_MODE == 6 || TC_TEST_MODE == 7 || TC_TEST_MODE == 8
// ------------------------------------------------------
// Stage C2 byte-local edge history (also reused by C3)
// ------------------------------------------------------
// A 17-slot byte can only produce a small number of transitions.
// 32 gives ample diagnostic margin without touching the ISR ring.
static constexpr uint8_t C2_BYTE_EDGE_MAX = 32;

static bool     c2HaveByteStart = false;
static uint32_t c2ByteStartUs   = 0;

// Edges strictly after c2ByteStartUs and up to the next boundary.
// The next boundary may be present in this array; all sample points
// occur before it, so it does not change the decoded value.
static uint32_t c2ByteEdgeTimes[C2_BYTE_EDGE_MAX];
static uint8_t  c2ByteEdgeLevels[C2_BYTE_EDGE_MAX];
static uint8_t  c2ByteEdgeCount = 0;

static uint32_t c2DecodedBytes      = 0;
static uint32_t c2DecodeErrors      = 0;
static uint32_t c2TimingErrors      = 0;
static uint32_t c2LocalEdgeOverflow = 0;

// Diagnostic only. Production tolerance is NOT frozen in C2.
// Stage C1 measured roughly 55.97..56.04ms versus nominal 56.1ms.
// Keep a deliberately generous window for detecting gross anomalies.
static constexpr uint32_t C2_BYTE_DT_MIN_US = 54000UL;
static constexpr uint32_t C2_BYTE_DT_MAX_US = 58000UL;

static uint32_t c2LastSeenRingOverflow = 0;
#endif

#endif

// ======================================================
// GPIO4 CHANGE ISR
// ======================================================
// ISR remains deliberately minimal.
// No Serial / decode / frame logic in ISR.
static void IRAM_ATTR p11TcRxIsr() {
#if TC_TEST_MODE == 5 || TC_TEST_MODE == 6 || TC_TEST_MODE == 7 || TC_TEST_MODE == 8
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
#else
  if (p11EdgeCount < P11_EDGE_BUF_SIZE) {
    p11EdgeTimes[p11EdgeCount]  = micros();
    p11EdgeLevels[p11EdgeCount] = (uint8_t)digitalRead(PIN_TC_RX);
    p11EdgeCount++;
  }
#endif
}

// ======================================================
// Queue / RTOS objects
// ======================================================
static constexpr uint8_t QUEUE_DEPTH_PI_TO_TC = 8;
static constexpr uint8_t QUEUE_DEPTH_TC_TO_PI = 8;

static constexpr uint32_t TC_RESPONSE_TIMEOUT_MS = 150;

HardwareSerial SerialPi(1);

static QueueHandle_t qPiToTc = nullptr;
static QueueHandle_t qTcToPi = nullptr;
static SemaphoreHandle_t tcBusMutex = nullptr;

#if TC_TEST_MODE == 8
static bool c4ForwardFrameToPi(const uint8_t* frame) {
  if (!qTcToPi || !frame) {
    return false;
  }

  tc::TcMessage m;
  m.timestamp_us = micros();
  m.source = tc::MsgSource::Tc;
  m.type = tc::MsgType::Response;
  m.len = P11_FRAME_BYTES;
  memcpy(m.data, frame, P11_FRAME_BYTES);

  // Direction-B 0x7F is a footer, not a checksum.
  // Do not claim FLAG_CHECKSUM_OK.
  m.flags = tc::FLAG_NONE;

  // Continuous TC receive must not be stalled by a slow/disconnected Pi.
  return xQueueSend(qTcToPi, &m, 0) == pdTRUE;
}
#endif

// ======================================================
// Utilities
// ======================================================
static void debugDumpMessage(const char* tag, const tc::TcMessage& m) {
  Serial.print(tag);
  Serial.print(" len=");
  Serial.print(m.len);
  Serial.print(" flags=0x");
  Serial.print(m.flags, HEX);
  Serial.print(" data=");
  m.dumpTo(Serial);
  Serial.println();
}

static void setStatusLed(bool on) {
#ifdef LED_BUILTIN
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
#endif
}

// ======================================================
// Legacy TC physical TX
// ======================================================
static void tcWriteLogicalBit(bool logicalHigh) {
  if (TC_OC_ACTIVE_LOW) {
    digitalWrite(PIN_TC_TX_TRIG, logicalHigh ? LOW : HIGH);
  } else {
    digitalWrite(PIN_TC_TX_TRIG, logicalHigh ? HIGH : LOW);
  }

  delayMicroseconds(BIT_US);
}

static void tcSend9BitByte(uint8_t data, bool isCommand) {
  tcWriteLogicalBit(false);

  for (uint8_t i = 0; i < 8; i++) {
    tcWriteLogicalBit((data >> i) & 0x01);
  }

  tcWriteLogicalBit(isCommand);
  tcWriteLogicalBit(true);
}

static bool tcSendFrame(const tc::TcMessage& msg) {
  if (msg.len == 0) return false;

  tc::Packet p;
  p.len = min<uint8_t>(msg.len, tc::PI_MAX);
  memcpy(p.buf, msg.data, p.len);

  tc::TcFrames f = tc::toTcFrames(p);

  for (uint8_t i = 0; i < tc::TC_DATA_LEN; i++) {
    tcSend9BitByte(f.data[i], false);
  }

  tcSend9BitByte(f.cmd, true);
  return true;
}

// ======================================================
// Legacy Phase1.1 receive implementation
// ======================================================
// Retained only for regression modes 1/4.
// IMPORTANT: this receiver assumes the historical opposite polarity/burst model.
// It is NOT the Step4C Original250201 receiver.
static bool p11TryReceiveFrame(uint8_t out[P11_FRAME_BYTES], uint32_t timeoutMs) {
  const uint32_t giveUpAtMs = millis() + timeoutMs;

  noInterrupts();
  p11EdgeCount = 0;
  interrupts();

  while (true) {
    if ((int32_t)(millis() - giveUpAtMs) > 0) return false;
    noInterrupts();
    uint16_t n = p11EdgeCount;
    interrupts();
    if (n > 0) break;
    vTaskDelay(1);
  }

  const uint32_t QUIET_MS = 50;
  uint16_t lastCount = 0;
  uint32_t lastChangeAtMs = millis();

  while (true) {
    if ((int32_t)(millis() - giveUpAtMs) > 0) return false;

    noInterrupts();
    uint16_t n = p11EdgeCount;
    interrupts();

    if (n != lastCount) {
      lastCount = n;
      lastChangeAtMs = millis();
    } else if (millis() - lastChangeAtMs > QUIET_MS) {
      break;
    }
    vTaskDelay(1);
  }

  static uint32_t times[P11_EDGE_BUF_SIZE];
  static uint8_t  levels[P11_EDGE_BUF_SIZE];
  uint16_t n;

  noInterrupts();
  n = p11EdgeCount;
  for (uint16_t i = 0; i < n; i++) {
    times[i]  = p11EdgeTimes[i];
    levels[i] = p11EdgeLevels[i];
  }
  p11EdgeCount = 0;
  interrupts();

  if (n < 2) return false;

  uint32_t footerEnd[P11_FRAME_BYTES];
  uint8_t footerCount = 0;

  for (uint16_t i = 0; i + 1 < n && footerCount < P11_FRAME_BYTES; i++) {
    if (levels[i] == LOW) {
      uint32_t dur = times[i + 1] - times[i];
      if (dur >= P11_FOOTER_MIN_US) {
        footerEnd[footerCount++] = times[i + 1];
      }
    }
  }

  if (footerCount < P11_FRAME_BYTES - 1) {
    return false;
  }

  uint32_t byteT0[P11_FRAME_BYTES];
  byteT0[0] = footerEnd[0] - P11_BYTE_US;

  for (uint8_t b = 1; b < P11_FRAME_BYTES; b++) {
    byteT0[b] = footerEnd[b - 1];
  }

  auto levelAt = [&](uint32_t t) -> int {
    if (n == 0 || t < times[0]) return HIGH;
    int lvl = HIGH;

    for (uint16_t i = 0; i < n; i++) {
      if (times[i] <= t) {
        lvl = levels[i];
      } else {
        break;
      }
    }
    return lvl;
  };

  for (uint8_t b = 0; b < P11_FRAME_BYTES; b++) {
    uint8_t value = 0;

    for (uint8_t k = 1; k <= P11_DATA_SLOTS; k++) {
      uint32_t target =
        byteT0[b] + (uint32_t)k * P11_SLOT_US + (P11_SLOT_US / 2);

      if (levelAt(target) == LOW) {
        value |= (uint8_t)(1u << (k - 1));
      }
    }

    out[b] = value;
  }

  return true;
}

// ======================================================
// Legacy Phase1.2 send implementation
// ======================================================
static void p11SetBusLogical(bool logicalHigh) {
  if (TC_OC_ACTIVE_LOW) {
    digitalWrite(PIN_TC_TX_TRIG, logicalHigh ? LOW : HIGH);
  } else {
    digitalWrite(PIN_TC_TX_TRIG, logicalHigh ? HIGH : LOW);
  }
}

static void p11WaitUntilMicros(uint32_t targetMicros) {
  while ((int32_t)(micros() - targetMicros) < 0) {
    // busy wait
  }
}

static void p11SendByte17Slot(uint8_t value) {
  const uint32_t t0 = micros();

  p11SetBusLogical(true);
  p11WaitUntilMicros(t0 + P11_SLOT_US);

  for (uint8_t k = 1; k <= P11_DATA_SLOTS; k++) {
    const bool bitIsOne = (value >> (k - 1)) & 0x01;
    p11SetBusLogical(!bitIsOne);
    p11WaitUntilMicros(t0 + (uint32_t)(k + 1) * P11_SLOT_US);
  }

  p11SetBusLogical(false);
  p11WaitUntilMicros(
    t0 + (uint32_t)P11_SLOTS_PER_BYTE * P11_SLOT_US
  );
}

static void p11SendFrame17Slot(const uint8_t data[P11_FRAME_BYTES]) {
  for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
    p11SendByte17Slot(data[i]);
  }

  p11SetBusLogical(true);
}

static bool tcReceiveFrame(tc::TcMessage& out, uint32_t timeoutMs) {
  (void)out;
  (void)timeoutMs;
  return false;
}

// ======================================================
// Step4C C1/C2 helpers
// ======================================================
#if TC_TEST_MODE == 5 || TC_TEST_MODE == 6 || TC_TEST_MODE == 7 || TC_TEST_MODE == 8

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

#if TC_TEST_MODE == 5

static void c1ProcessEdge(uint32_t t, uint8_t lvl) {
  c1ConsumedEdgeCount++;

  if (!c1HavePrevEdge) {
    c1HavePrevEdge = true;
    c1PrevEdgeUs   = t;
    c1PrevLevel    = lvl;
    return;
  }

  const uint32_t dt = t - c1PrevEdgeUs;

  if ((c1PrevLevel == HIGH) &&
      (lvl == LOW) &&
      (dt >= C1_GUARD_MIN_HIGH_US)) {

    c1GuardCount++;

    Serial.print("[C1 GUARD] #");
    Serial.print(c1GuardCount);
    Serial.print(" high_us=");
    Serial.print(dt);
    Serial.print(" high_slots=");
    Serial.print((double)dt / (double)P11_SLOT_US, 2);

    if (c1HavePrevGuard) {
      const uint32_t boundaryDt = t - c1PrevGuardUs;
      Serial.print(" boundary_dt_us=");
      Serial.print(boundaryDt);
      Serial.print(" byte_slots=");
      Serial.print((double)boundaryDt / (double)P11_SLOT_US, 2);
    } else {
      Serial.print(" boundary_dt_us=FIRST");
    }

    Serial.println();

    c1PrevGuardUs   = t;
    c1HavePrevGuard = true;
  }

  c1PrevEdgeUs = t;
  c1PrevLevel  = lvl;
}

static void c1PrintSummaryIfDue() {
  const uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - c1LastSummaryMs) < 1000UL) {
    return;
  }
  c1LastSummaryMs = nowMs;

  uint16_t w;
  uint16_t r;
  uint32_t isrEdges;
  uint32_t overflow;

  portENTER_CRITICAL(&c1RingMux);
  w = c1WriteIndex;
  r = c1ReadIndex;
  isrEdges = c1IsrEdgeCount;
  overflow = c1EdgeOverflowCount;
  portEXIT_CRITICAL(&c1RingMux);

  const uint16_t pending =
    (uint16_t)((w - r) & C1_EDGE_RING_MASK);

  Serial.print("[C1] isr_edges=");
  Serial.print(isrEdges);
  Serial.print(" consumed=");
  Serial.print(c1ConsumedEdgeCount);
  Serial.print(" guard_candidates=");
  Serial.print(c1GuardCount);
  Serial.print(" pending=");
  Serial.print(pending);
  Serial.print(" overflow=");
  Serial.println(overflow);
}

#endif // TC_TEST_MODE == 5

#if TC_TEST_MODE == 6 || TC_TEST_MODE == 7 || TC_TEST_MODE == 8

#if TC_TEST_MODE == 7 || TC_TEST_MODE == 8
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

#if TC_TEST_MODE == 8
static uint32_t c4QueuedFrames = 0;
static uint32_t c4QueueDrops   = 0;
#endif

static void c3InvalidateFrame(const char* reason) {
  if (c3Aligned || c3DataCount != 0 || c3BadSpan) {
    c3FrameErrors++;
  }
  c3Aligned = false;
  c3DataCount = 0;
  c3BadSpan = false;
  c3DiscardUntilFooter = true;

  Serial.print("[C3 INVALID] ");
  Serial.println(reason);
}

static void c3PrintFrame() {
  Serial.print("[C3 FRAME] #");
  Serial.print(c3ValidFrames);
  Serial.print(" ");
  for (uint8_t i = 0; i < C3_FRAME_BYTES; i++) {
    if (c3Frame[i] < 0x10) Serial.print('0');
    Serial.print(c3Frame[i], HEX);
    if (i + 1u < C3_FRAME_BYTES) Serial.print(' ');
  }
  Serial.println();
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
      Serial.println("[C3 RESYNC] footer found after lower-layer invalidation");
    }
    return;
  }

  if (value == C3_FOOTER) {
    if (c3DataCount == 5 && !c3BadSpan) {
      c3Frame[5] = C3_FOOTER;
      c3ValidFrames++;
      c3Aligned = true;
      c3PrintFrame();

#if TC_TEST_MODE == 8
      if (c4ForwardFrameToPi(c3Frame)) {
        c4QueuedFrames++;
      } else {
        c4QueueDrops++;
      }
#endif
    } else {
      // Cold start may legitimately enter in the middle of a frame.
      // This footer establishes the phase for the NEXT frame.
      c3SyncMisses++;
      if (c3Aligned || c3BadSpan) {
        c3FrameErrors++;
      }
      c3Resyncs++;
      Serial.print("[C3 RESYNC] footer at data_count=");
      Serial.println(c3DataCount);
      c3Aligned = true;
    }

    c3DataCount = 0;
    c3BadSpan = false;
    return;
  }

  if (c3DataCount < 5) {
    c3Frame[c3DataCount++] = value;
  } else {
    // More than five non-footer bytes means frame phase is broken.
    // Do not slide a 6-byte window; wait for the unique footer instead.
    c3BadSpan = true;
    if (c3DataCount < 0xFF) c3DataCount++;
  }
}
#endif

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

    uint8_t level = LOW; // slot0 begins LOW at byte boundary.

    for (uint8_t i = 0; i < c2ByteEdgeCount; i++) {
      // Signed subtraction is safe for these short (<60ms) intervals
      // and remains correct across micros() wrap.
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
#if TC_TEST_MODE == 7 || TC_TEST_MODE == 8
    c3InvalidateFrame("byte-local edge overflow");
#endif
    c2ResetByteState();
  }
}

static void c2HandleBoundary(uint32_t boundaryUs, uint32_t highUs) {
  if (!c2HaveByteStart) {
    // Cold-start: this boundary only establishes the first byte start.
    c2StartNewByte(boundaryUs);
    c1GuardCount++;
    c1PrevGuardUs   = boundaryUs;
    c1HavePrevGuard = true;

    Serial.print("[C2 SYNC] first_boundary_us=");
    Serial.print(boundaryUs);
    Serial.print(" preceding_high_us=");
    Serial.println(highUs);
    return;
  }

  const uint32_t boundaryDt = boundaryUs - c2ByteStartUs;

  if (boundaryDt < C2_BYTE_DT_MIN_US ||
      boundaryDt > C2_BYTE_DT_MAX_US) {
    c2TimingErrors++;
#if TC_TEST_MODE == 7 || TC_TEST_MODE == 8
    c3InvalidateFrame("byte boundary timing error");
#endif
  }

  uint8_t value = 0;
  if (c2DecodeCurrentByte(value)) {
    c2DecodedBytes++;

#if TC_TEST_MODE == 7 || TC_TEST_MODE == 8
    c3ConsumeByte(value);
#else
    Serial.print("[C2 BYTE] #");
    Serial.print(c2DecodedBytes);
    Serial.print(" value=");
    if (value < 0x10) Serial.print('0');
    Serial.print(value, HEX);
    Serial.print(" start_us=");
    Serial.print(c2ByteStartUs);
    Serial.print(" boundary_dt_us=");
    Serial.print(boundaryDt);
    Serial.print(" byte_slots=");
    Serial.print((double)boundaryDt / (double)P11_SLOT_US, 2);
    Serial.print(" edges=");
    Serial.println(c2ByteEdgeCount);
#endif
  } else {
    c2DecodeErrors++;
#if TC_TEST_MODE == 7 || TC_TEST_MODE == 8
    c3InvalidateFrame("byte decode failure");
#endif
  }

  c1GuardCount++;
  c1PrevGuardUs   = boundaryUs;
  c1HavePrevGuard = true;

  // Current falling edge is slot0 of the NEXT byte.
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
#if TC_TEST_MODE == 7 || TC_TEST_MODE == 8
    c3InvalidateFrame("ISR ring overflow");
#endif
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

static void c2PrintSummaryIfDue() {
  const uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - c1LastSummaryMs) < 1000UL) {
    return;
  }
  c1LastSummaryMs = nowMs;

  uint16_t w;
  uint16_t r;
  uint32_t isrEdges;
  uint32_t overflow;

  portENTER_CRITICAL(&c1RingMux);
  w = c1WriteIndex;
  r = c1ReadIndex;
  isrEdges = c1IsrEdgeCount;
  overflow = c1EdgeOverflowCount;
  portEXIT_CRITICAL(&c1RingMux);

  const uint16_t pending =
    (uint16_t)((w - r) & C1_EDGE_RING_MASK);

#if TC_TEST_MODE == 7 || TC_TEST_MODE == 8
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
#if TC_TEST_MODE == 8
  Serial.print(" queued=");
  Serial.print(c4QueuedFrames);
  Serial.print(" queue_drops=");
  Serial.print(c4QueueDrops);
#endif
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
#else
  Serial.print("[C2] isr_edges=");
  Serial.print(isrEdges);
  Serial.print(" consumed=");
  Serial.print(c1ConsumedEdgeCount);
  Serial.print(" bytes=");
  Serial.print(c2DecodedBytes);
  Serial.print(" guards=");
  Serial.print(c1GuardCount);
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
#endif
}

#endif // TC_TEST_MODE == 6 || TC_TEST_MODE == 7 || TC_TEST_MODE == 8

#endif // TC_TEST_MODE == 5 || TC_TEST_MODE == 6 || TC_TEST_MODE == 7 || TC_TEST_MODE == 8

// ======================================================
// Core1: Raspberry Pi side task
// ======================================================
static void taskPiUart(void* pv) {
  uint8_t ring[tc::RING_SIZE]{};
  uint8_t head = 0;
  uint64_t lastSig = 0;

  // Step4C C4 Pi UART diagnostic.
  // Non-invasive scheduling: keep the normal qTcToPi drain active while
  // transmitting a fixed test frame 5 times after startup.
  static constexpr uint32_t PI_UART_DIAG_START_DELAY_MS = 5000;
  static constexpr uint32_t PI_UART_DIAG_INTERVAL_MS    = 2000;
  static constexpr uint8_t  PI_UART_DIAG_SEND_COUNT     = 5;
  static const uint8_t PI_UART_DIAG_FRAME[6] = {
    0x11, 0x22, 0x33, 0x44, 0x55, 0x7F
  };

  const uint32_t piTaskStartedMs = millis();
  uint32_t nextDiagMs = piTaskStartedMs + PI_UART_DIAG_START_DELAY_MS;
  uint8_t diagSentCount = 0;

  Serial.println("[PiTask] start on Core1");
  Serial.println("[PiTask DIAG] fixed 6-byte UART test armed");

  while (true) {
    while (SerialPi.available()) {
      ring[head] = static_cast<uint8_t>(SerialPi.read());
      head = (head + 1) & (tc::RING_SIZE - 1);

      tc::Packet p;

      if (tc::PacketFactory::tryParse(ring, head, lastSig, p)) {
        tc::TcMessage msg = tc::TcMessage::fromPacket(
          p, tc::MsgSource::Pi, tc::MsgType::Command
        );

        if (xQueueSend(qPiToTc, &msg, 0) != pdTRUE) {
          Serial.println("[PiTask] qPiToTc full, dropping packet");
        } else {
          debugDumpMessage("[PiTask RX]", msg);
        }
      }
    }

    tc::TcMessage rx;

    while (xQueueReceive(qTcToPi, &rx, 0) == pdTRUE) {
      debugDumpMessage("[PiTask TX]", rx);

      if (rx.len > 0) {
        SerialPi.write(rx.data, rx.len);
        SerialPi.flush();
      }
    }

    // Diagnostic transmission is deliberately interleaved with the existing
    // receive/queue-drain loop.  Do not block taskPiUart for 5-15 seconds:
    // qTcToPi has finite depth and normal C4 forwarding must keep running.
    if (diagSentCount < PI_UART_DIAG_SEND_COUNT) {
      const uint32_t nowMs = millis();
      if ((int32_t)(nowMs - nextDiagMs) >= 0) {
        const size_t written =
          SerialPi.write(PI_UART_DIAG_FRAME, sizeof(PI_UART_DIAG_FRAME));
        SerialPi.flush();

        diagSentCount++;

        // USB CDC log is auxiliary only. Keep it short so the diagnostic
        // itself does not depend on host-side Serial monitor behaviour.
        Serial.print("[PiTask DIAG] #");
        Serial.print(diagSentCount);
        Serial.print(" write_result=");
        Serial.println(written);

        nextDiagMs += PI_UART_DIAG_INTERVAL_MS;
      }
    }

    vTaskDelay(1);
  }
}

// ======================================================
// Core0: TC106 side task
// ======================================================
#if TC_TEST_MODE == 2 || TC_TEST_MODE == 4
static const uint8_t kP12TestFrames[][P11_FRAME_BYTES] = {
  {0x00, 0x64, 0x00, 0x64, 0x00, 0x7F},
  {0x11, 0x22, 0x33, 0x44, 0x55, 0x7F},
  {0x01, 0x00, 0x00, 0x1E, 0x00, 0x7F},
  {0x64, 0x00, 0x00, 0x32, 0x00, 0x7F},
};

static constexpr uint8_t kP12TestFrameCount =
  sizeof(kP12TestFrames) / sizeof(kP12TestFrames[0]);
#endif

#if TC_TEST_MODE == 4
static constexpr uint32_t P135_RX_WINDOW_MS = 450;
static constexpr uint32_t P135_TX_START_MS  = 500;
static constexpr uint32_t P135_CYCLE_MS     = 1000;
#endif

static void taskTcBus(void* pv) {
  Serial.println("[TcTask] start on Core0");

  digitalWrite(PIN_TC_TX_TRIG, LOW);

#if TC_TEST_MODE == 1

  Serial.println("[TcTask] Phase1.1 standalone listen mode");

  uint8_t frame[P11_FRAME_BYTES];
  uint32_t rxSeq = 0;

  while (true) {
    if (p11TryReceiveFrame(frame, 3000)) {
      setStatusLed(true);
      rxSeq++;

      Serial.print('[');
      Serial.print(millis());
      Serial.print(" ms] [TcTask RX] #");
      Serial.print(rxSeq);
      Serial.print(" ");

      for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
        if (frame[i] < 0x10) Serial.print('0');
        Serial.print(frame[i], HEX);
        Serial.print(' ');
      }

      Serial.println();
      setStatusLed(false);
    }
  }

#elif TC_TEST_MODE == 2

  Serial.println("[TcTask] Phase1.2 standalone send mode");

  uint8_t idx = 0;
  uint32_t txSeq = 0;

  while (true) {
    const uint8_t* f = kP12TestFrames[idx];

    setStatusLed(true);
    p11SendFrame17Slot(f);
    setStatusLed(false);

    txSeq++;

    Serial.print('[');
    Serial.print(millis());
    Serial.print(" ms] [TcTask TX] #");
    Serial.print(txSeq);
    Serial.print(" ");

    for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
      if (f[i] < 0x10) Serial.print('0');
      Serial.print(f[i], HEX);
      Serial.print(' ');
    }

    Serial.println();

    idx = (idx + 1) % kP12TestFrameCount;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

#elif TC_TEST_MODE == 4

  Serial.println("[TcTask] Phase1.3.5-A time-windowed simultaneous mode");

  uint8_t txIdx = 0;
  uint32_t txSeq = 0;
  uint32_t rxSeq = 0;
  uint32_t cycleStart = millis();

  while (true) {
    uint32_t elapsed = millis() - cycleStart;

    if (elapsed < P135_RX_WINDOW_MS) {
      uint32_t remaining = P135_RX_WINDOW_MS - elapsed;
      uint8_t frame[P11_FRAME_BYTES];

      if (p11TryReceiveFrame(frame, remaining)) {
        rxSeq++;

        Serial.print('[');
        Serial.print(millis());
        Serial.print(" ms] [RX] #");
        Serial.print(rxSeq);
        Serial.print(" ");

        for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
          if (frame[i] < 0x10) Serial.print('0');
          Serial.print(frame[i], HEX);
          Serial.print(' ');
        }

        Serial.println();
      }

      while (millis() - cycleStart < P135_RX_WINDOW_MS) {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }

    while (millis() - cycleStart < P135_TX_START_MS) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    const uint8_t* f = kP12TestFrames[txIdx];

    setStatusLed(true);
    txSeq++;

    Serial.print('[');
    Serial.print(millis());
    Serial.print(" ms] [TX] #");
    Serial.print(txSeq);
    Serial.print(" ");

    for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
      if (f[i] < 0x10) Serial.print('0');
      Serial.print(f[i], HEX);
      Serial.print(' ');
    }

    Serial.println();

    p11SendFrame17Slot(f);
    setStatusLed(false);

    txIdx = (txIdx + 1) % kP12TestFrameCount;

    while (millis() - cycleStart < P135_CYCLE_MS) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    cycleStart = millis();
  }

#elif TC_TEST_MODE == 5

  Serial.println("[TcTask] Step4C Stage C1 Original250201 continuous RX");
  Serial.println("[TcTask] GPIO4 CHANGE -> ring buffer -> long HIGH / falling-edge monitor");
  Serial.print("[TcTask] provisional guard threshold = ");
  Serial.print(C1_GUARD_MIN_HIGH_US);
  Serial.println(" us (NOT frozen spec)");
  Serial.println("[TcTask] NO byte decode / NO frame decode / NO Pi forwarding");

  c1LastSummaryMs = millis();

  while (true) {
    uint32_t t;
    uint8_t lvl;

    while (c1PopEdge(t, lvl)) {
      c1ProcessEdge(t, lvl);
    }

    c1PrintSummaryIfDue();
    vTaskDelay(pdMS_TO_TICKS(1));
  }

#elif TC_TEST_MODE == 6

  Serial.println("[TcTask] Step4C Stage C2 Original250201 1-byte decoder");
  Serial.println("[TcTask] boundary = long HIGH -> falling edge");
  Serial.println("[TcTask] decode = captured edge history, 7-bit LSB first, LOW=0 HIGH=1");
  Serial.print("[TcTask] provisional guard threshold = ");
  Serial.print(C1_GUARD_MIN_HIGH_US);
  Serial.println(" us (NOT frozen spec)");
  Serial.println("[TcTask] NO 6-byte frame assembly / NO Pi forwarding");

  c1LastSummaryMs = millis();

  while (true) {
    uint32_t t;
    uint8_t lvl;

    while (c1PopEdge(t, lvl)) {
      c2ProcessEdge(t, lvl);
    }

    c2PrintSummaryIfDue();
    vTaskDelay(pdMS_TO_TICKS(1));
  }

#elif TC_TEST_MODE == 7

  Serial.println("[TcTask] Step4C Stage C3 Original250201 6-byte frame assembler");
  Serial.println("[TcTask] lower layer = Stage C2 7-bit LSB-first decoder");
  Serial.println("[TcTask] frame = 5 data bytes + footer 0x7F");
  Serial.println("[TcTask] footer validates phase and re-synchronizes after cold/mid-stream start");
  Serial.println("[TcTask] NO Pi forwarding (Stage C4)");

  c1LastSummaryMs = millis();

  while (true) {
    uint32_t t;
    uint8_t lvl;

    while (c1PopEdge(t, lvl)) {
      c2ProcessEdge(t, lvl);
    }

    c2PrintSummaryIfDue();
    vTaskDelay(pdMS_TO_TICKS(1));
  }

#elif TC_TEST_MODE == 8

  Serial.println("[TcTask] Step4C Stage C4 Original250201 Pi forwarding");
  Serial.println("[TcTask] C3 valid frame -> TcMessage -> qTcToPi -> Core1 Pi UART");
  Serial.println("[TcTask] Pi payload = raw 6 bytes, footer 0x7F");
  Serial.println("[TcTask] qTcToPi send is non-blocking; queue drops are counted");

  c1LastSummaryMs = millis();

  while (true) {
    uint32_t t;
    uint8_t lvl;

    while (c1PopEdge(t, lvl)) {
      c2ProcessEdge(t, lvl);
    }

    c2PrintSummaryIfDue();
    vTaskDelay(pdMS_TO_TICKS(1));
  }

#else

  while (true) {
    tc::TcMessage cmd;

    if (xQueueReceive(qPiToTc, &cmd, portMAX_DELAY) == pdTRUE) {
      debugDumpMessage("[TcTask CMD]", cmd);

      if (tcBusMutex) {
        xSemaphoreTake(tcBusMutex, portMAX_DELAY);
      }

      setStatusLed(true);

      const bool sent = tcSendFrame(cmd);

      tc::TcMessage resp;
      const bool gotResp =
        sent && tcReceiveFrame(resp, TC_RESPONSE_TIMEOUT_MS);

      setStatusLed(false);

      if (tcBusMutex) {
        xSemaphoreGive(tcBusMutex);
      }

      if (!gotResp) {
        resp = tc::TcMessage::timeout(tc::MsgSource::Tc);
      }

      if (xQueueSend(qTcToPi, &resp, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("[TcTask] qTcToPi full, dropping response");
      }
    }
  }

#endif
}

// ======================================================
// setup / loop
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== ESP32-S3 TC Bridge RTOS Phase3 / Step4C C4 PiUART-DIAG ===");
  Serial.println("Core0=TC, Core1=Pi UART");
  Serial.print("TC_TEST_MODE=");
  Serial.println(TC_TEST_MODE);

  pinMode(PIN_STATUS_LED, OUTPUT);
  setStatusLed(false);

  pinMode(PIN_TC_TX_TRIG, OUTPUT);
  digitalWrite(PIN_TC_TX_TRIG, LOW);

  // Keep existing hardware assumption for now.
  // Stage C1/C2/C3/C4 only observe GPIO4; they never drive GPIO4.
  pinMode(PIN_TC_RX, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(PIN_TC_RX),
    p11TcRxIsr,
    CHANGE
  );

  SerialPi.begin(
    PI_BAUD,
    SERIAL_8N1,
    PIN_PI_RX,
    PIN_PI_TX
  );

  qPiToTc = xQueueCreate(
    QUEUE_DEPTH_PI_TO_TC,
    sizeof(tc::TcMessage)
  );

  qTcToPi = xQueueCreate(
    QUEUE_DEPTH_TC_TO_PI,
    sizeof(tc::TcMessage)
  );

  tcBusMutex = xSemaphoreCreateMutex();

  if (!qPiToTc || !qTcToPi || !tcBusMutex) {
    Serial.println("[FATAL] RTOS object creation failed");

    while (true) {
      setStatusLed(true);
      delay(100);
      setStatusLed(false);
      delay(100);
    }
  }

  xTaskCreatePinnedToCore(
    taskTcBus,
    "TcBus",
    4096,
    nullptr,
    5,
    nullptr,
    0
  );

  xTaskCreatePinnedToCore(
    taskPiUart,
    "PiUart",
    4096,
    nullptr,
    3,
    nullptr,
    1
  );

  Serial.println("[setup] tasks created");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
