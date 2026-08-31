/**
 * ESP32_GPIO4_EdgeLogger_Step4C_C1_GuardMonitor.ino
 *
 * Step4C Stage C1 diagnostic sketch.
 *
 * Purpose:
 *   - Continuously capture GPIO4 CHANGE edges from TC106 Direction-B.
 *   - Replace the Step4B one-shot/fixed-buffer model with a continuous ring buffer.
 *   - Detect candidate byte boundaries as:
 *
 *         sufficiently-long HIGH  ->  falling edge (H -> L)
 *
 *     which corresponds to the Original 250201 ESP/Main-view:
 *
 *         guard HIGH -> slot0 LOW
 *
 *   - NO byte decode.
 *   - NO 6-byte frame decode.
 *   - NO Pi forwarding.
 *   - GPIO4 is INPUT ONLY.
 *
 * Important:
 *   GUARD_THRESHOLD_US is a provisional Stage-C1 diagnostic threshold.
 *   It is NOT yet a frozen protocol constant.
 *
 * Expected reference:
 *   slot ~= 3300 us
 *   guard = 9 slots ~= 29700 us minimum when the HIGH begins at slot8.
 *   A data-only HIGH run is at most 7 slots ~= 23100 us.
 *
 *   Therefore 26000 us is used only as a provisional separation threshold.
 */

#include <Arduino.h>

// ============================================================
// Configuration
// ============================================================
static const uint8_t  GPIO4_PIN = 4;

static const uint32_t SLOT_US = 3300UL;

// Diagnostic-only provisional threshold.
// Do not promote this value to the production protocol specification
// until Nano Original250201 runtime measurements are reviewed.
static const uint32_t GUARD_THRESHOLD_US = 26000UL;

// Power-of-two size to make wrapping cheap.
// Effective capacity is RING_SIZE - 1 entries.
static const uint16_t RING_SIZE = 1024;
static const uint16_t RING_MASK = RING_SIZE - 1;

static_assert((RING_SIZE & (RING_SIZE - 1)) == 0,
              "RING_SIZE must be a power of two");

// ============================================================
// Edge ring buffer
// ============================================================
static uint32_t edgeTimeUs[RING_SIZE];
static uint8_t  edgeLevel[RING_SIZE];

static volatile uint16_t ringWrite = 0;  // ISR producer
static volatile uint16_t ringRead  = 0;  // loop consumer

static volatile uint32_t isrEdgeCount      = 0;
static volatile uint32_t ringOverflowCount = 0;

static portMUX_TYPE muxRing = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// Consumer / diagnostic state
// ============================================================
static bool     havePreviousEdge = false;
static uint32_t previousEdgeUs   = 0;
static uint8_t  previousLevel    = 0;

static bool     havePreviousGuard = false;
static uint32_t previousGuardUs   = 0;

static uint32_t guardCandidateCount = 0;
static uint32_t consumerEdgeCount   = 0;

static uint32_t lastSummaryMs = 0;

// ============================================================
// ISR
// ============================================================
// ISR responsibility is deliberately minimal:
//   1) timestamp
//   2) level
//   3) append to ring
//   4) overflow counter
//
// No Serial, decode, guard timing judgement, or frame processing here.
void IRAM_ATTR onGpio4Change() {
    const uint32_t t   = micros();
    const uint8_t  lvl = (uint8_t)digitalRead(GPIO4_PIN);

    portENTER_CRITICAL_ISR(&muxRing);

    const uint16_t w    = ringWrite;
    const uint16_t next = (uint16_t)((w + 1u) & RING_MASK);

    if (next == ringRead) {
        // Ring full: preserve unread data and drop this new edge.
        ringOverflowCount++;
        portEXIT_CRITICAL_ISR(&muxRing);
        return;
    }

    edgeTimeUs[w] = t;
    edgeLevel[w]  = lvl;
    ringWrite     = next;
    isrEdgeCount++;

    portEXIT_CRITICAL_ISR(&muxRing);
}

// ============================================================
// Ring pop
// ============================================================
static bool popEdge(uint32_t &t, uint8_t &lvl) {
    bool ok = false;

    portENTER_CRITICAL(&muxRing);

    const uint16_t r = ringRead;
    if (r != ringWrite) {
        t   = edgeTimeUs[r];
        lvl = edgeLevel[r];
        ringRead = (uint16_t)((r + 1u) & RING_MASK);
        ok = true;
    }

    portEXIT_CRITICAL(&muxRing);
    return ok;
}

// ============================================================
// Reset diagnostics
// ============================================================
static void resetDiagnostics() {
    portENTER_CRITICAL(&muxRing);
    ringRead  = ringWrite;   // discard only currently unread diagnostic data
    ringOverflowCount = 0;
    isrEdgeCount = 0;
    portEXIT_CRITICAL(&muxRing);

    havePreviousEdge  = false;
    previousEdgeUs    = 0;
    previousLevel     = 0;

    havePreviousGuard = false;
    previousGuardUs   = 0;

    guardCandidateCount = 0;
    consumerEdgeCount   = 0;

    Serial.println();
    Serial.println("=== C1 counters/reset: waiting for continuous guard boundaries ===");
}

// ============================================================
// Edge consumer
// ============================================================
static void processEdge(uint32_t t, uint8_t lvl) {
    consumerEdgeCount++;

    if (!havePreviousEdge) {
        havePreviousEdge = true;
        previousEdgeUs   = t;
        previousLevel    = lvl;
        return;
    }

    const uint32_t dt = t - previousEdgeUs;

    // A falling edge means the signal was HIGH since the previous edge.
    // If that HIGH interval is sufficiently long, this is a candidate
    // guard HIGH -> slot0 LOW boundary.
    if ((previousLevel == HIGH) &&
        (lvl == LOW) &&
        (dt >= GUARD_THRESHOLD_US)) {

        guardCandidateCount++;

        Serial.print("GUARD#");
        Serial.print(guardCandidateCount);
        Serial.print("  high_us=");
        Serial.print(dt);
        Serial.print("  high_slots=");
        Serial.print((double)dt / (double)SLOT_US, 2);

        if (havePreviousGuard) {
            const uint32_t boundaryDt = t - previousGuardUs;
            Serial.print("  boundary_dt_us=");
            Serial.print(boundaryDt);
            Serial.print("  byte_slots=");
            Serial.print((double)boundaryDt / (double)SLOT_US, 2);
        } else {
            Serial.print("  boundary_dt_us=FIRST");
        }

        Serial.println();

        previousGuardUs   = t;
        havePreviousGuard = true;
    }

    previousEdgeUs = t;
    previousLevel  = lvl;
}

// ============================================================
// Periodic summary
// ============================================================
static void printSummaryIfDue() {
    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastSummaryMs) < 1000UL) {
        return;
    }
    lastSummaryMs = nowMs;

    uint32_t isrEdges = 0;
    uint32_t overflow = 0;
    uint16_t w = 0;
    uint16_t r = 0;

    portENTER_CRITICAL(&muxRing);
    isrEdges = isrEdgeCount;
    overflow = ringOverflowCount;
    w = ringWrite;
    r = ringRead;
    portEXIT_CRITICAL(&muxRing);

    const uint16_t pending =
        (uint16_t)((w - r) & RING_MASK);

    Serial.print("[C1] isr_edges=");
    Serial.print(isrEdges);
    Serial.print(" consumed=");
    Serial.print(consumerEdgeCount);
    Serial.print(" guard_candidates=");
    Serial.print(guardCandidateCount);
    Serial.print(" pending=");
    Serial.print(pending);
    Serial.print(" overflow=");
    Serial.println(overflow);
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(GPIO4_PIN, INPUT);

    Serial.println();
    Serial.println("=== ESP32 Step4C Stage C1 Guard Monitor ===");
    Serial.println("GPIO4 continuous CHANGE-edge capture / ring buffer / guard-boundary diagnostic.");
    Serial.println("NO byte decode. NO frame decode. NO GPIO4 output.");
    Serial.print("Provisional guard threshold: ");
    Serial.print(GUARD_THRESHOLD_US);
    Serial.println(" us (diagnostic only, NOT frozen spec)");
    Serial.println("Expected boundary signature: long HIGH -> FALLING edge -> slot0 LOW");
    Serial.println("Send 'r' to clear counters/resync diagnostics.");

    attachInterrupt(digitalPinToInterrupt(GPIO4_PIN), onGpio4Change, CHANGE);

    lastSummaryMs = millis();
}

void loop() {
    // Drain continuously. No quiet-time wait and no one-shot capture.
    uint32_t t;
    uint8_t lvl;

    while (popEdge(t, lvl)) {
        processEdge(t, lvl);
    }

    // Simple diagnostic command.
    if (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == 'r' || c == 'R') {
            resetDiagnostics();
        }
    }

    printSummaryIfDue();

    // Yield to the scheduler without imposing protocol timing.
    delay(1);
}
