#include "ChessTestRun.h"
#include "PathPlanner.h"
#include "MotionCoreXY.h"
#include "BoardMapping.h"
#include "Magnet.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

// ============================================================
// Test sequence definition
// ============================================================

// Each step exercises one distinct movement pattern.
// Steps are executed in order; the board starts from standard position.
// Dead zone slots are on the left side (low X) of the calibrated area.
//
// useSquareMove: true  → planSquareMove()      (L-shape routing, castling detection)
//               false → planSquareMoveDirect() (straight A→B, 2 waypoints — matches
//                        real game play for sliding pieces on clear paths)
struct TestStep {
    const char* label;
    bool        deadZone;     // true → deliver piece at (ff,fr) to dead zone
    bool        useSquareMove; // true → planSquareMove, false → planSquareMoveDirect
    uint8_t     ff, fr;
    uint8_t     tf, tr;
};

static const TestStep TEST_STEPS[] = {
    // Pawns (short straight moves)
    { "Pion 1 case  : a2→a3",          false, false, 0,2, 0,3 },
    { "Pion 2 cases : e2→e4",          false, false, 4,2, 4,4 },
    { "Pion adversaire : d7→d5",       false, false, 3,7, 3,5 },

    // Knights — must use planSquareMove for L-shape routing
    { "Cavalier : g1→f3",              false, true,  6,1, 5,3 },
    { "Cavalier : b1→c3",              false, true,  1,1, 2,3 },

    // Bishops (diagonal direct — 2 waypoints, same physical path as multi-wp)
    { "Fou court : c1→e3",             false, false, 2,1, 4,3 },
    { "Fou long  : f1→b5",             false, false, 5,1, 1,5 },

    // Rooks (straight direct)
    { "Tour courte : a1→a4",           false, false, 0,1, 0,4 },
    { "Tour longue : h1→h7",           false, false, 7,1, 7,7 },

    // Queen (diagonal + straight direct)
    { "Reine diagonale : d1→h5",       false, false, 3,1, 7,5 },
    { "Reine droite    : d8→d5",       false, false, 3,8, 3,5 },

    // King (adjacent)
    { "Roi adjacent : e1→f1",          false, false, 4,1, 5,1 },

    // Castling — must use planSquareMove (2-piece detection inside)
    { "Roque côté roi : e8→g8",        false, true,  4,8, 6,8 },

    // Dead zone delivery + capture
    { "Zone morte : retirer pièce d5", true,  false, 3,5, 0,0 },
    { "Capture : e4×d5",               false, false, 4,4, 3,5 },

    // Long traversals — stress test high-speed motion
    { "Tour longue : a4→a8",           false, false, 0,4, 0,8 },
    { "Reine longue : h5→a5",          false, false, 7,5, 0,5 },
};
static const uint8_t TEST_STEP_COUNT =
    (uint8_t)(sizeof(TEST_STEPS) / sizeof(TEST_STEPS[0]));

// ============================================================
// Internal helpers
// ============================================================

static TaskHandle_t s_trTask = nullptr;

static void trLog(const char* fmt, ...) {
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    Serial.print("[TR] ");
    Serial.println(buf);

    // Push into the AutoTune log ring buffer so the WebSocket drains it
    // (AutoTune and TestRun never run simultaneously).
    uint8_t next = (g_atLogWr + 1) % AT_LOG_SLOTS;
    if (next != g_atLogRd) {
        snprintf(g_atLogBuf[g_atLogWr].msg, AT_LOG_MSG_LEN, "[Test] %s", buf);
        g_atLogWr = next;
    }
}

// Wait until the path planner finishes the current move sequence.
static bool waitPathDone(unsigned long timeoutMs = 25000UL) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (g_testRunAbortReq) return false;
        bool active;
        portENTER_CRITICAL(&gMux);
        active = g_pathActive;
        portEXIT_CRITICAL(&gMux);
        if (!active) return true;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    // Timeout — forcibly clear path
    portENTER_CRITICAL(&gMux);
    g_pathActive = false;
    g_wpCount    = 0;
    g_wpIndex    = 0;
    portEXIT_CRITICAL(&gMux);
    trLog("TIMEOUT waiting for path completion");
    return false;
}

// Execute a board move.
// planSquareMoveDirect: straight A→B (2 waypoints) — matches real game play for
//   sliding pieces; fast, no intermediate settling at in-between squares.
// planSquareMove: full routing with L-shape waypoints — used for knights and castling.
static bool doMove(const TestStep& s) {
    if (g_testRunAbortReq) return false;
    trLog("%s", s.label);
    if (s.useSquareMove) {
        planSquareMove(s.ff, s.fr, s.tf, s.tr);
    } else {
        planSquareMoveDirect(s.ff, s.fr, s.tf, s.tr);
    }
    if (!waitPathDone()) return false;
    return true;
}

// Deliver the piece currently at (srcFile, srcRank) to the dead zone on the
// left side of the calibrated area (low X, centre Y).
// Uses beginMoveSeq() directly so we can target an off-board absolute position.
static bool doDeadZone(const TestStep& s) {
    if (g_testRunAbortReq) return false;
    trLog("%s", s.label);

    long srcX, srcY;
    squareCenterSteps(s.ff, s.fr, srcX, srcY);

    long xMin, yCenter;
    portENTER_CRITICAL(&gMux);
    xMin    = g_xHardMin;
    yCenter = g_yCenterTarget;
    portEXIT_CRITICAL(&gMux);

    // Left dead zone — just inside the hard limit, well off the board edge.
    long dzX = xMin + EDGE_STOP_DIST + 200;
    long dzY = yCenter;

    Waypoint wps[2];
    wps[0] = { srcX, srcY, 1 };  // arrive at piece → magnet ON (pick up)
    wps[1] = { dzX,  dzY,  0 };  // arrive at dead zone → magnet OFF (drop)

    beginMoveSeq(wps, 2);
    if (!waitPathDone()) return false;

    return true;
}

// ============================================================
// FreeRTOS task
// ============================================================

static void chessTestTaskFn(void* /*param*/) {
    trLog("=== Chess Test Run started (%u steps) ===", (unsigned)TEST_STEP_COUNT);

    for (uint8_t i = 0; i < TEST_STEP_COUNT && !g_testRunAbortReq; i++) {
        const TestStep& s = TEST_STEPS[i];

        // Update progress globals (read by telemetry push).
        portENTER_CRITICAL(&gMux);
        g_trStepIdx   = i;
        g_trStepTotal = TEST_STEP_COUNT;
        strncpy((char*)g_trStepName, s.label, sizeof(g_trStepName) - 1);
        ((char*)g_trStepName)[sizeof(g_trStepName) - 1] = '\0';
        portEXIT_CRITICAL(&gMux);

        trLog("Step %u/%u: %s", (unsigned)(i + 1), (unsigned)TEST_STEP_COUNT, s.label);

        bool ok = s.deadZone ? doDeadZone(s) : doMove(s);

        if (!ok) {
            if (g_testRunAbortReq) {
                trLog("Test Run annulé par l'utilisateur.");
            } else {
                trLog("Step %u TIMEOUT — poursuite du test...", (unsigned)(i + 1));
                // Continue to the next step rather than aborting.
            }
        }
    }

    if (!g_testRunAbortReq) {
        trLog("=== Chess Test Run terminé avec succès (%u steps) ===",
              (unsigned)TEST_STEP_COUNT);
    }

    // Return carriage to centre.
    portENTER_CRITICAL(&gMux);
    g_xCenterTarget  = g_xCenterTarget;  // already at center after last move
    g_yCenterTarget  = g_yCenterTarget;
    g_recenter       = true;
    g_vx_xy          = 0.0f;
    g_vy_xy          = 0.0f;
    g_lastCmdMs      = millis();
    portEXIT_CRITICAL(&gMux);

    // Clear state
    portENTER_CRITICAL(&gMux);
    g_trStepName[0] = '\0';
    portEXIT_CRITICAL(&gMux);

    g_testRunActive  = false;
    g_testRunAbortReq = false;
    s_trTask = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================
// Public API
// ============================================================

void chessTestRunInit() {
    // Nothing to load from NVS — test run uses current AutoTune settings.
}

bool chessTestRunStart() {
    if (g_testRunActive) {
        Serial.println("[TR] Already running.");
        return false;
    }
    if (g_tuneActive) {
        Serial.println("[TR] Cannot start: AutoTune is running.");
        return false;
    }
    bool calOk;
    portENTER_CRITICAL(&gMux);
    calOk = g_yLimitsCalibrated && g_xLimitsCalibrated;
    portEXIT_CRITICAL(&gMux);
    if (!calOk) {
        Serial.println("[TR] Cannot start: board not calibrated.");
        return false;
    }
    if (g_calibState != CALIB_IDLE) {
        Serial.println("[TR] Cannot start: calibration in progress.");
        return false;
    }

    g_testRunActive   = true;
    g_testRunAbortReq = false;
    g_trStepIdx       = 0;
    g_trStepTotal     = TEST_STEP_COUNT;
    g_trStepName[0]   = '\0';

    BaseType_t ret = xTaskCreatePinnedToCore(
        chessTestTaskFn, "chessTest",
        4096, nullptr, 2, &s_trTask, 1);

    if (ret != pdPASS) {
        Serial.println("[TR] xTaskCreate FAILED");
        g_testRunActive = false;
        return false;
    }
    Serial.println("[TR] Test run task started.");
    return true;
}

void chessTestRunStop() {
    if (g_testRunActive) {
        g_testRunAbortReq = true;
        Serial.println("[TR] Abort requested.");
    }
}
