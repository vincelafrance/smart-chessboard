// ============================================================
// AutoTune.cpp
// Comprehensive trajectory test suite for CoreXY Smart Chessboard.
//
// HOW IT WORKS
// ------------
// The tune sequence runs in a separate FreeRTOS task (core 1, pri 2).
// When the task calls vTaskDelay(), the RTOS yields to loop() (pri 1),
// which drives calibrationLoop().  This is how the task orchestrates
// full calibrations without busy-waiting.
//
// MOTION TESTING — DESIGN PHILOSOPHY
// -----------------------------------
// Rather than testing only the calibration path (corner ↔ corner),
// the system characterises the full variety of motion the chessboard
// actually performs: axis-only, diagonal, direction-reversal, multi-
// segment, near-edge, precision placement, chess-piece trajectories.
//
// All trajectories are specified in normalised coordinates [0.0..1.0]
// of the calibrated board span.  normToAbs() converts them to step
// positions at runtime, so the suite is board-independent.
//
// Each phase (Repeatability / Speed / Accel / Current) runs the same
// runTrajectorySession() engine, which:
//   1. Executes every test in the active suite (full or quick subset).
//   2. Recovers gracefully from any single-trajectory timeout.
//   3. Finishes with ONE drift check (full calibration) to detect
//      accumulated missed steps across the whole session.
//
// The safe parameter stored in NVS is therefore the worst-case-reliable
// value across the broadest realistic motion set, not just one path.
//
// DRIFT MEASUREMENT — FAST REFERENCE CHECKS
// ------------------------------------------
// Drift cannot be detected from the step counter alone.
// fastHomeCornerCheck() pre-approaches the home corner at session speed,
// then uses the calibration state machine to detect both hall sensors at
// the home corner (Y then X).  After both sensors fire, calibration is
// aborted and the carriage returns to centre.  Total ~3-4 s per check.
//
// Checks are performed after every drift-prone test (diagonals, multi-seg,
// long axis, edge-region, repetition-heavy chess) AND at the end of each
// session.  Drift is measured relative to the phase-start reference so
// it reflects cumulative step loss since the parameter level began.
// A single check exceeding AT_MAX_DRIFT_STEPS fails the entire level.
//
// REUSED EXISTING CODE (unchanged)
//   startFullCalibration()   Calibration.cpp
//   calibrationLoop()        Calibration.cpp  (driven by loop() while task waits)
//   getXLimits/getYLimits    MotionCoreXY.cpp
//   setCurrentOverrides()    DriversUART.cpp
// ============================================================

#include "AutoTune.h"
#include "Calibration.h"
#include "MotionCoreXY.h"
#include "DriversUART.h"
#include "WebUI.h"
#include "Utils.h"
#include <Preferences.h>

// ============================================================
// Tuning constants
// ============================================================

static const unsigned long AT_CALIB_TIMEOUT_MS = 120000UL; // 120 s per calib pass
static const unsigned long AT_MOVE_TIMEOUT_MS  =  15000UL; // 15 s per test move
static const float         AT_MAX_DRIFT_STEPS  =   50.0f;  // 50 steps ≈ 1.6 mm

// Repeatability passes
static const int AT_REPEAT_FULL  = 5;
static const int AT_REPEAT_QUICK = 3;

// Speed ramp (steps/s, low → high)
static const float    AT_SPEEDS[]        = {3000.f,4500.f,6000.f,7500.f,9000.f,10500.f};
static const uint8_t  AT_N_SPEEDS        = sizeof(AT_SPEEDS)/sizeof(AT_SPEEDS[0]);
static const uint8_t  AT_N_SPEEDS_QUICK  = 4;

// Acceleration ramp (steps/s²)
static const float    AT_ACCELS[]        = {6000.f,10000.f,14000.f,18000.f,22000.f};
static const uint8_t  AT_N_ACCELS        = sizeof(AT_ACCELS)/sizeof(AT_ACCELS[0]);
static const uint8_t  AT_N_ACCELS_QUICK  = 3;

// Current sweep (mA, high → low; find lowest that works)
static const uint16_t AT_CURRENTS[]       = {1050,950,850,750,650,550};
static const uint8_t  AT_N_CURRENTS       = sizeof(AT_CURRENTS)/sizeof(AT_CURRENTS[0]);
static const uint8_t  AT_N_CURRENTS_QUICK = 4;

// NVS keys
static const char NVS_NS[]        = "chess_tune";
static const char NVS_SPEED[]     = "safeSpeed";
static const char NVS_ACCEL[]     = "safeAccel";
static const char NVS_CURRENT[]   = "motorMa";
static const char NVS_TUNE_OK[]   = "tuneValid";
static const char NVS_BOUNDS_OK[] = "boundsValid";

static const float    NVS_DEF_SPEED   = 8600.0f;
static const float    NVS_DEF_ACCEL   = 6000.0f;
static const uint16_t NVS_DEF_CURRENT = 850;

// ============================================================
// Module state
// ============================================================
static TaskHandle_t s_taskHandle = nullptr;
static bool         s_fullMode   = true;
static Preferences  s_prefs;

// ============================================================
// Logging  (UNCHANGED)
// ============================================================
static void atLog(const char *fmt, ...) {
    char buf[AT_LOG_MSG_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    Serial.print("[AT] ");
    Serial.println(buf);

    uint8_t next = (g_atLogWr + 1) % AT_LOG_SLOTS;
    if (next != g_atLogRd) {
        strncpy(g_atLogBuf[g_atLogWr].msg, buf, AT_LOG_MSG_LEN - 1);
        g_atLogBuf[g_atLogWr].msg[AT_LOG_MSG_LEN - 1] = '\0';
        g_atLogWr = next;
    }
}

static void setPhaseProgress(TunePhase ph, int pct) {
    g_tunePhase    = ph;
    g_tuneProgress = pct;
}

// ============================================================
// NVS persistence  (UNCHANGED)
// ============================================================
void saveSettings() {
    s_prefs.begin(NVS_NS, false);
    s_prefs.putFloat(NVS_SPEED,    g_tuneSettings.safeSpeed);
    s_prefs.putFloat(NVS_ACCEL,    g_tuneSettings.safeAccel);
    s_prefs.putUShort(NVS_CURRENT, g_tuneSettings.motorCurrent);
    s_prefs.putBool(NVS_TUNE_OK,   g_tuneSettings.tuningValid);
    s_prefs.putBool(NVS_BOUNDS_OK, g_tuneSettings.boundsValid);
    s_prefs.end();
    Serial.printf("[AT] Saved: speed=%.0f accel=%.0f current=%u\n",
                  g_tuneSettings.safeSpeed, g_tuneSettings.safeAccel,
                  (unsigned)g_tuneSettings.motorCurrent);
}

void loadSettings() {
    s_prefs.begin(NVS_NS, true);
    g_tuneSettings.safeSpeed    = s_prefs.getFloat(NVS_SPEED,    NVS_DEF_SPEED);
    g_tuneSettings.safeAccel    = s_prefs.getFloat(NVS_ACCEL,    NVS_DEF_ACCEL);
    g_tuneSettings.motorCurrent = s_prefs.getUShort(NVS_CURRENT, NVS_DEF_CURRENT);
    g_tuneSettings.tuningValid  = s_prefs.getBool(NVS_TUNE_OK,   false);
    g_tuneSettings.boundsValid  = s_prefs.getBool(NVS_BOUNDS_OK, false);
    s_prefs.end();

    if (g_tuneSettings.tuningValid) {
        g_overrideVmax  = g_tuneSettings.safeSpeed;
        g_overrideAccel = g_tuneSettings.safeAccel;
        setCurrentOverrides(g_tuneSettings.motorCurrent,
                            (uint16_t)((float)g_tuneSettings.motorCurrent * 1.20f));
        Serial.printf("[AT] Loaded: speed=%.0f accel=%.0f current=%u (VALID)\n",
                      g_tuneSettings.safeSpeed, g_tuneSettings.safeAccel,
                      (unsigned)g_tuneSettings.motorCurrent);
    } else {
        Serial.println("[AT] No valid tune in NVS — using firmware defaults.");
    }
}

// ============================================================
// Safety  (UNCHANGED)
// ============================================================
static void atEmergencyStop(const char *reason) {
    portENTER_CRITICAL(&gMux);
    g_recenter       = false;
    g_vx_xy          = 0.0f;
    g_vy_xy          = 0.0f;
    g_calibState     = CALIB_IDLE;
    g_pathActive     = false;
    g_wpCount        = 0;
    g_wpIndex        = 0;
    g_autoMagnetPath = false;
    portEXIT_CRITICAL(&gMux);
    g_overrideVmax  = 0.0f;
    g_overrideAccel = 0.0f;
    setDriversEnabled(false);
    atLog("EMERGENCY STOP: %s", reason);
}

// ============================================================
// Wait helpers  (UNCHANGED)
// ============================================================
static bool waitForCalibration() {
    unsigned long start = millis();
    while (millis() - start < AT_CALIB_TIMEOUT_MS) {
        if (g_tuneAbortReq) return false;
        CalibState st;
        portENTER_CRITICAL(&gMux);
        st = g_calibState;
        portEXIT_CRITICAL(&gMux);
        if (st == CALIB_IDLE) return true;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    atLog("waitForCalibration: TIMEOUT");
    return false;
}

enum ArriveResult : uint8_t { AR_OK = 0, AR_TIMEOUT, AR_ABORT };

static ArriveResult tuneMoveTo(long x, long y,
                                unsigned long timeoutMs = AT_MOVE_TIMEOUT_MS) {
    long xMin, xMax, yMin, yMax;
    getXLimits(xMin, xMax);
    getYLimits(yMin, yMax);

    const long guard = EDGE_STOP_DIST + 80;
    x = clampl(x, xMin + guard, xMax - guard);
    y = clampl(y, yMin + guard, yMax - guard);

    portENTER_CRITICAL(&gMux);
    g_xCenterTarget = x;
    g_yCenterTarget = y;
    g_recenter      = true;
    g_vx_xy         = 0.0f;
    g_vy_xy         = 0.0f;
    g_lastCmdMs     = millis();
    portEXIT_CRITICAL(&gMux);

    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (g_tuneAbortReq) {
            portENTER_CRITICAL(&gMux);
            g_recenter = false;
            g_vx_xy = 0.0f; g_vy_xy = 0.0f;
            portEXIT_CRITICAL(&gMux);
            return AR_ABORT;
        }
        bool r;
        portENTER_CRITICAL(&gMux);
        r = g_recenter;
        portEXIT_CRITICAL(&gMux);
        if (!r) return AR_OK;
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    portENTER_CRITICAL(&gMux);
    g_recenter = false;
    g_vx_xy = 0.0f; g_vy_xy = 0.0f;
    portEXIT_CRITICAL(&gMux);
    return AR_TIMEOUT;
}

// Drive to (x, y) using the PATH controller — true simultaneous X+Y motion.
//
// Unlike tuneMoveTo() (recenter, Y-then-X sequential), this sets up a
// single-waypoint path so StepTask's path P-controller drives both axes
// at the same time.  That is the same motion model used for all actual
// chess-piece moves, and the only way to test genuine diagonal traversal.
//
// Note: does NOT call beginMoveSeq() to avoid selectCurrentsForMove()
// overriding the test current, and to avoid the auto-magnet path logic.
static ArriveResult tuneMoveToPath(long x, long y,
                                    unsigned long timeoutMs = AT_MOVE_TIMEOUT_MS) {
    long xMin, xMax, yMin, yMax;
    getXLimits(xMin, xMax);
    getYLimits(yMin, yMax);

    const long guard = EDGE_STOP_DIST + 80;
    x = clampl(x, xMin + guard, xMax - guard);
    y = clampl(y, yMin + guard, yMax - guard);

    portENTER_CRITICAL(&gMux);
    g_waypoints[0]   = { x, y, -1 };
    g_wpCount        = 1;
    g_wpIndex        = 0;
    g_pathTargetX    = x;
    g_pathTargetY    = y;
    g_pathActive     = true;
    g_recenter       = false;
    g_vx_xy          = 0.0f;
    g_vy_xy          = 0.0f;
    g_autoMagnetPath = false;   // never touch the magnet during tuning
    g_lastCmdMs      = millis();
    portEXIT_CRITICAL(&gMux);

    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (g_tuneAbortReq) {
            portENTER_CRITICAL(&gMux);
            g_pathActive     = false;
            g_wpCount        = 0;
            g_wpIndex        = 0;
            g_autoMagnetPath = false;
            portEXIT_CRITICAL(&gMux);
            return AR_ABORT;
        }
        bool active;
        portENTER_CRITICAL(&gMux);
        active = g_pathActive;
        portEXIT_CRITICAL(&gMux);
        if (!active) return AR_OK;
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    portENTER_CRITICAL(&gMux);
    g_pathActive     = false;
    g_wpCount        = 0;
    g_wpIndex        = 0;
    g_autoMagnetPath = false;
    portEXIT_CRITICAL(&gMux);
    return AR_TIMEOUT;
}

// ============================================================
// REFERENCE LAYER  (UNCHANGED)
// ============================================================
static bool referenceHomeCorner(long &yMin, long &xMin) {
    atLog("referenceHomeCorner...");
    startFullCalibration();
    if (!waitForCalibration()) {
        atLog("referenceHomeCorner: FAILED");
        return false;
    }
    portENTER_CRITICAL(&gMux);
    yMin = g_yHardMin;
    xMin = g_xHardMin;
    portEXIT_CRITICAL(&gMux);
    atLog("homeCorner: y=%ld x=%ld", yMin, xMin);
    return true;
}

static bool referenceMaxCorner(long &yMax, long &xMax) {
    long yd, xd;
    if (!referenceHomeCorner(yd, xd)) return false;
    portENTER_CRITICAL(&gMux);
    yMax = g_yHardMax;
    xMax = g_xHardMax;
    portEXIT_CRITICAL(&gMux);
    atLog("maxCorner: y=%ld x=%ld", yMax, xMax);
    return true;
}

// ============================================================
// FAST HOME-CORNER REFERENCE CHECK
// ============================================================
// Drive toward the home corner using the normal calibration state
// machine; abort after both hall sensors have fired at the bottom
// corner.  Much faster than a full 4-corner calibration because
// only the home (min) corner is used.
//
// Pre-approach:  tuneMoveTo() drives at full session speed to within
//   AT_FASTREF_APPROACH_STEPS of the expected corner position.
// Slow-seek:     startFullCalibration() then takes over at CALIB speed
//   (~700 steps/s) for the final approach and hall detection.
// After both sensors fire, the calibration is aborted and the carriage
// is returned to the board centre.
//
// Typical duration from board centre:
//   ~0.7s pre-approach  +  ~0.9s Y-seek  +  ~0.9s X-seek  +  ~0.7s return
//   ≈ 3-4 seconds total
// ============================================================

static const long          AT_FASTREF_APPROACH_STEPS = 600L;
static const unsigned long AT_FASTREF_TIMEOUT_MS     = 18000UL;

enum FastRefResult : uint8_t { FR_OK = 0, FR_TIMEOUT, FR_ABORT };

// Result of one fast reference check.
// driftY / driftX / magnitude are all relative to the reference positions
// established at the START of the current tuning phase (not updated between
// checks), so they reflect cumulative drift since phase start.
struct DriftMeasure {
    long  actualY;    // g_yHallPos at moment Y sensor fired
    long  actualX;    // g_xHallPos at moment X sensor fired
    float driftY;     // actualY - refYMin  (+ = drifted toward max)
    float driftX;     // actualX - refXMin
    float magnitude;  // sqrt(dX² + dY²)
    bool  exceeded;   // magnitude > AT_MAX_DRIFT_STEPS
};

static FastRefResult fastHomeCornerCheck(long refYMin, long refXMin,
                                          DriftMeasure &out) {
    long cx, cy;
    portENTER_CRITICAL(&gMux);
    cx = g_xCenterTarget;
    cy = g_yCenterTarget;
    portEXIT_CRITICAL(&gMux);

    // ── Step 1: Fast pre-approach — drive to just outside the home corner ──
    // Clamped so we stay within the safe zone even if the reference is stale.
    long xMin, xMax, yMin, yMax;
    getXLimits(xMin, xMax);
    getYLimits(yMin, yMax);
    const long guard = EDGE_STOP_DIST + 80;

    long preX = refXMin + AT_FASTREF_APPROACH_STEPS;
    long preY = refYMin + AT_FASTREF_APPROACH_STEPS;
    preX = clampl(preX, xMin + guard, xMax - guard);
    preY = clampl(preY, yMin + guard, yMax - guard);

    ArriveResult ar = tuneMoveTo(preX, preY, 10000UL);
    if (ar == AR_ABORT)   return FR_ABORT;
    if (ar == AR_TIMEOUT) {
        atLog("fastRef: pre-approach TIMEOUT");
        return FR_TIMEOUT;
    }
    vTaskDelay(60 / portTICK_PERIOD_MS);

    // ── Step 2: Verify hall sensors are not yet triggered ──
    // At preX/preY (~600 steps from the corner) they should always be HIGH.
    // If they are LOW, drift has moved the corner position significantly.
    if (digitalRead(HALL_Y_PIN) == LOW || digitalRead(HALL_X_PIN) == LOW) {
        atLog("fastRef: hall LOW at pre-approach — drift too large to measure");
        tuneMoveTo(cx, cy);
        return FR_TIMEOUT;
    }

    // ── Step 3: Launch calibration slow-seek toward home corner ──
    // StepTask drives at CALIB_Y_SPEED (700 steps/s) in -Y, then -X.
    // calibrationLoop() watches hall sensors and advances the state machine.
    startFullCalibration();

    // ── Step 4: Wait until BOTH bottom-corner sensors have fired ──
    // Transition path:  CALIB_Y_BOTTOM → (Y fires) → CALIB_X_BOTTOM
    //                   CALIB_X_BOTTOM → (X fires) → CALIB_Y_TOP
    // When state reaches CALIB_Y_TOP, both g_yHallPos and g_xHallPos
    // hold the detected positions.
    unsigned long start = millis();
    bool found = false;
    while (millis() - start < AT_FASTREF_TIMEOUT_MS) {
        if (g_tuneAbortReq) {
            portENTER_CRITICAL(&gMux);
            g_calibState = CALIB_IDLE;
            g_vx_xy = 0.0f; g_vy_xy = 0.0f;
            portEXIT_CRITICAL(&gMux);
            return FR_ABORT;
        }
        CalibState st;
        portENTER_CRITICAL(&gMux);
        st = g_calibState;
        portEXIT_CRITICAL(&gMux);
        // CALIB_Y_TOP or beyond means both bottom-corner sensors fired
        if (st == CALIB_Y_TOP  || st == CALIB_X_TOP ||
            st == CALIB_RECENTER || st == CALIB_IDLE) {
            found = true;
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (!found) {
        portENTER_CRITICAL(&gMux);
        g_calibState = CALIB_IDLE;
        g_vx_xy = 0.0f; g_vy_xy = 0.0f;
        portEXIT_CRITICAL(&gMux);
        atLog("fastRef: TIMEOUT waiting for home-corner sensors");
        tuneMoveTo(cx, cy);
        return FR_TIMEOUT;
    }

    // ── Step 5: Capture detected positions; abort rest of calibration ──
    long hitY, hitX;
    portENTER_CRITICAL(&gMux);
    hitY = g_yHallPos;
    hitX = g_xHallPos;
    g_calibState = CALIB_IDLE;
    g_vx_xy = 0.0f;
    g_vy_xy = 0.0f;
    portEXIT_CRITICAL(&gMux);

    // ── Step 6: Compute drift vs phase-start reference ──
    out.actualY   = hitY;
    out.actualX   = hitX;
    out.driftY    = (float)(hitY - refYMin);
    out.driftX    = (float)(hitX - refXMin);
    out.magnitude = sqrtf(out.driftX * out.driftX + out.driftY * out.driftY);
    out.exceeded  = (out.magnitude > AT_MAX_DRIFT_STEPS);

    if (g_driftCount < MAX_DRIFT_LOG) {
        g_driftLog[g_driftCount].driftX = out.driftX;
        g_driftLog[g_driftCount].driftY = out.driftY;
        g_driftCount++;
    }

    atLog("fastRef: Y=%ld(ref%ld dY=%.1f) X=%ld(ref%ld dX=%.1f) |%.1f| %s",
          hitY, refYMin, out.driftY,
          hitX, refXMin, out.driftX,
          out.magnitude, out.exceeded ? "FAIL" : "OK");

    // ── Step 7: Return carriage to board centre ──
    // g_overrideVmax is still set by the caller (session speed); the
    // return move uses it for full-speed travel back.
    if (tuneMoveTo(cx, cy) == AR_ABORT) return FR_ABORT;
    vTaskDelay(80 / portTICK_PERIOD_MS);

    return FR_OK;
}

static bool recoverReferenceAfterDrift(long &newYMin, long &newXMin) {
    atLog("recoverReferenceAfterDrift: stopping and re-homing...");
    atEmergencyStop("pre-recovery");
    vTaskDelay(600 / portTICK_PERIOD_MS);
    return referenceHomeCorner(newYMin, newXMin);
}

// ============================================================
// TRAJECTORY TEST INFRASTRUCTURE  — NEW
// ============================================================

// Maximum waypoints per test.  Most tests use 2 (start + end).
// Multi-segment tests use up to 8.
#define AT_MAX_TRAJ_PTS 8

// Trajectory category — used for per-category failure counting in the report.
enum TrajCategory : uint8_t {
    TC_AXIS_X    = 0,
    TC_AXIS_Y    = 1,
    TC_DIAGONAL  = 2,
    TC_REGION    = 3,
    TC_MULTI_SEG = 4,
    TC_CHESS     = 5,
    TC_NUM       = 6
};

// A single waypoint expressed as normalised board fractions [0.0 .. 1.0].
// 0.0 → calibrated minimum edge (+ guard)
// 1.0 → calibrated maximum edge (- guard)
// normToAbs() converts to absolute step positions at runtime.
struct TrajPoint { float u; float v; };

// One trajectory entry in the test suite.
// ──────────────────────────────────────────────────────────────
// name         : human-readable label (logged + shown in UI)
// pts          : waypoints in normalised coordinates
// nPts         : number of valid points (2 = simple A→B move)
// speedScale   : multiply against the current session speed
//                (< 1.0 to slow corner/precision approaches)
// accelScale   : multiply against current session acceleration
// reps         : full path repetitions per test invocation
// bidirectional: also traverse in reverse after each forward pass
// inQuickSuite : include when ramp phases need fast feedback
// category     : used for per-category failure reporting
// ──────────────────────────────────────────────────────────────
struct TrajectoryTest {
    const char   *name;
    TrajPoint     pts[AT_MAX_TRAJ_PTS];
    uint8_t       nPts;
    float         speedScale;
    float         accelScale;
    uint8_t       reps;
    bool          bidirectional;
    bool          inQuickSuite;
    TrajCategory  category;
};

// ──────────────────────────────────────────────────────────────
// TEST SUITE
// 36 tests across 6 categories.
// Add new entries at any time — the phase runners iterate the whole array.
// ──────────────────────────────────────────────────────────────
static const TrajectoryTest AT_TESTS[] = {

// ==== AXIS-ONLY: X ============================================
//  name            pts                                                           n  spd   acc  rep  bidi   quick  cat
{"X-short",       {{.45f,.50f},{.55f,.50f}},                                     2, 1.0f, 1.0f,  3, true,  false, TC_AXIS_X},
{"X-medium",      {{.20f,.50f},{.80f,.50f}},                                     2, 1.0f, 1.0f,  2, true,  false, TC_AXIS_X},
{"X-long",        {{.05f,.50f},{.95f,.50f}},                                     2, 1.0f, 1.0f,  2, true,  true,  TC_AXIS_X},
{"X-reversal",    {{.35f,.50f},{.65f,.50f}},                                     2, 1.0f, 1.0f,  5, true,  true,  TC_AXIS_X},

// ==== AXIS-ONLY: Y ============================================
{"Y-short",       {{.50f,.45f},{.50f,.55f}},                                     2, 1.0f, 1.0f,  3, true,  false, TC_AXIS_Y},
{"Y-medium",      {{.50f,.20f},{.50f,.80f}},                                     2, 1.0f, 1.0f,  2, true,  false, TC_AXIS_Y},
{"Y-long",        {{.50f,.05f},{.50f,.95f}},                                     2, 1.0f, 1.0f,  2, true,  true,  TC_AXIS_Y},
{"Y-reversal",    {{.50f,.35f},{.50f,.65f}},                                     2, 1.0f, 1.0f,  5, true,  true,  TC_AXIS_Y},

// ==== DIAGONALS ===============================================
{"Diag-NE-short", {{.40f,.40f},{.60f,.60f}},                                     2, 1.0f, 1.0f,  2, true,  true,  TC_DIAGONAL},
{"Diag-NW-short", {{.60f,.40f},{.40f,.60f}},                                     2, 1.0f, 1.0f,  2, true,  true,  TC_DIAGONAL},
{"Diag-NE-long",  {{.10f,.10f},{.90f,.90f}},                                     2, 1.0f, 1.0f,  2, true,  true,  TC_DIAGONAL},
{"Diag-NW-long",  {{.90f,.10f},{.10f,.90f}},                                     2, 1.0f, 1.0f,  2, true,  true,  TC_DIAGONAL},
{"Corner-corner", {{.05f,.05f},{.95f,.95f}},                                     2, 0.8f, 1.0f,  2, true,  true,  TC_DIAGONAL},
{"Anti-diagonal", {{.05f,.95f},{.95f,.05f}},                                     2, 0.8f, 1.0f,  2, true,  true,  TC_DIAGONAL},
{"Diag-reversal", {{.30f,.30f},{.70f,.70f}},                                     2, 1.0f, 1.0f,  5, true,  true,  TC_DIAGONAL},

// ==== BOARD REGIONS ===========================================
{"Center-micro",  {{.47f,.47f},{.53f,.53f}},                                     2, 1.0f, 1.0f,  4, true,  false, TC_REGION},
{"Left-edge-V",   {{.05f,.20f},{.05f,.80f}},                                     2, 1.0f, 1.0f,  2, true,  false, TC_REGION},
{"Right-edge-V",  {{.95f,.20f},{.95f,.80f}},                                     2, 1.0f, 1.0f,  2, true,  false, TC_REGION},
{"Bottom-edge-H", {{.20f,.05f},{.80f,.05f}},                                     2, 1.0f, 1.0f,  2, true,  false, TC_REGION},
{"Top-edge-H",    {{.20f,.95f},{.80f,.95f}},                                     2, 1.0f, 1.0f,  2, true,  false, TC_REGION},
{"Cross-board-H", {{.05f,.50f},{.95f,.50f}},                                     2, 1.0f, 1.0f,  2, true,  true,  TC_REGION},
{"Cross-board-V", {{.50f,.05f},{.50f,.95f}},                                     2, 1.0f, 1.0f,  2, true,  true,  TC_REGION},
{"Corner-NE-loc", {{.80f,.80f},{.95f,.95f}},                                     2, 0.7f, 1.0f,  3, true,  false, TC_REGION},
{"Corner-SW-loc", {{.05f,.05f},{.20f,.20f}},                                     2, 0.7f, 1.0f,  3, true,  false, TC_REGION},
{"Zone-transit",  {{.10f,.10f},{.90f,.50f}},                                     2, 1.0f, 1.0f,  2, true,  false, TC_REGION},

// ==== MULTI-SEGMENT ===========================================
// pts form closed loops; each segment is a separate recenter move.
{"Square-CW",    {{.30f,.30f},{.70f,.30f},{.70f,.70f},{.30f,.70f},{.30f,.30f}},  5, 1.0f, 1.0f,  2, false, true,  TC_MULTI_SEG},
{"Diamond",      {{.50f,.15f},{.85f,.50f},{.50f,.85f},{.15f,.50f},{.50f,.15f}},  5, 1.0f, 1.0f,  2, false, false, TC_MULTI_SEG},
{"Zig-zag-H",    {{.10f,.30f},{.30f,.70f},{.50f,.30f},{.70f,.70f},{.90f,.30f}},  5, 1.0f, 1.0f,  2, true,  false, TC_MULTI_SEG},
{"Zig-zag-V",    {{.30f,.10f},{.70f,.30f},{.30f,.50f},{.70f,.70f},{.30f,.90f}},  5, 1.0f, 1.0f,  2, false, false, TC_MULTI_SEG},
// Star/cross: center → top → right → bottom → left → center (5 arms)
{"Cross-star",   {{.50f,.50f},{.50f,.92f},{.92f,.50f},{.50f,.08f},{.08f,.50f},{.50f,.50f}}, 6, 1.0f, 1.0f, 2, false, true, TC_MULTI_SEG},
{"Rectangle-H",  {{.15f,.40f},{.85f,.40f},{.85f,.60f},{.15f,.60f},{.15f,.40f}},  5, 1.0f, 1.0f,  2, false, false, TC_MULTI_SEG},

// ==== CHESS-RELEVANT ==========================================
// Distances based on 1/8 of board span ≈ one chessboard square.
{"Chess-1sq-H",  {{.40f,.50f},{.525f,.50f}},                                     2, 0.65f,1.0f,  6, true,  false, TC_CHESS},
{"Chess-2sq-H",  {{.35f,.50f},{.60f,.50f}},                                      2, 0.80f,1.0f,  4, true,  false, TC_CHESS},
{"Rook-H",       {{.10f,.50f},{.90f,.50f}},                                      2, 1.0f, 1.0f,  3, true,  true,  TC_CHESS},
{"Rook-V",       {{.50f,.10f},{.50f,.90f}},                                      2, 1.0f, 1.0f,  3, true,  true,  TC_CHESS},
{"Bishop-full",  {{.10f,.10f},{.90f,.90f}},                                      2, 1.0f, 1.0f,  3, true,  true,  TC_CHESS},
// Knight L-shape: 2 squares right, 1 square up (2-segment path)
{"Knight-L",     {{.40f,.40f},{.65f,.40f},{.65f,.525f}},                         3, 0.85f,1.0f,  4, true,  false, TC_CHESS},
// Ultra-short precision placement
{"Micro-place",  {{.49f,.50f},{.51f,.50f}},                                      2, 0.40f,1.0f,  8, true,  false, TC_CHESS},

};  // end AT_TESTS

static const uint8_t AT_NUM_TESTS = sizeof(AT_TESTS) / sizeof(AT_TESTS[0]);

// Category names for logging
static const char * const TC_NAMES[TC_NUM] = {
    "Axis-X", "Axis-Y", "Diagonal", "Region", "Multi-Seg", "Chess"
};

// Per-category reliability weights used by computeLevelScore().
// A category's drift failures contribute this multiple to the overall score
// penalty.  Diagonals stress both motors simultaneously — highest weight.
// Multi-seg accumulates direction reversals — second highest.
// Single-axis moves are easiest to recover and carry the least weight.
static const float CAT_WEIGHT[TC_NUM] = {
    0.8f,  // TC_AXIS_X    — single-motor; easiest to recover
    0.8f,  // TC_AXIS_Y
    1.5f,  // TC_DIAGONAL  — simultaneous dual-motor; highest mechanical stress
    1.0f,  // TC_REGION    — edge proximity adds belt tension variance
    1.3f,  // TC_MULTI_SEG — multi-direction changes accumulate errors
    1.0f,  // TC_CHESS     — real-world trajectories; representative baseline
};

// A speed/accel level must achieve at least this weighted score to be
// accepted.  Below this, the ramp stops and the last good level is kept.
static const float AT_SCORE_PASS_THRESHOLD = 0.60f;

// ──────────────────────────────────────────────────────────────
// normToAbs()
// Convert a normalised coordinate (0.0..1.0) to an absolute step
// position within the calibrated, guard-cleared board span.
// ──────────────────────────────────────────────────────────────
static void normToAbs(float u, float v, long &xAbs, long &yAbs) {
    long xMin, xMax, yMin, yMax;
    getXLimits(xMin, xMax);
    getYLimits(yMin, yMax);

    const long gx = EDGE_STOP_DIST + 80;
    const long gy = EDGE_STOP_DIST + 80;
    long spanX = xMax - xMin - 2 * gx;
    long spanY = yMax - yMin - 2 * gy;
    if (spanX < 1) spanX = 1;
    if (spanY < 1) spanY = 1;

    u = clampf(u, 0.0f, 1.0f);
    v = clampf(v, 0.0f, 1.0f);
    xAbs = xMin + gx + (long)(u * (float)spanX);
    yAbs = yMin + gy + (long)(v * (float)spanY);
}

// ──────────────────────────────────────────────────────────────
// runSingleTrajectory()
// Execute one TrajectoryTest entry at the given session speed/accel.
// Handles: multi-waypoint, reps, bidirectional (reverse pass).
// Speed/accel are set per-test using the test's scale factors, then
// restored to session values after the test completes.
// ──────────────────────────────────────────────────────────────
enum TrajRunResult : uint8_t { TR_OK = 0, TR_TIMEOUT, TR_ABORT };

static TrajRunResult runSingleTrajectory(const TrajectoryTest &t,
                                          float sessionSpeed,
                                          float sessionAccel) {
    if (g_tuneAbortReq) return TR_ABORT;

    // Apply per-test speed/accel scale (e.g. slower for precision/corner tests)
    g_overrideVmax  = sessionSpeed * clampf(t.speedScale, 0.05f, 2.0f);
    g_overrideAccel = sessionAccel * clampf(t.accelScale, 0.05f, 2.0f);

    TrajRunResult result = TR_OK;

    // ---- Move to starting waypoint ----
    // Uses the path controller (simultaneous X+Y) so even the approach to the
    // start position exercises the same motion model as real chess moves.
    {
        long sx, sy;
        normToAbs(t.pts[0].u, t.pts[0].v, sx, sy);
        ArriveResult ar = tuneMoveToPath(sx, sy);
        if      (ar == AR_ABORT)   result = TR_ABORT;
        else if (ar == AR_TIMEOUT) result = TR_TIMEOUT;
        else                       vTaskDelay(80 / portTICK_PERIOD_MS);
    }

    // ---- Repetitions ----
    for (uint8_t rep = 0; rep < t.reps && result == TR_OK && !g_tuneAbortReq; rep++) {

        // Forward: pts[1] … pts[nPts-1]
        // Each segment uses the path controller for true simultaneous X+Y motion.
        for (uint8_t pi = 1; pi < t.nPts && result == TR_OK && !g_tuneAbortReq; pi++) {
            long x, y;
            normToAbs(t.pts[pi].u, t.pts[pi].v, x, y);
            ArriveResult ar = tuneMoveToPath(x, y);
            if      (ar == AR_ABORT)   result = TR_ABORT;
            else if (ar == AR_TIMEOUT) result = TR_TIMEOUT;
            else                       vTaskDelay(80 / portTICK_PERIOD_MS);
        }

        // Reverse: pts[nPts-2] … pts[0]  (bidirectional only)
        if (t.bidirectional) {
            for (int pi = (int)t.nPts - 2; pi >= 0 && result == TR_OK && !g_tuneAbortReq; pi--) {
                long x, y;
                normToAbs(t.pts[pi].u, t.pts[pi].v, x, y);
                ArriveResult ar = tuneMoveToPath(x, y);
                if      (ar == AR_ABORT)   result = TR_ABORT;
                else if (ar == AR_TIMEOUT) result = TR_TIMEOUT;
                else                       vTaskDelay(80 / portTICK_PERIOD_MS);
            }
        }
    }

    // Restore session defaults (don't carry per-test scale into next test)
    g_overrideVmax  = sessionSpeed;
    g_overrideAccel = sessionAccel;
    return result;
}

// Return true if this test warrants a fast reference check after it runs.
// Diagonals and multi-segment paths stress both motors simultaneously and
// accumulate the most drift.  Long axis sweeps, near-edge tests, and
// repetition-heavy chess tests are also flagged.
// In full mode, ALL region and chess tests get a post-check.
static bool needsPostCheck(const TrajectoryTest &t, bool quickMode) {
    if (t.category == TC_DIAGONAL)   return true;  // always: simultaneous-motor stress
    if (t.category == TC_MULTI_SEG)  return true;  // always: complex direction changes
    if (t.category == TC_AXIS_X  && t.inQuickSuite)  return true;  // long / reversal
    if (t.category == TC_AXIS_Y  && t.inQuickSuite)  return true;
    if (t.category == TC_REGION  && t.inQuickSuite)  return true;  // edge + cross-board
    if (t.category == TC_CHESS   && t.reps >= 4)     return true;  // rep-heavy
    if (!quickMode && t.category == TC_REGION)        return true;  // all edge tests
    if (!quickMode && t.category == TC_CHESS)         return true;  // all chess tests
    return false;
}

// ──────────────────────────────────────────────────────────────
// SessionStats — results of one runTrajectorySession() call.
// driftOk is false if ANY fast reference check exceeded the
// AT_MAX_DRIFT_STEPS threshold during the session.
// ──────────────────────────────────────────────────────────────
struct SessionStats {
    uint32_t movesRun;
    uint32_t timeouts;
    uint32_t refChecks;            // total fast reference checks performed
    uint32_t driftExceeded;        // checks where magnitude > AT_MAX_DRIFT_STEPS
    float    maxDriftY;            // worst |driftY| across all checks (steps)
    float    maxDriftX;            // worst |driftX| across all checks (steps)
    float    maxDriftTotal;        // worst magnitude across all checks (steps)
    bool     driftOk;              // true only if zero checks exceeded threshold
    uint8_t  catFails[TC_NUM];      // trajectory timeouts per category
    uint8_t  catDriftChecks[TC_NUM]; // ref checks attributed per category
    uint8_t  catDriftFails[TC_NUM];  // drift-threshold failures per category
};

// ──────────────────────────────────────────────────────────────
// computeLevelScore()
// Returns a weighted reliability score 0.0 (unusable) .. 1.0 (perfect)
// for a completed session.
//
// Two components are multiplied together:
//
//   Drift component — how far was the worst measured drift from the
//     hard threshold?  0 drift → 1.0; drift == threshold → 0.0.
//     This rewards levels that passed cleanly, not just barely.
//
//   Category component — weighted drift-failure rate across test
//     categories.  A diagonal failure penalises the score 1.5× more
//     than an axis failure, reflecting the higher mechanical stress.
//     If a category had no ref checks this session it is excluded.
//
// Any trajectory timeout returns 0.0 immediately.  A missed-step event
// severe enough to stall the carriage is an unconditional disqualifier.
// ──────────────────────────────────────────────────────────────
static float computeLevelScore(const SessionStats &s) {
    if (s.timeouts > 0) return 0.0f;

    // Drift component: linear 1.0 → 0.0 as maxDriftTotal approaches threshold
    float driftScore = 1.0f - clampf(s.maxDriftTotal / AT_MAX_DRIFT_STEPS, 0.0f, 1.0f);

    // Category component: weighted failure rate
    float failWeight  = 0.0f;
    float totalWeight = 0.0f;
    for (uint8_t c = 0; c < TC_NUM; c++) {
        if (s.catDriftChecks[c] == 0) continue;
        float failRate = (float)s.catDriftFails[c] / (float)s.catDriftChecks[c];
        failWeight  += failRate * CAT_WEIGHT[c];
        totalWeight += CAT_WEIGHT[c];
    }
    float catScore = (totalWeight > 0.0f)
                   ? (1.0f - clampf(failWeight / totalWeight, 0.0f, 1.0f))
                   : 1.0f;  // no ref checks yet — don't penalise

    return driftScore * catScore;
}

// ──────────────────────────────────────────────────────────────
// graduatedMargin()
// Converts a level score into the safety margin applied to the raw
// parameter value (speed or accel).
//
// A level that sailed through (score → 1.0) gets margin 0.90 —
// nearly all the headroom is kept.
// A level that barely cleared the pass threshold gets margin 0.75 —
// a larger haircut to guard against hardware variance day-to-day.
//
// Score range [AT_SCORE_PASS_THRESHOLD .. 1.0] maps linearly to
// [0.75 .. 0.90].
// ──────────────────────────────────────────────────────────────
static float graduatedMargin(float score) {
    float t = clampf((score - AT_SCORE_PASS_THRESHOLD) /
                     (1.0f - AT_SCORE_PASS_THRESHOLD), 0.0f, 1.0f);
    return 0.75f + t * 0.15f;
}

// ──────────────────────────────────────────────────────────────
// runTrajectorySession()
// Executes the trajectory test suite (full or quick subset).
//
// REFERENCE CHECKING STRATEGY
// ----------------------------
// A fast home-corner reference check (fastHomeCornerCheck) is
// performed after every drift-prone test — specifically after all
// diagonals, all multi-segment paths, long/reversal axis tests,
// near-edge region tests, and repetition-heavy chess moves.
//
// Each check takes ~3-4 s (pre-approach at session speed +
// ~0.9 s Y sensor seek + ~0.9 s X sensor seek + return).
//
// A final check runs at the end of the session as well.
//
// All drift measurements compare against refYMin/refXMin, which is
// fixed at the start of the phase (not updated between checks).
// This means the reported drift is cumulative since phase start.
// The reference is reset only after a timeout recovery.
//
// Returns false only on abort request; driftOk in stats encodes pass/fail.
// ──────────────────────────────────────────────────────────────
static bool runTrajectorySession(bool quickMode,
                                  float sessionSpeed, float sessionAccel,
                                  long &refYMin, long &refXMin,
                                  SessionStats &stats) {
    memset(&stats, 0, sizeof(stats));
    stats.driftOk = true;

    long cx, cy;
    portENTER_CRITICAL(&gMux);
    cx = g_xCenterTarget;
    cy = g_yCenterTarget;
    portEXIT_CRITICAL(&gMux);

    g_overrideVmax  = sessionSpeed;
    g_overrideAccel = sessionAccel;

    if (tuneMoveTo(cx, cy) != AR_OK) {
        atLog("Session: failed to reach center at start");
        g_overrideVmax  = 0.0f;
        g_overrideAccel = 0.0f;
        return false;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // ── Helper: run a fast ref check and fold results into stats ──
    // Called inline after each drift-prone test and at session end.
    // cat is the category of the test that just ran (TC_NUM = no category,
    // used for the end-of-session check).  Per-category drift accounting
    // feeds computeLevelScore() in the phase runners.
    // After a successful check the carriage is back at the board centre
    // and session overrides are restored.
    auto doRefCheck = [&](const char *label, TrajCategory cat) -> bool {
        stats.refChecks++;
        if (cat < TC_NUM) stats.catDriftChecks[cat]++;

        DriftMeasure dm = {};
        FastRefResult fr = fastHomeCornerCheck(refYMin, refXMin, dm);

        if (fr == FR_ABORT) {
            g_overrideVmax  = 0.0f;
            g_overrideAccel = 0.0f;
            return false;  // propagate abort
        }

        if (fr == FR_TIMEOUT) {
            stats.driftExceeded++;
            stats.driftOk = false;
            if (cat < TC_NUM) stats.catDriftFails[cat]++;
            atLog("fastRef TIMEOUT after '%s' — counted as drift fail", label);
            // Reference unknown after timeout — leave refYMin/refXMin unchanged
            // so the next check starts from the same baseline.
        } else {  // FR_OK
            if (fabsf(dm.driftY) > stats.maxDriftY) stats.maxDriftY = fabsf(dm.driftY);
            if (fabsf(dm.driftX) > stats.maxDriftX) stats.maxDriftX = fabsf(dm.driftX);
            if (dm.magnitude > stats.maxDriftTotal) stats.maxDriftTotal = dm.magnitude;
            if (dm.exceeded) {
                stats.driftExceeded++;
                stats.driftOk = false;
                if (cat < TC_NUM) stats.catDriftFails[cat]++;
                atLog(">> DRIFT EXCEEDED after '%s': |%.1f| steps (Y:%.1f X:%.1f)",
                      label, dm.magnitude, dm.driftY, dm.driftX);
            }
            // ── Advance the reference to the freshly detected corner position ──
            // Each subsequent check now measures drift caused by its own
            // preceding test only — not cumulative drift from phase start.
            // Without this, a single FAIL shifts the baseline permanently and
            // causes every following check to also exceed the threshold.
            refYMin = dm.actualY;
            refXMin = dm.actualX;
        }
        // fastHomeCornerCheck() returns with carriage at centre.
        // Restore overrides that the calibration approach may have stalled.
        g_overrideVmax  = sessionSpeed;
        g_overrideAccel = sessionAccel;
        return true;
    };

    // ── Iterate through the test suite ──
    for (uint8_t ti = 0; ti < AT_NUM_TESTS && !g_tuneAbortReq; ti++) {
        const TrajectoryTest &t = AT_TESTS[ti];

        if (quickMode && !t.inQuickSuite) continue;

        TrajectoryTest t_run = t;
        if (quickMode && t_run.reps > 1) t_run.reps = 1;

        atLog("[%d/%d] %s  spd=%.0fx%.2f  rep=%u%s",
              (int)ti + 1, (int)AT_NUM_TESTS,
              t.name, sessionSpeed, t.speedScale,
              (unsigned)t_run.reps, t.bidirectional ? " bidi" : "");

        TrajRunResult r = runSingleTrajectory(t_run, sessionSpeed, sessionAccel);

        if (r == TR_ABORT) {
            g_overrideVmax  = 0.0f;
            g_overrideAccel = 0.0f;
            return false;
        }

        if (r == TR_TIMEOUT) {
            stats.timeouts++;
            stats.catFails[t.category]++;
            atLog("  TIMEOUT on '%s' — recovering and re-homing", t.name);

            long newY, newX;
            if (!recoverReferenceAfterDrift(newY, newX)) {
                g_overrideVmax  = 0.0f;
                g_overrideAccel = 0.0f;
                return false;
            }
            // After a full re-home, reset the drift reference to the new
            // detected corner so subsequent checks measure from here.
            refYMin = newY;
            refXMin = newX;

            g_overrideVmax  = sessionSpeed;
            g_overrideAccel = sessionAccel;
            tuneMoveTo(cx, cy);
            continue;
        }

        stats.movesRun++;
        vTaskDelay(60 / portTICK_PERIOD_MS);

        // ── Post-test fast reference check for drift-prone trajectories ──
        if (needsPostCheck(t, quickMode) && !g_tuneAbortReq) {
            if (!doRefCheck(t.name, t.category)) return false;
        }
    }

    if (g_tuneAbortReq) {
        g_overrideVmax  = 0.0f;
        g_overrideAccel = 0.0f;
        return false;
    }

    // ── Final end-of-session reference check ──
    atLog("Session tests done — %u run, %u timeout, %u ref-checks so far",
          (unsigned)stats.movesRun, (unsigned)stats.timeouts, (unsigned)stats.refChecks);

    if (!doRefCheck("session-end", TC_NUM)) return false;

    g_overrideVmax  = 0.0f;
    g_overrideAccel = 0.0f;

    atLog("Session: %u tests, %u timeouts, %u ref-checks, %u exceeded, "
          "maxDrift=%.1f(Y:%.1f X:%.1f) -> %s",
          (unsigned)stats.movesRun, (unsigned)stats.timeouts,
          (unsigned)stats.refChecks, (unsigned)stats.driftExceeded,
          stats.maxDriftTotal, stats.maxDriftY, stats.maxDriftX,
          stats.driftOk ? "PASS" : "FAIL");

    return true;
}

// ============================================================
// PHASE RUNNERS  (revised to use trajectory sessions)
// ============================================================

// ---- Phase 2: Repeatability ----
// Full suite at default speed; characterises system baseline.
static bool phaseRepeatability(int cycles, bool quickMode,
                                long &refYMin, long &refXMin) {
    setPhaseProgress(AT_PHASE_REPEATABILITY, 0);
    atLog("=== Phase: Repeatability (%d cycles, %s) ===",
          cycles, quickMode ? "quick suite" : "full suite");

    float maxDrift = 0.0f;
    bool  passAll  = true;

    // Use a conservative speed for the characterisation baseline
    const float baseSpeed = AT_SPEEDS[2];  // 6000 steps/s
    const float baseAccel = AT_ACCELS[1];  // 10000 steps/s²

    for (int i = 0; i < cycles && !g_tuneAbortReq; i++) {
        atLog("Repeat pass %d/%d", i + 1, cycles);
        SessionStats s;
        if (!runTrajectorySession(quickMode, baseSpeed, baseAccel,
                                   refYMin, refXMin, s)) return false;

        if (s.maxDriftTotal > maxDrift) maxDrift = s.maxDriftTotal;
        if (!s.driftOk || s.timeouts > 0) {
            passAll = false;
            atLog("Pass %d WARN: drift=%.1f(Y:%.1f X:%.1f) refFails=%u timeouts=%u",
                  i + 1, s.maxDriftTotal, s.maxDriftY, s.maxDriftX,
                  (unsigned)s.driftExceeded, (unsigned)s.timeouts);
            for (uint8_t c = 0; c < TC_NUM; c++) {
                if (s.catFails[c]) atLog("  cat %s: %u fails", TC_NAMES[c], (unsigned)s.catFails[c]);
            }
        }
        setPhaseProgress(AT_PHASE_REPEATABILITY,
                          (int)(100.0f * (i + 1) / cycles));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    atLog("Repeatability: maxDrift=%.1f  %s",
          maxDrift, passAll ? "PASS" : "WARN");
    return true;
}

// ---- Phase 3: Speed Ramp ----
// Runs the quick test suite at increasing speeds; finds max reliable speed.
// Pass/fail uses a weighted category score (computeLevelScore) rather than
// a binary drift threshold.  The safety margin applied to the last-good
// speed is proportional to how comfortably that level passed (graduatedMargin).
static float phaseSpeedRamp(long refYMin, long refXMin,
                             int numSteps, float baseAccel) {
    setPhaseProgress(AT_PHASE_SPEED_RAMP, 0);
    atLog("=== Phase: Speed Ramp (%d levels, quick suite) ===", numSteps);

    float lastGoodSpeed = AT_SPEEDS[0];
    float lastGoodScore = 1.0f;  // initialise to "perfect" in case level 0 is never tested

    for (int si = 0; si < numSteps && !g_tuneAbortReq; si++) {
        float speed = AT_SPEEDS[si];
        atLog("Speed level %d/%d: %.0f steps/s", si + 1, numSteps, speed);

        SessionStats s;
        if (!runTrajectorySession(/*quick=*/true, speed, baseAccel,
                                   refYMin, refXMin, s)) {
            // Abort — return best so far with graduated margin
            return lastGoodSpeed * graduatedMargin(lastGoodScore);
        }

        float score    = computeLevelScore(s);
        bool  levelOk  = (score >= AT_SCORE_PASS_THRESHOLD);
        atLog("Speed %.0f: score=%.2f %s (drift=%.1f Y:%.1f X:%.1f refFails=%u timeouts=%u)",
              speed, score, levelOk ? "PASS" : "FAIL",
              s.maxDriftTotal, s.maxDriftY, s.maxDriftX,
              (unsigned)s.driftExceeded, (unsigned)s.timeouts);

        if (!levelOk) {
            // Log per-category breakdown to help diagnose which motion type failed
            for (uint8_t c = 0; c < TC_NUM; c++) {
                if (s.catFails[c] || s.catDriftFails[c])
                    atLog("  cat %s: tmo=%u driftFail=%u/%u (w=%.1f)",
                          TC_NAMES[c],
                          (unsigned)s.catFails[c],
                          (unsigned)s.catDriftFails[c],
                          (unsigned)s.catDriftChecks[c],
                          CAT_WEIGHT[c]);
            }
            break;  // stop ramp; last good is already recorded
        }
        lastGoodSpeed = speed;
        lastGoodScore = score;
        setPhaseProgress(AT_PHASE_SPEED_RAMP, (int)(100.0f * (si + 1) / numSteps));
    }

    float margin = graduatedMargin(lastGoodScore);
    float result = lastGoodSpeed * margin;
    if (result < 3000.0f) result = 3000.0f;
    atLog("Speed ramp done: lastGood=%.0f score=%.2f margin=%.2f safe=%.0f",
          lastGoodSpeed, lastGoodScore, margin, result);
    return result;
}

// ---- Phase 4: Acceleration Ramp ----
// Same scoring approach as phaseSpeedRamp: weighted category score for
// pass/fail, graduated margin on the last-good acceleration.
static float phaseAccelRamp(long refYMin, long refXMin,
                             int numSteps, float safeSpeed) {
    setPhaseProgress(AT_PHASE_ACCEL_RAMP, 0);
    atLog("=== Phase: Accel Ramp (%d levels, quick suite) ===", numSteps);

    float lastGoodAccel = AT_ACCELS[0];
    float lastGoodScore = 1.0f;

    for (int ai = 0; ai < numSteps && !g_tuneAbortReq; ai++) {
        float accel = AT_ACCELS[ai];
        atLog("Accel level %d/%d: %.0f steps/s²", ai + 1, numSteps, accel);

        SessionStats s;
        if (!runTrajectorySession(/*quick=*/true, safeSpeed, accel,
                                   refYMin, refXMin, s)) {
            return lastGoodAccel * graduatedMargin(lastGoodScore);
        }

        float score   = computeLevelScore(s);
        bool  levelOk = (score >= AT_SCORE_PASS_THRESHOLD);
        atLog("Accel %.0f: score=%.2f %s (drift=%.1f Y:%.1f X:%.1f refFails=%u timeouts=%u)",
              accel, score, levelOk ? "PASS" : "FAIL",
              s.maxDriftTotal, s.maxDriftY, s.maxDriftX,
              (unsigned)s.driftExceeded, (unsigned)s.timeouts);

        if (!levelOk) {
            for (uint8_t c = 0; c < TC_NUM; c++) {
                if (s.catFails[c] || s.catDriftFails[c])
                    atLog("  cat %s: tmo=%u driftFail=%u/%u (w=%.1f)",
                          TC_NAMES[c],
                          (unsigned)s.catFails[c],
                          (unsigned)s.catDriftFails[c],
                          (unsigned)s.catDriftChecks[c],
                          CAT_WEIGHT[c]);
            }
            break;
        }
        lastGoodAccel = accel;
        lastGoodScore = score;
        setPhaseProgress(AT_PHASE_ACCEL_RAMP, (int)(100.0f * (ai + 1) / numSteps));
    }

    float margin = graduatedMargin(lastGoodScore);
    float result = lastGoodAccel * margin;
    if (result < 6000.0f) result = 6000.0f;
    atLog("Accel ramp done: lastGood=%.0f score=%.2f margin=%.2f safe=%.0f",
          lastGoodAccel, lastGoodScore, margin, result);
    return result;
}

// ---- Phase 5: Current Sweep ----
// Tests from highest to lowest current; stores the lowest that passes.
// No margin is applied to current — going lower already increases the risk
// of step loss, so lastGoodCurrent is kept as-is.
// Score is logged per level for diagnostics.  If the last good level had
// a low score (borderline pass), the log will indicate which categories
// were the cause.
static uint16_t phaseCurrentTune(long refYMin, long refXMin,
                                   int numCurrents,
                                   float safeSpeed, float safeAccel) {
    setPhaseProgress(AT_PHASE_CURRENT, 0);
    atLog("=== Phase: Current Tune (%d levels, quick suite) ===", numCurrents);

    uint16_t lastGoodCurrent = AT_CURRENTS[0];  // 1050 mA — known working
    float    lastGoodScore   = 1.0f;

    for (int ci = 0; ci < numCurrents && !g_tuneAbortReq; ci++) {
        uint16_t mA = AT_CURRENTS[ci];
        setCurrentOverrides(mA, (uint16_t)((float)mA * 1.20f));
        atLog("Current level %d/%d: %u mA", ci + 1, numCurrents, (unsigned)mA);
        vTaskDelay(250 / portTICK_PERIOD_MS);  // let drivers settle

        SessionStats s;
        if (!runTrajectorySession(/*quick=*/true, safeSpeed, safeAccel,
                                   refYMin, refXMin, s)) {
            return lastGoodCurrent;
        }

        float score   = computeLevelScore(s);
        bool  levelOk = (score >= AT_SCORE_PASS_THRESHOLD);
        atLog("Current %u mA: score=%.2f %s (drift=%.1f Y:%.1f X:%.1f refFails=%u timeouts=%u)",
              (unsigned)mA, score, levelOk ? "PASS" : "FAIL",
              s.maxDriftTotal, s.maxDriftY, s.maxDriftX,
              (unsigned)s.driftExceeded, (unsigned)s.timeouts);

        if (!levelOk) {
            for (uint8_t c = 0; c < TC_NUM; c++) {
                if (s.catFails[c] || s.catDriftFails[c])
                    atLog("  cat %s: tmo=%u driftFail=%u/%u (w=%.1f)",
                          TC_NAMES[c],
                          (unsigned)s.catFails[c],
                          (unsigned)s.catDriftFails[c],
                          (unsigned)s.catDriftChecks[c],
                          CAT_WEIGHT[c]);
            }
            break;
        }
        lastGoodCurrent = mA;  // this lower current works — try the next lower
        lastGoodScore   = score;
        setPhaseProgress(AT_PHASE_CURRENT, (int)(100.0f * (ci + 1) / numCurrents));
    }

    atLog("Current sweep done: safe=%u mA (score=%.2f)", (unsigned)lastGoodCurrent, lastGoodScore);
    return lastGoodCurrent;
}

// ---- Phase 6: Final characterisation (full suite at tuned params) ----
// Runs the FULL trajectory suite once at the tuned parameters to generate
// a comprehensive per-category report of the final system behaviour.
static bool phaseFinalCharacterize(long refYMin, long refXMin,
                                    float safeSpeed, float safeAccel) {
    setPhaseProgress(AT_PHASE_APPROACH, 0);  // reuse APPROACH slot for this phase
    atLog("=== Phase: Final Characterisation (full suite at tuned params) ===");

    SessionStats s;
    if (!runTrajectorySession(/*quick=*/false, safeSpeed, safeAccel,
                               refYMin, refXMin, s)) return false;

    atLog("--- Final characterisation report ---");
    atLog("  Tests run:   %u", (unsigned)s.movesRun);
    atLog("  Timeouts:    %u", (unsigned)s.timeouts);
    atLog("  Ref checks:  %u  (%u exceeded)", (unsigned)s.refChecks, (unsigned)s.driftExceeded);
    atLog("  Max drift:   %.1f steps (Y:%.1f X:%.1f)  %s",
          s.maxDriftTotal, s.maxDriftY, s.maxDriftX, s.driftOk ? "PASS" : "WARN");
    for (uint8_t c = 0; c < TC_NUM; c++) {
        atLog("  %s failures: %u", TC_NAMES[c], (unsigned)s.catFails[c]);
    }
    setPhaseProgress(AT_PHASE_APPROACH, 100);
    return s.driftOk;
}

// ============================================================
// Main tune sequence
// ============================================================
static bool runTuneSequence(bool fullMode) {
    // Pre-flight: calibration must be valid before we start
    bool calOk;
    portENTER_CRITICAL(&gMux);
    calOk = g_yLimitsCalibrated && g_xLimitsCalibrated;
    portEXIT_CRITICAL(&gMux);
    if (!calOk) {
        atLog("ABORT: calibration not valid — run 'Calibrate Board' first.");
        return false;
    }

    g_driftCount = 0;

    // ---- Phase 1: Establish reference ----
    setPhaseProgress(AT_PHASE_REFERENCE, 0);
    atLog("=== Phase: Establish Reference ===");
    long refYMin, refXMin;
    if (!referenceHomeCorner(refYMin, refXMin)) return false;

    long refYMax, refXMax;
    portENTER_CRITICAL(&gMux);
    refYMax = g_yHardMax;
    refXMax = g_xHardMax;
    portEXIT_CRITICAL(&gMux);
    atLog("Ref: y=[%ld..%ld]  x=[%ld..%ld]", refYMin, refYMax, refXMin, refXMax);
    setPhaseProgress(AT_PHASE_REFERENCE, 100);
    if (g_tuneAbortReq) return false;

    // ---- Phase 2: Repeatability / baseline characterisation ----
    int repeatCycles = fullMode ? AT_REPEAT_FULL : AT_REPEAT_QUICK;
    bool quickRepeat  = !fullMode;  // full suite in full mode; quick in quick mode
    if (!phaseRepeatability(repeatCycles, quickRepeat, refYMin, refXMin)) return false;
    if (g_tuneAbortReq) return false;

    // ---- Phase 3: Speed Ramp ----
    int nSpeeds = fullMode ? (int)AT_N_SPEEDS : (int)AT_N_SPEEDS_QUICK;
    float testAccelBase = AT_ACCELS[1];   // 10000 steps/s² (conservative)
    float safeSpeed = phaseSpeedRamp(refYMin, refXMin, nSpeeds, testAccelBase);
    if (g_tuneAbortReq) return false;

    portENTER_CRITICAL(&gMux);
    refYMin = g_yHardMin;
    refXMin = g_xHardMin;
    portEXIT_CRITICAL(&gMux);

    // ---- Phase 4: Acceleration Ramp ----
    int nAccels = fullMode ? (int)AT_N_ACCELS : (int)AT_N_ACCELS_QUICK;
    float safeAccel = phaseAccelRamp(refYMin, refXMin, nAccels, safeSpeed);
    if (g_tuneAbortReq) return false;

    portENTER_CRITICAL(&gMux);
    refYMin = g_yHardMin;
    refXMin = g_xHardMin;
    portEXIT_CRITICAL(&gMux);

    // ---- Phase 5: Current Sweep ----
    int nCurrents = fullMode ? (int)AT_N_CURRENTS : (int)AT_N_CURRENTS_QUICK;
    uint16_t safeCurrent = phaseCurrentTune(refYMin, refXMin, nCurrents,
                                             safeSpeed, safeAccel);
    if (g_tuneAbortReq) return false;

    portENTER_CRITICAL(&gMux);
    refYMin = g_yHardMin;
    refXMin = g_xHardMin;
    portEXIT_CRITICAL(&gMux);

    // ---- Phase 6: Final characterisation at tuned params ----
    phaseFinalCharacterize(refYMin, refXMin, safeSpeed, safeAccel);
    // Non-fatal: even a WARN here doesn't invalidate the tune result.
    if (g_tuneAbortReq) return false;

    // ---- Apply and persist results ----
    g_tuneSettings.safeSpeed    = safeSpeed;
    g_tuneSettings.safeAccel    = safeAccel;
    g_tuneSettings.motorCurrent = safeCurrent;
    g_tuneSettings.tuningValid  = true;
    g_tuneSettings.boundsValid  = true;

    g_overrideVmax  = safeSpeed;
    g_overrideAccel = safeAccel;
    setCurrentOverrides(safeCurrent, (uint16_t)((float)safeCurrent * 1.20f));
    saveSettings();

    atLog("=== AutoTune COMPLETE ===");
    atLog("  safeSpeed  = %.0f steps/s", safeSpeed);
    atLog("  safeAccel  = %.0f steps/s²", safeAccel);
    atLog("  current    = %u mA", (unsigned)safeCurrent);
    atLog("  driftLog   = %u entries", (unsigned)g_driftCount);
    setPhaseProgress(AT_PHASE_DONE, 100);
    return true;
}

// ============================================================
// FreeRTOS task  (UNCHANGED)
// ============================================================
static void autoTuneTaskFn(void *param) {
    (void)param;
    atLog("AutoTune task started (%s mode)", s_fullMode ? "FULL" : "QUICK");
    g_systemState = SYS_TUNING;

    bool ok = runTuneSequence(s_fullMode);

    if (!ok || g_tuneAbortReq) {
        atEmergencyStop("tune end");
        setPhaseProgress(g_tuneAbortReq ? AT_PHASE_ABORTED : AT_PHASE_ERROR, 0);
        g_systemState = g_tuneAbortReq ? SYS_READY : SYS_ERROR;
        atLog(g_tuneAbortReq ? "AutoTune ABORTED by user" : "AutoTune ERROR");
    } else {
        g_systemState = SYS_READY;
    }

    g_tuneActive    = false;
    g_tuneAbortReq  = false;
    g_overrideVmax  = g_tuneSettings.tuningValid ? g_tuneSettings.safeSpeed  : 0.0f;
    g_overrideAccel = g_tuneSettings.tuningValid ? g_tuneSettings.safeAccel  : 0.0f;
    s_taskHandle    = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================
// Public API  (UNCHANGED)
// ============================================================

void autoTuneInit()  { loadSettings(); }

bool autoTuneStart(bool fullMode) {
    if (g_tuneActive) {
        Serial.println("[AT] Already running.");
        return false;
    }
    bool calOk;
    portENTER_CRITICAL(&gMux);
    calOk = g_yLimitsCalibrated && g_xLimitsCalibrated;
    portEXIT_CRITICAL(&gMux);
    if (!calOk) {
        Serial.println("[AT] Cannot start: no valid calibration.");
        return false;
    }
    if (g_calibState != CALIB_IDLE) {
        Serial.println("[AT] Cannot start: calibration in progress.");
        return false;
    }

    s_fullMode      = fullMode;
    g_tuneActive    = true;
    g_tuneAbortReq  = false;
    g_driftCount    = 0;
    setPhaseProgress(AT_PHASE_REFERENCE, 0);

    BaseType_t ret = xTaskCreatePinnedToCore(
        autoTuneTaskFn, "autoTune",
        8192, nullptr, 2, &s_taskHandle, 1);

    if (ret != pdPASS) {
        Serial.println("[AT] xTaskCreate FAILED");
        g_tuneActive = false;
        s_taskHandle = nullptr;
        return false;
    }
    Serial.printf("[AT] Started (%s mode)\n", fullMode ? "FULL" : "QUICK");
    return true;
}

void autoTuneStop() {
    if (!g_tuneActive) return;
    g_tuneAbortReq = true;
    Serial.println("[AT] Abort requested.");
}

void autoTuneLoop(unsigned long now) {
    (void)now;

    // Drain log ring-buffer to WebSocket (max 2 frames per loop tick)
    for (int drain = 0; drain < 2 && g_atLogRd != g_atLogWr; drain++) {
        char json[AT_LOG_MSG_LEN + 24];
        snprintf(json, sizeof(json), "{\"type\":\"tuneLog\",\"msg\":\"%s\"}",
                 g_atLogBuf[g_atLogRd].msg);
        extern WebSocketsServer webSocket;
        webSocket.broadcastTXT(json);
        g_atLogRd = (g_atLogRd + 1) % AT_LOG_SLOTS;
    }

    // Update g_systemState from calibration activity (only when not tuning)
    if (!g_tuneActive) {
        CalibState cs;
        bool calOk;
        portENTER_CRITICAL(&gMux);
        cs    = g_calibState;
        calOk = g_yLimitsCalibrated && g_xLimitsCalibrated;
        portEXIT_CRITICAL(&gMux);

        if (cs != CALIB_IDLE) {
            g_systemState = SYS_CALIBRATING;
        } else if (g_systemState == SYS_CALIBRATING || g_systemState == SYS_UNHOMED) {
            g_systemState = calOk ? SYS_READY : SYS_UNHOMED;
        }
    }
}
