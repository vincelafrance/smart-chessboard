#include <cstddef>
#include <cstring>
#include <cstdint>

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
static const unsigned long AT_MOVE_TIMEOUT_MS  =  12000UL; // 12 s per test move
static const float         AT_MAX_DRIFT_STEPS  =   50.0f;  // 50 steps ≈ 1.6 mm
static const uint8_t       AT_LOCKED_MICROSTEPS = 8;

// Hard wall-clock limit for the entire tuning sequence (excluding initial calib).
// If a phase is still running when this fires, that phase commits its best value

// Repeatability passes
static const int AT_REPEAT_QUICK = 1;

// Speed ramp — diagonal tests (steps/s, low → high).
// ONE motor runs at √2× carriage speed during 45° moves — the stall-critical case.
// Diagonals are intentionally kept around half the straight-line speed range.
static const float    AT_SPEEDS_DIAG[]      = {3500.f, 4500.f, 6000.f};
static const uint8_t  AT_N_SPEEDS_DIAG      = sizeof(AT_SPEEDS_DIAG)/sizeof(AT_SPEEDS_DIAG[0]);
// Speed ramp — axis-only tests (steps/s).
// Both motors share load more evenly on straight moves, so the carriage can
// usually run substantially faster than on 45° diagonals at the same current.
static const float    AT_SPEEDS_AXIS[]      = {7000.f, 9000.f, 11000.f, 12500.f};
static const uint8_t  AT_N_SPEEDS_AXIS      = sizeof(AT_SPEEDS_AXIS)/sizeof(AT_SPEEDS_AXIS[0]);

// Fixed acceleration used during speed-ramp test sessions (not tuned; hardware ramp constants apply at runtime).
static const float    AT_BASELINE_ACCEL  = 9000.0f;

// Current sweep (mA, low → high).
// Auto Tune now starts from the lowest current and only climbs if needed.
static const uint16_t AT_CURRENTS[]       = {950, 1050, 1150, 1300, 1500};
static const uint8_t  AT_N_CURRENTS       = sizeof(AT_CURRENTS)/sizeof(AT_CURRENTS[0]);

// NVS keys
static const char NVS_NS[]         = "chess_tune";
static const char NVS_SPEED[]      = "safeSpeed";
static const char NVS_SPEED_DIAG[] = "safeSpdDiag";
static const char NVS_CURRENT[]    = "motorMa";
static const char NVS_TUNE_OK[]    = "tuneValid";
static const char NVS_BOUNDS_OK[]  = "boundsValid";

static const float    NVS_DEF_SPEED      = 8600.0f;
static const float    NVS_DEF_SPEED_DIAG = 6000.0f;
static const uint16_t NVS_DEF_CURRENT    = 850;

// ============================================================
// Module state
// ============================================================
static TaskHandle_t s_taskHandle = nullptr;
static Preferences  s_prefs;

static void applyTuneCurrentMa(uint16_t mA);
static void setTuneLiveSettings(float axisSpeed, float diagSpeed, float accel, float decel);
static float axisSpeedForDiagSpeed(float diagSpeed);

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
    s_prefs.putFloat(NVS_SPEED,      g_tuneSettings.safeSpeed);
    s_prefs.putFloat(NVS_SPEED_DIAG, g_tuneSettings.safeSpeedDiag);
    s_prefs.putUShort(NVS_CURRENT,   g_tuneSettings.motorCurrent);
    s_prefs.putBool(NVS_TUNE_OK,     g_tuneSettings.tuningValid);
    s_prefs.putBool(NVS_BOUNDS_OK,   g_tuneSettings.boundsValid);
    s_prefs.end();
    Serial.printf("[AT] Saved: axisSpd=%.0f diagSpd=%.0f current=%u\n",
                  g_tuneSettings.safeSpeed, g_tuneSettings.safeSpeedDiag,
                  (unsigned)g_tuneSettings.motorCurrent);
}

void loadSettings() {
    s_prefs.begin(NVS_NS, true);
    g_tuneSettings.safeSpeed     = s_prefs.getFloat(NVS_SPEED,      NVS_DEF_SPEED);
    g_tuneSettings.safeSpeedDiag = s_prefs.getFloat(NVS_SPEED_DIAG, NVS_DEF_SPEED_DIAG);
    g_tuneSettings.motorCurrent  = s_prefs.getUShort(NVS_CURRENT,   NVS_DEF_CURRENT);
    g_tuneSettings.tuningValid   = s_prefs.getBool(NVS_TUNE_OK,     false);
    g_tuneSettings.boundsValid   = s_prefs.getBool(NVS_BOUNDS_OK,   false);
    s_prefs.end();

    // Accel/decel are NOT overridden from NVS — hardware ramp constants handle them.
    g_tuneSettings.safeAccel = 0.0f;
    g_tuneSettings.safeDecel = 0.0f;

    if (g_tuneSettings.tuningValid) {
        g_overrideVmax     = g_tuneSettings.safeSpeed;
        g_overrideDiagVmax = g_tuneSettings.safeSpeedDiag;
        setMotionProfileLock(true, AT_LOCKED_MICROSTEPS, g_tuneSettings.motorCurrent);
        Serial.printf("[AT] Loaded: axisSpd=%.0f diagSpd=%.0f current=%u (VALID)\n",
                      g_tuneSettings.safeSpeed, g_tuneSettings.safeSpeedDiag,
                      (unsigned)g_tuneSettings.motorCurrent);
    } else {
        setMotionProfileLock(false, AT_LOCKED_MICROSTEPS, 0);
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
    //
    // EARLY-EXIT: When the state machine enters CALIB_X_BOTTOM it starts
    // driving -X.  If the carriage has drifted enough that X is already
    // sitting on its hall sensor (HALL_X_PIN == LOW), driving further into
    // the wall causes a physical crash.  We detect this immediately and
    // abort the calibration, treating the current X position as the hall hit.
    unsigned long start = millis();
    bool found     = false;
    bool xEarlyHit = false;  // true when we used the early-exit path for X
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

        // ── Early-exit: X hall already triggered at CALIB_X_BOTTOM entry ──
        // The calibration just finished Y-seek and is about to drive -X.
        // If X hall is already LOW the carriage is at the corner in both axes;
        // record current X position and abort instead of driving into the wall.
        if (st == CALIB_X_BOTTOM && digitalRead(HALL_X_PIN) == LOW) {
            portENTER_CRITICAL(&gMux);
            g_calibState = CALIB_IDLE;
            g_vx_xy = 0.0f; g_vy_xy = 0.0f;
            portEXIT_CRITICAL(&gMux);
            xEarlyHit = true;
            found     = true;
            atLog("fastRef: X hall already LOW at CALIB_X_BOTTOM — skipping X seek");
            break;
        }

        // Normal path: both bottom sensors fired, state advanced past X-bottom
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
    // For the early-exit path, g_xHallPos was never written by the state machine,
    // so we use g_xAbs (the current physical position) as the X hit position.
    long hitY, hitX;
    portENTER_CRITICAL(&gMux);
    hitY = g_yHallPos;
    hitX = xEarlyHit ? g_xAbs : g_xHallPos;
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
    bool          inQuickSuite; // included in the diagonal-capable quick suite
    bool          inAxisSuite;  // axis-aligned moves only — used for the axis speed ramp
    bool          highDrift;    // triggers an intermediate ref check right after this test
    TrajCategory  category;
};

// ──────────────────────────────────────────────────────────────
// TEST SUITE
//
// inQuickSuite — the 5-test minimum used during ALL ramp phases.
//   Cut to the 5 tests that exercise the actual failure modes:
//   X/Y direction-reversal stress, both motors at full diagonal load,
//   and a full axis traversal.  Running more tests during ramp phases
//   makes each level too slow on large boards.
//
// inAxisSuite  — axis-aligned tests only (straight segments and L-shapes)
//   for the axis speed-extension ramp.
//
// highDrift    — triggers ONE intermediate ref check after this specific
//   test.  Only Diag-NE-long carries this flag: it is the single test most
//   likely to stall a motor and leave the carriage mis-positioned.
//   An end-of-session check covers the rest.
// ──────────────────────────────────────────────────────────────
static const TrajectoryTest AT_TESTS[] = {

// ==== AXIS-ONLY: X ============================================
//  name            pts                                                           n  spd   acc  rep  bidi   quick  axis   hiDrft  cat
{"X-short",       {{.45f,.50f},{.55f,.50f}},                                     2, 1.0f, 1.0f,  3, true,  false, true,  false, TC_AXIS_X},
{"X-L-medium",    {{.20f,.45f},{.80f,.45f},{.80f,.60f}},                         3, 1.0f, 1.0f,  2, true,  false, true,  false, TC_AXIS_X},
{"X-long",        {{.05f,.50f},{.95f,.50f}},                                     2, 1.0f, 1.0f,  2, true,  false, true,  false, TC_AXIS_X},
{"X-L-reversal",  {{.35f,.45f},{.65f,.45f},{.65f,.58f}},                         3, 1.0f, 1.0f,  3, true,  true,  true,  false, TC_AXIS_X},  // ← quick

// ==== AXIS-ONLY: Y ============================================
{"Y-short",       {{.50f,.45f},{.50f,.55f}},                                     2, 1.0f, 1.0f,  3, true,  false, true,  false, TC_AXIS_Y},
{"Y-L-medium",    {{.45f,.20f},{.45f,.80f},{.60f,.80f}},                         3, 1.0f, 1.0f,  2, true,  false, true,  false, TC_AXIS_Y},
{"Y-long",        {{.50f,.05f},{.50f,.95f}},                                     2, 1.0f, 1.0f,  2, true,  false, true,  false, TC_AXIS_Y},
{"Y-L-reversal",  {{.45f,.35f},{.45f,.65f},{.58f,.65f}},                         3, 1.0f, 1.0f,  3, true,  true,  true,  false, TC_AXIS_Y},  // ← quick

// ==== DIAGONALS ===============================================
{"Diag-NE-short", {{.40f,.40f},{.60f,.60f}},                                     2, 1.0f, 1.0f,  2, true,  false, false, false, TC_DIAGONAL},
{"Diag-NW-short", {{.60f,.40f},{.40f,.60f}},                                     2, 1.0f, 1.0f,  2, true,  false, false, false, TC_DIAGONAL},
{"Diag-NE-long",  {{.10f,.10f},{.90f,.90f}},                                     2, 1.0f, 1.0f,  1, true,  true,  false, true,  TC_DIAGONAL},  // ← quick + ref check
{"Diag-NW-long",  {{.90f,.10f},{.10f,.90f}},                                     2, 1.0f, 1.0f,  1, true,  true,  false, false, TC_DIAGONAL},  // ← quick
{"Corner-corner", {{.05f,.05f},{.95f,.95f}},                                     2, 0.8f, 1.0f,  2, true,  false, false, false, TC_DIAGONAL},
{"Anti-diagonal", {{.05f,.95f},{.95f,.05f}},                                     2, 0.8f, 1.0f,  2, true,  false, false, false, TC_DIAGONAL},
{"Diag-reversal", {{.30f,.30f},{.70f,.70f}},                                     2, 1.0f, 1.0f,  5, true,  false, false, false, TC_DIAGONAL},

// ==== BOARD REGIONS ===========================================
{"Center-micro",  {{.47f,.47f},{.53f,.53f}},                                     2, 1.0f, 1.0f,  4, true,  false, false, false, TC_REGION},
{"Left-edge-V",   {{.05f,.20f},{.05f,.80f}},                                     2, 1.0f, 1.0f,  2, true,  false, true,  false, TC_REGION},
{"Right-edge-V",  {{.95f,.20f},{.95f,.80f}},                                     2, 1.0f, 1.0f,  2, true,  false, true,  false, TC_REGION},
{"Bottom-edge-H", {{.20f,.05f},{.80f,.05f}},                                     2, 1.0f, 1.0f,  2, true,  false, true,  false, TC_REGION},
{"Top-edge-H",    {{.20f,.95f},{.80f,.95f}},                                     2, 1.0f, 1.0f,  2, true,  false, true,  false, TC_REGION},
{"Cross-board-H", {{.05f,.50f},{.95f,.50f}},                                     2, 1.0f, 1.0f,  2, true,  false, true,  false, TC_REGION},
{"Cross-board-V", {{.50f,.05f},{.50f,.95f}},                                     2, 1.0f, 1.0f,  2, true,  false, true,  false, TC_REGION},
{"Corner-NE-loc", {{.80f,.80f},{.95f,.95f}},                                     2, 0.7f, 1.0f,  3, true,  false, false, false, TC_REGION},
{"Corner-SW-loc", {{.05f,.05f},{.20f,.20f}},                                     2, 0.7f, 1.0f,  3, true,  false, false, false, TC_REGION},
{"Zone-transit",  {{.10f,.10f},{.90f,.50f}},                                     2, 1.0f, 1.0f,  2, true,  false, false, false, TC_REGION},

// ==== MULTI-SEGMENT ===========================================
{"Square-CW",    {{.30f,.30f},{.70f,.30f},{.70f,.70f},{.30f,.70f},{.30f,.30f}},  5, 1.0f, 1.0f,  2, false, false, false, false, TC_MULTI_SEG},
{"Diamond",      {{.50f,.15f},{.85f,.50f},{.50f,.85f},{.15f,.50f},{.50f,.15f}},  5, 1.0f, 1.0f,  2, false, false, false, false, TC_MULTI_SEG},
{"Zig-zag-H",    {{.10f,.30f},{.30f,.70f},{.50f,.30f},{.70f,.70f},{.90f,.30f}},  5, 1.0f, 1.0f,  2, true,  false, false, false, TC_MULTI_SEG},
{"Zig-zag-V",    {{.30f,.10f},{.70f,.30f},{.30f,.50f},{.70f,.70f},{.30f,.90f}},  5, 1.0f, 1.0f,  2, false, false, false, false, TC_MULTI_SEG},
{"Cross-star",   {{.50f,.50f},{.50f,.92f},{.92f,.50f},{.50f,.08f},{.08f,.50f},{.50f,.50f}}, 6, 1.0f, 1.0f, 2, false, false, false, false, TC_MULTI_SEG},
{"Rectangle-H",  {{.15f,.40f},{.85f,.40f},{.85f,.60f},{.15f,.60f},{.15f,.40f}},  5, 1.0f, 1.0f,  2, false, false, false, false, TC_MULTI_SEG},

// ==== CHESS-RELEVANT ==========================================
{"Chess-1sq-H",  {{.40f,.50f},{.525f,.50f}},                                     2, 0.65f,1.0f,  6, true,  false, true,  false, TC_CHESS},
{"Chess-2sq-H",  {{.35f,.50f},{.60f,.50f}},                                      2, 0.80f,1.0f,  4, true,  false, true,  false, TC_CHESS},
{"Rook-H",       {{.10f,.50f},{.90f,.50f}},                                      2, 1.0f, 1.0f,  1, true,  true,  true,  false, TC_CHESS},  // ← quick + axis
{"Rook-V",       {{.50f,.10f},{.50f,.90f}},                                      2, 1.0f, 1.0f,  1, true,  false, true,  false, TC_CHESS},
{"Bishop-full",  {{.10f,.10f},{.90f,.90f}},                                      2, 1.0f, 1.0f,  3, true,  false, false, false, TC_CHESS},
{"Knight-L",     {{.40f,.40f},{.65f,.40f},{.65f,.525f}},                         3, 0.85f,1.0f,  4, true,  false, false, false, TC_CHESS},
{"Micro-place",  {{.49f,.50f},{.51f,.50f}},                                      2, 0.40f,1.0f,  8, true,  false, true,  false, TC_CHESS},

};  // end AT_TESTS

static const uint8_t AT_NUM_TESTS = sizeof(AT_TESTS) / sizeof(AT_TESTS[0]);

// =====================
// Gestion des tests bannis (timeouts persistants)
// =====================
#define AT_BANNED_KEY "bannedTests"
static bool bannedTests[AT_NUM_TESTS] = {0};

static void loadBannedTests() {
    s_prefs.begin(NVS_NS, true);
    size_t sz = s_prefs.getBytes(AT_BANNED_KEY, bannedTests, sizeof(bannedTests));
    s_prefs.end();
    if (sz != sizeof(bannedTests)) {
        memset(bannedTests, 0, sizeof(bannedTests));
    }
}

static void saveBannedTests() {
    s_prefs.begin(NVS_NS, false);
    s_prefs.putBytes(AT_BANNED_KEY, bannedTests, sizeof(bannedTests));
    s_prefs.end();
}

static void banTest(uint8_t ti) {
    if (ti < AT_NUM_TESTS) {
        bannedTests[ti] = true;
        saveBannedTests();
    }
}

static void autoTuneInitBanned() {
    loadBannedTests();
}

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
                                          float sessionAxisSpeed,
                                          float sessionDiagSpeed,
                                          float sessionAccel) {
    if (g_tuneAbortReq) return TR_ABORT;

    // Apply per-test speed/accel scale (e.g. slower for precision/corner tests)
    const float speedScale = clampf(t.speedScale, 0.05f, 2.0f);
    g_overrideVmax     = sessionAxisSpeed * speedScale;
    g_overrideDiagVmax = sessionDiagSpeed * speedScale;
    g_overrideAccel    = sessionAccel * clampf(t.accelScale, 0.05f, 2.0f);

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
        else                       vTaskDelay(20 / portTICK_PERIOD_MS);
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
            else                       vTaskDelay(20 / portTICK_PERIOD_MS);
        }

        // Reverse: pts[nPts-2] … pts[0]  (bidirectional only)
        if (t.bidirectional) {
            for (int pi = (int)t.nPts - 2; pi >= 0 && result == TR_OK && !g_tuneAbortReq; pi--) {
                long x, y;
                normToAbs(t.pts[pi].u, t.pts[pi].v, x, y);
                ArriveResult ar = tuneMoveToPath(x, y);
                if      (ar == AR_ABORT)   result = TR_ABORT;
                else if (ar == AR_TIMEOUT) result = TR_TIMEOUT;
                else                       vTaskDelay(20 / portTICK_PERIOD_MS);
            }
        }
    }

    // Restore session defaults (don't carry per-test scale into next test)
    g_overrideVmax     = sessionAxisSpeed;
    g_overrideDiagVmax = sessionDiagSpeed;
    g_overrideAccel    = sessionAccel;
    return result;
}

// (needsPostCheck replaced by t.highDrift flag in the test table)

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

// Axis-only motion is mechanically easier than diagonal stress cases, so keep
// a smaller safety haircut when selecting final straight-line speed.
static float axisGraduatedMargin(float score) {
    float t = clampf((score - AT_SCORE_PASS_THRESHOLD) /
                     (1.0f - AT_SCORE_PASS_THRESHOLD), 0.0f, 1.0f);
    return 0.88f + t * 0.08f; // 0.88 .. 0.96
}

static bool runTrajectorySession(bool quickMode, bool axisOnly,
                                  float sessionAxisSpeed, float sessionDiagSpeed, float sessionAccel,
                                  long &refYMin, long &refXMin,
                                  SessionStats &stats);

static int currentIndexForStart(uint16_t targetmA) {
    int idx = (int)AT_N_CURRENTS - 1;
    for (int i = 0; i < (int)AT_N_CURRENTS; i++) {
        if (AT_CURRENTS[i] >= targetmA) {
            idx = i;
            break;
        }
    }
    return idx;
}

static uint16_t currentForIndex(int idx) {
    if (idx < 0) idx = 0;
    if (idx >= (int)AT_N_CURRENTS) idx = (int)AT_N_CURRENTS - 1;
    return AT_CURRENTS[idx];
}

static void applyTuneCurrent(int idx) {
    uint16_t mA = currentForIndex(idx);
    g_tuneCurrentMa = mA;
    if (isMotionProfileLocked()) setCurrentOverrides(mA, mA);
    else setCurrentOverrides(mA, (uint16_t)((float)mA * 1.20f));
}

static void applyTuneCurrentMa(uint16_t mA) {
    g_tuneCurrentMa = mA;
    if (isMotionProfileLocked()) setCurrentOverrides(mA, mA);
    else setCurrentOverrides(mA, (uint16_t)((float)mA * 1.20f));
}

static void setTuneLiveSettings(float axisSpeed, float diagSpeed, float accel, float decel) {
    g_tuneLiveAxisSpeed = axisSpeed;
    g_tuneLiveDiagSpeed = diagSpeed;
    g_tuneLiveAccel     = accel;
    g_tuneLiveDecel     = decel;
}

static float axisSpeedForDiagSpeed(float diagSpeed) {
    float desired = diagSpeed * 2.0f;
    float axisSpeed = AT_SPEEDS_AXIS[0];
    for (int i = 0; i < (int)AT_N_SPEEDS_AXIS; i++) {
        if (AT_SPEEDS_AXIS[i] <= desired) axisSpeed = AT_SPEEDS_AXIS[i];
    }
    if (axisSpeed < diagSpeed) axisSpeed = diagSpeed;
    return axisSpeed;
}

static bool runAdaptiveSessionLevel(const char *label,
                                     bool quickMode, bool axisOnly,
                                     float sessionAxisSpeed, float sessionDiagSpeed,
                                     float sessionAccel, float sessionDecel,
                                     long &refYMin, long &refXMin,
                                     int &currentIdx,
                                     SessionStats &stats,
                                     float &scoreOut, bool &levelOkOut,
                                     int maxCurrentIdx = (int)AT_N_CURRENTS - 1) {
    if (maxCurrentIdx < 0) maxCurrentIdx = 0;
    if (maxCurrentIdx >= (int)AT_N_CURRENTS) maxCurrentIdx = (int)AT_N_CURRENTS - 1;

    int retryBudget = maxCurrentIdx - currentIdx + 3;
    while (!g_tuneAbortReq) {
        if (retryBudget-- <= 0) {
            atLog("%s: aborting adaptive loop after too many retries", label);
            g_overrideDecel = 0.0f;
            return false;
        }

        uint16_t mA = currentForIndex(currentIdx);
        applyTuneCurrent(currentIdx);
        g_overrideDecel = sessionDecel;
        setTuneLiveSettings(sessionAxisSpeed, sessionDiagSpeed, sessionAccel, sessionDecel);

        atLog("%s @ %u mA", label, (unsigned)mA);
        if (!runTrajectorySession(quickMode, axisOnly,
                                  sessionAxisSpeed, sessionDiagSpeed, sessionAccel,
                                  refYMin, refXMin, stats)) {
            g_overrideDecel = 0.0f;
            return false;
        }

        scoreOut = computeLevelScore(stats);
        bool hardFail = (stats.timeouts > 0) || (!stats.driftOk) || (stats.driftExceeded > 0);
        levelOkOut = !hardFail && (scoreOut >= AT_SCORE_PASS_THRESHOLD);
        atLog("%s @ %u mA: score=%.2f %s (drift=%.1f timeouts=%u refFails=%u)",
              label, (unsigned)mA, scoreOut, levelOkOut ? "PASS" : "FAIL",
              stats.maxDriftTotal, (unsigned)stats.timeouts, (unsigned)stats.driftExceeded);

        if (levelOkOut) {
            g_overrideDecel = 0.0f;
            return true;
        }

        if (currentIdx >= maxCurrentIdx) {
            atLog("%s FAIL even at current cap %u mA", label, (unsigned)mA);
            g_overrideDecel = 0.0f;
            return true;
        }

        int strongerIdx = currentIdx + 1;
        atLog("%s FAIL at %u mA -> retry with %u mA",
              label, (unsigned)mA, (unsigned)currentForIndex(strongerIdx));
        currentIdx = strongerIdx;
        applyTuneCurrent(currentIdx);
        setTuneLiveSettings(sessionAxisSpeed, sessionDiagSpeed, sessionAccel, sessionDecel);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    g_overrideDecel = 0.0f;
    return false;
}

// Intermediate ref checks are performed only after tests flagged highDrift=true
// (currently only Diag-NE-long — the single most crash-prone trajectory).
// A mandatory end-of-session check covers cumulative drift for all other tests.

// ──────────────────────────────────────────────────────────────
// runTrajectorySession()
// Executes the trajectory test suite filtered by the mode flag:
//   quickMode  — runs only tests with inQuickSuite=true
//   axisOnly   — runs only tests with inAxisSuite=true (for axis speed ramp)
//
// REFERENCE CHECKING STRATEGY
// ----------------------------
// One fast home-corner reference check is performed after each completed
// test and once at session end. Trajectory timeouts still trigger a full
// recovery/re-home if the carriage stalls.
//
// Returns false only on abort; driftOk in stats encodes pass/fail.
// ──────────────────────────────────────────────────────────────
static bool runTrajectorySession(bool quickMode, bool axisOnly,
                                  float sessionAxisSpeed, float sessionDiagSpeed, float sessionAccel,
                                  long &refYMin, long &refXMin,
                                  SessionStats &stats) {
    memset(&stats, 0, sizeof(stats));
    stats.driftOk = true;
    bool sessionHardFail = false;

    long cx, cy;
    portENTER_CRITICAL(&gMux);
    cx = g_xCenterTarget;
    cy = g_yCenterTarget;
    portEXIT_CRITICAL(&gMux);

    g_overrideVmax     = sessionAxisSpeed;
    g_overrideDiagVmax = sessionDiagSpeed;
    g_overrideAccel    = sessionAccel;

    if (tuneMoveTo(cx, cy) != AR_OK) {
        atLog("Session: failed to reach center at start");
        g_overrideVmax     = 0.0f;
        g_overrideDiagVmax = 0.0f;
        g_overrideAccel    = 0.0f;
        return false;
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);

    // ── Helper: run a fast ref check and fold results into stats ──
    // Used both for selective post-test checks (highDrift tests only) and
    // the mandatory end-of-session check.
    auto doRefCheck = [&](const char *label, TrajCategory cat) -> bool {
        stats.refChecks++;
        if (cat < TC_NUM) stats.catDriftChecks[cat]++;

        DriftMeasure dm = {};
        FastRefResult fr = fastHomeCornerCheck(refYMin, refXMin, dm);
        bool needsRecovery = false;

        if (fr == FR_ABORT) {
            g_overrideVmax     = 0.0f;
            g_overrideDiagVmax = 0.0f;
            g_overrideAccel    = 0.0f;
            return false;
        }

        if (fr == FR_TIMEOUT) {
            stats.driftExceeded++;
            stats.driftOk = false;
            if (cat < TC_NUM) stats.catDriftFails[cat]++;
            atLog("fastRef TIMEOUT after '%s' — counted as drift fail", label);
            needsRecovery = true;
            sessionHardFail = true;
        } else {
            if (fabsf(dm.driftY) > stats.maxDriftY) stats.maxDriftY = fabsf(dm.driftY);
            if (fabsf(dm.driftX) > stats.maxDriftX) stats.maxDriftX = fabsf(dm.driftX);
            if (dm.magnitude > stats.maxDriftTotal)  stats.maxDriftTotal = dm.magnitude;
            if (dm.exceeded) {
                stats.driftExceeded++;
                stats.driftOk = false;
                if (cat < TC_NUM) stats.catDriftFails[cat]++;
                atLog(">> DRIFT EXCEEDED after '%s': |%.1f| (Y:%.1f X:%.1f)",
                      label, dm.magnitude, dm.driftY, dm.driftX);
                needsRecovery = true;
                sessionHardFail = true;
            }
            refYMin = dm.actualY;
            refXMin = dm.actualX;
        }

        if (needsRecovery) {
            long newY, newX;
            atLog("fastRef fail after '%s' — refreshing calibration reference", label);
            if (!recoverReferenceAfterDrift(newY, newX)) {
                g_overrideVmax     = 0.0f;
                g_overrideDiagVmax = 0.0f;
                g_overrideAccel    = 0.0f;
                return false;
            }
            refYMin = newY;
            refXMin = newX;
        }

        g_overrideVmax     = sessionAxisSpeed;
        g_overrideDiagVmax = sessionDiagSpeed;
        g_overrideAccel    = sessionAccel;
        return true;
    };

    // ── Iterate through the test suite ──
    for (uint8_t ti = 0; ti < AT_NUM_TESTS && !g_tuneAbortReq; ti++) {
        if (bannedTests[ti]) continue; // Ignore les tests bannis
        const TrajectoryTest &t = AT_TESTS[ti];

        if (axisOnly) {
            if (!t.inAxisSuite) continue;
            if (quickMode && !t.inQuickSuite) continue;
        } else {
            if (quickMode && !t.inQuickSuite) continue;
        }

        TrajectoryTest t_run = t;
        if (quickMode && t_run.reps > 1) t_run.reps = 1;

        atLog("[%d/%d] %s  spd=%.0fx%.2f  rep=%u%s",
              (int)ti + 1, (int)AT_NUM_TESTS,
              t.name, sessionDiagSpeed, t.speedScale,
              (unsigned)t_run.reps, t.bidirectional ? " bidi" : "");

        TrajRunResult r = runSingleTrajectory(t_run, sessionAxisSpeed, sessionDiagSpeed, sessionAccel);

        if (r == TR_ABORT) {
            g_overrideVmax     = 0.0f;
            g_overrideDiagVmax = 0.0f;
            g_overrideAccel    = 0.0f;
            return false;
        }

        if (r == TR_TIMEOUT) {
            stats.timeouts++;
            stats.catFails[t.category]++;
            atLog("  TIMEOUT on '%s' — recovering and re-homing", t.name);
            sessionHardFail = true;

            long newY, newX;
            if (!recoverReferenceAfterDrift(newY, newX)) {
                g_overrideVmax     = 0.0f;
                g_overrideDiagVmax = 0.0f;
                g_overrideAccel    = 0.0f;
                return false;
            }
            refYMin = newY;
            refXMin = newX;

            g_overrideVmax     = sessionAxisSpeed;
            g_overrideDiagVmax = sessionDiagSpeed;
            g_overrideAccel    = sessionAccel;
            tuneMoveTo(cx, cy);
            break;
        }

        stats.movesRun++;
        vTaskDelay(20 / portTICK_PERIOD_MS);

        // Intermediate ref check only after high-drift tests (Diag-NE-long).
        if (t.highDrift && !g_tuneAbortReq) {
            if (!doRefCheck(t.name, t.category)) return false;
            if (sessionHardFail) break;
        }
    }

    if (g_tuneAbortReq) {
        g_overrideVmax     = 0.0f;
        g_overrideDiagVmax = 0.0f;
        g_overrideAccel    = 0.0f;
        return false;
    }

    if (sessionHardFail) {
        g_overrideVmax     = 0.0f;
        g_overrideDiagVmax = 0.0f;
        g_overrideAccel    = 0.0f;
        return true;
    }

    // ── End-of-session reference check ──
    atLog("Session tests done — %u run, %u timeout, %u ref-checks so far",
          (unsigned)stats.movesRun, (unsigned)stats.timeouts, (unsigned)stats.refChecks);

    if (!doRefCheck("session-end", TC_NUM)) return false;

    atLog("Session: %u tests, %u timeouts, %u ref-checks, drift=%.1f(Y:%.1f X:%.1f) -> %s",
          (unsigned)stats.movesRun, (unsigned)stats.timeouts, (unsigned)stats.refChecks,
          stats.maxDriftTotal, stats.maxDriftY, stats.maxDriftX,
          stats.driftOk ? "PASS" : "FAIL");

    g_overrideVmax     = 0.0f;
    g_overrideDiagVmax = 0.0f;
    g_overrideAccel    = 0.0f;
    return true;
}

// ============================================================

// Appeler autoTuneInitBanned() dans autoTuneInit() pour charger la liste au boot
// PHASE RUNNERS  (revised to use trajectory sessions)
// ============================================================


// ---- Phase 3a: Diagonal Speed Ramp ----
// Uses the full quick suite (axis + diagonal moves).
// Limited by diagonal performance: one motor runs at √2× carriage speed.
// Returns the safe carriage speed for diagonal moves.
static float phaseSpeedRampDiag(long refYMin, long refXMin, float baseAccel,
                                int &currentIdx,
                                int maxCurrentIdx = (int)AT_N_CURRENTS - 1,
                                TunePhase phase = AT_PHASE_SPEED_RAMP) {
    setPhaseProgress(phase, 0);
    atLog("=== Phase: Diagonal Speed Ramp (%d levels) ===", (int)AT_N_SPEEDS_DIAG);

    float lastGoodSpeed = AT_SPEEDS_DIAG[0];
    float lastGoodScore = 1.0f;
    bool  anyPass = false;

    for (int si = 0; si < (int)AT_N_SPEEDS_DIAG && !g_tuneAbortReq; si++) {
        float speed = AT_SPEEDS_DIAG[si];
        float axisSpeed = axisSpeedForDiagSpeed(speed);
        char label[64];
        snprintf(label, sizeof(label), "Diag speed %d/%d: diag=%.0f axis=%.0f",
                 si + 1, (int)AT_N_SPEEDS_DIAG, speed, axisSpeed);
        atLog("%s", label);

        SessionStats s;
        float score = 0.0f;
        bool  levelOk = false;
        if (!runAdaptiveSessionLevel(label, /*quick=*/true, /*axisOnly=*/false,
                                     axisSpeed, speed, baseAccel, /*decel=*/0.0f,
                                     refYMin, refXMin, currentIdx, s, score, levelOk,
                                     maxCurrentIdx)) {
            atLog("Diag speed ramp aborted during adaptive current recovery");
            return 0.0f;
        }

        if (!levelOk) {
            if (!anyPass) {
                atLog("Diag speed ramp: even the lowest diagonal speed could not be validated");
                return 0.0f;
            }
            for (uint8_t c = 0; c < TC_NUM; c++) {
                if (s.catFails[c]) atLog("  cat %s: tmo=%u", TC_NAMES[c], (unsigned)s.catFails[c]);
            }
            break;
        }
        lastGoodSpeed = speed;
        lastGoodScore = score;
        anyPass = true;
        setPhaseProgress(phase, (int)(50.0f * (si + 1) / AT_N_SPEEDS_DIAG));
    }

    float margin = graduatedMargin(lastGoodScore);
    float result = lastGoodSpeed * margin;
    if (result < 3000.0f) result = 3000.0f;
    atLog("Diag speed done: lastGood=%.0f score=%.2f margin=%.2f safe=%.0f",
          lastGoodSpeed, lastGoodScore, margin, result);
    return result;
}

// ---- Phase 3b: Axis Speed Ramp ----
// Runs axis-only tests (both motors equal load) above the diagonal safe speed.
// Carriage can go faster on straight lines because neither motor is overloaded.
// Returns the safe carriage speed for axis-only moves.
static float phaseSpeedRampAxis(long refYMin, long refXMin,
                                 float diagSafeSpeed, float baseAccel,
                                 int &currentIdx,
                                 int maxCurrentIdx = (int)AT_N_CURRENTS - 1,
                                 TunePhase phase = AT_PHASE_SPEED_RAMP) {
    atLog("=== Phase: Axis Speed Extension (above %.0f) ===", diagSafeSpeed);

    // Find the first AT_SPEEDS_AXIS level above the diagonal safe speed.
    int startIdx = (int)AT_N_SPEEDS_AXIS;  // default: no levels to test
    for (int i = 0; i < (int)AT_N_SPEEDS_AXIS; i++) {
        if (AT_SPEEDS_AXIS[i] > diagSafeSpeed) { startIdx = i; break; }
    }

    if (startIdx >= (int)AT_N_SPEEDS_AXIS) {
        atLog("Axis ramp: no levels above diag safe speed — using diagSafe=%.0f as axis speed", diagSafeSpeed);
        setPhaseProgress(phase, 100);
        return diagSafeSpeed;  // already as fast as axis allows at this range
    }

    float lastGoodSpeed = diagSafeSpeed;
    float lastGoodScore = 1.0f;
    int   numTested     = (int)AT_N_SPEEDS_AXIS - startIdx;

    for (int si = startIdx; si < (int)AT_N_SPEEDS_AXIS && !g_tuneAbortReq; si++) {
        float speed = AT_SPEEDS_AXIS[si];
        char label[64];
        snprintf(label, sizeof(label), "Axis speed %d/%d: %.0f steps/s",
                 si - startIdx + 1, numTested, speed);
        atLog("%s", label);

        SessionStats s;
        float score = 0.0f;
        bool  levelOk = false;
        if (!runAdaptiveSessionLevel(label, /*quick=*/true, /*axisOnly=*/true,
                                     speed, diagSafeSpeed, baseAccel, /*decel=*/0.0f,
                                     refYMin, refXMin, currentIdx, s, score, levelOk,
                                     maxCurrentIdx)) {
            atLog("Axis speed ramp aborted during adaptive current recovery");
            return diagSafeSpeed;
        }

        if (!levelOk) {
            for (uint8_t c = 0; c < TC_NUM; c++) {
                if (s.catFails[c]) atLog("  cat %s: tmo=%u", TC_NAMES[c], (unsigned)s.catFails[c]);
            }
            break;
        }
        lastGoodSpeed = speed;
        lastGoodScore = score;
        setPhaseProgress(phase,
                          50 + (int)(50.0f * (si - startIdx + 1) / numTested));
    }

    float margin = axisGraduatedMargin(lastGoodScore);
    float result = lastGoodSpeed * margin;
    if (result < diagSafeSpeed) result = diagSafeSpeed;  // never go below diagonal safe
    atLog("Axis speed done: lastGood=%.0f score=%.2f margin=%.2f safe=%.0f",
          lastGoodSpeed, lastGoodScore, margin, result);
    setPhaseProgress(phase, 100);
    return result;
}


// ---- Phase 5: Current Sweep ----
// Tests from lowest to highest current; stores the first level that passes.
// This gives the minimum current that still preserves reliable motion.
// Score is logged per level for diagnostics.  If the last good level had
// a low score (borderline pass), the log will indicate which categories
// were the cause.
static uint16_t phaseCurrentTune(long &refYMin, long &refXMin,
                                   int startCurrentIdx,
                                   float safeSpeedAxis, float safeSpeedDiag, float safeAccel,
                                   float safeDecel) {
    if (startCurrentIdx < 0) startCurrentIdx = 0;
    if (startCurrentIdx >= (int)AT_N_CURRENTS) startCurrentIdx = (int)AT_N_CURRENTS - 1;
    const int numCurrents = startCurrentIdx + 1;
    setPhaseProgress(AT_PHASE_CURRENT, 0);
    atLog("=== Phase: Current Tune (%d levels, quick suite) ===", numCurrents);
    atLog("Current sweep params: axisSpeed=%.0f diagSpeed=%.0f accel=%.0f decel=%.0f score>=%.2f",
          safeSpeedAxis, safeSpeedDiag, safeAccel, safeDecel, AT_SCORE_PASS_THRESHOLD);
    atLog("Current minimisation: retesting from lowest current up to %u mA",
          (unsigned)currentForIndex(startCurrentIdx));

    uint16_t lastGoodCurrent = currentForIndex(startCurrentIdx);
    float    lastGoodScore   = 1.0f;
    bool     foundPass       = false;

    for (int ci = 0; ci <= startCurrentIdx && !g_tuneAbortReq; ci++) {
        uint16_t mA = AT_CURRENTS[ci];
        applyTuneCurrentMa(mA);
        g_overrideDecel = safeDecel;
        setTuneLiveSettings(safeSpeedAxis, safeSpeedDiag, safeAccel, safeDecel);
        atLog("Current level %d/%d: %u mA", ci + 1, numCurrents, (unsigned)mA);
        vTaskDelay(250 / portTICK_PERIOD_MS);  // let drivers settle

        SessionStats s;
        if (!runTrajectorySession(/*quick=*/true, /*axisOnly=*/false,
                                   safeSpeedAxis, safeSpeedDiag, safeAccel, refYMin, refXMin, s)) {
            g_overrideDecel = 0.0f;
            return lastGoodCurrent;
        }

        float score   = computeLevelScore(s);
        bool  hardFail = (s.timeouts > 0) || (!s.driftOk) || (s.driftExceeded > 0);
        bool  levelOk = !hardFail && (score >= AT_SCORE_PASS_THRESHOLD);
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
            continue;
        }
        lastGoodCurrent = mA;
        lastGoodScore   = score;
        foundPass = true;
        setPhaseProgress(AT_PHASE_CURRENT, (int)(100.0f * (ci + 1) / numCurrents));
        break;
    }

    g_overrideDecel = 0.0f;
    if (!foundPass) {
        atLog("Current sweep: no lower current passed, keeping adaptive current %u mA",
              (unsigned)lastGoodCurrent);
    }
    atLog("Current sweep done: safe=%u mA (score=%.2f)", (unsigned)lastGoodCurrent, lastGoodScore);
    return lastGoodCurrent;
}

// ---- Phase 6: Final characterisation at tuned params (quick suite) ----
static bool phaseFinalCharacterize(long &refYMin, long &refXMin,
                                    float safeSpeedAxis, float safeSpeedDiag, float safeAccel,
                                    float safeDecel, uint16_t safeCurrent) {
    setPhaseProgress(AT_PHASE_APPROACH, 0);  // reuse APPROACH slot for this phase
    atLog("=== Phase: Final Characterisation (quick suite at tuned params) ===");
    atLog("Final params: axisSpeed=%.0f diagSpeed=%.0f accel=%.0f decel=%.0f current=%u",
          safeSpeedAxis, safeSpeedDiag, safeAccel, safeDecel, (unsigned)safeCurrent);

    applyTuneCurrentMa(safeCurrent);
    g_overrideDecel = safeDecel;
    setTuneLiveSettings(safeSpeedAxis, safeSpeedDiag, safeAccel, safeDecel);

    SessionStats s;
    if (!runTrajectorySession(/*quick=*/true, /*axisOnly=*/false,
                               safeSpeedAxis, safeSpeedDiag, safeAccel, refYMin, refXMin, s)) {
        g_overrideDecel = 0.0f;
        return false;
    }

    atLog("--- Final characterisation report ---");
    atLog("  Tests run:   %u", (unsigned)s.movesRun);
    atLog("  Timeouts:    %u", (unsigned)s.timeouts);
    atLog("  Ref checks:  %u  (%u exceeded)", (unsigned)s.refChecks, (unsigned)s.driftExceeded);
    atLog("  Max drift:   %.1f steps (Y:%.1f X:%.1f)  %s",
          s.maxDriftTotal, s.maxDriftY, s.maxDriftX, s.driftOk ? "PASS" : "WARN");
    float score = computeLevelScore(s);
    bool finalOk = (s.timeouts == 0) && s.driftOk && (s.driftExceeded == 0) &&
                   (score >= AT_SCORE_PASS_THRESHOLD);
    atLog("  Session score: %.2f (threshold %.2f)", score, AT_SCORE_PASS_THRESHOLD);
    for (uint8_t c = 0; c < TC_NUM; c++) {
        atLog("  %s failures: %u", TC_NAMES[c], (unsigned)s.catFails[c]);
    }
    atLog("  Final verdict: %s", finalOk ? "PASS" : "FAIL");
    g_overrideDecel = 0.0f;
    setPhaseProgress(AT_PHASE_APPROACH, 100);
    return finalOk;
}

static void reclaimFinalSpeeds(long &refYMin, long &refXMin,
                               float &safeSpeedAxis, float &safeSpeedDiag,
                               float safeAccel, float safeDecel,
                               uint16_t safeCurrent) {
    const float diagCandidates[] = {
        safeSpeedDiag * 1.05f,
        safeSpeedDiag * 1.10f,
        safeSpeedDiag * 1.15f
    };
    const float axisCandidates[] = {
        safeSpeedAxis * 1.04f,
        safeSpeedAxis * 1.08f,
        safeSpeedAxis * 1.12f
    };

    atLog("=== Phase: Speed Reclaim ===");

    for (uint8_t i = 0; i < (sizeof(diagCandidates) / sizeof(diagCandidates[0])) && !g_tuneAbortReq; i++) {
        float candidateDiag = diagCandidates[i];
        if (candidateDiag > AT_SPEEDS_DIAG[AT_N_SPEEDS_DIAG - 1]) {
            candidateDiag = AT_SPEEDS_DIAG[AT_N_SPEEDS_DIAG - 1];
        }
        if (candidateDiag <= safeSpeedDiag + 1.0f) continue;

        atLog("Reclaim diag speed: %.0f -> %.0f", safeSpeedDiag, candidateDiag);
        if (!phaseFinalCharacterize(refYMin, refXMin, safeSpeedAxis, candidateDiag, safeAccel,
                                    safeDecel, safeCurrent)) {
            break;
        }
        safeSpeedDiag = candidateDiag;
        if (safeSpeedAxis < safeSpeedDiag) safeSpeedAxis = safeSpeedDiag;
    }

    for (uint8_t i = 0; i < (sizeof(axisCandidates) / sizeof(axisCandidates[0])) && !g_tuneAbortReq; i++) {
        float candidateAxis = axisCandidates[i];
        if (candidateAxis > AT_SPEEDS_AXIS[AT_N_SPEEDS_AXIS - 1]) {
            candidateAxis = AT_SPEEDS_AXIS[AT_N_SPEEDS_AXIS - 1];
        }
        if (candidateAxis < safeSpeedDiag) candidateAxis = safeSpeedDiag;
        if (candidateAxis <= safeSpeedAxis + 1.0f) continue;

        atLog("Reclaim axis speed: %.0f -> %.0f", safeSpeedAxis, candidateAxis);
        if (!phaseFinalCharacterize(refYMin, refXMin, candidateAxis, safeSpeedDiag, safeAccel,
                                    safeDecel, safeCurrent)) {
            break;
        }
        safeSpeedAxis = candidateAxis;
    }
}

static bool stabilizeFinalSettings(long &refYMin, long &refXMin,
                                   float &safeSpeedAxis, float &safeSpeedDiag,
                                   float &safeAccel, float &safeDecel,
                                   uint16_t &safeCurrent) {
    const uint16_t maxSafeCurrent = currentForIndex((int)AT_N_CURRENTS - 1);
    const float origSpeedAxis = safeSpeedAxis;
    const float origSpeedDiag = safeSpeedDiag;

    // Step 1: Try at current (tuned) settings
    if (phaseFinalCharacterize(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag, safeAccel,
                               safeDecel, safeCurrent)) {
        reclaimFinalSpeeds(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag, safeAccel,
                           safeDecel, safeCurrent);
        return true;
    }

    // Step 2: Bump to max current and retry
    if (safeCurrent < maxSafeCurrent) {
        atLog("Final validation failed at %u mA -> retrying at max safe current %u mA",
              (unsigned)safeCurrent, (unsigned)maxSafeCurrent);
        safeCurrent = maxSafeCurrent;
        if (phaseFinalCharacterize(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag, safeAccel,
                                   safeDecel, safeCurrent)) {
            reclaimFinalSpeeds(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag, safeAccel,
                               safeDecel, safeCurrent);
            return true;
        }
    }

    // Step 3: Step down from 90% to 50% of tuned speed until a level passes.
    // Each passing level is saved as the new safe setting; reclaim then tries to push back up.
    static const float SPEED_STEPS[] = { 0.90f, 0.80f, 0.70f, 0.60f, 0.50f };
    for (int i = 0; i < 5 && !g_tuneAbortReq; i++) {
        float ratio = SPEED_STEPS[i];
        safeSpeedDiag = origSpeedDiag * ratio;
        safeSpeedAxis = origSpeedAxis * ratio;
        if (safeSpeedAxis < safeSpeedDiag) safeSpeedAxis = safeSpeedDiag;

        atLog("Stabilisation step %d/5: %.0f%% speed (axis=%.0f diag=%.0f current=%u)",
              i + 1, ratio * 100.0f, safeSpeedAxis, safeSpeedDiag, (unsigned)safeCurrent);

        if (phaseFinalCharacterize(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag, safeAccel,
                                   safeDecel, safeCurrent)) {
            atLog("Stabilisation: stable at %.0f%% of tuned speed (axis=%.0f diag=%.0f)",
                  ratio * 100.0f, safeSpeedAxis, safeSpeedDiag);
            reclaimFinalSpeeds(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag, safeAccel,
                               safeDecel, safeCurrent);
            return true;
        }
    }

    // Nothing passed even at 50% — caller will save the 50% floor as a last resort
    atLog("Stabilisation: no stable configuration found down to 50%% of tuned speed");
    return false;
}

// ============================================================
// Main tune sequence
// ============================================================
static bool runTuneSequence() {
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
    memset(bannedTests, 0, sizeof(bannedTests));  // clear bans from any previous run

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

    int currentIdx = 0;
    setMotionProfileLock(true, AT_LOCKED_MICROSTEPS, currentForIndex(currentIdx));
    applyTuneCurrent(currentIdx);
    setTuneLiveSettings(0.0f, 0.0f, 0.0f, 0.0f);
    atLog("Adaptive current start: %u mA", (unsigned)currentForIndex(currentIdx));

    // ---- Phase 2: Speed Ramp (diagonal then axis) ----
    const float testAccelBase = AT_BASELINE_ACCEL;

    float safeSpeedDiag = phaseSpeedRampDiag(refYMin, refXMin, testAccelBase, currentIdx);
    if (safeSpeedDiag <= 0.0f) {
        atLog("ABORT: unable to validate even the lowest diagonal speed");
        return false;
    }
    if (g_tuneAbortReq) return false;

    portENTER_CRITICAL(&gMux);
    refYMin = g_yHardMin;
    refXMin = g_xHardMin;
    portEXIT_CRITICAL(&gMux);

    float safeSpeedAxis = phaseSpeedRampAxis(refYMin, refXMin, safeSpeedDiag, testAccelBase, currentIdx);
    if (g_tuneAbortReq) return false;

    // ---- Efficiency comparison: re-run speed ramp capped at one current level below ----
    // If the lower-current result still achieves >= 90% of the higher-current speed for
    // both axis and diagonal, it is more efficient (less heat, longer battery life).
    // In that case we accept the lower-current result as the winner.
    {
        const float speedDiagHigh  = safeSpeedDiag;
        const float speedAxisHigh  = safeSpeedAxis;
        const int   currentIdxHigh = currentIdx;

        if (currentIdxHigh > 0 && !g_tuneAbortReq) {
            const int capIdx = currentIdxHigh - 1;
            int currentIdxLow = 0;

            portENTER_CRITICAL(&gMux);
            refYMin = g_yHardMin;
            refXMin = g_xHardMin;
            portEXIT_CRITICAL(&gMux);

            atLog("=== Efficiency check: re-running speed ramp capped at %u mA ===",
                  (unsigned)currentForIndex(capIdx));

            setPhaseProgress(AT_PHASE_ACCEL_RAMP, 0);
            float speedDiagLow = phaseSpeedRampDiag(refYMin, refXMin, testAccelBase,
                                                     currentIdxLow, capIdx, AT_PHASE_ACCEL_RAMP);
            float speedAxisLow = speedDiagLow;

            if (speedDiagLow > 0.0f && !g_tuneAbortReq) {
                portENTER_CRITICAL(&gMux);
                refYMin = g_yHardMin;
                refXMin = g_xHardMin;
                portEXIT_CRITICAL(&gMux);
                speedAxisLow = phaseSpeedRampAxis(refYMin, refXMin, speedDiagLow,
                                                  testAccelBase, currentIdxLow, capIdx,
                                                  AT_PHASE_ACCEL_RAMP);
            }

            atLog("Efficiency: HIGH %u mA => diag=%.0f axis=%.0f",
                  (unsigned)currentForIndex(currentIdxHigh), speedDiagHigh, speedAxisHigh);
            atLog("Efficiency: LOW  %u mA => diag=%.0f axis=%.0f",
                  (unsigned)currentForIndex(capIdx), speedDiagLow, speedAxisLow);

            const float diagRatio = (speedDiagHigh > 0.0f) ? (speedDiagLow / speedDiagHigh) : 0.0f;
            const float axisRatio = (speedAxisHigh > 0.0f) ? (speedAxisLow / speedAxisHigh) : 0.0f;
            atLog("Efficiency ratios: diag=%.2f axis=%.2f (threshold 0.90)", diagRatio, axisRatio);

            if (speedDiagLow > 0.0f && diagRatio >= 0.90f && axisRatio >= 0.90f) {
                atLog("Efficiency: LOW current is >= 90%% as fast on both axes — using %u mA",
                      (unsigned)currentForIndex(capIdx));
                safeSpeedDiag = speedDiagLow;
                safeSpeedAxis = speedAxisLow;
                currentIdx    = currentIdxLow;
            } else {
                atLog("Efficiency: HIGH current wins (diag=%.2f axis=%.2f) — keeping %u mA",
                      diagRatio, axisRatio, (unsigned)currentForIndex(currentIdxHigh));
                safeSpeedDiag = speedDiagHigh;
                safeSpeedAxis = speedAxisHigh;
                currentIdx    = currentIdxHigh;
                portENTER_CRITICAL(&gMux);
                refYMin = g_yHardMin;
                refXMin = g_xHardMin;
                portEXIT_CRITICAL(&gMux);
            }
        }
    }
    if (g_tuneAbortReq) return false;

    // ---- Phase 4: Accel/Decel — not tuned; hardware ramp constants used at runtime ----
    // safeAccel/safeDecel stay 0.0f (no override) so MOTION_RAMP_UP_AB / MOTION_RAMP_DOWN_AB apply.
    float safeAccel = 0.0f;
    float safeDecel = 0.0f;
    setPhaseProgress(AT_PHASE_ACCEL_RAMP, 100);
    atLog("=== Phase: Accel/Decel — fixed hardware ramp constants (not tuned) ===");
    atLog("Ramp up=%.0f down=%.0f stop=%.0f steps/s² (from firmware constants)",
          (float)MOTION_RAMP_UP_AB, (float)MOTION_RAMP_DOWN_AB, (float)MOTION_RAMP_STOP_AB);
    atLog("Current at accel/decel skip: %u mA", (unsigned)currentForIndex(currentIdx));

    // ---- Phase 5: Current Sweep (step down from the adaptive current) ----
    uint16_t safeCurrent = phaseCurrentTune(refYMin, refXMin, currentIdx,
                                             safeSpeedAxis, safeSpeedDiag, safeAccel, safeDecel);
    if (g_tuneAbortReq) return false;

    portENTER_CRITICAL(&gMux);
    refYMin = g_yHardMin;
    refXMin = g_xHardMin;
    portEXIT_CRITICAL(&gMux);

    // ---- Phase 6: Final characterisation / stabilisation at tuned params ----
    bool stabOk = stabilizeFinalSettings(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag,
                                         safeAccel, safeDecel, safeCurrent);
    if (g_tuneAbortReq) return false;
    if (!stabOk) {
        atLog("WARNING: final stabilisation could not pass at any tested speed level");
        atLog("  Saving conservative floor settings (axis=%.0f diag=%.0f current=%u)",
              safeSpeedAxis, safeSpeedDiag, (unsigned)safeCurrent);
    }
    setPhaseProgress(AT_PHASE_APPROACH, 100);

    // ---- Apply and persist results ----

    g_tuneSettings.safeSpeed     = safeSpeedAxis;
    g_tuneSettings.safeSpeedDiag = safeSpeedDiag;
    g_tuneSettings.safeAccel     = 0.0f;  // not tuned; firmware ramp constants apply
    g_tuneSettings.safeDecel     = 0.0f;  // not tuned; firmware ramp constants apply
    g_tuneSettings.motorCurrent  = safeCurrent;
    g_tuneSettings.tuningValid   = true;
    g_tuneSettings.boundsValid   = true;

    g_overrideVmax     = safeSpeedAxis;
    g_overrideDiagVmax = safeSpeedDiag;
    // g_overrideAccel and g_overrideDecel left at 0 — hardware constants used
    setMotionProfileLock(true, AT_LOCKED_MICROSTEPS, safeCurrent);
    applyTuneCurrentMa(safeCurrent);
    saveSettings();

    atLog("=== AutoTune COMPLETE ===");
    atLog("  axisSpeed  = %.0f steps/s", safeSpeedAxis);
    atLog("  diagSpeed  = %.0f steps/s", safeSpeedDiag);
    atLog("  safeAccel  = %.0f steps/s²", safeAccel);
    atLog("  safeDecel  = %.0f steps/s²", safeDecel);
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
    atLog("AutoTune task started");
    g_systemState = SYS_TUNING;

    bool ok = runTuneSequence();

    if (!ok || g_tuneAbortReq) {
        atEmergencyStop("tune end");
        setPhaseProgress(g_tuneAbortReq ? AT_PHASE_ABORTED : AT_PHASE_ERROR, 0);
        g_systemState = g_tuneAbortReq ? SYS_READY : SYS_ERROR;
        atLog(g_tuneAbortReq ? "AutoTune ABORTED by user" : "AutoTune ERROR");
    } else {
        g_systemState = SYS_READY;
    }

    g_tuneActive       = false;
    g_tuneAbortReq     = false;
    g_tuneCurrentMa    = 0;
    g_tuneLiveAxisSpeed = 0.0f;
    g_tuneLiveDiagSpeed = 0.0f;
    g_tuneLiveAccel     = 0.0f;
    g_tuneLiveDecel     = 0.0f;
    setMotionProfileLock(g_tuneSettings.tuningValid, AT_LOCKED_MICROSTEPS,
                         g_tuneSettings.tuningValid ? g_tuneSettings.motorCurrent : 0);
    g_overrideVmax     = g_tuneSettings.tuningValid ? g_tuneSettings.safeSpeed     : 0.0f;
    g_overrideDiagVmax = g_tuneSettings.tuningValid ? g_tuneSettings.safeSpeedDiag : 0.0f;
    // g_overrideAccel / g_overrideDecel left at 0 — hardware ramp constants apply
    s_taskHandle    = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================
// Public API  (UNCHANGED)
// ============================================================

void autoTuneInit()  {
    loadSettings();
    autoTuneInitBanned();
}

bool autoTuneStart() {
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
    Serial.println("[AT] Started");
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
