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
static const float    AT_SPEEDS_DIAG[]      = {1800.f, 2500.f, 3200.f, 4000.f, 5000.f, 6000.f, 7000.f};
static const uint8_t  AT_N_SPEEDS_DIAG      = sizeof(AT_SPEEDS_DIAG)/sizeof(AT_SPEEDS_DIAG[0]);
// Speed ramp — axis-only tests (steps/s).
// Both motors share load more evenly on straight moves, so the carriage can
// usually run substantially faster than on 45° diagonals at the same current.
static const float    AT_SPEEDS_AXIS[]      = {4500.f, 6500.f, 8500.f, 10500.f, 12500.f, 14500.f};
static const uint8_t  AT_N_SPEEDS_AXIS      = sizeof(AT_SPEEDS_AXIS)/sizeof(AT_SPEEDS_AXIS[0]);

// Fixed acceleration used during speed-ramp test sessions (not tuned; hardware ramp constants apply at runtime).
static const float    AT_BASELINE_ACCEL  = 6000.0f;

// Current sweep (mA, low → high).
// Auto Tune now starts from the lowest current and only climbs if needed.
static const uint16_t AT_CURRENTS[]       = {950, 1050, 1150, 1300, 1500};
static const uint8_t  AT_N_CURRENTS       = sizeof(AT_CURRENTS)/sizeof(AT_CURRENTS[0]);

// Keep the normal AutoTune path focused on reaching a stable, usable result.
// These optional optimisation passes add a lot of runtime and can turn an
// otherwise good tune into a late-stage failure on small mechanical variance.
static const bool AT_ENABLE_EFFICIENCY_CHECK = false;
static const bool AT_ENABLE_SPEED_RECLAIM    = false;
static const bool AT_USE_SPREADCYCLE         = true;

// NVS keys
static const char NVS_NS[]         = "chess_tune";
static const char NVS_SPEED[]      = "safeSpeed";
static const char NVS_SPEED_DIAG[] = "safeSpdDiag";
static const char NVS_CALIB_SPEED[]= "calibSpd";
static const char NVS_CURRENT[]    = "motorMa";
static const char NVS_TUNE_OK[]    = "tuneValid";
static const char NVS_BOUNDS_OK[]  = "boundsValid";

static const float    NVS_DEF_SPEED      = 8600.0f;
static const float    NVS_DEF_SPEED_DIAG = 6000.0f;
static const float    NVS_DEF_CALIB_SPEED= 1200.0f;
static const uint16_t NVS_DEF_CURRENT    = 850;

// ============================================================
// Module state
// ============================================================
static TaskHandle_t s_taskHandle = nullptr;
static Preferences  s_prefs;
static bool         s_prevSpreadCycle = false;

static void applyTuneCurrentMa(uint16_t mA);
static void setTuneLiveSettings(float axisSpeed, float diagSpeed, float accel, float decel);
static float axisSpeedForDiagSpeed(float diagSpeed);
static bool guardedSeekHallMin(char axis, long &hitPos, float seekSpeed = 850.0f);

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
    s_prefs.putFloat(NVS_CALIB_SPEED,g_tuneSettings.calibSpeed);
    s_prefs.putUShort(NVS_CURRENT,   g_tuneSettings.motorCurrent);
    s_prefs.putBool(NVS_TUNE_OK,     g_tuneSettings.tuningValid);
    s_prefs.putBool(NVS_BOUNDS_OK,   g_tuneSettings.boundsValid);
    s_prefs.end();
    Serial.printf("[AT] Saved: axisSpd=%.0f diagSpd=%.0f calibSpd=%.0f current=%u\n",
                  g_tuneSettings.safeSpeed, g_tuneSettings.safeSpeedDiag,
                  g_tuneSettings.calibSpeed,
                  (unsigned)g_tuneSettings.motorCurrent);
}

void loadSettings() {
    s_prefs.begin(NVS_NS, true);
    g_tuneSettings.safeSpeed     = s_prefs.getFloat(NVS_SPEED,      NVS_DEF_SPEED);
    g_tuneSettings.safeSpeedDiag = s_prefs.getFloat(NVS_SPEED_DIAG, NVS_DEF_SPEED_DIAG);
    g_tuneSettings.calibSpeed    = s_prefs.getFloat(NVS_CALIB_SPEED,NVS_DEF_CALIB_SPEED);
    g_tuneSettings.motorCurrent  = s_prefs.getUShort(NVS_CURRENT,   NVS_DEF_CURRENT);
    g_tuneSettings.tuningValid   = s_prefs.getBool(NVS_TUNE_OK,     false);
    g_tuneSettings.boundsValid   = s_prefs.getBool(NVS_BOUNDS_OK,   false);
    s_prefs.end();

    // Accel/decel are NOT overridden from NVS — hardware ramp constants handle them.
    g_tuneSettings.safeAccel = 0.0f;
    g_tuneSettings.safeDecel = 0.0f;
    if (g_tuneSettings.calibSpeed < 600.0f) g_tuneSettings.calibSpeed = NVS_DEF_CALIB_SPEED;

    if (g_tuneSettings.tuningValid) {
        setDriversSpreadCycle(AT_USE_SPREADCYCLE);
        g_overrideVmax     = g_tuneSettings.safeSpeed;
        g_overrideDiagVmax = g_tuneSettings.safeSpeedDiag;
        g_calibYSpeed      = g_tuneSettings.calibSpeed;
        g_calibXSpeed      = g_tuneSettings.calibSpeed;
        setMotionProfileLock(true, AT_LOCKED_MICROSTEPS, g_tuneSettings.motorCurrent);
        Serial.printf("[AT] Loaded: axisSpd=%.0f diagSpd=%.0f calibSpd=%.0f current=%u (VALID)\n",
                      g_tuneSettings.safeSpeed, g_tuneSettings.safeSpeedDiag,
                      g_tuneSettings.calibSpeed,
                      (unsigned)g_tuneSettings.motorCurrent);
    } else {
        g_calibYSpeed = CALIB_Y_SPEED;
        g_calibXSpeed = CALIB_X_SPEED;
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
    g_pathSpeedScale = 1.0f;
    g_tuneRefSeekActive = false;
    g_tuneRefSeekAxis   = 0;
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
    g_liveLimitFault = false;
    g_liveLimitAxis  = 0;
    g_liveLimitDir   = 0;
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
        bool liveLimit;
        char liveAxis;
        int8_t liveDir;
        portENTER_CRITICAL(&gMux);
        r = g_recenter;
        liveLimit = g_liveLimitFault;
        liveAxis = g_liveLimitAxis;
        liveDir = g_liveLimitDir;
        portEXIT_CRITICAL(&gMux);
        if (liveLimit) {
            long px, py, tx, ty;
            int hx = digitalRead(HALL_X_PIN);
            int hy = digitalRead(HALL_Y_PIN);
            portENTER_CRITICAL(&gMux);
            px = g_xAbs;
            py = g_yAbs;
            tx = g_xCenterTarget;
            ty = g_yCenterTarget;
            portEXIT_CRITICAL(&gMux);
            atLog("tuneMoveTo: live %c-%s limit hit pos=(%ld,%ld) target=(%ld,%ld) hallX=%s hallY=%s",
                  liveAxis,
                  liveDir < 0 ? "MIN" : "MAX",
                  px, py,
                  tx, ty,
                  hx == LOW ? "LOW" : "HIGH",
                  hy == LOW ? "LOW" : "HIGH");
            return AR_TIMEOUT;
        }
        if (!r) return AR_OK;
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    portENTER_CRITICAL(&gMux);
    long px = g_xAbs;
    long py = g_yAbs;
    long tx = g_xCenterTarget;
    long ty = g_yCenterTarget;
    g_recenter = false;
    g_vx_xy = 0.0f; g_vy_xy = 0.0f;
    portEXIT_CRITICAL(&gMux);
    atLog("tuneMoveTo: TIMEOUT pos=(%ld,%ld) target=(%ld,%ld) hallX=%s hallY=%s",
          px, py,
          tx, ty,
          digitalRead(HALL_X_PIN) == LOW ? "LOW" : "HIGH",
          digitalRead(HALL_Y_PIN) == LOW ? "LOW" : "HIGH");
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
    g_liveLimitFault = false;
    g_liveLimitAxis  = 0;
    g_liveLimitDir   = 0;
    g_waypoints[0]   = { x, y, -1 };
    g_wpCount        = 1;
    g_wpIndex        = 0;
    g_pathTargetX    = x;
    g_pathTargetY    = y;
    g_pathActive     = true;
    g_recenter       = false;
    g_vx_xy          = 0.0f;
    g_vy_xy          = 0.0f;
    g_pathSpeedScale = 1.0f;
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
            g_pathSpeedScale = 1.0f;
            portEXIT_CRITICAL(&gMux);
            return AR_ABORT;
        }
        bool active;
        bool liveLimit;
        char liveAxis;
        int8_t liveDir;
        portENTER_CRITICAL(&gMux);
        active = g_pathActive;
        liveLimit = g_liveLimitFault;
        liveAxis = g_liveLimitAxis;
        liveDir = g_liveLimitDir;
        portEXIT_CRITICAL(&gMux);
        if (liveLimit) {
            long px, py, tx, ty;
            int hx = digitalRead(HALL_X_PIN);
            int hy = digitalRead(HALL_Y_PIN);
            portENTER_CRITICAL(&gMux);
            px = g_xAbs;
            py = g_yAbs;
            tx = g_pathTargetX;
            ty = g_pathTargetY;
            g_pathActive     = false;
            g_wpCount        = 0;
            g_wpIndex        = 0;
            g_autoMagnetPath = false;
            g_pathSpeedScale = 1.0f;
            portEXIT_CRITICAL(&gMux);
            atLog("tuneMoveToPath: live %c-%s limit hit pos=(%ld,%ld) target=(%ld,%ld) hallX=%s hallY=%s",
                  liveAxis,
                  liveDir < 0 ? "MIN" : "MAX",
                  px, py,
                  tx, ty,
                  hx == LOW ? "LOW" : "HIGH",
                  hy == LOW ? "LOW" : "HIGH");
            return AR_TIMEOUT;
        }
        if (!active) return AR_OK;
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    portENTER_CRITICAL(&gMux);
    long px = g_xAbs;
    long py = g_yAbs;
    long tx = g_pathTargetX;
    long ty = g_pathTargetY;
    g_pathActive     = false;
    g_wpCount        = 0;
    g_wpIndex        = 0;
    g_autoMagnetPath = false;
    g_pathSpeedScale = 1.0f;
    portEXIT_CRITICAL(&gMux);
    atLog("tuneMoveToPath: TIMEOUT pos=(%ld,%ld) target=(%ld,%ld) hallX=%s hallY=%s",
          px, py,
          tx, ty,
          digitalRead(HALL_X_PIN) == LOW ? "LOW" : "HIGH",
          digitalRead(HALL_Y_PIN) == LOW ? "LOW" : "HIGH");
    return AR_TIMEOUT;
}

// ============================================================
// REFERENCE LAYER  (UNCHANGED)
// ============================================================
static bool referenceHomeCorner(long &yMin, long &xMin) {
    atLog("referenceHomeCorner: guarded Hall probe...");
    long hitY, hitX;
    if (!guardedSeekHallMin('Y', hitY)) {
        atLog("referenceHomeCorner: Y probe FAILED");
        return false;
    }
    vTaskDelay(80 / portTICK_PERIOD_MS);
    if (!guardedSeekHallMin('X', hitX)) {
        atLog("referenceHomeCorner: X probe FAILED");
        return false;
    }

    long oldYMin, oldYMax, oldXMin, oldXMax;
    portENTER_CRITICAL(&gMux);
    oldYMin = g_yHardMin;
    oldYMax = g_yHardMax;
    oldXMin = g_xHardMin;
    oldXMax = g_xHardMax;
    portEXIT_CRITICAL(&gMux);

    long spanY = labs(oldYMax - oldYMin);
    long spanX = labs(oldXMax - oldXMin);
    if (spanY < 1000L) spanY = labs(Y_HARD_MAX_DEFAULT - Y_HARD_MIN_DEFAULT);
    if (spanX < 1000L) spanX = labs(X_HARD_MAX_DEFAULT - X_HARD_MIN_DEFAULT);

    long newYMin = HALL_AT_Y_MIN ? hitY : hitY - spanY;
    long newYMax = HALL_AT_Y_MIN ? hitY + spanY : hitY;
    long newXMin = HALL_AT_X_MIN ? hitX : hitX - spanX;
    long newXMax = HALL_AT_X_MIN ? hitX + spanX : hitX;
    if (newYMin > newYMax) { long t = newYMin; newYMin = newYMax; newYMax = t; }
    if (newXMin > newXMax) { long t = newXMin; newXMin = newXMax; newXMax = t; }

    const long newYCenter = (newYMin + newYMax) / 2L;
    const long newXCenter = (newXMin + newXMax) / 2L;

    portENTER_CRITICAL(&gMux);
    g_yHardMin = newYMin;
    g_yHardMax = newYMax;
    g_xHardMin = newXMin;
    g_xHardMax = newXMax;
    g_yCenterTarget = newYCenter;
    g_xCenterTarget = newXCenter;
    g_yLimitsCalibrated = true;
    g_xLimitsCalibrated = true;
    yMin = g_yHardMin;
    xMin = g_xHardMin;
    portEXIT_CRITICAL(&gMux);

    setRawABfromXY(xMin, yMin);
    atLog("homeCorner: y=%ld x=%ld (hit y=%ld x=%ld spanY=%ld spanX=%ld center y=%ld x=%ld)",
          yMin, xMin, hitY, hitX, spanY, spanX, newYCenter, newXCenter);
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
// AutoTune drift checks cannot trust the current step position if missed steps
// are exactly what we are trying to detect.  So the reference check uses a
// guarded low-speed Hall probe: seek Y-min until Hall, stop, then seek X-min
// until Hall.  The step counter is only used as a maximum-travel failsafe.
// ============================================================

static const float         AT_REF_SEEK_SPEED         = 850.0f;
static const long          AT_REF_MAX_EXTRA_STEPS    = 1200L;

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

static void stopTuneVelocity() {
    portENTER_CRITICAL(&gMux);
    g_vx_xy      = 0.0f;
    g_vy_xy      = 0.0f;
    g_recenter   = false;
    g_pathActive = false;
    g_wpCount    = 0;
    g_wpIndex    = 0;
    g_pathSpeedScale = 1.0f;
    g_tuneRefSeekActive = false;
    g_tuneRefSeekAxis   = 0;
    g_lastCmdMs  = millis();
    portEXIT_CRITICAL(&gMux);
}

static bool guardedSeekHallMin(char axis, long &hitPos, float seekSpeed) {
    seekSpeed = clampf(seekSpeed, 400.0f, 6000.0f);
    long xMin, xMax, yMin, yMax;
    getXLimits(xMin, xMax);
    getYLimits(yMin, yMax);

    const float seekDir = (axis == 'Y')
                        ? (HALL_AT_Y_MIN ? (float)CALIB_Y_DIR : -(float)CALIB_Y_DIR)
                        : (HALL_AT_X_MIN ? (float)CALIB_X_DIR : -(float)CALIB_X_DIR);

    const int hallPin = (axis == 'Y') ? HALL_Y_PIN : HALL_X_PIN;
    const int hallAtMin = (axis == 'Y') ? HALL_AT_Y_MIN : HALL_AT_X_MIN;
    const long minLimit = (axis == 'Y') ? yMin : xMin;
    const long maxLimit = (axis == 'Y') ? yMax : xMax;

    long startPos;
    long startX;
    long startY;
    int hallStartRaw;
    portENTER_CRITICAL(&gMux);
    startX = g_xAbs;
    startY = g_yAbs;
    startPos = (axis == 'Y') ? startY : startX;
    g_liveLimitFault = false;
    g_liveLimitAxis  = 0;
    g_liveLimitDir   = 0;
    g_recenter       = false;
    g_pathActive     = false;
    g_wpCount        = 0;
    g_wpIndex        = 0;
    g_autoMagnetPath = false;
    g_pathSpeedScale = 1.0f;
    g_tuneRefSeekActive = true;
    g_tuneRefSeekAxis   = axis;
    g_vx_xy          = (axis == 'X') ? seekDir * seekSpeed : 0.0f;
    g_vy_xy          = (axis == 'Y') ? seekDir * seekSpeed : 0.0f;
    g_lastCmdMs      = millis();
    portEXIT_CRITICAL(&gMux);
    hallStartRaw = digitalRead(hallPin);

    const long span = (axis == 'Y') ? labs(yMax - yMin) : labs(xMax - xMin);
    const long maxTravel = span + AT_REF_MAX_EXTRA_STEPS;
    const unsigned long seekTimeoutMs =
        (unsigned long)(((float)maxTravel / seekSpeed) * 1000.0f) + 8000UL;
    const unsigned long startMs = millis();
    unsigned long lastProgressLogMs = startMs;
    unsigned long lastNoMoveLogMs = startMs;
    long lastLogPos = startPos;

    atLog("guardedRef: %c-min start pos=%ld xy=(%ld,%ld) limits=[%ld..%ld] span=%ld maxTravel=%ld dir=%+.0f speed=%.0f hallPin=%d hallStart=%s hallAtMin=%d timeout=%lu",
          axis,
          startPos,
          startX,
          startY,
          minLimit,
          maxLimit,
          span,
          maxTravel,
          seekDir,
          seekSpeed,
          hallPin,
          hallStartRaw == LOW ? "LOW" : "HIGH",
          hallAtMin,
          seekTimeoutMs);

    while (millis() - startMs < seekTimeoutMs) {
        if (g_tuneAbortReq) {
            stopTuneVelocity();
            atLog("guardedRef: %c-min aborted by request", axis);
            return false;
        }

        const bool hallLow = (axis == 'Y')
                           ? (digitalRead(HALL_Y_PIN) == LOW)
                           : (digitalRead(HALL_X_PIN) == LOW);

        long posNow;
        long xNow;
        long yNow;
        float vxNow;
        float vyNow;
        bool liveLimit;
        char liveAxis;
        int8_t liveDir;
        portENTER_CRITICAL(&gMux);
        xNow = g_xAbs;
        yNow = g_yAbs;
        posNow = (axis == 'Y') ? yNow : xNow;
        vxNow = g_vx_xy;
        vyNow = g_vy_xy;
        liveLimit = g_liveLimitFault;
        liveAxis = g_liveLimitAxis;
        liveDir = g_liveLimitDir;
        g_lastCmdMs = millis();
        portEXIT_CRITICAL(&gMux);

        if (hallLow || liveLimit) {
            stopTuneVelocity();
            vTaskDelay(40 / portTICK_PERIOD_MS);
            portENTER_CRITICAL(&gMux);
            hitPos = (axis == 'Y') ? g_yAbs : g_xAbs;
            if (axis == 'Y') g_yHallPos = hitPos;
            else             g_xHallPos = hitPos;
            portEXIT_CRITICAL(&gMux);
            atLog("guardedRef: %c-min stop by %s at %ld travel=%ld live=%d liveAxis=%c liveDir=%d",
                  axis,
                  hallLow ? "Hall" : "liveLimit",
                  hitPos,
                  labs(hitPos - startPos),
                  liveLimit ? 1 : 0,
                  liveAxis ? liveAxis : '-',
                  (int)liveDir);
            return true;
        }

        if (labs(posNow - startPos) > maxTravel) {
            stopTuneVelocity();
            atLog("guardedRef: %c-min seek exceeded max travel: pos=%ld start=%ld travel=%ld maxTravel=%ld hall=%s vx=%.1f vy=%.1f",
                  axis,
                  posNow,
                  startPos,
                  labs(posNow - startPos),
                  maxTravel,
                  hallLow ? "LOW" : "HIGH",
                  vxNow,
                  vyNow);
            return false;
        }

        const unsigned long nowMs = millis();
        if (nowMs - lastProgressLogMs >= 1000UL) {
            atLog("guardedRef: %c-min progress t=%lu pos=%ld travel=%ld deltaLast=%ld xy=(%ld,%ld) hall=%s vx=%.1f vy=%.1f live=%d",
                  axis,
                  nowMs - startMs,
                  posNow,
                  posNow - startPos,
                  posNow - lastLogPos,
                  xNow,
                  yNow,
                  hallLow ? "LOW" : "HIGH",
                  vxNow,
                  vyNow,
                  liveLimit ? 1 : 0);
            lastProgressLogMs = nowMs;
            lastLogPos = posNow;
        }

        if ((nowMs - startMs >= 1500UL) && (labs(posNow - startPos) < 50L) &&
            (nowMs - lastNoMoveLogMs >= 1500UL)) {
            atLog("guardedRef: %c-min WARNING little/no movement: pos=%ld start=%ld travel=%ld commanded vx=%.1f vy=%.1f hall=%s",
                  axis,
                  posNow,
                  startPos,
                  labs(posNow - startPos),
                  vxNow,
                  vyNow,
                  hallLow ? "LOW" : "HIGH");
            lastNoMoveLogMs = nowMs;
        }

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    stopTuneVelocity();
    long endPos;
    long endX;
    long endY;
    bool endLive;
    char endLiveAxis;
    int8_t endLiveDir;
    portENTER_CRITICAL(&gMux);
    endX = g_xAbs;
    endY = g_yAbs;
    endPos = (axis == 'Y') ? endY : endX;
    endLive = g_liveLimitFault;
    endLiveAxis = g_liveLimitAxis;
    endLiveDir = g_liveLimitDir;
    portEXIT_CRITICAL(&gMux);
    const int hallEndRaw = digitalRead(hallPin);
    atLog("guardedRef: %c-min seek TIMEOUT after %lu ms: start=%ld end=%ld travel=%ld xy=(%ld,%ld) hallEnd=%s live=%d liveAxis=%c liveDir=%d",
          axis,
          seekTimeoutMs,
          startPos,
          endPos,
          endPos - startPos,
          endX,
          endY,
          hallEndRaw == LOW ? "LOW" : "HIGH",
          endLive ? 1 : 0,
          endLiveAxis ? endLiveAxis : '-',
          (int)endLiveDir);
    return false;
}

static FastRefResult fastHomeCornerCheck(long refYMin, long refXMin,
                                          DriftMeasure &out) {
    long cx, cy;
    portENTER_CRITICAL(&gMux);
    cx = g_xCenterTarget;
    cy = g_yCenterTarget;
    portEXIT_CRITICAL(&gMux);

    long hitY, hitX;
    if (!guardedSeekHallMin('Y', hitY)) {
        tuneMoveTo(cx, cy);
        return g_tuneAbortReq ? FR_ABORT : FR_TIMEOUT;
    }
    vTaskDelay(80 / portTICK_PERIOD_MS);

    if (!guardedSeekHallMin('X', hitX)) {
        tuneMoveTo(cx, cy);
        return g_tuneAbortReq ? FR_ABORT : FR_TIMEOUT;
    }
    vTaskDelay(80 / portTICK_PERIOD_MS);

    // ── Compute drift vs phase-start reference ──
    out.actualY   = hitY;
    out.actualX   = hitX;
    out.driftY    = (float)(hitY - refYMin);
    out.driftX    = (float)(hitX - refXMin);
    out.magnitude = sqrtf(out.driftX * out.driftX + out.driftY * out.driftY);
    out.exceeded  = (out.magnitude > AT_MAX_DRIFT_STEPS);

    // The Hall hit tells us the physical carriage is at the calibrated home
    // corner.  After measuring drift, snap the logical step counters back to
    // that known physical reference so the next return-to-centre and test do
    // not inherit the drifted coordinate frame.
    setRawABfromXY(refXMin, refYMin);

    if (g_driftCount < MAX_DRIFT_LOG) {
        g_driftLog[g_driftCount].driftX = out.driftX;
        g_driftLog[g_driftCount].driftY = out.driftY;
        g_driftCount++;
    }

    atLog("fastRef: Y=%ld(ref%ld dY=%.1f) X=%ld(ref%ld dX=%.1f) |%.1f| %s",
          hitY, refYMin, out.driftY,
          hitX, refXMin, out.driftX,
          out.magnitude, out.exceeded ? "FAIL" : "OK");

    // ── Return carriage to board centre ──
    // g_overrideVmax is still set by the caller (session speed); the
    // return move uses it for full-speed travel back.
    if (tuneMoveTo(cx, cy) == AR_ABORT) return FR_ABORT;
    vTaskDelay(80 / portTICK_PERIOD_MS);

    return FR_OK;
}

static bool recoverReferenceAfterDrift(long &newYMin, long &newXMin) {
    atLog("recoverReferenceAfterDrift: stopping and guarded re-reference...");
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

    long sx, sy;
    portENTER_CRITICAL(&gMux);
    sx = g_xAbs;
    sy = g_yAbs;
    portEXIT_CRITICAL(&gMux);
    const long centerTravel = max(labs(cx - sx), labs(cy - sy));
    float centerSpeed = sessionAxisSpeed;
    if (centerSpeed < 600.0f) centerSpeed = 600.0f;
    unsigned long centerTimeoutMs =
        (unsigned long)(((float)centerTravel / centerSpeed) * 3000.0f) + 9000UL;
    if (centerTimeoutMs < 18000UL) centerTimeoutMs = 18000UL;
    if (centerTimeoutMs > 45000UL) centerTimeoutMs = 45000UL;

    atLog("Session: center start from (%ld,%ld) to (%ld,%ld), travel=%ld timeout=%lu",
          sx, sy, cx, cy, centerTravel, centerTimeoutMs);
    if (tuneMoveToPath(cx, cy, centerTimeoutMs) != AR_OK) {
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

        atLog("[%d/%d] %s  axis=%.0f diag=%.0f scale=%.2f  rep=%u%s",
              (int)ti + 1, (int)AT_NUM_TESTS,
              t.name, sessionAxisSpeed, sessionDiagSpeed, t.speedScale,
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
    const uint16_t adaptiveCurrent = currentForIndex(startCurrentIdx);
    setPhaseProgress(AT_PHASE_CURRENT, 0);
    atLog("=== Phase: Current Validation (fast) ===");
    atLog("Current params: axisSpeed=%.0f diagSpeed=%.0f accel=%.0f decel=%.0f score>=%.2f",
          safeSpeedAxis, safeSpeedDiag, safeAccel, safeDecel, AT_SCORE_PASS_THRESHOLD);
    atLog("Validating adaptive current first: %u mA", (unsigned)adaptiveCurrent);

    uint16_t bestCurrent = adaptiveCurrent;
    float    bestScore   = 0.0f;

    auto validateCurrent = [&](int ci, const char *label, SessionStats &s, float &score) -> bool {
        uint16_t mA = currentForIndex(ci);
        applyTuneCurrentMa(mA);
        g_overrideDecel = safeDecel;
        setTuneLiveSettings(safeSpeedAxis, safeSpeedDiag, safeAccel, safeDecel);
        atLog("%s: %u mA", label, (unsigned)mA);
        vTaskDelay(250 / portTICK_PERIOD_MS);  // let drivers settle

        if (!runTrajectorySession(/*quick=*/true, /*axisOnly=*/false,
                                   safeSpeedAxis, safeSpeedDiag, safeAccel, refYMin, refXMin, s)) {
            g_overrideDecel = 0.0f;
            return false;
        }

        score = computeLevelScore(s);
        bool hardFail = (s.timeouts > 0) || (!s.driftOk) || (s.driftExceeded > 0);
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
        }
        return levelOk;
    };

    SessionStats s;
    float score = 0.0f;
    if (!validateCurrent(startCurrentIdx, "Adaptive current validation", s, score)) {
        g_overrideDecel = 0.0f;
        atLog("Adaptive current did not revalidate cleanly; keeping %u mA with final stabilisation fallback",
              (unsigned)adaptiveCurrent);
        setPhaseProgress(AT_PHASE_CURRENT, 100);
        return adaptiveCurrent;
    }

    bestScore = score;
    setPhaseProgress(AT_PHASE_CURRENT, 70);

    // Efficiency trim: only try one step lower.  Older code swept from the
    // lowest current upward, which was slow and often caused avoidable
    // recovery cycles before reaching the already-known working current.
    if (startCurrentIdx > 0 && !g_tuneAbortReq) {
        int lowerIdx = startCurrentIdx - 1;
        SessionStats lowStats;
        float lowScore = 0.0f;
        if (validateCurrent(lowerIdx, "One-step lower current check", lowStats, lowScore)) {
            bestCurrent = currentForIndex(lowerIdx);
            bestScore   = lowScore;
            atLog("Current trim accepted: %u mA", (unsigned)bestCurrent);
        } else {
            atLog("Current trim rejected; keeping adaptive current %u mA",
                  (unsigned)adaptiveCurrent);
        }
    }

    g_overrideDecel = 0.0f;
    setPhaseProgress(AT_PHASE_CURRENT, 100);
    atLog("Current validation done: safe=%u mA (score=%.2f)", (unsigned)bestCurrent, bestScore);
    return bestCurrent;
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
        if (AT_ENABLE_SPEED_RECLAIM) {
            reclaimFinalSpeeds(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag, safeAccel,
                               safeDecel, safeCurrent);
        } else {
            atLog("Speed reclaim skipped for fast/seamless tune.");
        }
        return true;
    }

    // Step 2: Bump to max current and retry
    if (safeCurrent < maxSafeCurrent) {
        atLog("Final validation failed at %u mA -> retrying at max safe current %u mA",
              (unsigned)safeCurrent, (unsigned)maxSafeCurrent);
        safeCurrent = maxSafeCurrent;
        if (phaseFinalCharacterize(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag, safeAccel,
                                   safeDecel, safeCurrent)) {
            if (AT_ENABLE_SPEED_RECLAIM) {
                reclaimFinalSpeeds(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag, safeAccel,
                                   safeDecel, safeCurrent);
            } else {
                atLog("Speed reclaim skipped for fast/seamless tune.");
            }
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
            if (AT_ENABLE_SPEED_RECLAIM) {
                reclaimFinalSpeeds(refYMin, refXMin, safeSpeedAxis, safeSpeedDiag, safeAccel,
                                   safeDecel, safeCurrent);
            } else {
                atLog("Speed reclaim skipped for fast/seamless tune.");
            }
            return true;
        }
    }

    // Nothing passed even at 50% — caller will save the 50% floor as a last resort
    atLog("Stabilisation: no stable configuration found down to 50%% of tuned speed");
    return false;
}

static float phaseCalibrationSpeedTune(long refYMin, long refXMin, float safeSpeedAxis) {
    static const float CANDIDATES[] = {1200.f, 1800.f, 2400.f, 3000.f, 3600.f, 4200.f};
    static const uint8_t N_CANDIDATES = sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);
    const float cap = clampf(safeSpeedAxis * 0.45f, 1200.0f, 4200.0f);
    const float driftLimit = fminf(AT_MAX_DRIFT_STEPS * 0.60f, 30.0f);

    atLog("=== Phase: Calibration Speed Tune ===");
    atLog("Calibration seek candidates capped at %.0f steps/s, drift limit %.1f steps",
          cap, driftLimit);

    float lastGood = 1200.0f;
    float lastDrift = 0.0f;

    for (uint8_t i = 0; i < N_CANDIDATES && !g_tuneAbortReq; i++) {
        const float spd = CANDIDATES[i];
        if (spd > cap + 1.0f) break;

        bool ok = true;
        float maxDrift = 0.0f;

        for (uint8_t rep = 0; rep < 2 && ok && !g_tuneAbortReq; rep++) {
            long hitY, hitX;
            if (!guardedSeekHallMin('Y', hitY, spd)) {
                ok = false;
                break;
            }
            vTaskDelay(50 / portTICK_PERIOD_MS);
            if (!guardedSeekHallMin('X', hitX, spd)) {
                ok = false;
                break;
            }

            const float dY = (float)(hitY - refYMin);
            const float dX = (float)(hitX - refXMin);
            const float mag = sqrtf(dX * dX + dY * dY);
            if (mag > maxDrift) maxDrift = mag;

            setRawABfromXY(refXMin, refYMin);

            long cx, cy;
            portENTER_CRITICAL(&gMux);
            cx = g_xCenterTarget;
            cy = g_yCenterTarget;
            portEXIT_CRITICAL(&gMux);
            tuneMoveToPath(cx, cy, 18000UL);

            atLog("Calib speed %.0f rep %u: Y=%ld dY=%.1f X=%ld dX=%.1f |%.1f|",
                  spd, (unsigned)(rep + 1), hitY, dY, hitX, dX, mag);

            if (mag > driftLimit) ok = false;
            vTaskDelay(60 / portTICK_PERIOD_MS);
        }

        atLog("Calibration speed %.0f: %s maxDrift=%.1f",
              spd, ok ? "PASS" : "FAIL", maxDrift);

        if (!ok) break;
        lastGood = spd;
        lastDrift = maxDrift;
    }

    float safe = lastGood * 0.90f;
    if (safe < 1200.0f) safe = 1200.0f;
    atLog("Calibration speed done: lastGood=%.0f maxDrift=%.1f safe=%.0f",
          lastGood, lastDrift, safe);
    return safe;
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
    s_prevSpreadCycle = getDriversSpreadCycle();
    setDriversSpreadCycle(AT_USE_SPREADCYCLE);
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

        if (AT_ENABLE_EFFICIENCY_CHECK && currentIdxHigh > 0 && !g_tuneAbortReq) {
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
        } else if (!AT_ENABLE_EFFICIENCY_CHECK) {
            atLog("Efficiency re-run skipped for fast/seamless tune.");
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

    float safeCalibSpeed = phaseCalibrationSpeedTune(refYMin, refXMin, safeSpeedAxis);
    if (g_tuneAbortReq) return false;
    setPhaseProgress(AT_PHASE_APPROACH, 100);

    // ---- Apply and persist results ----

    g_tuneSettings.safeSpeed     = safeSpeedAxis;
    g_tuneSettings.safeSpeedDiag = safeSpeedDiag;
    g_tuneSettings.calibSpeed    = safeCalibSpeed;
    g_tuneSettings.safeAccel     = 0.0f;  // not tuned; firmware ramp constants apply
    g_tuneSettings.safeDecel     = 0.0f;  // not tuned; firmware ramp constants apply
    g_tuneSettings.motorCurrent  = safeCurrent;
    g_tuneSettings.tuningValid   = true;
    g_tuneSettings.boundsValid   = true;

    g_overrideVmax     = safeSpeedAxis;
    g_overrideDiagVmax = safeSpeedDiag;
    g_calibYSpeed      = safeCalibSpeed;
    g_calibXSpeed      = safeCalibSpeed;
    // g_overrideAccel and g_overrideDecel left at 0 — hardware constants used
    setMotionProfileLock(true, AT_LOCKED_MICROSTEPS, safeCurrent);
    applyTuneCurrentMa(safeCurrent);
    saveSettings();

    atLog("=== AutoTune COMPLETE ===");
    atLog("  axisSpeed  = %.0f steps/s", safeSpeedAxis);
    atLog("  diagSpeed  = %.0f steps/s", safeSpeedDiag);
    atLog("  calibSpeed = %.0f steps/s", safeCalibSpeed);
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
    atLog("AutoTune build: guarded-ref v2026-07-04j");
    g_systemState = SYS_TUNING;

    bool ok = runTuneSequence();

    if (!ok || g_tuneAbortReq) {
        atEmergencyStop("tune end");
        setDriversSpreadCycle(s_prevSpreadCycle);
        setPhaseProgress(g_tuneAbortReq ? AT_PHASE_ABORTED : AT_PHASE_ERROR, 0);
        g_systemState = g_tuneAbortReq ? SYS_READY : SYS_ERROR;
        atLog(g_tuneAbortReq ? "AutoTune ABORTED by user" : "AutoTune ERROR");
    } else {
        setDriversSpreadCycle(AT_USE_SPREADCYCLE);
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
        atLog("Cannot start: AutoTune is already running.");
        return false;
    }
    bool calOk;
    CalibState cs;
    SystemState sys;
    portENTER_CRITICAL(&gMux);
    calOk = g_yLimitsCalibrated && g_xLimitsCalibrated;
    cs = g_calibState;
    sys = g_systemState;
    portEXIT_CRITICAL(&gMux);
    if (!calOk) {
        Serial.println("[AT] Cannot start: no valid calibration.");
        atLog("Cannot start: no valid calibration. Run board calibration first.");
        return false;
    }
    if (cs != CALIB_IDLE) {
        Serial.println("[AT] Cannot start: calibration in progress.");
        atLog("Cannot start: calibration is still active (state=%u).", (unsigned)cs);
        return false;
    }

    if (sys == SYS_ERROR) {
        portENTER_CRITICAL(&gMux);
        g_systemState = SYS_READY;
        portEXIT_CRITICAL(&gMux);
        atLog("Previous error cleared; starting AutoTune.");
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
