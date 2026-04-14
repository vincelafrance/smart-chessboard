#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <INA3221.h>

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#include <ArduinoJson.h>
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

// =======================================================
// Shared lock
// =======================================================
extern portMUX_TYPE gMux;

// =======================================================
// HALL
// =======================================================
static const int HALL_Y_PIN = 34;
static const int HALL_X_PIN = 35;

extern volatile bool g_hallYDetected;
extern volatile bool g_hallXDetected;

extern int lastHallYState;
extern int lastHallXState;
extern unsigned long lastHallPollMs;

// =======================================================
// CALIBRATION
// =======================================================
static const long  Y_SPAN_STEPS   = 9300;
static const long  X_SPAN_STEPS   = 8525;

static const float CALIB_Y_SPEED  = 2000.0f;
static const float CALIB_X_SPEED  = 2000.0f;

static const int   CALIB_Y_DIR    = -1;
static const int   CALIB_X_DIR    = -1;

static const bool  HALL_AT_Y_MIN  = true;
static const bool  HALL_AT_X_MIN  = true;

enum CalibState : uint8_t {
	CALIB_IDLE = 0,
	CALIB_Y_BOTTOM = 1,
	CALIB_X_BOTTOM = 2,
	CALIB_Y_TOP = 3,
	CALIB_X_TOP = 4,
	CALIB_RECENTER = 5,
};

extern volatile CalibState g_calibState;

extern volatile bool g_yLimitsCalibrated;
extern volatile bool g_xLimitsCalibrated;

extern volatile long g_yHallPos;
extern volatile long g_xHallPos;

extern volatile long g_yHardMin;
extern volatile long g_yHardMax;
extern volatile long g_xHardMin;
extern volatile long g_xHardMax;

// =======================================================
// XY origin / center targets
// =======================================================
static const long XY_ORIGIN_X = 4000;
static const long XY_ORIGIN_Y = 4650;

extern volatile long g_xCenterTarget;
extern volatile long g_yCenterTarget;

// =======================================================
// BOARD GEOMETRY
// =======================================================
static const long ORIGIN_REF_X = 0;
static const long ORIGIN_REF_Y = 0;

static const long OFF_A1C_X = -3660;
static const long OFF_A1C_Y = -3860;

static const long OFF_H1C_X =  3710;
static const long OFF_H1C_Y = -3780;

static const long OFF_A8C_X = -3675;
static const long OFF_A8C_Y =  3475;

static const long OFF_H8C_X =  3695;
static const long OFF_H8C_Y =  3575;

static const long A1C_REF_X = ORIGIN_REF_X + OFF_A1C_X;
static const long A1C_REF_Y = ORIGIN_REF_Y + OFF_A1C_Y;
static const long H1C_REF_X = ORIGIN_REF_X + OFF_H1C_X;
static const long H1C_REF_Y = ORIGIN_REF_Y + OFF_H1C_Y;
static const long A8C_REF_X = ORIGIN_REF_X + OFF_A8C_X;
static const long A8C_REF_Y = ORIGIN_REF_Y + OFF_A8C_Y;
static const long H8C_REF_X = ORIGIN_REF_X + OFF_H8C_X;
static const long H8C_REF_Y = ORIGIN_REF_Y + OFF_H8C_Y;

extern volatile long g_A1C_X; extern volatile long g_A1C_Y;
extern volatile long g_H1C_X; extern volatile long g_H1C_Y;
extern volatile long g_A8C_X; extern volatile long g_A8C_Y;
extern volatile long g_H8C_X; extern volatile long g_H8C_Y;

// Last origin used by boardUpdateFromOrigin — used to compute delta on recalibration
// so advanced corner fine-tuning is preserved when normal calibration runs again.
extern volatile long g_lastOrigin_X;
extern volatile long g_lastOrigin_Y;

extern volatile long g_BO_A1_X; extern volatile long g_BO_A1_Y;
extern volatile long g_BO_H1_X; extern volatile long g_BO_H1_Y;
extern volatile long g_BO_A8_X; extern volatile long g_BO_A8_Y;
extern volatile long g_BO_H8_X; extern volatile long g_BO_H8_Y;

// =======================================================
// INA3221
// =======================================================
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

extern INA3221 ina;
static const ina3221_ch_t INA_CHANNEL = INA3221_CH1;

// =======================================================
// TMC2209 pins
// =======================================================
static const int TMC_UART_RX_PIN = 16; // ESP32 RX2
static const int TMC_UART_TX_PIN = 17; // ESP32 TX2

static const int LEFT_EN   = 13;
static const int LEFT_STEP = 14;
static const int LEFT_DIR  = 26;

static const int RIGHT_EN   = 27;
static const int RIGHT_STEP = 25;
static const int RIGHT_DIR  = 33;

// =======================================================
// Magnet PWM
// =======================================================
static const int MAGNET_PIN = 32;
static const int MAGNET_PWM_FREQ = 20000;
static const int MAGNET_PWM_RES  = 8;
static const uint8_t MAGNET_DUTY_100 = 255;

// =======================================================
// HARD LIMITS defaults
// =======================================================
static const long X_HARD_MIN_DEFAULT = -225;
static const long X_HARD_MAX_DEFAULT = 8300;

static const long Y_HARD_MIN_DEFAULT = 0;
static const long Y_HARD_MAX_DEFAULT = 9300;

// Edge slowdown
static const long  EDGE_SLOW_DIST  = 450;
static const long  EDGE_STOP_DIST  = 25;
static const float EDGE_MIN_SPEED  = 400.0f;

// CoreXY options
static const bool INVERT_X = true;
static const bool INVERT_Y = false;
static const bool SWAP_MOTORS_AB = false;
static const bool ENABLE_ACTIVE_LOW = true;

static const unsigned long CMD_TIMEOUT_MS = 600;

// Speed modes
enum SpeedMode : uint8_t { SPEED_SLOW = 0, SPEED_NORMAL = 1, SPEED_FAST = 2 };
static const float SPEED_XY_SLOW   = 2000.0f;
static const float SPEED_XY_NORMAL = 4500.0f;
static const float SPEED_XY_FAST   = 8000.0f;

// Recenter / Path
static const float RECENTER_KP = 2.0f;
static const float RECENTER_VMAX_XY = 7000.0f;
static const float RECENTER_MIN_VXY = 350.0f;
static const float RECENTER_X_CAP_SCALE = 0.55f;
static const float RECENTER_X_MIN_VXY = 220.0f;
static const long  RECENTER_DEADBAND_XY = 12;
static const unsigned long RECENTER_SETTLE_MS = 60;

// Faster piece movement profile (independent from recenter calibration behavior)
static const float PATH_VMAX_XY = 8600.0f;
static const float PATH_KP = 8.0f;
static const float PATH_MIN_VXY = 380.0f;
static const unsigned long PATH_PICKUP_SETTLE_MS = 20;

// Global motor slew limiter.
// Keep this very short so moves stay snappy while avoiding hard step jumps
// at launch, stop, and direction reversals.
static const float MOTION_RAMP_UP_AB   = 70000.0f;
static const float MOTION_RAMP_DOWN_AB = 450000.0f;
static const float MOTION_RAMP_STOP_AB = 900000.0f;

// Auto disable
static const bool DRIVERS_AUTO_DISABLE = true;
static const unsigned long IDLE_DISABLE_MS = 3000;

// Battery rest detect
static const float REST_CURRENT_A = 0.20;
static const unsigned long REST_TIME_MS = 8000;

// =======================================================
// WiFi + Web
// =======================================================
extern const char* WIFI_SSID;
extern const char* WIFI_PASS;

extern const char* AP_SSID;
extern const char* AP_PASS;

extern WebServer server;
extern WebSocketsServer webSocket;

// =======================================================
// Shared motion state
// =======================================================
extern volatile float g_vx_xy;
extern volatile float g_vy_xy;
extern volatile bool  g_recenter;
extern volatile SpeedMode g_speedMode;
extern volatile unsigned long g_lastCmdMs;

extern volatile bool g_driversEnabled;
extern volatile unsigned long g_lastMotionMs;

extern volatile long g_Apos;
extern volatile long g_Bpos;

extern volatile long g_xAbs;
extern volatile long g_yAbs;

extern volatile float g_battV;
extern volatile float g_battI;
extern volatile float g_battPct;

extern volatile bool g_magnetOn;

// =======================================================
// Waypoints / Path
// =======================================================
struct Waypoint { long x; long y; int8_t mag; /* -1=no change, 0=off, 1=on */ };

static const uint8_t MAX_WAYPOINTS = 12;

extern Waypoint g_waypoints[MAX_WAYPOINTS];
extern volatile uint8_t g_wpCount;
extern volatile uint8_t g_wpIndex;
extern volatile bool g_pathActive;
extern volatile long g_pathTargetX;
extern volatile long g_pathTargetY;

extern volatile bool g_autoMagnetPath;

static const long EDGE_MARGIN_STEPS = 120;

extern volatile bool g_autoCalibRequested;
extern volatile bool g_autoCalibStarted;

// =======================================================
// Dead zone physical endpoints
// Left  = whites captured (low X side)
// Right = blacks captured (high X side)
// Slot 15 = bas (first piece placed), slot 0 = haut (last)
// =======================================================
extern volatile long g_DZ_L_Bas_X,  g_DZ_L_Bas_Y;
extern volatile long g_DZ_L_Haut_X, g_DZ_L_Haut_Y;
extern volatile long g_DZ_R_Bas_X,  g_DZ_R_Bas_Y;
extern volatile long g_DZ_R_Haut_X, g_DZ_R_Haut_Y;
extern volatile bool g_DZ_L_Calibrated;   // true once both L endpoints saved
extern volatile bool g_DZ_R_Calibrated;

// Extended Y-axis soft limits (+50 steps each side) active only when:
//   g_dzCalibYExpanded — dead-zone calibration panel is open (set by gotoDeadZone,
//                         cleared by calibDeadZone or endDZCalib command)
//   g_dzPathYExpanded  — a deadZoneMove path is in flight (set by deadZoneMove,
//                         cleared by StepTask on path completion or abortPath)
extern volatile bool g_dzCalibYExpanded;
extern volatile bool g_dzPathYExpanded;
static const long DZ_Y_EXTRA = 100;

// =======================================================
// WebSocket deferred push (set by onWsEvent on connect, consumed by loop())
extern volatile bool g_wsPushPending;

// Deferred NVS calibration save.  Any code that mutates calibration globals
// sets this flag instead of calling saveCalibToNVS() directly.  loop() performs
// the actual (slow, blocking) write when the system is otherwise idle.
extern volatile bool     g_calibNvsDirty;
extern volatile unsigned long g_calibNvsDirtyMs;  // millis() when flag was last set

// Chess Test Run
// =======================================================
extern volatile bool    g_testRunActive;
extern volatile bool    g_testRunAbortReq;
extern volatile char    g_trStepName[64];  // current step label
extern volatile uint8_t g_trStepIdx;       // 0-based step index
extern volatile uint8_t g_trStepTotal;     // total step count

// =======================================================
// System state machine
// =======================================================
enum SystemState : uint8_t {
    SYS_UNHOMED    = 0,   // boot — no calibration yet
    SYS_CALIBRATING = 1,  // calibration sequence in progress
    SYS_HOMED      = 2,   // calibrated but tuning not yet run
    SYS_READY      = 3,   // calibrated (+ optionally tuned) — normal ops
    SYS_TUNING     = 4,   // AutoTune task running
    SYS_ERROR      = 5,   // unrecoverable fault
};
extern volatile SystemState g_systemState;

// =======================================================
// AutoTune phase
// =======================================================
enum TunePhase : uint8_t {
    AT_PHASE_IDLE         = 0,
    AT_PHASE_REFERENCE    = 1,
    AT_PHASE_REPEATABILITY= 2,
    AT_PHASE_SPEED_RAMP   = 3,
    AT_PHASE_ACCEL_RAMP   = 4,
    AT_PHASE_CURRENT      = 5,
    AT_PHASE_APPROACH     = 6,
    AT_PHASE_DONE         = 7,
    AT_PHASE_ABORTED      = 8,
    AT_PHASE_ERROR        = 9,
};
extern volatile TunePhase g_tunePhase;
extern volatile int       g_tuneProgress;   // 0-100
extern volatile bool      g_tuneActive;
extern volatile bool      g_tuneAbortReq;

// =======================================================
// Tuning settings (loaded from NVS at boot)
// =======================================================
struct TuneSettings {
    float    safeSpeed;      // validated safe axis speed (steps/s) — straight lines
    float    safeSpeedDiag;  // validated safe diagonal speed (steps/s) — 45° moves
    float    safeAccel;      // validated safe acceleration (steps/s²)
    float    safeDecel;      // validated safe deceleration (steps/s²)
    uint16_t motorCurrent;   // validated cruise current (mA)
    bool     tuningValid;    // true if a complete tune was saved
    bool     boundsValid;    // true if bounds were confirmed by tuning
};
extern TuneSettings g_tuneSettings;

// =======================================================
// Motion overrides (set by AutoTune; 0 = use firmware defaults)
// Checked by StepTask in both recenter and path sections.
// =======================================================
extern volatile float g_overrideVmax;     // axis speed override (steps/s)
extern volatile float g_overrideDiagVmax; // diagonal speed override (steps/s); 0 = same as g_overrideVmax
extern volatile float g_overrideAccel;    // acceleration override (steps/s²)
extern volatile float g_overrideDecel;    // deceleration override (steps/s²)
extern volatile uint16_t g_tuneCurrentMa; // current level presently under test by AutoTune
extern volatile float g_tuneLiveAxisSpeed; // Auto Tune axis speed presently being evaluated
extern volatile float g_tuneLiveDiagSpeed; // Auto Tune diagonal speed presently being evaluated
extern volatile float g_tuneLiveAccel;     // Auto Tune acceleration presently being evaluated
extern volatile float g_tuneLiveDecel;     // Auto Tune deceleration presently being evaluated

// =======================================================
// Drift log
// =======================================================
struct DriftRecord {
    float driftX;
    float driftY;
};
static const uint8_t MAX_DRIFT_LOG = 8;
extern DriftRecord g_driftLog[MAX_DRIFT_LOG];
extern volatile uint8_t g_driftCount;

// =======================================================
// AutoTune log ring-buffer (drained to WebSocket by autoTuneLoop)
// =======================================================
static const uint8_t AT_LOG_SLOTS   = 8;
static const uint8_t AT_LOG_MSG_LEN = 110;
struct AtLogEntry { char msg[AT_LOG_MSG_LEN]; };
extern AtLogEntry       g_atLogBuf[AT_LOG_SLOTS];
extern volatile uint8_t g_atLogWr;   // writer index (AutoTune task)
extern volatile uint8_t g_atLogRd;   // reader index (loop / autoTuneLoop)

// =======================================================
// Global init
// =======================================================
void initGlobals();
