#include "Globals.h"

// lock
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

// Hall
volatile bool g_hallYDetected = false;
volatile bool g_hallXDetected = false;

int lastHallYState = HIGH;
int lastHallXState = HIGH;
unsigned long lastHallPollMs = 0;

// Calib
volatile CalibState g_calibState = CALIB_IDLE;

volatile bool g_yLimitsCalibrated = false;
volatile bool g_xLimitsCalibrated = false;

volatile long g_yHallPos = 0;
volatile long g_xHallPos = 0;

volatile long g_yHardMin = 0;
volatile long g_yHardMax = 0;
volatile long g_xHardMin = 0;
volatile long g_xHardMax = 0;

// Center targets
volatile long g_xCenterTarget = XY_ORIGIN_X;
volatile long g_yCenterTarget = XY_ORIGIN_Y;

// Board mapping globals
volatile long g_A1C_X = A1C_REF_X, g_A1C_Y = A1C_REF_Y;
volatile long g_H1C_X = H1C_REF_X, g_H1C_Y = H1C_REF_Y;
volatile long g_A8C_X = A8C_REF_X, g_A8C_Y = A8C_REF_Y;
volatile long g_H8C_X = H8C_REF_X, g_H8C_Y = H8C_REF_Y;

// ORIGIN_REF_X/Y = 0, so initial corners are already at OFF_xxx values.
// Initialise to 0 so the first boardUpdateFromOrigin delta = (newCenter - 0)
// which correctly shifts corners from their REF positions to the calibrated center.
volatile long g_lastOrigin_X = ORIGIN_REF_X;
volatile long g_lastOrigin_Y = ORIGIN_REF_Y;

volatile long g_BO_A1_X = 0, g_BO_A1_Y = 0;
volatile long g_BO_H1_X = 0, g_BO_H1_Y = 0;
volatile long g_BO_A8_X = 0, g_BO_A8_Y = 0;
volatile long g_BO_H8_X = 0, g_BO_H8_Y = 0;

// INA
INA3221 ina(INA3221_ADDR40_GND);

// WiFi + Web
const char* WIFI_SSID = "";  // géré via le WebUI → NVS
const char* WIFI_PASS = "";

const char* AP_SSID = "ESP32-Chessboard";
const char* AP_PASS = "12345678";

WebServer server(80);
WebSocketsServer webSocket(81);

// Motion shared
volatile float g_vx_xy = 0.0f;
volatile float g_vy_xy = 0.0f;
volatile bool  g_recenter = false;
volatile SpeedMode g_speedMode = SPEED_NORMAL;
volatile unsigned long g_lastCmdMs = 0;

volatile bool g_driversEnabled = false;
volatile unsigned long g_lastMotionMs = 0;

volatile long g_Apos = 0;
volatile long g_Bpos = 0;

volatile long g_xAbs = XY_ORIGIN_X;
volatile long g_yAbs = XY_ORIGIN_Y;

volatile float g_battV = 0, g_battI = 0, g_battPct = 0;
volatile bool g_magnetOn = false;

// Path
Waypoint g_waypoints[MAX_WAYPOINTS];
volatile uint8_t g_wpCount = 0;
volatile uint8_t g_wpIndex = 0;
volatile bool g_pathActive = false;
volatile long g_pathTargetX = XY_ORIGIN_X;
volatile long g_pathTargetY = XY_ORIGIN_Y;

volatile bool g_autoMagnetPath = false;

// Auto calib
volatile bool g_autoCalibRequested = false;
volatile bool g_autoCalibStarted   = false;

// Dead zone endpoints (0 = not yet calibrated — fallback used)
volatile long g_DZ_L_Bas_X  = 0, g_DZ_L_Bas_Y  = 0;
volatile long g_DZ_L_Haut_X = 0, g_DZ_L_Haut_Y = 0;
volatile long g_DZ_R_Bas_X  = 0, g_DZ_R_Bas_Y  = 0;
volatile long g_DZ_R_Haut_X = 0, g_DZ_R_Haut_Y = 0;
volatile bool g_DZ_L_Calibrated = false;
volatile bool g_DZ_R_Calibrated = false;

volatile bool g_dzCalibYExpanded = false;
volatile bool g_dzPathYExpanded  = false;

volatile bool g_wsPushPending = false;

volatile bool          g_calibNvsDirty   = false;
volatile unsigned long g_calibNvsDirtyMs = 0;

// Chess Test Run
volatile bool    g_testRunActive   = false;
volatile bool    g_testRunAbortReq = false;
volatile char    g_trStepName[64]  = {};
volatile uint8_t g_trStepIdx       = 0;
volatile uint8_t g_trStepTotal     = 0;

// System state
volatile SystemState g_systemState = SYS_UNHOMED;

// AutoTune
volatile TunePhase g_tunePhase    = AT_PHASE_IDLE;
volatile int       g_tuneProgress = 0;
volatile bool      g_tuneActive   = false;
volatile bool      g_tuneAbortReq = false;

TuneSettings g_tuneSettings = {
    /* safeSpeed     */ 8600.0f,
    /* safeSpeedDiag */ 6000.0f,
    /* safeAccel     */ 6000.0f,
    /* safeDecel     */ 6000.0f,
    /* motorCurrent  */ 850,
    /* tuningValid   */ false,
    /* boundsValid   */ false,
};

volatile float g_overrideVmax     = 0.0f;
volatile float g_overrideDiagVmax = 0.0f;
volatile float g_overrideAccel    = 0.0f;
volatile float g_overrideDecel    = 0.0f;
volatile uint16_t g_tuneCurrentMa = 0;
volatile float g_tuneLiveAxisSpeed = 0.0f;
volatile float g_tuneLiveDiagSpeed = 0.0f;
volatile float g_tuneLiveAccel     = 0.0f;
volatile float g_tuneLiveDecel     = 0.0f;

DriftRecord     g_driftLog[MAX_DRIFT_LOG];
volatile uint8_t g_driftCount = 0;

AtLogEntry       g_atLogBuf[AT_LOG_SLOTS];
volatile uint8_t g_atLogWr = 0;
volatile uint8_t g_atLogRd = 0;

void initGlobals() {
  portENTER_CRITICAL(&gMux);

  g_Apos = 0;
  g_Bpos = 0;

  g_xAbs = XY_ORIGIN_X;
  g_yAbs = XY_ORIGIN_Y;

  g_xCenterTarget = XY_ORIGIN_X;
  g_yCenterTarget = XY_ORIGIN_Y;

  g_lastCmdMs = millis();
  g_lastMotionMs = millis();

  g_hallYDetected = false;
  g_hallXDetected = false;

  g_yLimitsCalibrated = false;
  g_xLimitsCalibrated = false;
  g_yHardMin = g_yHardMax = 0;
  g_xHardMin = g_xHardMax = 0;
  g_yHallPos = 0;
  g_xHallPos = 0;

  g_calibState = CALIB_IDLE;
  g_recenter = false;

  g_pathActive = false;
  g_wpCount = 0;
  g_wpIndex = 0;
  g_pathTargetX = XY_ORIGIN_X;
  g_pathTargetY = XY_ORIGIN_Y;

  g_autoMagnetPath = false;

  g_autoCalibRequested = true;
  g_autoCalibStarted = false;

  portEXIT_CRITICAL(&gMux);
}
