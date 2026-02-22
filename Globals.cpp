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

volatile long g_BO_A1_X = 0, g_BO_A1_Y = 0;
volatile long g_BO_H1_X = 0, g_BO_H1_Y = 0;
volatile long g_BO_A8_X = 0, g_BO_A8_Y = 0;
volatile long g_BO_H8_X = 0, g_BO_H8_Y = 0;

// INA
INA3221 ina(INA3221_ADDR40_GND);

// WiFi + Web
const char* WIFI_SSID = "";
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