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

static const float CALIB_Y_SPEED  = 700.0f;
static const float CALIB_X_SPEED  = 700.0f;

static const int   CALIB_Y_DIR    = -1;
static const int   CALIB_X_DIR    = -1;

static const bool  HALL_AT_Y_MIN  = true;
static const bool  HALL_AT_X_MIN  = true;

enum CalibState : uint8_t { CALIB_IDLE=0, CALIB_Y=1, CALIB_X=2, CALIB_RECENTER=3 };

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

// Accel
static const float ACCEL_AB = 32000.0f;

// Recenter / Path
static const float RECENTER_KP = 2.0f;
static const float RECENTER_VMAX_XY = 7000.0f;
static const float RECENTER_RAMP_ACC = 9000.0f;
static const float RECENTER_MIN_VXY = 350.0f;
static const long  RECENTER_DEADBAND_XY = 12;
static const unsigned long RECENTER_SETTLE_MS = 120;

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
// Global init
// =======================================================
void initGlobals();