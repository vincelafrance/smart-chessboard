#include <Arduino.h>
#include "Globals.h"
#include "WebUI.h"
#include "WiFiNet.h"
#include "Magnet.h"
#include "BatteryINA.h"
#include "BoardMapping.h"
#include "Commands.h"
#include "Calibration.h"
#include "StepTask.h"
#include "MotionCoreXY.h"
#include "DriversUART.h"
#include "AutoTune.h"
#include "ChessTestRun.h"
#include "CalibNVS.h"

unsigned long lastBattMs = 0;
unsigned long lastPushMs = 0;

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\nHELLO ESP32 - serial ok");

  esp_task_wdt_deinit();

  // Hall
  pinMode(HALL_Y_PIN, INPUT);
  pinMode(HALL_X_PIN, INPUT);
  Serial.println("[HALL] GPIO34(Y) + GPIO35(X) ready (active LOW)");

  // Drivers pins
  pinMode(LEFT_STEP, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_EN, OUTPUT);

  pinMode(RIGHT_STEP, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);

  gpio_set_direction((gpio_num_t)LEFT_STEP, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)LEFT_DIR,  GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)RIGHT_STEP,GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)RIGHT_DIR, GPIO_MODE_OUTPUT);

  initDriversUART();

  setDriversEnabled(false);
  Serial.println("[DRIVERS] boot -> OFF");

  magnetInit();
  Serial.println("[MAGNET] init -> OFF");

  batteryInit();

  // Init globals
  initGlobals();

  // Restore calibration data from NVS.  If found, recompute board boundaries
  // from the saved corners directly.  If not found, fall back to the default
  // origin-based initialisation so the system is in a usable state before the
  // first physical calibration run.
  bool calibLoaded = loadCalibFromNVS();
  if (calibLoaded) {
    boardUpdateFromCorners();   // recompute BO_ boundaries from restored corners
  } else {
    long cx, cy;
    portENTER_CRITICAL(&gMux);
    cx = g_xCenterTarget;
    cy = g_yCenterTarget;
    portEXIT_CRITICAL(&gMux);
    boardUpdateFromOrigin(cx, cy);
  }

  // WiFi + Web
  startWiFi();
  initOTA();
  webInit();

  Serial.println("OK. Ouvre:");
  if (WiFi.getMode() == WIFI_MODE_AP) Serial.println("  http://192.168.4.1");
  else { Serial.print("  http://"); Serial.println(WiFi.localIP()); }

  stepTaskStart();

  // Load NVS-persisted tune settings and apply them to the motion system.
  // Must come after initDriversUART() so setCurrentOverrides() has a driver.
  autoTuneInit();
  chessTestRunInit();
}

void loop() {
  handleOTA();
  webLoop();

  const unsigned long now = millis();

  // Auto calibration at boot
  if (g_autoCalibRequested && !g_autoCalibStarted) {
    g_autoCalibStarted = true;
    Serial.println("[AUTO] Starting full calibration sequence...");
    startFullCalibration();
  }

  // Hall poll + calibration sequencing
  calibrationLoop(now);

  // AutoTune log drain + system-state tracking
  autoTuneLoop(now);

  // Execute queued move requests once motion is idle.
  commandsLoop();

  // Battery read
  if (now - lastBattMs >= 1000) {
    lastBattMs = now;

    float v, i, pct;
    readBatteryOnce(v, i, pct);

    portENTER_CRITICAL(&gMux);
    g_battV = v;
    g_battI = i;
    g_battPct = pct;
    portEXIT_CRITICAL(&gMux);
  }

  // Deferred NVS calibration save — triggered 3 s after the last mutation,
  // only when no path or calibration is in flight.  Runs in a one-shot
  // background task so loop() / webLoop() are never blocked by flash writes.
  if (g_calibNvsDirty && (now - g_calibNvsDirtyMs >= 3000)
      && !g_pathActive && (g_calibState == CALIB_IDLE)) {
    g_calibNvsDirty = false;
    xTaskCreate([](void*){ saveCalibToNVS(); vTaskDelete(nullptr); },
                "nvsSave", 4096, nullptr, 1, nullptr);
  }

  // Immediate push requested by onWsEvent when a new client connects.
  if (g_wsPushPending) {
    g_wsPushPending = false;
    webPushTelemetry();
  }

  // Push telemetry
  if (now - lastPushMs >= 100) {
    lastPushMs = now;
    webPushTelemetry();
  }
}
