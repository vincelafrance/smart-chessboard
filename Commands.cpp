#include "Commands.h"
#include "Utils.h"
#include "Magnet.h"
#include "PathPlanner.h"
#include "Calibration.h"

void setSpeedMode(uint8_t sp) {
  if (sp > 2) sp = 1;
  portENTER_CRITICAL(&gMux);
  g_speedMode = (SpeedMode)sp;
  g_lastCmdMs = millis();
  portEXIT_CRITICAL(&gMux);
}

void setDirCommand(const String& dir) {
  unsigned long now = millis();

  portENTER_CRITICAL(&gMux);
  g_lastCmdMs = now;

  g_calibState = CALIB_IDLE;
  g_pathActive = false;
  g_wpCount = 0;
  g_wpIndex = 0;
  g_autoMagnetPath = false;

  if (dir == "stop") {
    g_recenter = false;
    g_vx_xy = 0;
    g_vy_xy = 0;
    portEXIT_CRITICAL(&gMux);
    return;
  }

  if (dir == "center") {
    g_recenter = true;
    g_vx_xy = 0;
    g_vy_xy = 0;
    portEXIT_CRITICAL(&gMux);
    return;
  }

  g_recenter = false;

  float spd = speedForModeXY(g_speedMode);
  if (dir == "up")    { g_vx_xy = 0;     g_vy_xy = +spd; }
  if (dir == "down")  { g_vx_xy = 0;     g_vy_xy = -spd; }
  if (dir == "left")  { g_vx_xy = -spd;  g_vy_xy = 0; }
  if (dir == "right") { g_vx_xy = +spd;  g_vy_xy = 0; }

  portEXIT_CRITICAL(&gMux);
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  (void)num;
  if (type != WStype_TEXT) return;

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) return;

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  if (strcmp(cmd, "move") == 0) {
    const char* dir = doc["dir"];
    if (!dir) return;
    setDirCommand(String(dir));
    return;
  }

  if (strcmp(cmd, "speed") == 0) {
    if (!doc.containsKey("sp")) return;
    setSpeedMode((uint8_t)doc["sp"]);
    return;
  }

  if (strcmp(cmd, "magnet") == 0) {
    bool cur;
    portENTER_CRITICAL(&gMux);
    cur = g_magnetOn;
    portEXIT_CRITICAL(&gMux);

    bool next = cur;
    if (doc.containsKey("toggle") && (bool)doc["toggle"]) next = !cur;
    else if (doc.containsKey("on")) next = (bool)doc["on"];

    portENTER_CRITICAL(&gMux);
    g_autoMagnetPath = false;
    portEXIT_CRITICAL(&gMux);

    magnetSet(next);
    return;
  }

  if (strcmp(cmd, "calibAll") == 0) {
    startFullCalibration();
    return;
  }

  if (strcmp(cmd, "squareMove") == 0) {
    if (!doc.containsKey("ff") || !doc.containsKey("fr") || !doc.containsKey("tf") || !doc.containsKey("tr")) return;
    int ff = (int)doc["ff"];
    int fr = (int)doc["fr"];
    int tf = (int)doc["tf"];
    int tr = (int)doc["tr"];
    if (ff < 0 || ff > 7 || tf < 0 || tf > 7) return;
    if (fr < 1 || fr > 8 || tr < 1 || tr > 8) return;

    planSquareMove((uint8_t)ff, (uint8_t)fr, (uint8_t)tf, (uint8_t)tr);
    return;
  }
}