#include "Commands.h"
#include "Utils.h"
#include "Magnet.h"
#include "PathPlanner.h"
#include "Calibration.h"
#include "BoardMapping.h"
#include "MotionCoreXY.h"

enum PendingMoveType : uint8_t {
  PM_SQUARE = 0,
  PM_SQUARE_DIRECT = 1,
  PM_PATH = 2,
};

struct PendingMove {
  PendingMoveType type;
  uint8_t ff;
  uint8_t fr;
  uint8_t tf;
  uint8_t tr;
  Waypoint wps[MAX_WAYPOINTS];
  uint8_t n;
};

static const uint8_t PENDING_MOVE_CAP = 8;
static PendingMove g_pendingMoves[PENDING_MOVE_CAP];
static uint8_t g_pendingHead = 0;
static uint8_t g_pendingTail = 0;
static uint8_t g_pendingCount = 0;

static bool isMotionBusy() {
  CalibState calib;
  bool recenter;
  bool path;

  portENTER_CRITICAL(&gMux);
  calib = g_calibState;
  recenter = g_recenter;
  path = g_pathActive;
  portEXIT_CRITICAL(&gMux);

  return (calib != CALIB_IDLE) || recenter || path;
}

static void executePendingMove(const PendingMove &m) {
  if (m.type == PM_SQUARE) {
    planSquareMove(m.ff, m.fr, m.tf, m.tr);
    return;
  }
  if (m.type == PM_SQUARE_DIRECT) {
    planSquareMoveDirect(m.ff, m.fr, m.tf, m.tr);
    return;
  }
  beginMoveSeq(m.wps, m.n);
}

static bool enqueuePendingMove(const PendingMove &m) {
  if (g_pendingCount >= PENDING_MOVE_CAP) {
    Serial.println("[QUEUE] full, dropping new move");
    return false;
  }

  g_pendingMoves[g_pendingTail] = m;
  g_pendingTail = (uint8_t)((g_pendingTail + 1) % PENDING_MOVE_CAP);
  g_pendingCount++;

  Serial.printf("[QUEUE] move queued (pending=%u)\n", (unsigned)g_pendingCount);
  return true;
}

static bool dequeuePendingMove(PendingMove &out) {
  if (g_pendingCount == 0) return false;

  out = g_pendingMoves[g_pendingHead];
  g_pendingHead = (uint8_t)((g_pendingHead + 1) % PENDING_MOVE_CAP);
  g_pendingCount--;
  return true;
}

static void clearPendingMoves() {
  g_pendingHead = 0;
  g_pendingTail = 0;
  g_pendingCount = 0;
}

static void enqueueOrExecute(const PendingMove &m) {
  if (isMotionBusy()) {
    enqueuePendingMove(m);
    return;
  }
  executePendingMove(m);
}

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

  StaticJsonDocument<1024> doc;
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

  if (strcmp(cmd, "resetPieces") == 0) {
    clearPendingMoves();
    abortPath();

    portENTER_CRITICAL(&gMux);
    g_recenter = false;
    g_vx_xy = 0;
    g_vy_xy = 0;
    g_lastCmdMs = millis();
    portEXIT_CRITICAL(&gMux);

    magnetSet(false);
    Serial.println("[RESET] resetPieces received: path aborted and queue cleared");
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

    PendingMove m = {};
    m.type = PM_SQUARE;
    m.ff = (uint8_t)ff;
    m.fr = (uint8_t)fr;
    m.tf = (uint8_t)tf;
    m.tr = (uint8_t)tr;
    enqueueOrExecute(m);
    return;
  }

  if (strcmp(cmd, "squareMoveDirect") == 0) {
    if (!doc.containsKey("ff") || !doc.containsKey("fr") || !doc.containsKey("tf") || !doc.containsKey("tr")) return;
    int ff = (int)doc["ff"];
    int fr = (int)doc["fr"];
    int tf = (int)doc["tf"];
    int tr = (int)doc["tr"];
    if (ff < 0 || ff > 7 || tf < 0 || tf > 7) return;
    if (fr < 1 || fr > 8 || tr < 1 || tr > 8) return;

    PendingMove m = {};
    m.type = PM_SQUARE_DIRECT;
    m.ff = (uint8_t)ff;
    m.fr = (uint8_t)fr;
    m.tf = (uint8_t)tf;
    m.tr = (uint8_t)tr;
    enqueueOrExecute(m);
    return;
  }

  if (strcmp(cmd, "pathMove") == 0) {
    if (!doc.containsKey("path")) return;
    JsonArray path = doc["path"].as<JsonArray>();
    if (path.isNull()) return;
    if (path.size() < 2) return;

    Waypoint wps[MAX_WAYPOINTS];
    uint8_t n = 0;

    for (JsonVariant v : path) {
      if (n >= MAX_WAYPOINTS) break;
      if (!v.is<JsonObject>()) continue;
      JsonObject obj = v.as<JsonObject>();
      if (!obj.containsKey("f") || !obj.containsKey("r")) continue;
      int f = (int)obj["f"];
      int r = (int)obj["r"];
      if (f < 0 || f > 7 || r < 1 || r > 8) continue;

      long x, y;
      squareCenterSteps((uint8_t)f, (uint8_t)r, x, y);
      wps[n++] = { x, y, -1 };
    }

    if (n < 2) return;
    wps[0].mag = 1;
    wps[n - 1].mag = 0;

    PendingMove m = {};
    m.type = PM_PATH;
    m.n = n;
    for (uint8_t i = 0; i < n; i++) m.wps[i] = wps[i];
    enqueueOrExecute(m);
    return;
  }
}

void commandsLoop() {
  if (isMotionBusy()) return;

  PendingMove m;
  if (!dequeuePendingMove(m)) return;

  Serial.printf("[QUEUE] executing queued move (remaining=%u)\n", (unsigned)g_pendingCount);
  executePendingMove(m);
}

bool commandsIsBusy() {
  return isMotionBusy() || (g_pendingCount > 0);
}

uint8_t commandsPendingCount() {
  return g_pendingCount;
}