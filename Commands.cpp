

#include "Commands.h"
#include "Utils.h"
#include "Magnet.h"
#include "PathPlanner.h"
#include "Calibration.h"
#include "BoardMapping.h"
#include "MotionCoreXY.h"
#include "AutoTune.h"
#include "ChessTestRun.h"
#include "WiFiNet.h"

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
  // AutoTune holds exclusive motion control for the duration of its task.
  if (g_tuneActive) return true;

  CalibState calib;
  bool recenter;
  bool path;

  portENTER_CRITICAL(&gMux);
  calib    = g_calibState;
  recenter = g_recenter;
  path     = g_pathActive;
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
  // Request an immediate telemetry push for the new client.
  // Done via a flag rather than calling webPushTelemetry() directly here,
  // because we're inside webSocket.loop() — re-entering broadcastTXT()
  // from within the event callback corrupts the library's internal state.
  if (type == WStype_CONNECTED) {
    g_wsPushPending = true;
    return;
  }
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
    // Guard: refuse to start calibration while AutoTune holds the motion bus.
    if (g_tuneActive) {
      Serial.println("[CMD] calibAll ignored — AutoTune is running");
      return;
    }
    startFullCalibration();
    return;
  }

  // { "cmd": "calibCorner", "which": "a1"|"h1"|"a8"|"h8" }
  // The carriage is currently positioned at the centre of the named corner square.
  // Back-calculate the board origin so the whole grid is updated.
  if (strcmp(cmd, "calibCorner") == 0) {
    const char* which = doc["which"];
    if (!which) return;

    long xNow, yNow;
    portENTER_CRITICAL(&gMux);
    xNow = g_xAbs;
    yNow = g_yAbs;
    portEXIT_CRITICAL(&gMux);

    // Store the calibrated centre directly into the corner's own global —
    // the other 3 corners are NOT touched.  boardUpdateFromCorners() then
    // recomputes the bilinear boundary so every square reinterpolates correctly.
    portENTER_CRITICAL(&gMux);
    if      (strcmp(which, "a1") == 0) { g_A1C_X = xNow; g_A1C_Y = yNow; }
    else if (strcmp(which, "h1") == 0) { g_H1C_X = xNow; g_H1C_Y = yNow; }
    else if (strcmp(which, "a8") == 0) { g_A8C_X = xNow; g_A8C_Y = yNow; }
    else if (strcmp(which, "h8") == 0) { g_H8C_X = xNow; g_H8C_Y = yNow; }
    else { portEXIT_CRITICAL(&gMux); Serial.printf("[CALIB] calibCorner unknown which=%s\n", which); return; }
    portEXIT_CRITICAL(&gMux);

    boardUpdateFromCorners();
    g_calibNvsDirty   = true;
    g_calibNvsDirtyMs = millis();
    Serial.printf("[CALIB] corner %s set to abs=(%ld,%ld) — all squares reinterpolated\n", which, xNow, yNow);
    return;
  }

  // { "cmd": "calibDeadZone", "side": "L"|"R", "ext": "bas"|"haut" }
  // Carriage is at the desired position for this dead-zone endpoint.
  if (strcmp(cmd, "calibDeadZone") == 0) {
    const char* side = doc["side"];
    const char* ext  = doc["ext"];
    if (!side || !ext) return;

    long xNow, yNow;
    portENTER_CRITICAL(&gMux);
    xNow = g_xAbs;
    yNow = g_yAbs;
    portEXIT_CRITICAL(&gMux);

    portENTER_CRITICAL(&gMux);
    if (strcmp(side, "L") == 0) {
      if (strcmp(ext, "bas")  == 0) { g_DZ_L_Bas_X  = xNow; g_DZ_L_Bas_Y  = yNow; }
      else                          { g_DZ_L_Haut_X = xNow; g_DZ_L_Haut_Y = yNow; g_DZ_L_Calibrated = true; }
    } else {
      if (strcmp(ext, "bas")  == 0) { g_DZ_R_Bas_X  = xNow; g_DZ_R_Bas_Y  = yNow; }
      else                          { g_DZ_R_Haut_X = xNow; g_DZ_R_Haut_Y = yNow; g_DZ_R_Calibrated = true; }
    }
    g_dzCalibYExpanded = false;  // calibration validated — restore normal Y limits
    portEXIT_CRITICAL(&gMux);
    g_calibNvsDirty   = true;
    g_calibNvsDirtyMs = millis();
    Serial.printf("[DZ] calibDeadZone side=%s ext=%s pos=(%ld,%ld)\n", side, ext, xNow, yNow);
    return;
  }

  // { "cmd": "deadZoneMove", "ff": file, "fr": rank, "side": "L"|"R", "slot": 0..15,
  //   "path": [{u,v},...] (optional intersection UV waypoints, integers 0..8) }
  // Pick up the piece at (ff,fr), navigate via BFS path if provided to avoid
  // collisions, then deliver to the physical dead-zone slot.
  if (strcmp(cmd, "deadZoneMove") == 0) {
    if (!doc.containsKey("ff") || !doc.containsKey("fr") ||
        !doc.containsKey("side") || !doc.containsKey("slot")) return;
    int ff   = (int)doc["ff"];
    int fr   = (int)doc["fr"];
    int slot = (int)doc["slot"];
    const char* side = doc["side"];
    if (ff < 0 || ff > 7 || fr < 1 || fr > 8) return;
    if (slot < 0 || slot > 15 || !side) return;

    long srcX, srcY;
    squareCenterSteps((uint8_t)ff, (uint8_t)fr, srcX, srcY);

    long dzX, dzY;
    deadZoneSlotPos(side[0], (uint8_t)slot, dzX, dzY);

    portENTER_CRITICAL(&gMux);
    g_dzPathYExpanded = true;  // expand Y limits for the duration of this path
    portEXIT_CRITICAL(&gMux);

    PendingMove m = {};
    m.type = PM_PATH;
    uint8_t n = 0;

    // Parse optional capturer move (capFF/capFR = FROM, capTF/capTR = TO).
    // When present the path continues directly to pick up and place the
    // capturing piece, making the whole capture a single uninterrupted motion.
    bool hasCapMove = doc.containsKey("capFF") && doc.containsKey("capFR") &&
                      doc.containsKey("capTF") && doc.containsKey("capTR");
    int capFF_v = hasCapMove ? (int)doc["capFF"] : -1;
    int capFR_v = hasCapMove ? (int)doc["capFR"] : -1;
    int capTF_v = hasCapMove ? (int)doc["capTF"] : -1;
    int capTR_v = hasCapMove ? (int)doc["capTR"] : -1;
    if (hasCapMove && (capFF_v < 0 || capFF_v > 7 || capFR_v < 1 || capFR_v > 8 ||
                       capTF_v < 0 || capTF_v > 7 || capTR_v < 1 || capTR_v > 8))
      hasCapMove = false;

    // First waypoint is always the captured piece's centre (pick-up point).
    m.wps[n++] = { srcX, srcY, -1 };  // mag set to 1 below

    // If the UI provided an intersection path, follow it to avoid piece collisions.
    // Path entries are {u,v} UV integers (0..8) for intersection-grid waypoints.
    // Reserve 3 slots: dead-zone + (capturer FROM + TO) or (return waypoint).
    if (doc.containsKey("path")) {
      JsonArray pathArr = doc["path"].as<JsonArray>();
      if (!pathArr.isNull() && pathArr.size() >= 1) {
        for (JsonVariant v : pathArr) {
          if (n >= MAX_WAYPOINTS - 3) break;
          if (!v.is<JsonObject>()) continue;
          JsonObject obj = v.as<JsonObject>();
          long wx, wy;
          if (obj.containsKey("u") && obj.containsKey("v")) {
            float pu = obj["u"].as<float>();
            float pv = obj["v"].as<float>();
            boardUVToXY(pu, pv, wx, wy);
            long xMin, xMax, yMin, yMax;
            getXLimits(xMin, xMax); getYLimits(yMin, yMax);
            wx = clampl(wx, xMin + EDGE_STOP_DIST, xMax - EDGE_STOP_DIST);
            wy = clampl(wy, yMin + EDGE_STOP_DIST, yMax - EDGE_STOP_DIST);
          } else if (obj.containsKey("f") && obj.containsKey("r")) {
            int pf = (int)obj["f"], pr = (int)obj["r"];
            if (pf < 0 || pf > 7 || pr < 1 || pr > 8) continue;
            squareCenterSteps((uint8_t)pf, (uint8_t)pr, wx, wy);
          } else {
            continue;
          }
          if (wx == m.wps[n-1].x && wy == m.wps[n-1].y) continue;
          m.wps[n++] = { wx, wy, -1 };
        }
      }
    }

    // Remember the last board-side waypoint (within normal Y limits).
    long retX = m.wps[n - 1].x;
    long retY = m.wps[n - 1].y;

    // Dead zone drop waypoint.
    uint8_t dzIdx = n;
    m.wps[n++] = { dzX, dzY, -1 };

    if (hasCapMove) {
      // Continue directly to the capturing piece then move it to its destination.
      // The path ends at a normal board square so g_dzPathYExpanded is cleared
      // only once the carriage is back within normal Y limits.
      long capFromX, capFromY, capToX, capToY;
      squareCenterSteps((uint8_t)capFF_v, (uint8_t)capFR_v, capFromX, capFromY);
      squareCenterSteps((uint8_t)capTF_v, (uint8_t)capTR_v, capToX,   capToY);
      m.wps[n++] = { capFromX, capFromY, 1 };  // pick up capturing piece
      m.wps[n++] = { capToX,   capToY,   0 };  // drop at destination
    } else {
      // No capturer chaining: return to the board-edge square so the carriage
      // exits the expanded Y zone before g_dzPathYExpanded is cleared.
      m.wps[n++] = { retX, retY, -1 };
    }

    m.n = n;

    // Magnet assignments:
    //   wps[0]     = pick up captured piece
    //   wps[dzIdx] = drop at dead-zone slot
    //   capturer FROM/TO mags are already set inline above (1 / 0)
    m.wps[0].mag    = 1;
    m.wps[dzIdx].mag = 0;

    enqueueOrExecute(m);
    Serial.printf("[DZ] deadZoneMove (%d,%d) -> side=%s slot=%d abs=(%ld,%ld) wps=%d\n",
                  ff, fr, side, slot, dzX, dzY, (int)n);
    return;
  }

  // { "cmd": "gotoSquare", "f": 0..7, "r": 1..8 }
  // Move carriage to the centre of the given square unconditionally (no same-square skip).
  // Used by calibration UI to position the carriage on a corner.
  if (strcmp(cmd, "gotoSquare") == 0) {
    if (!doc.containsKey("f") || !doc.containsKey("r")) return;
    int f = (int)doc["f"];
    int r = (int)doc["r"];
    if (f < 0 || f > 7 || r < 1 || r > 8) return;

    long tx, ty;
    squareCenterSteps((uint8_t)f, (uint8_t)r, tx, ty);

    PendingMove m = {};
    m.type   = PM_PATH;
    m.wps[0] = { tx, ty, -1 };
    m.n      = 1;
    enqueueOrExecute(m);
    Serial.printf("[CALIB] gotoSquare f=%d r=%d abs=(%ld,%ld)\n", f, r, tx, ty);
    return;
  }

  // { "cmd": "gotoDeadZone", "side": "L"|"R", "slot": 0..15 }
  // Move carriage to the stored (or fallback) dead-zone position for that endpoint.
  // Used by calibration UI to show the user where the current position is.
  if (strcmp(cmd, "gotoDeadZone") == 0) {
    const char* side = doc["side"];
    if (!side || !doc.containsKey("slot")) return;
    uint8_t slot = (uint8_t)(int)doc["slot"];
    long tx, ty;
    deadZoneSlotPos(side[0], slot, tx, ty);

    portENTER_CRITICAL(&gMux);
    g_dzCalibYExpanded = true;  // keep Y limits expanded while calib panel is open
    portEXIT_CRITICAL(&gMux);

    PendingMove m = {};
    m.type   = PM_PATH;
    m.wps[0] = { tx, ty, -1 };
    m.n      = 1;
    enqueueOrExecute(m);
    Serial.printf("[DZ] gotoDeadZone side=%s slot=%d abs=(%ld,%ld)\n", side, slot, tx, ty);
    return;
  }

  // { "cmd": "endDZCalib" }
  // Sent by the UI when the dead-zone calibration panel is closed without validating
  // (cancel or switching away). Restores normal Y limits.
  if (strcmp(cmd, "endDZCalib") == 0) {
    portENTER_CRITICAL(&gMux);
    g_dzCalibYExpanded = false;
    portEXIT_CRITICAL(&gMux);
    Serial.println("[DZ] endDZCalib — Y limits restored");
    return;
  }

  // { "cmd": "start_tuning" }
  if (strcmp(cmd, "start_tuning") == 0) {
    if (!autoTuneStart()) {
      Serial.println("[CMD] start_tuning rejected (busy or no calib)");
    }
    return;
  }

  // { "cmd": "start_test_run" }
  if (strcmp(cmd, "start_test_run") == 0) {
    if (!chessTestRunStart()) {
      Serial.println("[CMD] start_test_run rejected (busy or no calib)");
    }
    return;
  }

  // { "cmd": "wifiConfig", "ssid": "...", "pass": "..." }
  // Save WiFi credentials to NVS and reboot to connect to the home network.
  if (strcmp(cmd, "wifiConfig") == 0) {
    const char* ssid = doc["ssid"];
    const char* pass = doc["pass"];
    if (!ssid || strlen(ssid) == 0) {
      Serial.println("[WIFI] wifiConfig ignored — empty SSID");
      return;
    }
    saveWifiCreds(ssid, pass ? pass : "");
    return;  // never reached (saveWifiCreds reboots)
  }

  // { "cmd": "stop" }
  // Emergency stop: halts motion and aborts any running AutoTune or TestRun.
  if (strcmp(cmd, "stop") == 0) {
    autoTuneStop();
    chessTestRunStop();
    clearPendingMoves();
    abortPath();
    portENTER_CRITICAL(&gMux);
    g_recenter = false;
    g_vx_xy    = 0.0f;
    g_vy_xy    = 0.0f;
    g_lastCmdMs = millis();
    portEXIT_CRITICAL(&gMux);
    Serial.println("[CMD] STOP — motion halted, tune aborted if running");
    return;
  }

  // { "cmd": "status" }
  // Telemetry push handles status; this just acknowledges receipt.
  if (strcmp(cmd, "status") == 0) {
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