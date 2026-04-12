#include "Calibration.h"
#include "Utils.h"
#include "MotionCoreXY.h"
#include "PathPlanner.h"
#include "BoardMapping.h"

static const unsigned long CALIB_HALL_POLL_MS = 2;
static long s_yBottom = 0;
static long s_xBottom = 0;
static long s_yTop = 0;
static long s_xTop = 0;

void startFullCalibration() {
  int sy = digitalRead(HALL_Y_PIN);
  int sx = digitalRead(HALL_X_PIN);
  if (sy == LOW || sx == LOW) {
    Serial.println("[CALIB] Refused: hall already DETECTED. Move away then retry.");
    portENTER_CRITICAL(&gMux);
    g_calibState = CALIB_IDLE;
    g_recenter = false;
    g_vx_xy = 0;
    g_vy_xy = 0;
    g_lastCmdMs = millis();
    portEXIT_CRITICAL(&gMux);
    return;
  }

  unsigned long now = millis();
  portENTER_CRITICAL(&gMux);
  abortPath();
  g_recenter = false;
  g_vx_xy = 0;
  g_vy_xy = 0;
  g_calibState = CALIB_Y_BOTTOM;
  g_lastCmdMs = now;
  portEXIT_CRITICAL(&gMux);

  s_yBottom = s_xBottom = s_yTop = s_xTop = 0;
  Serial.println("[CALIB] Full sequence started: Bas(Y->X) -> Haut(Y->X) -> Recenter");
}

// Start calibration when one corner is already active (both Hall sensors LOW).
// Records the known corner and jumps directly to finding the opposite corner,
// bypassing startFullCalibration()'s Hall-LOW guard — no escape move required.
// lastHallY/XState are forced to LOW so calibrationLoop waits for the sensors
// to go HIGH (leaving corner) before recognising the far-corner LOW edge.
void startCalibrationFromKnownCorner(long yKnown, long xKnown, bool knownIsBottom) {
  unsigned long now = millis();
  portENTER_CRITICAL(&gMux);
  abortPath();
  g_recenter = false;
  g_vx_xy    = 0;
  g_vy_xy    = 0;
  portEXIT_CRITICAL(&gMux);

  s_yBottom = s_xBottom = s_yTop = s_xTop = 0;
  // Force edge history to LOW: both pins are currently LOW at the known corner.
  lastHallYState = LOW;
  lastHallXState = LOW;

  if (knownIsBottom) {
    s_yBottom = yKnown;
    s_xBottom = xKnown;
    portENTER_CRITICAL(&gMux);
    g_yHallPos   = s_yBottom;
    g_xHallPos   = s_xBottom;
    g_calibState = CALIB_Y_TOP;
    g_lastCmdMs  = now;
    portEXIT_CRITICAL(&gMux);
    Serial.printf("[CALIB] Corner-start: Bas connu (y=%ld, x=%ld) -> recherche Haut\n",
                  yKnown, xKnown);
  } else {
    s_yTop = yKnown;
    s_xTop = xKnown;
    portENTER_CRITICAL(&gMux);
    g_calibState = CALIB_Y_BOTTOM;
    g_lastCmdMs  = now;
    portEXIT_CRITICAL(&gMux);
    Serial.printf("[CALIB] Corner-start: Haut connu (y=%ld, x=%ld) -> recherche Bas\n",
                  yKnown, xKnown);
  }
}

// Start calibration from a single known Hall axis (Y only or X only), or both.
// This is used when tuning fails and the carriage is already touching one sensor.
// knownIsBottom selects which side is currently known:
//   true  -> bottom/ref side
//   false -> top/far side
void startCalibrationFromKnownHall(bool hasY, long yKnown,
                                   bool hasX, long xKnown,
                                   bool knownIsBottom) {
  if (hasY && hasX) {
    startCalibrationFromKnownCorner(yKnown, xKnown, knownIsBottom);
    return;
  }
  if (!hasY && !hasX) {
    startFullCalibration();
    return;
  }

  unsigned long now = millis();
  portENTER_CRITICAL(&gMux);
  abortPath();
  g_recenter = false;
  g_vx_xy    = 0;
  g_vy_xy    = 0;
  portEXIT_CRITICAL(&gMux);

  s_yBottom = s_xBottom = s_yTop = s_xTop = 0;
  // Match edge history to current physical state so calibrationLoop sees clean
  // transitions while leaving the known Hall and searching the opposite corner.
  lastHallYState = digitalRead(HALL_Y_PIN);
  lastHallXState = digitalRead(HALL_X_PIN);

  if (hasY) {
    if (knownIsBottom) {
      s_yBottom = yKnown;
      portENTER_CRITICAL(&gMux);
      g_yHallPos   = s_yBottom;
      g_calibState = CALIB_X_BOTTOM;  // complete this known side by finding X
      g_lastCmdMs  = now;
      portEXIT_CRITICAL(&gMux);
      Serial.printf("[CALIB] Side-start: Y Bas connu (y=%ld) -> recherche X Bas\n", yKnown);
    } else {
      s_yTop = yKnown;
      portENTER_CRITICAL(&gMux);
      g_calibState = CALIB_X_TOP;     // complete this known side by finding X
      g_lastCmdMs  = now;
      portEXIT_CRITICAL(&gMux);
      Serial.printf("[CALIB] Side-start: Y Haut connu (y=%ld) -> recherche X Haut\n", yKnown);
    }
    return;
  }

  // hasX only
  if (knownIsBottom) {
    s_xBottom = xKnown;
    portENTER_CRITICAL(&gMux);
    g_xHallPos   = s_xBottom;
    g_calibState = CALIB_Y_BOTTOM;    // complete this known side by finding Y
    g_lastCmdMs  = now;
    portEXIT_CRITICAL(&gMux);
    Serial.printf("[CALIB] Side-start: X Bas connu (x=%ld) -> recherche Y Bas\n", xKnown);
  } else {
    s_xTop = xKnown;
    portENTER_CRITICAL(&gMux);
    g_calibState = CALIB_Y_TOP;       // complete this known side by finding Y
    g_lastCmdMs  = now;
    portEXIT_CRITICAL(&gMux);
    Serial.printf("[CALIB] Side-start: X Haut connu (x=%ld) -> recherche Y Haut\n", xKnown);
  }
}

void calibrationLoop(unsigned long now) {
  if (now - lastHallPollMs < CALIB_HALL_POLL_MS) return;
  lastHallPollMs = now;

  int stateY = digitalRead(HALL_Y_PIN);
  int stateX = digitalRead(HALL_X_PIN);

  bool detectedY = (stateY == LOW);
  bool detectedX = (stateX == LOW);

  CalibState st;
  long xNowAbs, yNowAbs;
  portENTER_CRITICAL(&gMux);
  st = g_calibState;
  xNowAbs = g_xAbs;
  yNowAbs = g_yAbs;
  portEXIT_CRITICAL(&gMux);

  if (stateY != lastHallYState) {
    int prev = lastHallYState;
    lastHallYState = stateY;

    if (prev == HIGH && stateY == LOW && st == CALIB_Y_BOTTOM) {
      s_yBottom = yNowAbs;

      portENTER_CRITICAL(&gMux);
      g_yHallPos = s_yBottom;
      g_calibState = CALIB_X_BOTTOM;
      g_lastCmdMs = now;
      portEXIT_CRITICAL(&gMux);

      Serial.print("[CALIB BAS] Y detecte a y=");
      Serial.println(s_yBottom);

      if (digitalRead(HALL_X_PIN) == LOW) {
        Serial.println("[CALIB BAS] Hall X encore LOW au changement d'etat, poursuite vers Haut pour sortir de la zone capteur.");
      }
    } else if (prev == HIGH && stateY == LOW && st == CALIB_Y_TOP) {
      s_yTop = yNowAbs;

      Serial.printf("[CALIB HAUT] Y detecte a y=%ld\n", s_yTop);
      if (digitalRead(HALL_X_PIN) == LOW) {
        // X is simultaneously LOW at the top corner — record X now and jump
        // straight to RECENTER.  Entering CALIB_X_TOP to wait for an edge that
        // already happened would spin forever.
        s_xTop = xNowAbs;
        long _yMin = min(s_yBottom, s_yTop);
        long _yMax = max(s_yBottom, s_yTop);
        long _xMin = min(s_xBottom, s_xTop);
        long _xMax = max(s_xBottom, s_xTop);
        long _yCenter = (_yMin + _yMax) / 2L;
        long _xCenter = (_xMin + _xMax) / 2L;
        portENTER_CRITICAL(&gMux);
        g_yHardMin = _yMin;  g_yHardMax = _yMax;
        g_xHardMin = _xMin;  g_xHardMax = _xMax;
        g_yCenterTarget = _yCenter;
        g_xCenterTarget = _xCenter;
        g_yLimitsCalibrated = true;
        g_xLimitsCalibrated = true;
        g_calibState = CALIB_RECENTER;
        g_recenter   = true;
        g_vx_xy = 0;  g_vy_xy = 0;
        g_lastCmdMs  = now;
        portEXIT_CRITICAL(&gMux);
        lastHallXState = LOW;
        Serial.printf("[CALIB HAUT] X simultane a x=%ld\n", s_xTop);
        Serial.printf("[CALIB] Coin Bas=(x=%ld,y=%ld) Coin Haut=(x=%ld,y=%ld)\n",
                      s_xBottom, s_yBottom, s_xTop, s_yTop);
        Serial.printf("[CALIB] Centre calcule=(x=%ld,y=%ld)\n", _xCenter, _yCenter);
        long oxAbs, oyAbs;
        portENTER_CRITICAL(&gMux);
        oxAbs = g_xCenterTarget;
        oyAbs = g_yCenterTarget;
        portEXIT_CRITICAL(&gMux);
        boardUpdateFromOrigin(oxAbs, oyAbs);
      } else {
        portENTER_CRITICAL(&gMux);
        g_calibState = CALIB_X_TOP;
        g_lastCmdMs  = now;
        portEXIT_CRITICAL(&gMux);
      }
    } else if (prev == LOW && stateY == HIGH && st == CALIB_X_BOTTOM) {
      // While searching X at the bottom corner, Y must stay latched on Hall Y.
      // If Y drops out, reacquire Y first, then return to X search.
      portENTER_CRITICAL(&gMux);
      g_calibState = CALIB_Y_BOTTOM;
      g_lastCmdMs = now;
      portEXIT_CRITICAL(&gMux);
      Serial.println("[CALIB BAS] Hall Y perdu pendant recherche X -> reacquisition Y");
    } else if (prev == LOW && stateY == HIGH && st == CALIB_X_TOP) {
      // Same protection on the top corner: keep Y detected while X is searching.
      portENTER_CRITICAL(&gMux);
      g_calibState = CALIB_Y_TOP;
      g_lastCmdMs = now;
      portEXIT_CRITICAL(&gMux);
      Serial.println("[CALIB HAUT] Hall Y perdu pendant recherche X -> reacquisition Y");
    }
  }

  if (stateX != lastHallXState) {
    int prev = lastHallXState;
    lastHallXState = stateX;

    if (prev == HIGH && stateX == LOW && st == CALIB_X_BOTTOM) {
      s_xBottom = xNowAbs;

      portENTER_CRITICAL(&gMux);
      g_xHallPos = s_xBottom;
      g_calibState = CALIB_Y_TOP;
      g_lastCmdMs = now;
      portEXIT_CRITICAL(&gMux);

      Serial.print("[CALIB BAS] X detecte a x=");
      Serial.println(s_xBottom);

      if (digitalRead(HALL_Y_PIN) == LOW) {
        Serial.println("[CALIB BAS] Hall Y encore LOW au changement d'etat, poursuite vers Haut pour sortir de la zone capteur.");
      }
    } else if (prev == HIGH && stateX == LOW && st == CALIB_X_TOP) {
      s_xTop = xNowAbs;

      long yMin = min(s_yBottom, s_yTop);
      long yMax = max(s_yBottom, s_yTop);
      long xMin = min(s_xBottom, s_xTop);
      long xMax = max(s_xBottom, s_xTop);
      long yCenter = (yMin + yMax) / 2L;
      long xCenter = (xMin + xMax) / 2L;

      portENTER_CRITICAL(&gMux);
      g_yHardMin = yMin;
      g_yHardMax = yMax;
      g_xHardMin = xMin;
      g_xHardMax = xMax;
      g_yCenterTarget = yCenter;
      g_xCenterTarget = xCenter;
      g_yLimitsCalibrated = true;
      g_xLimitsCalibrated = true;

      g_calibState = CALIB_RECENTER;
      g_recenter = true;
      g_vx_xy = 0; g_vy_xy = 0;
      g_lastCmdMs = now;
      portEXIT_CRITICAL(&gMux);

      Serial.printf("[CALIB HAUT] X detecte a x=%ld\n", s_xTop);
      Serial.printf("[CALIB] Coin Bas=(x=%ld,y=%ld) Coin Haut=(x=%ld,y=%ld)\n", s_xBottom, s_yBottom, s_xTop, s_yTop);
      Serial.printf("[CALIB] Centre calcule=(x=%ld,y=%ld)\n", xCenter, yCenter);

      long oxAbs, oyAbs;
      portENTER_CRITICAL(&gMux);
      oxAbs = g_xCenterTarget;
      oyAbs = g_yCenterTarget;
      portEXIT_CRITICAL(&gMux);

      boardUpdateFromOrigin(oxAbs, oyAbs);
    }
  }

  if (st == CALIB_RECENTER) {
    bool r;
    portENTER_CRITICAL(&gMux);
    r = g_recenter;
    portEXIT_CRITICAL(&gMux);
    if (!r) {
      portENTER_CRITICAL(&gMux);
      g_calibState = CALIB_IDLE;
      g_lastCmdMs = now;
      portEXIT_CRITICAL(&gMux);
      Serial.println("[CALIB] Done. Ready.");
    }
  }

  portENTER_CRITICAL(&gMux);
  g_hallYDetected = detectedY;
  g_hallXDetected = detectedX;
  portEXIT_CRITICAL(&gMux);
}