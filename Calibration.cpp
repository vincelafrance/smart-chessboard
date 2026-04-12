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

      portENTER_CRITICAL(&gMux);
      g_calibState = CALIB_X_TOP;
      g_lastCmdMs = now;
      portEXIT_CRITICAL(&gMux);

      Serial.print("[CALIB HAUT] Y detecte a y=");
      Serial.println(s_yTop);

      if (digitalRead(HALL_X_PIN) == LOW) {
        Serial.println("[CALIB HAUT] Refuse: hall X deja DETECTED. Move away and retry.");
        portENTER_CRITICAL(&gMux);
        g_calibState = CALIB_IDLE;
        g_vx_xy = 0; g_vy_xy = 0;
        g_recenter = false;
        g_lastCmdMs = now;
        portEXIT_CRITICAL(&gMux);
      }
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