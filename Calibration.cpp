#include "Calibration.h"
#include "Utils.h"
#include "MotionCoreXY.h"
#include "PathPlanner.h"
#include "BoardMapping.h"

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
  g_calibState = CALIB_Y;
  g_lastCmdMs = now;
  portEXIT_CRITICAL(&gMux);

  Serial.println("[CALIB] Full sequence started: Y -> X -> Recenter");
}

void calibrationLoop(unsigned long now) {
  if (now - lastHallPollMs < 10) return;
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

    if (st == CALIB_Y && prev == HIGH && stateY == LOW) {
      long hallY = yNowAbs;

      long newMin, newMax, newCenter;
      if (HALL_AT_Y_MIN) {
        newMin = hallY;
        newMax = hallY + Y_SPAN_STEPS;
        newCenter = hallY + (Y_SPAN_STEPS / 2);
      } else {
        newMax = hallY;
        newMin = hallY - Y_SPAN_STEPS;
        newCenter = hallY - (Y_SPAN_STEPS / 2);
      }

      portENTER_CRITICAL(&gMux);
      g_yHallPos = hallY;
      g_yHardMin = newMin;
      g_yHardMax = newMax;
      g_yCenterTarget = newCenter;
      g_yLimitsCalibrated = true;

      g_calibState = CALIB_X;
      g_lastCmdMs = now;
      portEXIT_CRITICAL(&gMux);

      Serial.print("[Y CAL] hallY=");
      Serial.print(hallY);
      Serial.print(" -> ymin=");
      Serial.print(newMin);
      Serial.print(" ymax=");
      Serial.print(newMax);
      Serial.print(" centerTarget=");
      Serial.println(newCenter);

      if (digitalRead(HALL_X_PIN) == LOW) {
        Serial.println("[X CAL] Refused mid-sequence: hall X already DETECTED. Move away and retry.");
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

    if (st == CALIB_X && prev == HIGH && stateX == LOW) {
      long hallX = xNowAbs;

      long newMin, newMax, newCenter;
      if (HALL_AT_X_MIN) {
        newMin = hallX;
        newMax = hallX + X_SPAN_STEPS;
        newCenter = hallX + (X_SPAN_STEPS / 2);
      } else {
        newMax = hallX;
        newMin = hallX - X_SPAN_STEPS;
        newCenter = hallX - (X_SPAN_STEPS / 2);
      }

      portENTER_CRITICAL(&gMux);
      g_xHallPos = hallX;
      g_xHardMin = newMin;
      g_xHardMax = newMax;
      g_xCenterTarget = newCenter;
      g_xLimitsCalibrated = true;

      g_calibState = CALIB_RECENTER;
      g_recenter = true;
      g_vx_xy = 0; g_vy_xy = 0;
      g_lastCmdMs = now;
      portEXIT_CRITICAL(&gMux);

      Serial.print("[X CAL] hallX=");
      Serial.print(hallX);
      Serial.print(" -> xmin=");
      Serial.print(newMin);
      Serial.print(" xmax=");
      Serial.print(newMax);
      Serial.print(" centerTarget=");
      Serial.println(newCenter);

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