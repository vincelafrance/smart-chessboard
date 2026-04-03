#include "StepTask.h"
#include "MotionCoreXY.h"
#include "Utils.h"
#include "Magnet.h"
#include "PathPlanner.h"
#include "DriversUART.h"

static TaskHandle_t stepTaskHandle = nullptr;

static void stepTask(void *param) {
  (void)param;

  uint32_t lastA = micros();
  uint32_t lastB = micros();

  float vA_cur = 0.0f;
  float vB_cur = 0.0f;

  unsigned long settleStartMs = 0;
  float vmaxCur = 0.0f;
  uint32_t prevUsRamp = 0;
  static uint32_t prevUs = 0;

  for (;;) {
    uint32_t nowUs = micros();
    unsigned long nowMs = millis();

    long Apos, Bpos;
    portENTER_CRITICAL(&gMux);
    Apos = g_Apos;
    Bpos = g_Bpos;
    portEXIT_CRITICAL(&gMux);

    long xAbsRaw, yAbsRaw;
    getXYfromAB_raw(Apos, Bpos, xAbsRaw, yAbsRaw);

    CalibState calibState;
    float vx, vy;
    bool recenter;
    bool pathActive;
    long pathTx, pathTy;
    unsigned long lastCmd;

    portENTER_CRITICAL(&gMux);
    calibState = g_calibState;
    vx = g_vx_xy;
    vy = g_vy_xy;
    recenter = g_recenter;
    pathActive = g_pathActive;
    pathTx = g_pathTargetX;
    pathTy = g_pathTargetY;
    lastCmd = g_lastCmdMs;
    portEXIT_CRITICAL(&gMux);

    const bool calibY = (calibState == CALIB_Y);
    const bool calibX = (calibState == CALIB_X);

    long xMin, xMax, yMin, yMax;
    getXLimits(xMin, xMax);
    getYLimits(yMin, yMax);

    long xUseAbs, yUseAbs;
    if (calibY) {
      xUseAbs = clampl(xAbsRaw, xMin, xMax);
      yUseAbs = yAbsRaw;
    } else if (calibX) {
      xUseAbs = xAbsRaw;
      yUseAbs = clampl(yAbsRaw, yMin, yMax);
    } else {
      xUseAbs = clampl(xAbsRaw, xMin, xMax);
      yUseAbs = clampl(yAbsRaw, yMin, yMax);
    }

    portENTER_CRITICAL(&gMux);
    g_xAbs = xUseAbs;
    g_yAbs = yUseAbs;
    portEXIT_CRITICAL(&gMux);

    long xAbs = xUseAbs;
    long yAbs = yUseAbs;

    if (!calibY && !calibX && !recenter && !pathActive && (nowMs - lastCmd > CMD_TIMEOUT_MS)) {
      portENTER_CRITICAL(&gMux);
      g_vx_xy = 0;
      g_vy_xy = 0;
      portEXIT_CRITICAL(&gMux);
      vx = 0; vy = 0;
    }

    float vx_t = 0.0f, vy_t = 0.0f;

    if (calibY) {
      vx_t = 0.0f;
      vy_t = (float)CALIB_Y_DIR * CALIB_Y_SPEED;
      vmaxCur = 0.0f; prevUsRamp = 0; settleStartMs = 0;
    }
    else if (calibX) {
      vx_t = (float)CALIB_X_DIR * CALIB_X_SPEED;
      vy_t = 0.0f;
      vmaxCur = 0.0f; prevUsRamp = 0; settleStartMs = 0;
    }
    else if (pathActive) {
      long ex = pathTx - xAbs;
      long ey = pathTy - yAbs;

      float dtRamp = 0.001f;
      if (prevUsRamp != 0) {
        uint32_t du = (uint32_t)(nowUs - prevUsRamp);
        dtRamp = (float)du / 1000000.0f;
        if (dtRamp < 0.00005f) dtRamp = 0.00005f;
        if (dtRamp > 0.02f)    dtRamp = 0.02f;
      }
      prevUsRamp = nowUs;

      float rampMaxDelta = RECENTER_RAMP_ACC * dtRamp;
      vmaxCur = approachf(vmaxCur, RECENTER_VMAX_XY, rampMaxDelta);
      float vCap = clampf(vmaxCur, 400.0f, RECENTER_VMAX_XY);

      const bool xDone = (labs(ex) <= RECENTER_DEADBAND_XY);
      const bool yDone = (labs(ey) <= RECENTER_DEADBAND_XY);

      if (!xDone || !yDone) {
        if (!xDone && !yDone) {
          vCap *= 0.7f;
        }
        float vx_raw = RECENTER_KP * (float)ex;
        float vy_raw = RECENTER_KP * (float)ey;

        float maxAbs = fmaxf(fabsf(vx_raw), fabsf(vy_raw));
        float scale = 1.0f;
        if (maxAbs > vCap) scale = vCap / maxAbs;

        vx_t = vx_raw * scale;
        vy_t = vy_raw * scale;

        if (!xDone && fabsf(vx_t) < RECENTER_MIN_VXY) vx_t = (ex >= 0) ? RECENTER_MIN_VXY : -RECENTER_MIN_VXY;
        if (!yDone && fabsf(vy_t) < RECENTER_MIN_VXY) vy_t = (ey >= 0) ? RECENTER_MIN_VXY : -RECENTER_MIN_VXY;

        settleStartMs = 0;
      } else {
        if (settleStartMs == 0) settleStartMs = nowMs;

        if (nowMs - settleStartMs >= RECENTER_SETTLE_MS) {
          uint8_t arrivedIdx, cnt;
          bool autoMag;

          portENTER_CRITICAL(&gMux);
          arrivedIdx = g_wpIndex;
          cnt = g_wpCount;
          autoMag = g_autoMagnetPath;
          portEXIT_CRITICAL(&gMux);
if (autoMag) {
  // Waypoint-driven magnet control (supports multi-piece sequences, e.g. castling)
  int8_t magAction = -1;
  portENTER_CRITICAL(&gMux);
  if (arrivedIdx < g_wpCount) magAction = g_waypoints[arrivedIdx].mag;
  portEXIT_CRITICAL(&gMux);

  if (magAction == 1) {
    magnetSet(true);
    Serial.println("[MAGNET] AUTO ON at waypoint");
  } else if (magAction == 0) {
    magnetSet(false);
    Serial.println("[MAGNET] AUTO OFF at waypoint");
  }

  // Backwards-compat: if mag actions are not used (-1), keep old behavior
  if (magAction == -1) {
    if (arrivedIdx == 1) {
      magnetSet(true);
      Serial.println("[MAGNET] AUTO ON at FROM center");
    }
    if (cnt > 0 && arrivedIdx == (uint8_t)(cnt - 1)) {
      magnetSet(false);
      Serial.println("[MAGNET] AUTO OFF at TO center");
      portENTER_CRITICAL(&gMux);
      g_autoMagnetPath = false;
      portEXIT_CRITICAL(&gMux);
    }
  } else {
    // If we used waypoint actions, only auto-stop at the final waypoint.
    if (cnt > 0 && arrivedIdx == (uint8_t)(cnt - 1)) {
      portENTER_CRITICAL(&gMux);
      g_autoMagnetPath = false;
      portEXIT_CRITICAL(&gMux);
    }
  }
}portENTER_CRITICAL(&gMux);
          uint8_t idx = g_wpIndex;
          uint8_t c = g_wpCount;
          idx++;
          if (idx >= c) {
            g_pathActive = false;
            g_wpCount = 0;
            g_wpIndex = 0;
          } else {
            g_wpIndex = idx;
            g_pathTargetX = g_waypoints[idx].x;
            g_pathTargetY = g_waypoints[idx].y;
          }
          g_lastCmdMs = nowMs;
          portEXIT_CRITICAL(&gMux);

          settleStartMs = 0;
          vmaxCur = 0.0f;
          prevUsRamp = 0;
        }

        vx_t = 0.0f;
        vy_t = 0.0f;
      }
    }
    else if (recenter) {
      long xT, yT;
      portENTER_CRITICAL(&gMux);
      xT = g_xCenterTarget;
      yT = g_yCenterTarget;
      portEXIT_CRITICAL(&gMux);

      long ex = xT - xAbs;
      long ey = yT - yAbs;

      float dtRamp = 0.001f;
      if (prevUsRamp != 0) {
        uint32_t du = (uint32_t)(nowUs - prevUsRamp);
        dtRamp = (float)du / 1000000.0f;
        if (dtRamp < 0.00005f) dtRamp = 0.00005f;
        if (dtRamp > 0.02f)    dtRamp = 0.02f;
      }
      prevUsRamp = nowUs;

      float rampMaxDelta = RECENTER_RAMP_ACC * dtRamp;
      vmaxCur = approachf(vmaxCur, RECENTER_VMAX_XY, rampMaxDelta);
      float vCap = clampf(vmaxCur, 400.0f, RECENTER_VMAX_XY);

      const bool xDone = (labs(ex) <= RECENTER_DEADBAND_XY);
      const bool yDone = (labs(ey) <= RECENTER_DEADBAND_XY);

      if (!yDone) {
        vx_t = 0.0f;
        vy_t = clampf(RECENTER_KP * (float)ey, -vCap, +vCap);
        if (fabs(vy_t) > 0.0f && fabs(vy_t) < RECENTER_MIN_VXY) vy_t = (vy_t > 0) ? RECENTER_MIN_VXY : -RECENTER_MIN_VXY;
        settleStartMs = 0;
      } else if (!xDone) {
        vy_t = 0.0f;
        vx_t = clampf(RECENTER_KP * (float)ex, -vCap, +vCap);
        if (fabs(vx_t) > 0.0f && fabs(vx_t) < RECENTER_MIN_VXY) vx_t = (vx_t > 0) ? RECENTER_MIN_VXY : -RECENTER_MIN_VXY;
        settleStartMs = 0;
      } else {
        if (settleStartMs == 0) settleStartMs = nowMs;
        if (nowMs - settleStartMs >= RECENTER_SETTLE_MS) {
          portENTER_CRITICAL(&gMux);
          g_recenter = false;
          g_vx_xy = 0;
          g_vy_xy = 0;
          g_lastCmdMs = nowMs;
          portEXIT_CRITICAL(&gMux);

          settleStartMs = 0;
          vmaxCur = 0.0f;
          prevUsRamp = 0;
        }
        vx_t = 0.0f;
        vy_t = 0.0f;
      }
    }
    else {
      vx_t = vx;
      vy_t = vy;
      vmaxCur = 0.0f; prevUsRamp = 0; settleStartMs = 0;
    }

    long yMinUse, yMaxUse, xMinUse, xMaxUse;
    if (calibY) { yMinUse = -200000; yMaxUse = +200000; } else { getYLimits(yMinUse, yMaxUse); }
    if (calibX) { xMinUse = -200000; xMaxUse = +200000; } else { getXLimits(xMinUse, xMaxUse); }

    vx_t = applyEdgeLimit1D(vx_t, xAbs, xMinUse, xMaxUse);
    vy_t = applyEdgeLimit1D(vy_t, yAbs, yMinUse, yMaxUse);

    float vA_t, vB_t;
    xyToAB(vx_t, vy_t, vA_t, vB_t);

    float dt = 0.001f;
    if (prevUs != 0) {
      uint32_t du = (uint32_t)(nowUs - prevUs);
      dt = (float)du / 1000000.0f;
      if (dt < 0.00005f) dt = 0.00005f;
      if (dt > 0.02f)    dt = 0.02f;
    }
    prevUs = nowUs;

    float maxDv = ACCEL_AB * dt;
    vA_cur = approachf(vA_cur, vA_t, maxDv);
    vB_cur = approachf(vB_cur, vB_t, maxDv);

    uint8_t ms = getDriversUARTMicrosteps();
    if (ms == 0) ms = 8;
    const float motorScale = (float)ms / 8.0f;

    uint32_t pA = speedToPeriodUs(vA_cur * motorScale);
    uint32_t pB = speedToPeriodUs(vB_cur * motorScale);

    int8_t dirA = (vA_cur > 1.0f) ? +1 : (vA_cur < -1.0f) ? -1 : 0;
    int8_t dirB = (vB_cur > 1.0f) ? +1 : (vB_cur < -1.0f) ? -1 : 0;

    bool wantsMove = (pA != 0 || pB != 0) || recenter || calibY || calibX || pathActive;
    bool precisionMode = recenter || calibY || calibX || pathActive;

    serviceDriversUART(vA_cur, vB_cur, wantsMove, precisionMode);

    if (wantsMove) {
      driversOn();
      portENTER_CRITICAL(&gMux);
      markMotion();
      portEXIT_CRITICAL(&gMux);
    }

    if (DRIVERS_AUTO_DISABLE && !wantsMove) {
      unsigned long lastMot;
      portENTER_CRITICAL(&gMux);
      lastMot = g_lastMotionMs;
      portEXIT_CRITICAL(&gMux);
      if (g_driversEnabled && (nowMs - lastMot >= IDLE_DISABLE_MS)) driversOff();
    }

    if (dirA != 0) gpio_set_level((gpio_num_t)LEFT_DIR,  (dirA > 0) ? 1 : 0);
    if (dirB != 0) gpio_set_level((gpio_num_t)RIGHT_DIR, (dirB > 0) ? 1 : 0);

    if (pA != 0 && (uint32_t)(nowUs - lastA) >= pA) {
      lastA = nowUs;
      stepPulseFast((gpio_num_t)LEFT_STEP);
      portENTER_CRITICAL(&gMux);
      g_Apos += (dirA > 0 ? 1 : -1);
      portEXIT_CRITICAL(&gMux);
    }

    if (pB != 0 && (uint32_t)(nowUs - lastB) >= pB) {
      lastB = nowUs;
      stepPulseFast((gpio_num_t)RIGHT_STEP);
      portENTER_CRITICAL(&gMux);
      g_Bpos += (dirB > 0 ? 1 : -1);
      portEXIT_CRITICAL(&gMux);
    }

    if (!wantsMove) vTaskDelay(1);
    else taskYIELD();
  }
}

void stepTaskStart() {
  xTaskCreatePinnedToCore(stepTask, "stepTask", 4096, nullptr, 3, &stepTaskHandle, 0);
}