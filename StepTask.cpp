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
  uint32_t prevUs = lastA;

  float vA_cur = 0.0f;
  float vB_cur = 0.0f;

  unsigned long settleStartMs = 0;
  bool prevCalibY = false;
  bool prevCalibX = false;
  bool prevRecenter = false;
  static bool pickupWaitActive = false;
  static unsigned long pickupWaitStartMs = 0;
  static uint8_t pickupWaitWpIdx = 255;

  for (;;) {
    uint32_t nowUs = micros();
    unsigned long nowMs = millis();

    float dt = 0.001f;
    if (prevUs != 0) {
      uint32_t du = (uint32_t)(nowUs - prevUs);
      dt = (float)du / 1000000.0f;
      if (dt < 0.00005f) dt = 0.00005f;
      if (dt > 0.01f)    dt = 0.01f;
    }
    prevUs = nowUs;

    long Apos, Bpos;
    portENTER_CRITICAL(&gMux);
    Apos = g_Apos;
    Bpos = g_Bpos;
    portEXIT_CRITICAL(&gMux);

    long xAbsRaw, yAbsRaw;
    getXYfromAB_raw(Apos, Bpos, xAbsRaw, yAbsRaw);

    CalibState calibState;
    float vx, vy;
    float overrideAccel;
    bool recenter;
    bool pathActive;
    long pathTx, pathTy;
    unsigned long lastCmd;

    portENTER_CRITICAL(&gMux);
    calibState = g_calibState;
    vx = g_vx_xy;
    vy = g_vy_xy;
    overrideAccel = g_overrideAccel;
    recenter = g_recenter;
    pathActive = g_pathActive;
    pathTx = g_pathTargetX;
    pathTy = g_pathTargetY;
    lastCmd = g_lastCmdMs;
    portEXIT_CRITICAL(&gMux);

    const bool calibY = (calibState == CALIB_Y_BOTTOM || calibState == CALIB_Y_TOP);
    const bool calibX = (calibState == CALIB_X_BOTTOM || calibState == CALIB_X_TOP);

    // When calibration hands off to recenter, discard any residual motor
    // velocity from the previous seek phase so the carriage starts the
    // Y-then-X recenter cleanly instead of briefly blending into a diagonal.
    if (recenter && !prevRecenter && (prevCalibY || prevCalibX)) {
      vA_cur = 0.0f;
      vB_cur = 0.0f;
      settleStartMs = 0;
    }

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
      float dirY = (calibState == CALIB_Y_TOP) ? -(float)CALIB_Y_DIR : (float)CALIB_Y_DIR;
      vy_t = dirY * CALIB_Y_SPEED;
      settleStartMs = 0;
    }
    else if (calibX) {
      // Appliquer un taper sur la vitesse minimale et la capabilité près du centre pour éviter les coups.
      long xTarget = (calibState == CALIB_X_TOP) ? xMax : xMin;
      long ex = xTarget - xAbs;
      long exAbs = labs(ex);
      float dirX = (calibState == CALIB_X_TOP) ? -(float)CALIB_X_DIR : (float)CALIB_X_DIR;
      float vCapX = fmaxf(180.0f, CALIB_X_SPEED * 1.0f); // vCapX = CALIB_X_SPEED, mais on garde la structure pour le taper
      float minVX = CALIB_X_SPEED * 0.5f; // minVX = 50% de la vitesse de calibration

      // Taper près du centre (même logique que recentrage)
      if (exAbs < 120) {
        float t = clampf((float)exAbs / 120.0f, 0.0f, 1.0f);
        const float minVXNear = 40.0f; // plus doux que recentrage
        minVX = minVXNear + (minVX - minVXNear) * t;
        vCapX = fmaxf(100.0f, vCapX * (0.45f + 0.55f * t));
      }

      vx_t = clampf(dirX * CALIB_X_SPEED, -vCapX, +vCapX);
      if (fabs(vx_t) > 0.0f && fabs(vx_t) < minVX && exAbs > 2 * RECENTER_DEADBAND_XY) {
        vx_t = (vx_t > 0) ? minVX : -minVX;
      }
      vy_t = 0.0f;
      settleStartMs = 0;
    }
    else if (pathActive) {
      long ex = pathTx - xAbs;
      long ey = pathTy - yAbs;

      // AutoTune may inject a speed override; 0 means use firmware default.
      float pathVmax = (g_overrideVmax > 0.0f) ? g_overrideVmax : PATH_VMAX_XY;

      // Blend between axis speed and diagonal speed based on movement direction.
      // In CoreXY a 45° XY diagonal = one motor at √2× carriage speed (other idle).
      // diagFrac: 0.0 = pure axis (both motors equal), 1.0 = pure 45° diagonal (one motor).
      // Applies only when both overrides are set to distinct values.
      if (g_overrideVmax > 0.0f && g_overrideDiagVmax > 0.0f &&
          g_overrideDiagVmax != g_overrideVmax) {
          float ex_f = (float)(pathTx - xAbs);
          float ey_f = (float)(pathTy - yAbs);
          float eMag = fmaxf(fabsf(ex_f), fabsf(ey_f));
          if (eMag > 50.0f) {
              // Motor A speed ∝ |ex+ey|, motor B speed ∝ |ex-ey|
              float mA = fabsf(ex_f + ey_f);
              float mB = fabsf(ex_f - ey_f);
              float hi = fmaxf(mA, mB);
              float lo = fminf(mA, mB);
              float sum = hi + lo;
              float diagFrac = (sum > 0.0f) ? ((hi - lo) / sum) : 0.0f;
              pathVmax = g_overrideVmax + diagFrac * (g_overrideDiagVmax - g_overrideVmax);
          }
      }
      float pathMinV = PATH_MIN_VXY;
      float vCap = pathVmax;

      const bool xDone = (labs(ex) <= RECENTER_DEADBAND_XY);
      const bool yDone = (labs(ey) <= RECENTER_DEADBAND_XY);

      if (!xDone || !yDone) {
        float minPathV = pathMinV;
        const float errMag = fmaxf(fabsf((float)ex), fabsf((float)ey));
        if (!xDone && !yDone) {
          // Only soften diagonals close to the target; keep long traversals snappy.
          if (errMag < 220.0f) {
            vCap *= 0.55f;
            minPathV *= 0.65f;
          }
        }

        // Taper minimum speed near target to reduce end-of-diagonal bounce.
        if (errMag < 80.0f) {
          float t = clampf(errMag / 80.0f, 0.0f, 1.0f);
          const float minNear = 120.0f;
          minPathV = minNear + (minPathV - minNear) * t;
        }

        float vx_raw = PATH_KP * (float)ex;
        float vy_raw = PATH_KP * (float)ey;

        float maxAbs = fmaxf(fabsf(vx_raw), fabsf(vy_raw));
        float scale = 1.0f;
        if (maxAbs > vCap) scale = vCap / maxAbs;

        vx_t = vx_raw * scale;
        vy_t = vy_raw * scale;

        const bool xVeryNear = (labs(ex) <= RECENTER_DEADBAND_XY);
        const bool yVeryNear = (labs(ey) <= RECENTER_DEADBAND_XY);

        if (!xDone && !xVeryNear && fabsf(vx_t) < minPathV) vx_t = (ex >= 0) ? minPathV : -minPathV;
        if (!yDone && !yVeryNear && fabsf(vy_t) < minPathV) vy_t = (ey >= 0) ? minPathV : -minPathV;

        settleStartMs = 0;
      } else {
        uint8_t arrivedIdx = 0, cnt = 0;
        bool autoMag = false;
        int8_t magAction = -1;

        portENTER_CRITICAL(&gMux);
        arrivedIdx = g_wpIndex;
        cnt = g_wpCount;
        autoMag = g_autoMagnetPath;
        if (arrivedIdx < g_wpCount) magAction = g_waypoints[arrivedIdx].mag;
        portEXIT_CRITICAL(&gMux);

        const bool isFinalWp = (cnt > 0 && arrivedIdx == (uint8_t)(cnt - 1));
        bool needsSettle = isFinalWp;

        if (autoMag) {
          if (magAction == 1 || magAction == 0) {
            needsSettle = true;
          } else if (magAction == -1 && arrivedIdx == 1) {
            // Backwards-compat: old path format picks up at waypoint #1.
            needsSettle = true;
          }
        }

        // Intermediate waypoints without actions are passed continuously.
        if (!needsSettle) {
          portENTER_CRITICAL(&gMux);
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
          continue;
        }

        if (settleStartMs == 0) settleStartMs = nowMs;

        if (nowMs - settleStartMs >= PATH_SETTLE_MS) {
          bool needPickupDelay = false;

          if (autoMag) {
            // Waypoint-driven magnet control (supports multi-piece sequences, e.g. castling)
            if (magAction == 1) {
              magnetSet(true);
              Serial.println("[MAGNET] AUTO ON at waypoint");
              needPickupDelay = true;
            } else if (magAction == 0) {
              magnetSet(false);
              Serial.println("[MAGNET] AUTO OFF at waypoint");
            }

            // Backwards-compat: if mag actions are not used (-1), keep old behavior
            if (magAction == -1) {
              if (arrivedIdx == 1) {
                magnetSet(true);
                Serial.println("[MAGNET] AUTO ON at FROM center");
                needPickupDelay = true;
              }
              if (isFinalWp) {
                magnetSet(false);
                Serial.println("[MAGNET] AUTO OFF at TO center");
                portENTER_CRITICAL(&gMux);
                g_autoMagnetPath = false;
                portEXIT_CRITICAL(&gMux);
              }
            } else {
              // If we used waypoint actions, only auto-stop at the final waypoint.
              if (isFinalWp) {
                portENTER_CRITICAL(&gMux);
                g_autoMagnetPath = false;
                portEXIT_CRITICAL(&gMux);
              }
            }

            // Hold briefly after a pickup to let the piece fully attach before moving away.
            if (needPickupDelay) {
              if (!pickupWaitActive || pickupWaitWpIdx != arrivedIdx) {
                pickupWaitActive = true;
                pickupWaitWpIdx = arrivedIdx;
                pickupWaitStartMs = nowMs;
              }
              if ((nowMs - pickupWaitStartMs) < PATH_PICKUP_SETTLE_MS) {
                vx_t = 0.0f;
                vy_t = 0.0f;
                continue;
              }
              pickupWaitActive = false;
              pickupWaitWpIdx = 255;
            } else if (pickupWaitActive && pickupWaitWpIdx == arrivedIdx) {
              pickupWaitActive = false;
              pickupWaitWpIdx = 255;
            }
          }

          portENTER_CRITICAL(&gMux);
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

      // AutoTune may inject a speed override; 0 means use firmware default.
      float activeVmax = (g_overrideVmax > 0.0f) ? g_overrideVmax : RECENTER_VMAX_XY;
      float vCap = activeVmax;

      const bool xDone = (labs(ex) <= RECENTER_DEADBAND_XY);
      const bool yDone = (labs(ey) <= RECENTER_DEADBAND_XY);

      if (!yDone) {
        vx_t = 0.0f;
        vy_t = clampf(RECENTER_KP * (float)ey, -vCap, +vCap);
        if (fabs(vy_t) > 0.0f && fabs(vy_t) < RECENTER_MIN_VXY) vy_t = (vy_t > 0) ? RECENTER_MIN_VXY : -RECENTER_MIN_VXY;
        settleStartMs = 0;
      } else if (!xDone) {
        vy_t = 0.0f;
        long exAbs = labs(ex);
        float vCapX = fmaxf(140.0f, vCap * 0.42f);
        float minVX = 110.0f;
        float kpX = 1.15f;

        // Soften the Y->X handoff and the final X approach. On CoreXY the X-only
        // segment asks one motor to reverse while the other keeps moving, which
        // is where the carriage most easily feels "kicky".
        if (exAbs < 220) {
          float t = clampf((float)exAbs / 220.0f, 0.0f, 1.0f);
          const float minVXNear = 45.0f;
          minVX = minVXNear + (minVX - minVXNear) * t;
          vCapX = fmaxf(90.0f, vCapX * (0.30f + 0.70f * t));
          kpX = 0.70f + (kpX - 0.70f) * t;
        }

        vx_t = clampf(kpX * (float)ex, -vCapX, +vCapX);
        if (fabs(vx_t) > 0.0f && fabs(vx_t) < minVX && exAbs > 80) {
          vx_t = (vx_t > 0) ? minVX : -minVX;
        }
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
        }
        vx_t = 0.0f;
        vy_t = 0.0f;
      }
    }
    else {
      vx_t = vx;
      vy_t = vy;
      settleStartMs = 0;
    }

    long yMinUse, yMaxUse, xMinUse, xMaxUse;
    if (calibY) { yMinUse = -200000; yMaxUse = +200000; } else { getYLimits(yMinUse, yMaxUse); }
    if (calibX) { xMinUse = -200000; xMaxUse = +200000; } else { getXLimits(xMinUse, xMaxUse); }

    vx_t = applyEdgeLimit1D(vx_t, xAbs, xMinUse, xMaxUse);
    vy_t = applyEdgeLimit1D(vy_t, yAbs, yMinUse, yMaxUse);

    float vA_t, vB_t;
    xyToAB(vx_t, vy_t, vA_t, vB_t);


    float rampUpRate = MOTION_RAMP_UP_AB;
    if (overrideAccel > 0.0f) {
      // AutoTune path/recenter uses overrideAccel; keep launches smoother than
      // manual motion while still remaining responsive.
      rampUpRate = clampf(overrideAccel * 3.5f, 22000.0f, MOTION_RAMP_UP_AB);
    }
    float rampDownRate = MOTION_RAMP_DOWN_AB;
    if (g_overrideDecel > 0.0f) {
      rampDownRate = clampf(g_overrideDecel * 3.5f, 22000.0f, MOTION_RAMP_DOWN_AB);
    }

    float rampStopRate = MOTION_RAMP_STOP_AB;
    if (g_overrideDecel > 0.0f) {
      // When AutoTune supplies a decel override, apply it to full stops too
      // instead of letting the dedicated stop ramp hide the tuned behavior.
      rampStopRate = clampf(g_overrideDecel * 3.5f, 22000.0f, MOTION_RAMP_STOP_AB);
    }

    const float accelUp = rampUpRate * dt;
    const float accelDown = rampDownRate * dt;
    const float accelStop = rampStopRate * dt;

    float maxDeltaA = (fabsf(vA_t) <= 1.0f) ? accelStop :
              ((fabsf(vA_t) >= fabsf(vA_cur) || (vA_cur * vA_t) < 0.0f) ? accelUp : accelDown);
    float maxDeltaB = (fabsf(vB_t) <= 1.0f) ? accelStop :
              ((fabsf(vB_t) >= fabsf(vB_cur) || (vB_cur * vB_t) < 0.0f) ? accelUp : accelDown);

    vA_cur = approachf(vA_cur, vA_t, maxDeltaA);
    vB_cur = approachf(vB_cur, vB_t, maxDeltaB);

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

    prevCalibY = calibY;
    prevCalibX = calibX;
    prevRecenter = recenter;

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
