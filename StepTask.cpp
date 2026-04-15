#include "StepTask.h"
#include "StepGen.h"
#include "MotionCoreXY.h"
#include "Utils.h"
#include "Magnet.h"
#include "PathPlanner.h"
#include "DriversUART.h"

static TaskHandle_t stepTaskHandle = nullptr;

static void stepTask(void *param) {
  (void)param;

  // Fixed 1 ms timestep — step timing is now handled entirely by hardware
  // timers (StepGen).  The task only computes velocity and acceleration.
  const float dt = 0.001f;

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
    unsigned long nowMs = millis();

    // -----------------------------------------------------------------------
    // Read current motor positions (updated by StepGen ISR).
    // -----------------------------------------------------------------------
    long Apos, Bpos;
    portENTER_CRITICAL(&gMux);
    Apos = g_Apos;
    Bpos = g_Bpos;
    portEXIT_CRITICAL(&gMux);

    long xAbsRaw, yAbsRaw;
    getXYfromAB_raw(Apos, Bpos, xAbsRaw, yAbsRaw);

    // -----------------------------------------------------------------------
    // Snapshot shared state.
    // -----------------------------------------------------------------------
    CalibState calibState;
    float vx, vy;
    float overrideAccel;
    bool recenter;
    bool pathActive;
    long pathTx, pathTy;
    unsigned long lastCmd;

    portENTER_CRITICAL(&gMux);
    calibState    = g_calibState;
    vx            = g_vx_xy;
    vy            = g_vy_xy;
    overrideAccel = g_overrideAccel;
    recenter      = g_recenter;
    pathActive    = g_pathActive;
    pathTx        = g_pathTargetX;
    pathTy        = g_pathTargetY;
    lastCmd       = g_lastCmdMs;
    portEXIT_CRITICAL(&gMux);

    const bool calibY = (calibState == CALIB_Y_BOTTOM || calibState == CALIB_Y_TOP);
    const bool calibX = (calibState == CALIB_X_BOTTOM || calibState == CALIB_X_TOP);

    // When calibration hands off to recenter, discard residual motor velocity
    // so the recenter starts cleanly instead of blending with the seek phase.
    if (recenter && !prevRecenter && (prevCalibY || prevCalibX)) {
      vA_cur = 0.0f;
      vB_cur = 0.0f;
      settleStartMs = 0;
    }

    // -----------------------------------------------------------------------
    // Clamp position to soft limits (calibY/X bypass their own axis).
    // -----------------------------------------------------------------------
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

    // -----------------------------------------------------------------------
    // Command timeout — zero joystick velocity when idle.
    // -----------------------------------------------------------------------
    if (!calibY && !calibX && !recenter && !pathActive && (nowMs - lastCmd > CMD_TIMEOUT_MS)) {
      portENTER_CRITICAL(&gMux);
      g_vx_xy = 0;
      g_vy_xy = 0;
      portEXIT_CRITICAL(&gMux);
      vx = 0; vy = 0;
    }

    // -----------------------------------------------------------------------
    // Compute XY velocity target.
    // -----------------------------------------------------------------------
    float vx_t = 0.0f, vy_t = 0.0f;

    if (calibY) {
      vx_t = 0.0f;
      float dirY = (calibState == CALIB_Y_TOP) ? -(float)CALIB_Y_DIR : (float)CALIB_Y_DIR;
      vy_t = dirY * CALIB_Y_SPEED;
      settleStartMs = 0;
    }
    else if (calibX) {
      long xTarget = (calibState == CALIB_X_TOP) ? xMax : xMin;
      long ex = xTarget - xAbs;
      long exAbs = labs(ex);
      float dirX = (calibState == CALIB_X_TOP) ? -(float)CALIB_X_DIR : (float)CALIB_X_DIR;
      float vCapX = fmaxf(180.0f, CALIB_X_SPEED * 1.0f);
      float minVX = CALIB_X_SPEED * 0.5f;

      if (exAbs < 120) {
        float t = clampf((float)exAbs / 120.0f, 0.0f, 1.0f);
        const float minVXNear = 40.0f;
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

      // Determine whether the current waypoint needs a full settle (magnet
      // action or final destination) or is a fly-through corner.
      bool wpNeedsSettle;
      {
        uint8_t wpIdx2 = 0, wpCnt2 = 0;
        bool autoMag2 = false;
        int8_t magAct2 = -1;
        portENTER_CRITICAL(&gMux);
        wpIdx2   = g_wpIndex;
        wpCnt2   = g_wpCount;
        autoMag2 = g_autoMagnetPath;
        if (wpIdx2 < wpCnt2) magAct2 = g_waypoints[wpIdx2].mag;
        portEXIT_CRITICAL(&gMux);

        const bool isFinal2 = (wpCnt2 > 0 && wpIdx2 == (uint8_t)(wpCnt2 - 1));
        wpNeedsSettle = isFinal2;
        if (autoMag2) {
          if (magAct2 == 1 || magAct2 == 0) wpNeedsSettle = true;
          else if (magAct2 == -1 && wpIdx2 == 1) wpNeedsSettle = true;
        }
      }

      // AutoTune may inject a speed override; 0 means use firmware default.
      float pathVmax = (g_overrideVmax > 0.0f) ? g_overrideVmax : PATH_VMAX_XY;

      // Blend axis / diagonal speed based on movement direction.
      if (g_overrideVmax > 0.0f && g_overrideDiagVmax > 0.0f &&
          g_overrideDiagVmax != g_overrideVmax) {
        float ex_f = (float)(pathTx - xAbs);
        float ey_f = (float)(pathTy - yAbs);
        float eMag = fmaxf(fabsf(ex_f), fabsf(ey_f));
        if (eMag > 50.0f) {
          float mA = fabsf(ex_f + ey_f);
          float mB = fabsf(ex_f - ey_f);
          float hi  = fmaxf(mA, mB);
          float lo  = fminf(mA, mB);
          float sum = hi + lo;
          float diagFrac = (sum > 0.0f) ? ((hi - lo) / sum) : 0.0f;
          pathVmax = g_overrideVmax + diagFrac * (g_overrideDiagVmax - g_overrideVmax);
        }
      }
      float pathMinV = PATH_MIN_VXY;
      float vCap = pathVmax;

      // Fly-through waypoints use a wider deadband.
      const long wpDeadband = wpNeedsSettle ? (long)RECENTER_DEADBAND_XY : 120L;
      const bool xDone = (labs(ex) <= wpDeadband);
      const bool yDone = (labs(ey) <= wpDeadband);

      if (!xDone || !yDone) {
        float vx_raw = PATH_KP * (float)ex;
        float vy_raw = PATH_KP * (float)ey;

        float maxAbs = fmaxf(fabsf(vx_raw), fabsf(vy_raw));
        float scale  = 1.0f;
        if (maxAbs > vCap) scale = vCap / maxAbs;

        vx_t = vx_raw * scale;
        vy_t = vy_raw * scale;

        const bool xVeryNear = (labs(ex) <= RECENTER_DEADBAND_XY);
        const bool yVeryNear = (labs(ey) <= RECENTER_DEADBAND_XY);

        if (!xDone && !xVeryNear && fabsf(vx_t) < pathMinV) vx_t = (ex >= 0) ? pathMinV : -pathMinV;
        if (!yDone && !yVeryNear && fabsf(vy_t) < pathMinV) vy_t = (ey >= 0) ? pathMinV : -pathMinV;

        settleStartMs = 0;
      } else {
        uint8_t arrivedIdx = 0, cnt = 0;
        bool autoMag = false;
        int8_t magAction = -1;

        portENTER_CRITICAL(&gMux);
        arrivedIdx = g_wpIndex;
        cnt        = g_wpCount;
        autoMag    = g_autoMagnetPath;
        if (arrivedIdx < g_wpCount) magAction = g_waypoints[arrivedIdx].mag;
        portEXIT_CRITICAL(&gMux);

        const bool isFinalWp = (cnt > 0 && arrivedIdx == (uint8_t)(cnt - 1));
        bool needsSettle = isFinalWp;

        if (autoMag) {
          if (magAction == 1 || magAction == 0) {
            needsSettle = true;
          } else if (magAction == -1 && arrivedIdx == 1) {
            needsSettle = true;
          }
        }

        // Fly-through: advance waypoint and let the hardware timers keep the
        // carriage moving at its current rate for the 1 ms until the next
        // velocity update picks up the new target.
        if (!needsSettle) {
          portENTER_CRITICAL(&gMux);
          uint8_t idx = g_wpIndex;
          uint8_t c   = g_wpCount;
          idx++;
          if (idx >= c) {
            g_pathActive = false;
            g_wpCount    = 0;
            g_wpIndex    = 0;
            g_dzPathYExpanded = false;
          } else {
            g_wpIndex      = idx;
            g_pathTargetX  = g_waypoints[idx].x;
            g_pathTargetY  = g_waypoints[idx].y;
          }
          g_lastCmdMs = nowMs;
          portEXIT_CRITICAL(&gMux);

          settleStartMs = 0;
          vTaskDelay(1);
          continue;
        }

        // --- Magnet actions at settle waypoints ---
        bool needPickupDelay = false;

        if (autoMag) {
          if (magAction == 1) {
            magnetSet(true);
            Serial.println("[MAGNET] AUTO ON at waypoint");
            needPickupDelay = true;
          } else if (magAction == 0) {
            magnetSet(false);
            Serial.println("[MAGNET] AUTO OFF at waypoint");
          }

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
            if (isFinalWp) {
              portENTER_CRITICAL(&gMux);
              g_autoMagnetPath = false;
              portEXIT_CRITICAL(&gMux);
            }
          }

          // Hold briefly after pickup to let the piece attach before moving.
          if (needPickupDelay) {
            if (!pickupWaitActive || pickupWaitWpIdx != arrivedIdx) {
              pickupWaitActive    = true;
              pickupWaitWpIdx     = arrivedIdx;
              pickupWaitStartMs   = nowMs;
            }
            if ((nowMs - pickupWaitStartMs) < PATH_PICKUP_SETTLE_MS) {
              // Stop the motors during the settle delay.
              stepGenSetA(0, +1);
              stepGenSetB(0, +1);
              vTaskDelay(1);
              continue;
            }
            pickupWaitActive = false;
            pickupWaitWpIdx  = 255;
          } else if (pickupWaitActive && pickupWaitWpIdx == arrivedIdx) {
            pickupWaitActive = false;
            pickupWaitWpIdx  = 255;
          }
        }

        portENTER_CRITICAL(&gMux);
        uint8_t idx = g_wpIndex;
        uint8_t c   = g_wpCount;
        idx++;
        if (idx >= c) {
          g_pathActive = false;
          g_wpCount    = 0;
          g_wpIndex    = 0;
          g_dzPathYExpanded = false;
        } else {
          g_wpIndex      = idx;
          g_pathTargetX  = g_waypoints[idx].x;
          g_pathTargetY  = g_waypoints[idx].y;
        }
        g_lastCmdMs = nowMs;
        portEXIT_CRITICAL(&gMux);

        settleStartMs = 0;
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
        float kpX   = 1.15f;

        if (exAbs < 220) {
          float t = clampf((float)exAbs / 220.0f, 0.0f, 1.0f);
          const float minVXNear = 45.0f;
          minVX = minVXNear + (minVX - minVXNear) * t;
          vCapX = fmaxf(90.0f, vCapX * (0.30f + 0.70f * t));
          kpX   = 0.70f + (kpX - 0.70f) * t;
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
          g_vx_xy    = 0;
          g_vy_xy    = 0;
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

    // -----------------------------------------------------------------------
    // Edge soft limits.
    // -----------------------------------------------------------------------
    long yMinUse, yMaxUse, xMinUse, xMaxUse;
    if (calibY) { yMinUse = -200000; yMaxUse = +200000; } else { getYLimits(yMinUse, yMaxUse); }
    if (calibX) { xMinUse = -200000; xMaxUse = +200000; } else { getXLimits(xMinUse, xMaxUse); }

    vx_t = applyEdgeLimit1D(vx_t, xAbs, xMinUse, xMaxUse);
    vy_t = applyEdgeLimit1D(vy_t, yAbs, yMinUse, yMaxUse);

    // -----------------------------------------------------------------------
    // CoreXY transform + acceleration ramp.
    // -----------------------------------------------------------------------
    float vA_t, vB_t;
    xyToAB(vx_t, vy_t, vA_t, vB_t);

    float rampUpRate = MOTION_RAMP_UP_AB;
    if (overrideAccel > 0.0f) {
      rampUpRate = clampf(overrideAccel * 3.5f, 22000.0f, MOTION_RAMP_UP_AB);
    }
    float rampDownRate = MOTION_RAMP_DOWN_AB;
    if (g_overrideDecel > 0.0f) {
      rampDownRate = clampf(g_overrideDecel * 3.5f, 22000.0f, MOTION_RAMP_DOWN_AB);
    }
    float rampStopRate = MOTION_RAMP_STOP_AB;
    if (g_overrideDecel > 0.0f) {
      rampStopRate = clampf(g_overrideDecel * 3.5f, 22000.0f, MOTION_RAMP_STOP_AB);
    }

    const float accelUp   = rampUpRate   * dt;
    const float accelDown = rampDownRate * dt;
    const float accelStop = rampStopRate * dt;

    float maxDeltaA = (fabsf(vA_t) <= 1.0f) ? accelStop :
              ((fabsf(vA_t) >= fabsf(vA_cur) || (vA_cur * vA_t) < 0.0f) ? accelUp : accelDown);
    float maxDeltaB = (fabsf(vB_t) <= 1.0f) ? accelStop :
              ((fabsf(vB_t) >= fabsf(vB_cur) || (vB_cur * vB_t) < 0.0f) ? accelUp : accelDown);

    vA_cur = approachf(vA_cur, vA_t, maxDeltaA);
    vB_cur = approachf(vB_cur, vB_t, maxDeltaB);

    // -----------------------------------------------------------------------
    // Convert velocity to step period and direction.
    // -----------------------------------------------------------------------
    uint8_t ms = getDriversUARTMicrosteps();
    if (ms == 0) ms = 8;
    const float motorScale = (float)ms / 8.0f;

    uint32_t pA = speedToPeriodUs(vA_cur * motorScale);
    uint32_t pB = speedToPeriodUs(vB_cur * motorScale);

    int8_t dirA = (vA_cur > 1.0f) ? +1 : (vA_cur < -1.0f) ? -1 : 0;
    int8_t dirB = (vB_cur > 1.0f) ? +1 : (vB_cur < -1.0f) ? -1 : 0;

    // -----------------------------------------------------------------------
    // Driver management.
    // -----------------------------------------------------------------------
    bool wantsMove    = (pA != 0 || pB != 0) || recenter || calibY || calibX || pathActive;
    bool precisionMode = recenter || calibY || calibX || pathActive;

    serviceDriversUART(vA_cur, vB_cur, wantsMove, precisionMode);

    if (wantsMove) {
      driversOn();
      portENTER_CRITICAL(&gMux);
      markMotion();
      portEXIT_CRITICAL(&gMux);
    }

    prevCalibY   = calibY;
    prevCalibX   = calibX;
    prevRecenter = recenter;

    if (DRIVERS_AUTO_DISABLE && !wantsMove) {
      unsigned long lastMot;
      portENTER_CRITICAL(&gMux);
      lastMot = g_lastMotionMs;
      portEXIT_CRITICAL(&gMux);
      if (g_driversEnabled && (nowMs - lastMot >= IDLE_DISABLE_MS)) driversOff();
    }

    // -----------------------------------------------------------------------
    // Hand off to hardware step generator.
    // The ISR fires the actual GPIO pulses at hardware-timer precision,
    // completely independent of FreeRTOS scheduling or WiFi activity.
    // dir is +1 when dirX==0 (period=0 means stopped, dir is irrelevant).
    // -----------------------------------------------------------------------
    stepGenSetA(dirA != 0 ? pA : 0, dirA > 0 ? +1 : -1);
    stepGenSetB(dirB != 0 ? pB : 0, dirB > 0 ? +1 : -1);

    // Sleep 1 ms — gives loop()/webLoop() CPU time and keeps dt constant.
    vTaskDelay(1);
  }
}

void stepTaskStart() {
  stepGenInit();
  xTaskCreatePinnedToCore(stepTask, "stepTask", 4096, nullptr, 3, &stepTaskHandle, 0);
}
