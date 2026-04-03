#include "MotionCoreXY.h"
#include "Utils.h"
#include "DriversUART.h"

void setDriversEnabled(bool en) {
  bool lvl = ENABLE_ACTIVE_LOW ? (en ? LOW : HIGH) : (en ? HIGH : LOW);
  digitalWrite(LEFT_EN, lvl);
  digitalWrite(RIGHT_EN, lvl);
  g_driversEnabled = en;
}

void driversOn() {
  if (!g_driversEnabled) {
    setDriversEnabled(true);
    esp_rom_delay_us(200);
  }
}
void driversOff() { if (g_driversEnabled) setDriversEnabled(false); }

void markMotion() { g_lastMotionMs = millis(); }

void getYLimits(long &yMin, long &yMax) {
  portENTER_CRITICAL(&gMux);
  bool cal = g_yLimitsCalibrated;
  long mn  = g_yHardMin;
  long mx  = g_yHardMax;
  portEXIT_CRITICAL(&gMux);

  if (!cal) { yMin = Y_HARD_MIN_DEFAULT; yMax = Y_HARD_MAX_DEFAULT; }
  else      { yMin = mn;                 yMax = mx; }
}

void getXLimits(long &xMin, long &xMax) {
  portENTER_CRITICAL(&gMux);
  bool cal = g_xLimitsCalibrated;
  long mn  = g_xHardMin;
  long mx  = g_xHardMax;
  portEXIT_CRITICAL(&gMux);

  if (!cal) { xMin = X_HARD_MIN_DEFAULT; xMax = X_HARD_MAX_DEFAULT; }
  else      { xMin = mn;                 xMax = mx; }
}

float applyEdgeLimit1D(float v, long pos, long minL, long maxL) {
  if (v > 0.0f) {
    if (pos >= maxL) return 0.0f;
    long dist = maxL - pos;
    if (dist <= EDGE_STOP_DIST) return 0.0f;
    if (dist < EDGE_SLOW_DIST) {
      float s = (float)(dist - EDGE_STOP_DIST) / (float)(EDGE_SLOW_DIST - EDGE_STOP_DIST);
      s = clampf(s, 0.0f, 1.0f);
      float av = fabsf(v);
      float vlim = EDGE_MIN_SPEED + (av - EDGE_MIN_SPEED) * s;
      if (vlim > av) vlim = av;
      if (vlim < EDGE_MIN_SPEED) vlim = EDGE_MIN_SPEED;
      return +vlim;
    }
    return v;
  }
  if (v < 0.0f) {
    if (pos <= minL) return 0.0f;
    long dist = pos - minL;
    if (dist <= EDGE_STOP_DIST) return 0.0f;
    if (dist < EDGE_SLOW_DIST) {
      float s = (float)(dist - EDGE_STOP_DIST) / (float)(EDGE_SLOW_DIST - EDGE_STOP_DIST);
      s = clampf(s, 0.0f, 1.0f);
      float av = fabsf(v);
      float vlim = EDGE_MIN_SPEED + (av - EDGE_MIN_SPEED) * s;
      if (vlim > av) vlim = av;
      if (vlim < EDGE_MIN_SPEED) vlim = EDGE_MIN_SPEED;
      return -vlim;
    }
    return v;
  }
  return 0.0f;
}

void xyToAB(float vx, float vy, float &vA, float &vB) {
  float dx = vx, dy = vy;
  if (INVERT_X) dx = -dx;
  if (INVERT_Y) dy = -dy;
  vA = dx + dy;
  vB = dx - dy;
  if (SWAP_MOTORS_AB) { float t = vA; vA = vB; vB = t; }
}

void getXYfromAB_raw(long aPos, long bPos, long &xAbs, long &yAbs) {
  if (SWAP_MOTORS_AB) { long t = aPos; aPos = bPos; bPos = t; }

  const uint8_t ms = getDriversUARTMicrosteps();
  if (ms != 0 && ms != 8) {
    const float scaleToLogical = 8.0f / (float)ms;
    aPos = (long)lroundf((float)aPos * scaleToLogical);
    bPos = (long)lroundf((float)bPos * scaleToLogical);
  }

  long dx = (aPos + bPos) / 2;
  long dy = (aPos - bPos) / 2;

  if (INVERT_X) dx = -dx;
  if (INVERT_Y) dy = -dy;

  xAbs = XY_ORIGIN_X + dx;
  yAbs = XY_ORIGIN_Y + dy;
}

uint32_t speedToPeriodUs(float vStepsPerSec) {
  float av = fabsf(vStepsPerSec);
  if (av < 1.0f) return 0;
  if (av > 25000.0f) av = 25000.0f;
  return (uint32_t)(1000000.0f / av);
}

float approachf(float cur, float target, float maxDelta) {
  float d = target - cur;
  if (d >  maxDelta) d =  maxDelta;
  if (d < -maxDelta) d = -maxDelta;
  return cur + d;
}

void stepPulseFast(gpio_num_t pin) {
  gpio_set_level(pin, 1);
  esp_rom_delay_us(2);
  gpio_set_level(pin, 0);
}