#pragma once
#include <Arduino.h>
#include "Globals.h"

static inline float clampf(float x, float a, float b) { return (x < a) ? a : (x > b) ? b : x; }
static inline long  clampl(long  x, long a, long b)  { return (x < a) ? a : (x > b) ? b : x; }

static inline float speedForModeXY(SpeedMode m) {
  switch (m) {
    case SPEED_SLOW:   return SPEED_XY_SLOW;
    case SPEED_FAST:   return SPEED_XY_FAST;
    case SPEED_NORMAL:
    default:           return SPEED_XY_NORMAL;
  }
}