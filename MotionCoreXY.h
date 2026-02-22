#pragma once
#include "Globals.h"

void setDriversEnabled(bool en);
void driversOn();
void driversOff();
void markMotion();

void getYLimits(long &yMin, long &yMax);
void getXLimits(long &xMin, long &xMax);

float applyEdgeLimit1D(float v, long pos, long minL, long maxL);

void xyToAB(float vx, float vy, float &vA, float &vB);
void getXYfromAB_raw(long aPos, long bPos, long &xAbs, long &yAbs);

uint32_t speedToPeriodUs(float vStepsPerSec);
float approachf(float cur, float target, float maxDelta);

void stepPulseFast(gpio_num_t pin);