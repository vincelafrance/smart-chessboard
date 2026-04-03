#pragma once

#include "Globals.h"

void initDriversUART();
void serviceDriversUART(float vA_steps_s, float vB_steps_s, bool wantsMove, bool precisionMode);
uint8_t getDriversUARTMicrosteps();
