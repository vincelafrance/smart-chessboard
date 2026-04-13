#pragma once

#include "Globals.h"

void initDriversUART();
void serviceDriversUART(float vA_steps_s, float vB_steps_s, bool wantsMove, bool precisionMode);
uint8_t getDriversUARTMicrosteps();
void setMotionProfileLock(bool enabled, uint8_t microsteps, uint16_t moveCurrentMa);
bool isMotionProfileLocked();

// Runtime current override support
void setCurrentOverrides(uint16_t cruiseMa, uint16_t fastMa);
void setCurrentOverridesPerGroup(uint16_t diagMa, uint16_t lineMa, uint16_t shortMa);
void selectCurrentsForMove(long fromX, long fromY, long toX, long toY);
uint16_t getCurrentOverrideCruiseMa();
