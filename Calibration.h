#pragma once
#include "Globals.h"

void startFullCalibration();
void startCalibrationFromKnownCorner(long yKnown, long xKnown, bool knownIsBottom);
void startCalibrationFromKnownHall(bool hasY, long yKnown,
								   bool hasX, long xKnown,
								   bool knownIsBottom);
void calibrationLoop(unsigned long now);