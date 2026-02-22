#pragma once
#include "Globals.h"

void batteryInit();
void readBatteryOnce(float &v_bus, float &i_A, float &pct_stable);