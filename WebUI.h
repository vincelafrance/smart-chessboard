#pragma once
#include "Globals.h"

void webInit();
void webLoop();
void webPushTelemetry();
void webLog(const char* fmt, ...);