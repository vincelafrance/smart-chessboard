#pragma once
#include "Globals.h"

void setSpeedMode(uint8_t sp);
void setDirCommand(const String& dir);
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);