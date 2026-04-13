#pragma once
#include "Globals.h"

void startWiFi();
void initOTA();
void handleOTA();
void saveWifiCreds(const char* ssid, const char* pass);
