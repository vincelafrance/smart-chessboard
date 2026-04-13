#include "WiFiNet.h"
#include <ArduinoOTA.h>

static bool s_otaReady = false;

void startWiFi() {
  bool wantSTA = (strlen(WIFI_SSID) > 0);
  if (wantSTA) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(250);
    if (WiFi.status() == WL_CONNECTED) return;
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);
}

void initOTA() {
  s_otaReady = false;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA] Disabled: OTA upload from Arduino IDE needs STA WiFi.");
    return;
  }

  ArduinoOTA.setHostname("smartchessboard");

  ArduinoOTA.onStart([]() {
    const char *type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.printf("[OTA] Start updating %s\n", type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update complete");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned int lastPct = 0;
    unsigned int pct = (total > 0) ? (progress * 100U) / total : 0U;
    if (pct != lastPct && (pct == 100U || (pct % 10U) == 0U)) {
      lastPct = pct;
      Serial.printf("[OTA] Progress: %u%%\n", pct);
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", (unsigned)error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    else Serial.println("Unknown");
  });

  ArduinoOTA.begin();
  s_otaReady = true;
  Serial.printf("[OTA] Ready on %s.local (%s)\n",
                ArduinoOTA.getHostname().c_str(),
                WiFi.localIP().toString().c_str());
}

void handleOTA() {
  if (!s_otaReady) return;
  ArduinoOTA.handle();
}
