#include "WiFiNet.h"
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Preferences.h>

static bool s_otaReady = false;

// Load credentials saved via the WebUI. Returns true if a non-empty SSID was found.
static bool loadWifiCreds(char* ssid, size_t ssidLen, char* pass, size_t passLen) {
  Preferences prefs;
  prefs.begin("wifi", true);
  String s = prefs.getString("ssid", "");
  String p = prefs.getString("pass", "");
  prefs.end();
  if (s.length() == 0) return false;
  strncpy(ssid, s.c_str(), ssidLen - 1); ssid[ssidLen - 1] = '\0';
  strncpy(pass, p.c_str(), passLen - 1); pass[passLen - 1] = '\0';
  return true;
}

// Save credentials to NVS and reboot so the new network is used immediately.
void saveWifiCreds(const char* ssid, const char* pass) {
  Preferences prefs;
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass ? pass : "");
  prefs.end();
  Serial.printf("[WIFI] Credentials saved (SSID: %s) — rebooting\n", ssid);
  delay(300);
  ESP.restart();
}

void startWiFi() {
  char nvsSsid[64] = "";
  char nvsPass[64] = "";
  bool hasNvs = loadWifiCreds(nvsSsid, sizeof(nvsSsid), nvsPass, sizeof(nvsPass));

  // NVS credentials take priority; fall back to compile-time constants.
  const char* useSsid = hasNvs ? nvsSsid : WIFI_SSID;
  const char* usePass = hasNvs ? nvsPass : WIFI_PASS;

  if (strlen(useSsid) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_13dBm);   // reduce peak current on battery (~150 mA vs ~240 mA)
    WiFi.begin(useSsid, usePass);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(250);
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.println("[WIFI] STA connection failed — falling back to AP mode");
  }
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_13dBm);     // reduce peak current on battery
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);
}

void initOTA() {
  s_otaReady = false;

  bool isSTA = (WiFi.getMode() == WIFI_MODE_STA || WiFi.getMode() == WIFI_MODE_APSTA)
               && (WiFi.status() == WL_CONNECTED);
  bool isAP  = (WiFi.getMode() == WIFI_MODE_AP  || WiFi.getMode() == WIFI_MODE_APSTA);

  if (!isSTA && !isAP) {
    Serial.println("[OTA] Disabled: no WiFi active.");
    return;
  }

  // In AP mode, start mDNS manually so the hostname resolves on the AP subnet.
  if (isAP && !isSTA) {
    if (!MDNS.begin("smartchessboard")) {
      Serial.println("[OTA] mDNS start failed (AP mode) — use IP 192.168.4.1");
    }
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

  IPAddress ip = isAP ? WiFi.softAPIP() : WiFi.localIP();
  Serial.printf("[OTA] Ready — hostname: smartchessboard.local  IP: %s\n",
                ip.toString().c_str());
  Serial.println("[OTA] Dans Arduino IDE: Outils > Port > smartchessboard");
}

void handleOTA() {
  if (!s_otaReady) return;
  ArduinoOTA.handle();
}
