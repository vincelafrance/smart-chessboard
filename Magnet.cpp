#include "Magnet.h"

void magnetInit() {
  bool ok = ledcAttach(MAGNET_PIN, MAGNET_PWM_FREQ, MAGNET_PWM_RES);
  if (!ok) Serial.println("[MAGNET] ERROR: ledcAttach failed");
  ledcWrite(MAGNET_PIN, 0);

  portENTER_CRITICAL(&gMux);
  g_magnetOn = false;
  portEXIT_CRITICAL(&gMux);
}

void magnetSet(bool on) {
  ledcWrite(MAGNET_PIN, on ? MAGNET_DUTY_100 : 0);
  portENTER_CRITICAL(&gMux);
  g_magnetOn = on;
  portEXIT_CRITICAL(&gMux);
}