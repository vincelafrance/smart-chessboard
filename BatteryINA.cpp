#include "BatteryINA.h"
#include "Utils.h"

static const int AVG_SAMPLES = 20;
static float vbuf[AVG_SAMPLES];
static int   vindex = 0;
static bool  vfilled = false;

static float lastStablePercent = -1.0f;  // -1 = no stable reading yet
static unsigned long restStartMs = 0;

static float movingAverageVoltage(float v) {
  vbuf[vindex++] = v;
  if (vindex >= AVG_SAMPLES) { vindex = 0; vfilled = true; }
  int n = vfilled ? AVG_SAMPLES : vindex;
  if (n <= 0) n = 1;
  float sum = 0;
  for (int i = 0; i < n; i++) sum += vbuf[i];
  return sum / n;
}

static float voltageToPercent_3S(float v) {
  struct Point { float v; float p; };
  static const Point table[] = {
    {12.60, 100},{12.45, 95},{12.35, 90},{12.25, 80},{12.15, 70},{12.05, 60},
    {11.95, 50},{11.85, 40},{11.75, 30},{11.60, 20},{11.40, 10},{10.50, 0}
  };
  const size_t N = sizeof(table) / sizeof(table[0]);

  if (v >= table[0].v) return 100.0f;
  if (v <= table[N - 1].v) return 0.0f;

  for (size_t i = 0; i < N - 1; i++) {
    float v1 = table[i].v, p1 = table[i].p;
    float v2 = table[i+1].v, p2 = table[i+1].p;
    if (v <= v1 && v >= v2) {
      float t = (v - v2) / (v1 - v2);
      return p2 + t * (p1 - p2);
    }
  }
  return 0.0f;
}

void batteryInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  ina.begin(&Wire);
  ina.reset();
  ina.setShuntRes(100, 100, 100);
}

void readBatteryOnce(float &v_bus, float &i_A, float &pct_stable) {
  v_bus = ina.getVoltage(INA_CHANNEL);
  i_A   = ina.getCurrent(INA_CHANNEL);

  float v_avg = movingAverageVoltage(v_bus);
  float pct_raw = clampf(voltageToPercent_3S(v_avg), 0.0f, 100.0f);

  unsigned long now = millis();
  if (fabs(i_A) < REST_CURRENT_A) {
    if (restStartMs == 0) restStartMs = now;
    if (now - restStartMs >= REST_TIME_MS) lastStablePercent = pct_raw;
  } else {
    restStartMs = 0;
  }
  // Before the first stable reading, show the live raw percent rather than 0.
  pct_stable = (lastStablePercent >= 0.0f) ? lastStablePercent : pct_raw;
}