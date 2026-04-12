#include "DriversUART.h"

#include <TMCStepper.h>

// TMC2209 UART on ESP32 Serial2 (RX2/TX2).
// If both drivers share the same UART bus, each one must have a unique address.
static const uint8_t LEFT_TMC_ADDR = 0b00;
static const uint8_t RIGHT_TMC_ADDR = 0b01;

static const float TMC_R_SENSE = 0.11f;
static const uint8_t TMC_MICROSTEPS_FINE = 16;
static const uint8_t TMC_MICROSTEPS_COARSE = 8;
static const uint32_t TMC_UART_BAUD = 115200;

static const uint16_t TMC_CURRENT_HOLD_MA = 450;
static const uint16_t TMC_CURRENT_CRUISE_MA = 850;
static const uint16_t TMC_CURRENT_FAST_MA = 1050;

static const uint8_t TMC_HOLD_PCT_IDLE = 15;
static const uint8_t TMC_HOLD_PCT_MOVE = 35;

// Speed thresholds on motor axes (A/B), in steps/s, with hysteresis.
static const float TH_HOLD_TO_CRUISE = 700.0f;
static const float TH_CRUISE_TO_HOLD = 250.0f;
static const float TH_CRUISE_TO_FAST = 5200.0f;
static const float TH_FAST_TO_CRUISE = 4200.0f;

// Dynamic microstep policy (logical speed in "legacy 1/8-step units").
static const float TH_MS_16_TO_8 = 3600.0f;
static const float TH_MS_8_TO_16 = 2200.0f;

// Safety guards to avoid abrupt runtime reconfiguration.
static const float TH_SAFE_SWITCH_SPEED = 150.0f;
static const unsigned long MICROSTEP_SWITCH_MIN_MS = 220;
static const unsigned long PROFILE_SWITCH_MIN_MS = 120;

static HardwareSerial TMCSerial(2);
static TMC2209Stepper leftDriver(&TMCSerial, TMC_R_SENSE, LEFT_TMC_ADDR);
static TMC2209Stepper rightDriver(&TMCSerial, TMC_R_SENSE, RIGHT_TMC_ADDR);

enum DriverCurrentProfile : uint8_t {
  PROFILE_HOLD = 0,
  PROFILE_CRUISE = 1,
  PROFILE_FAST = 2,
};

static bool g_tmcReady = false;
static DriverCurrentProfile g_profile = PROFILE_CRUISE;
static uint8_t g_microsteps = TMC_MICROSTEPS_COARSE;
// Runtime current overrides (0 = use compiled defaults)
static uint16_t g_overrideCruiseMa = 0;
static uint16_t g_overrideFastMa   = 0;
// Per-movement-type overrides (0 = fall back to g_overrideCruiseMa)
static uint16_t g_overrideDiagMa   = 0;
static uint16_t g_overrideLineMa   = 0;
static uint16_t g_overrideShortMa  = 0;
// Threshold & classification constants for selectCurrentsForMove
static const long  SHORT_MOVE_STEPS  = 160;   // ~4 mm threshold
static const float DIAG_RATIO_MIN    = 0.30f; // 30% cross-axis component = diagonal
static unsigned long g_lastMicrostepSwitchMs = 0;
static unsigned long g_lastProfileSwitchMs = 0;

static void applyMicrosteps(uint8_t microsteps) {
  if (microsteps != TMC_MICROSTEPS_FINE && microsteps != TMC_MICROSTEPS_COARSE) return;
  if (microsteps == g_microsteps) return;
  leftDriver.microsteps(microsteps);
  rightDriver.microsteps(microsteps);
  g_microsteps = microsteps;
  g_lastMicrostepSwitchMs = millis();
  Serial.printf("[TMC-UART] Microsteps: 1/%u\n", (unsigned)microsteps);
}

static void applyProfile(DriverCurrentProfile profile) {
  uint16_t runCurrent = (g_overrideCruiseMa > 0) ? g_overrideCruiseMa : TMC_CURRENT_CRUISE_MA;
  uint8_t holdPct = TMC_HOLD_PCT_MOVE;

  if (profile == PROFILE_HOLD) {
    runCurrent = TMC_CURRENT_HOLD_MA;
    holdPct = TMC_HOLD_PCT_IDLE;
  } else if (profile == PROFILE_FAST) {
    runCurrent = (g_overrideFastMa > 0) ? g_overrideFastMa : TMC_CURRENT_FAST_MA;
    holdPct = TMC_HOLD_PCT_MOVE;
  }

  leftDriver.rms_current(runCurrent, holdPct);
  rightDriver.rms_current(runCurrent, holdPct);
  g_profile = profile;
  g_lastProfileSwitchMs = millis();

  const char *name = (profile == PROFILE_HOLD) ? "HOLD" : (profile == PROFILE_FAST) ? "FAST" : "CRUISE";
  Serial.printf("[TMC-UART] Current profile: %s (%u mA, hold %u%%)\n", name, runCurrent, holdPct);
}

void initDriversUART() {
  TMCSerial.begin(TMC_UART_BAUD, SERIAL_8N1, TMC_UART_RX_PIN, TMC_UART_TX_PIN);
  delay(20);

  leftDriver.begin();
  leftDriver.toff(4);
  leftDriver.pwm_autoscale(true);

  rightDriver.begin();
  rightDriver.toff(4);
  rightDriver.pwm_autoscale(true);

  uint8_t lconn = leftDriver.test_connection();
  uint8_t rconn = rightDriver.test_connection();
  g_tmcReady = (lconn == 0 && rconn == 0);

  // Start in fine mode for smoother low-speed control and calibration approach.
  applyMicrosteps(TMC_MICROSTEPS_FINE);
  applyProfile(PROFILE_CRUISE);

  Serial.printf("[TMC-UART] Serial2 started @ %lu bps (RX=%d TX=%d)\n", TMC_UART_BAUD, TMC_UART_RX_PIN, TMC_UART_TX_PIN);
  Serial.printf("[TMC-UART] Left addr=%u Right addr=%u\n", LEFT_TMC_ADDR, RIGHT_TMC_ADDR);
  if (!g_tmcReady) {
    Serial.printf("[TMC-UART] Warning: communication test failed (L=%u R=%u)\n", lconn, rconn);
  }
}

void serviceDriversUART(float vA_steps_s, float vB_steps_s, bool wantsMove, bool precisionMode) {
  if (!g_tmcReady) return;

  const float vAbs = fmaxf(fabsf(vA_steps_s), fabsf(vB_steps_s));
  const unsigned long nowMs = millis();

  // Keep 1/16 for precision phases and at idle; prefer 1/8 during regular travel.
  uint8_t nextMs = (precisionMode || !wantsMove) ? TMC_MICROSTEPS_FINE : TMC_MICROSTEPS_COARSE;
  bool canSwitchMs = (nowMs - g_lastMicrostepSwitchMs) >= MICROSTEP_SWITCH_MIN_MS;
  bool nearStop = (vAbs <= TH_SAFE_SWITCH_SPEED);
  if (nextMs != g_microsteps && canSwitchMs && (!wantsMove || nearStop)) {
    applyMicrosteps(nextMs);
  }

  DriverCurrentProfile next = g_profile;

  if (!wantsMove) {
    next = PROFILE_HOLD;
  } else if (g_profile == PROFILE_FAST) {
    if (vAbs <= TH_FAST_TO_CRUISE) next = PROFILE_CRUISE;
  } else if (g_profile == PROFILE_CRUISE) {
    if (vAbs >= TH_CRUISE_TO_FAST) next = PROFILE_FAST;
    else if (vAbs <= TH_CRUISE_TO_HOLD && !precisionMode) next = PROFILE_HOLD;
  } else {
    if (vAbs >= TH_HOLD_TO_CRUISE || precisionMode) next = PROFILE_CRUISE;
  }

  bool canSwitchProfile = (nowMs - g_lastProfileSwitchMs) >= PROFILE_SWITCH_MIN_MS;
  if (next != g_profile && canSwitchProfile) applyProfile(next);
}

uint8_t getDriversUARTMicrosteps() {
  return g_microsteps;
}

void setCurrentOverrides(uint16_t cruiseMa, uint16_t fastMa) {
  // Keep values in a conservative range supported by this profile.
  const uint16_t maxMa = 1500;
  g_overrideCruiseMa = (cruiseMa > maxMa) ? maxMa : cruiseMa;
  g_overrideFastMa   = (fastMa > maxMa) ? maxMa : fastMa;
  Serial.printf("[TMC-UART] Current overrides applied: cruise=%umA fast=%umA\n",
                (unsigned)g_overrideCruiseMa, (unsigned)g_overrideFastMa);
  // Re-apply current profile with current overrides.
  applyProfile(g_profile);
}

void setCurrentOverridesPerGroup(uint16_t diagMa, uint16_t lineMa, uint16_t shortMa) {
  const uint16_t maxMa = 1500;
  g_overrideDiagMa  = (diagMa  > maxMa) ? maxMa : diagMa;
  g_overrideLineMa  = (lineMa  > maxMa) ? maxMa : lineMa;
  g_overrideShortMa = (shortMa > maxMa) ? maxMa : shortMa;
  Serial.printf("[TMC-UART] Per-group currents stored: diag=%umA line=%umA short=%umA\n",
                (unsigned)g_overrideDiagMa, (unsigned)g_overrideLineMa, (unsigned)g_overrideShortMa);
}

void selectCurrentsForMove(long fromX, long fromY, long toX, long toY) {
  if (g_overrideDiagMa == 0 && g_overrideLineMa == 0 && g_overrideShortMa == 0) return;  // no per-group data

  long dx = labs(toX - fromX);
  long dy = labs(toY - fromY);
  long maxD = (dx > dy) ? dx : dy;
  long minD = (dx < dy) ? dx : dy;

  uint16_t selected;
  const char *label;

  if (maxD <= SHORT_MOVE_STEPS) {
    // Short move — small square distance
    selected = (g_overrideShortMa > 0) ? g_overrideShortMa : g_overrideCruiseMa;
    label = "SHORT";
  } else if (minD > 0 && (float)minD / (float)maxD >= DIAG_RATIO_MIN) {
    // Significant cross-axis component — diagonal
    selected = (g_overrideDiagMa > 0) ? g_overrideDiagMa : g_overrideCruiseMa;
    label = "DIAG";
  } else {
    // One-axis dominant — line slide
    selected = (g_overrideLineMa > 0) ? g_overrideLineMa : g_overrideCruiseMa;
    label = "LINE";
  }

  if (selected == g_overrideCruiseMa) return;  // no change needed

  g_overrideCruiseMa = selected;
  // Fast keeps its own margin above cruise; recompute here.
  // We don't alter g_overrideFastMa because it scales from the original override result.
  Serial.printf("[TMC-UART] selectCurrentsForMove: (%ld,%ld)->(%ld,%ld) => %s %umA\n",
                fromX, fromY, toX, toY, label, (unsigned)selected);

  // Apply immediately if we are already in CRUISE profile.
  if (g_profile == PROFILE_CRUISE) applyProfile(PROFILE_CRUISE);
}

