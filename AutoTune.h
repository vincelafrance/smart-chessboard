#pragma once
#include "Globals.h"

// ============================================================
// AutoTune.h
// Production motion validation, auto-tuning, and persistence.
//
// ARCHITECTURE:
//   - AutoTune runs as a dedicated FreeRTOS task (core 1, pri 2).
//   - It controls motion via shared globals exactly as Commands.cpp does.
//   - calibrationLoop() (in loop()) drives all calibration passes —
//     AutoTune only calls startFullCalibration() and waits for completion.
//   - g_overrideVmax / g_overrideAccel let the tune task inject test
//     speeds into the StepTask recenter and path controllers.
//   - NVS persistence uses Arduino Preferences (ESP32 flash).
//
// PREREQUISITE: Calibration must be valid before starting AutoTune.
//   autoTuneStart() rejects if g_yLimitsCalibrated || g_xLimitsCalibrated
//   are false.
// ============================================================

// ----- Public API -----

// Load saved TuneSettings from NVS at boot (before first move).
void autoTuneInit();

// Start the tuning sequence.  fullMode=true runs all 5 phases (Repeatability
// + SpeedRamp + AccelRamp + CurrentTune + Approach).  fullMode=false runs a
// reduced quick pass (fewer cycles per phase).
// Returns false if already running or calibration is not valid.
bool autoTuneStart(bool fullMode);

// Request graceful abort.  Motion is stopped, overrides are cleared.
void autoTuneStop();

// Called from loop() every iteration.  Drains the log ring-buffer to
// WebSocket, and updates g_systemState based on calibration/tune activity.
void autoTuneLoop(unsigned long now);

// NVS helpers (also called internally after a successful tune).
void saveSettings();
void loadSettings();
