#pragma once

// Save / load all calibration data (board corners, dead-zone endpoints, last origin)
// to / from ESP32 NVS so that advanced calibration survives reboots and OTA updates.
//
// Call loadCalibFromNVS() during setup before boardUpdateFromOrigin / boardUpdateFromCorners.
// Returns true if valid saved data was found and restored.
//
// Call saveCalibToNVS() whenever calibration data changes (after calibCorner,
// calibDeadZone, or boardUpdateFromOrigin).

bool loadCalibFromNVS();
void saveCalibToNVS();
