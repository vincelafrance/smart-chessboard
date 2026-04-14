#include "CalibNVS.h"
#include "Globals.h"
#include "BoardMapping.h"
#include <Preferences.h>

static const char NVS_NS[]   = "chess_calib";   // namespace (≤15 chars)
static const char KEY_VALID[] = "valid";         // bool sentinel — true when data was ever saved

// Corner centres (absolute step positions)
static const char KEY_A1CX[] = "a1cx";  static const char KEY_A1CY[] = "a1cy";
static const char KEY_H1CX[] = "h1cx";  static const char KEY_H1CY[] = "h1cy";
static const char KEY_A8CX[] = "a8cx";  static const char KEY_A8CY[] = "a8cy";
static const char KEY_H8CX[] = "h8cx";  static const char KEY_H8CY[] = "h8cy";

// Last origin used by boardUpdateFromOrigin (needed to compute delta on recalibration)
static const char KEY_ORIGX[] = "origx";  static const char KEY_ORIGY[] = "origy";

// Dead-zone endpoints
static const char KEY_DZLBX[] = "dzlbx";  static const char KEY_DZLBY[] = "dzlby";
static const char KEY_DZLHX[] = "dzlhx";  static const char KEY_DZLHY[] = "dzlhy";
static const char KEY_DZRBX[] = "dzrbx";  static const char KEY_DZRBY[] = "dzrby";
static const char KEY_DZRHX[] = "dzrhx";  static const char KEY_DZRHY[] = "dzrhy";
static const char KEY_DZLCAL[] = "dzlcal";
static const char KEY_DZRCAL[] = "dzrcal";

// ---------------------------------------------------------------------------

bool loadCalibFromNVS() {
    Preferences prefs;
    prefs.begin(NVS_NS, true);   // read-only
    bool valid = prefs.getBool(KEY_VALID, false);
    if (!valid) {
        prefs.end();
        Serial.println("[CALIB NVS] No saved calibration found — using firmware defaults.");
        return false;
    }

    long a1cx = prefs.getLong(KEY_A1CX, A1C_REF_X);
    long a1cy = prefs.getLong(KEY_A1CY, A1C_REF_Y);
    long h1cx = prefs.getLong(KEY_H1CX, H1C_REF_X);
    long h1cy = prefs.getLong(KEY_H1CY, H1C_REF_Y);
    long a8cx = prefs.getLong(KEY_A8CX, A8C_REF_X);
    long a8cy = prefs.getLong(KEY_A8CY, A8C_REF_Y);
    long h8cx = prefs.getLong(KEY_H8CX, H8C_REF_X);
    long h8cy = prefs.getLong(KEY_H8CY, H8C_REF_Y);

    long origx = prefs.getLong(KEY_ORIGX, ORIGIN_REF_X);
    long origy = prefs.getLong(KEY_ORIGY, ORIGIN_REF_Y);

    long dzlbx = prefs.getLong(KEY_DZLBX, 0);  long dzlby = prefs.getLong(KEY_DZLBY, 0);
    long dzlhx = prefs.getLong(KEY_DZLHX, 0);  long dzlhy = prefs.getLong(KEY_DZLHY, 0);
    long dzrbx = prefs.getLong(KEY_DZRBX, 0);  long dzrby = prefs.getLong(KEY_DZRBY, 0);
    long dzrhx = prefs.getLong(KEY_DZRHX, 0);  long dzrhy = prefs.getLong(KEY_DZRHY, 0);
    bool dzlcal = prefs.getBool(KEY_DZLCAL, false);
    bool dzrcal = prefs.getBool(KEY_DZRCAL, false);

    prefs.end();

    portENTER_CRITICAL(&gMux);
    g_A1C_X = a1cx;  g_A1C_Y = a1cy;
    g_H1C_X = h1cx;  g_H1C_Y = h1cy;
    g_A8C_X = a8cx;  g_A8C_Y = a8cy;
    g_H8C_X = h8cx;  g_H8C_Y = h8cy;

    g_lastOrigin_X = origx;
    g_lastOrigin_Y = origy;

    g_DZ_L_Bas_X  = dzlbx;  g_DZ_L_Bas_Y  = dzlby;
    g_DZ_L_Haut_X = dzlhx;  g_DZ_L_Haut_Y = dzlhy;
    g_DZ_R_Bas_X  = dzrbx;  g_DZ_R_Bas_Y  = dzrby;
    g_DZ_R_Haut_X = dzrhx;  g_DZ_R_Haut_Y = dzrhy;
    g_DZ_L_Calibrated = dzlcal;
    g_DZ_R_Calibrated = dzrcal;
    portEXIT_CRITICAL(&gMux);

    Serial.printf("[CALIB NVS] Loaded: A1=(%ld,%ld) H1=(%ld,%ld) A8=(%ld,%ld) H8=(%ld,%ld) origin=(%ld,%ld)\n",
                  a1cx, a1cy, h1cx, h1cy, a8cx, a8cy, h8cx, h8cy, origx, origy);
    return true;
}

void saveCalibToNVS() {
    long a1cx, a1cy, h1cx, h1cy, a8cx, a8cy, h8cx, h8cy;
    long origx, origy;
    long dzlbx, dzlby, dzlhx, dzlhy, dzrbx, dzrby, dzrhx, dzrhy;
    bool dzlcal, dzrcal;

    portENTER_CRITICAL(&gMux);
    a1cx = g_A1C_X;  a1cy = g_A1C_Y;
    h1cx = g_H1C_X;  h1cy = g_H1C_Y;
    a8cx = g_A8C_X;  a8cy = g_A8C_Y;
    h8cx = g_H8C_X;  h8cy = g_H8C_Y;
    origx = g_lastOrigin_X;
    origy = g_lastOrigin_Y;
    dzlbx = g_DZ_L_Bas_X;   dzlby = g_DZ_L_Bas_Y;
    dzlhx = g_DZ_L_Haut_X;  dzlhy = g_DZ_L_Haut_Y;
    dzrbx = g_DZ_R_Bas_X;   dzrby = g_DZ_R_Bas_Y;
    dzrhx = g_DZ_R_Haut_X;  dzrhy = g_DZ_R_Haut_Y;
    dzlcal = g_DZ_L_Calibrated;
    dzrcal = g_DZ_R_Calibrated;
    portEXIT_CRITICAL(&gMux);

    Preferences prefs;
    prefs.begin(NVS_NS, false);   // read-write
    prefs.putBool(KEY_VALID, true);

    prefs.putLong(KEY_A1CX, a1cx);  prefs.putLong(KEY_A1CY, a1cy);
    prefs.putLong(KEY_H1CX, h1cx);  prefs.putLong(KEY_H1CY, h1cy);
    prefs.putLong(KEY_A8CX, a8cx);  prefs.putLong(KEY_A8CY, a8cy);
    prefs.putLong(KEY_H8CX, h8cx);  prefs.putLong(KEY_H8CY, h8cy);

    prefs.putLong(KEY_ORIGX, origx);
    prefs.putLong(KEY_ORIGY, origy);

    prefs.putLong(KEY_DZLBX, dzlbx);  prefs.putLong(KEY_DZLBY, dzlby);
    prefs.putLong(KEY_DZLHX, dzlhx);  prefs.putLong(KEY_DZLHY, dzlhy);
    prefs.putLong(KEY_DZRBX, dzrbx);  prefs.putLong(KEY_DZRBY, dzrby);
    prefs.putLong(KEY_DZRHX, dzrhx);  prefs.putLong(KEY_DZRHY, dzrhy);
    prefs.putBool(KEY_DZLCAL, dzlcal);
    prefs.putBool(KEY_DZRCAL, dzrcal);

    prefs.end();
    Serial.printf("[CALIB NVS] Saved corners + dead zones to NVS.\n");
}
