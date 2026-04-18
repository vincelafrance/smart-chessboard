#include "BoardMapping.h"
#include "WebUI.h"
#include "Utils.h"
#include "MotionCoreXY.h"

void boardUpdateFromOrigin(long oxAbs, long oyAbs) {
  // Apply the shift between the previous center and the new center to all 4 corners.
  // This preserves any fine-tuning done via advanced calibration: if a corner was
  // manually adjusted, it keeps its relative offset and is just translated by the
  // same amount the center moved.  On first boot g_lastOrigin is ORIGIN_REF (0,0)
  // and the corners start at REF values (OFF_xxx), so the delta = oxAbs gives
  // exactly oxAbs + OFF_xxx — same as the old absolute reset.
  long lastX, lastY, A1cx, A1cy, H1cx, H1cy, A8cx, A8cy, H8cx, H8cy;
  long dzlbx, dzlby, dzlhx, dzlhy, dzrbx, dzrby, dzrhx, dzrhy;
  bool dzlcal, dzrcal;
  portENTER_CRITICAL(&gMux);
  lastX = g_lastOrigin_X; lastY = g_lastOrigin_Y;
  A1cx = g_A1C_X; A1cy = g_A1C_Y;
  H1cx = g_H1C_X; H1cy = g_H1C_Y;
  A8cx = g_A8C_X; A8cy = g_A8C_Y;
  H8cx = g_H8C_X; H8cy = g_H8C_Y;
  dzlbx = g_DZ_L_Bas_X;  dzlby = g_DZ_L_Bas_Y;
  dzlhx = g_DZ_L_Haut_X; dzlhy = g_DZ_L_Haut_Y;
  dzrbx = g_DZ_R_Bas_X;  dzrby = g_DZ_R_Bas_Y;
  dzrhx = g_DZ_R_Haut_X; dzrhy = g_DZ_R_Haut_Y;
  dzlcal = g_DZ_L_Calibrated;
  dzrcal = g_DZ_R_Calibrated;
  portEXIT_CRITICAL(&gMux);

  long dx = oxAbs - lastX;
  long dy = oyAbs - lastY;

  A1cx += dx; A1cy += dy;
  H1cx += dx; H1cy += dy;
  A8cx += dx; A8cy += dy;
  H8cx += dx; H8cy += dy;

  // Dead zone endpoints are stored in absolute step coordinates — shift them
  // by the same delta so they stay aligned with the board after recalibration.
  if (dzlcal) { dzlbx += dx; dzlby += dy; dzlhx += dx; dzlhy += dy; }
  if (dzrcal) { dzrbx += dx; dzrby += dy; dzrhx += dx; dzrhy += dy; }

  float fileVx = (float)(H1cx - A1cx) / 7.0f;
  float fileVy = (float)(H1cy - A1cy) / 7.0f;
  float rankVx = (float)(A8cx - A1cx) / 7.0f;
  float rankVy = (float)(A8cy - A1cy) / 7.0f;

  long BO_A1x = (long)lroundf((float)A1cx - 0.5f*fileVx - 0.5f*rankVx);
  long BO_A1y = (long)lroundf((float)A1cy - 0.5f*fileVy - 0.5f*rankVy);

  long BO_H1x = (long)lroundf((float)H1cx + 0.5f*fileVx - 0.5f*rankVx);
  long BO_H1y = (long)lroundf((float)H1cy + 0.5f*fileVy - 0.5f*rankVy);

  long BO_A8x = (long)lroundf((float)A8cx - 0.5f*fileVx + 0.5f*rankVx);
  long BO_A8y = (long)lroundf((float)A8cy - 0.5f*fileVy + 0.5f*rankVy);

  long BO_H8x = (long)lroundf((float)H8cx + 0.5f*fileVx + 0.5f*rankVx);
  long BO_H8y = (long)lroundf((float)H8cy + 0.5f*fileVy + 0.5f*rankVy);

  portENTER_CRITICAL(&gMux);
  g_lastOrigin_X = oxAbs; g_lastOrigin_Y = oyAbs;
  g_A1C_X=A1cx; g_A1C_Y=A1cy;
  g_H1C_X=H1cx; g_H1C_Y=H1cy;
  g_A8C_X=A8cx; g_A8C_Y=A8cy;
  g_H8C_X=H8cx; g_H8C_Y=H8cy;

  g_BO_A1_X=BO_A1x; g_BO_A1_Y=BO_A1y;
  g_BO_H1_X=BO_H1x; g_BO_H1_Y=BO_H1y;
  g_BO_A8_X=BO_A8x; g_BO_A8_Y=BO_A8y;
  g_BO_H8_X=BO_H8x; g_BO_H8_Y=BO_H8y;

  if (dzlcal) { g_DZ_L_Bas_X=dzlbx; g_DZ_L_Bas_Y=dzlby; g_DZ_L_Haut_X=dzlhx; g_DZ_L_Haut_Y=dzlhy; }
  if (dzrcal) { g_DZ_R_Bas_X=dzrbx; g_DZ_R_Bas_Y=dzrby; g_DZ_R_Haut_X=dzrhx; g_DZ_R_Haut_Y=dzrhy; }
  portEXIT_CRITICAL(&gMux);

  Serial.printf("[BOARD] centerAbs=%ld,%ld delta=%ld,%ld outerA1Abs=%ld,%ld dzShift=%s\n",
                oxAbs, oyAbs, dx, dy, BO_A1x, BO_A1y,
                (dzlcal || dzrcal) ? "yes" : "no");
  g_calibNvsDirty   = true;
  g_calibNvsDirtyMs = millis();
}

// Recompute outer-boundary (BO_) positions from the 4 independently-stored
// corner centres (g_A1C, g_H1C, g_A8C, g_H8C).  Call this after any single
// corner is updated by the calibration UI so that all squares reinterpolate.
void boardUpdateFromCorners() {
  long A1cx, A1cy, H1cx, H1cy, A8cx, A8cy, H8cx, H8cy;
  portENTER_CRITICAL(&gMux);
  A1cx = g_A1C_X; A1cy = g_A1C_Y;
  H1cx = g_H1C_X; H1cy = g_H1C_Y;
  A8cx = g_A8C_X; A8cy = g_A8C_Y;
  H8cx = g_H8C_X; H8cy = g_H8C_Y;
  portEXIT_CRITICAL(&gMux);

  // Half-square step vectors derived from the A1-H1 (file) and A1-A8 (rank) axes.
  float fileVx = (float)(H1cx - A1cx) / 7.0f;
  float fileVy = (float)(H1cy - A1cy) / 7.0f;
  float rankVx = (float)(A8cx - A1cx) / 7.0f;
  float rankVy = (float)(A8cy - A1cy) / 7.0f;

  // Outer boundary = extend each corner by half a square outward.
  long BO_A1x = (long)lroundf((float)A1cx - 0.5f*fileVx - 0.5f*rankVx);
  long BO_A1y = (long)lroundf((float)A1cy - 0.5f*fileVy - 0.5f*rankVy);
  long BO_H1x = (long)lroundf((float)H1cx + 0.5f*fileVx - 0.5f*rankVx);
  long BO_H1y = (long)lroundf((float)H1cy + 0.5f*fileVy - 0.5f*rankVy);
  long BO_A8x = (long)lroundf((float)A8cx - 0.5f*fileVx + 0.5f*rankVx);
  long BO_A8y = (long)lroundf((float)A8cy - 0.5f*fileVy + 0.5f*rankVy);
  long BO_H8x = (long)lroundf((float)H8cx + 0.5f*fileVx + 0.5f*rankVx);
  long BO_H8y = (long)lroundf((float)H8cy + 0.5f*fileVy + 0.5f*rankVy);

  portENTER_CRITICAL(&gMux);
  g_BO_A1_X=BO_A1x; g_BO_A1_Y=BO_A1y;
  g_BO_H1_X=BO_H1x; g_BO_H1_Y=BO_H1y;
  g_BO_A8_X=BO_A8x; g_BO_A8_Y=BO_A8y;
  g_BO_H8_X=BO_H8x; g_BO_H8_Y=BO_H8y;
  portEXIT_CRITICAL(&gMux);

  Serial.printf("[BOARD] corners recomputed from 4 independent centres\n");
}

void boardUVToXY(float u, float v, long &xOutAbs, long &yOutAbs) {
  long A1x,A1y,H1x,H1y,A8x,A8y,H8x,H8y;
  portENTER_CRITICAL(&gMux);
  A1x = g_BO_A1_X; A1y = g_BO_A1_Y;
  H1x = g_BO_H1_X; H1y = g_BO_H1_Y;
  A8x = g_BO_A8_X; A8y = g_BO_A8_Y;
  H8x = g_BO_H8_X; H8y = g_BO_H8_Y;
  portEXIT_CRITICAL(&gMux);

  float fu = clampf(u / 8.0f, 0.0f, 1.0f);
  float fv = clampf(v / 8.0f, 0.0f, 1.0f);

  float x =
    (1.0f - fu) * (1.0f - fv) * (float)A1x +
    (fu)        * (1.0f - fv) * (float)H1x +
    (1.0f - fu) * (fv)        * (float)A8x +
    (fu)        * (fv)        * (float)H8x;

  float y =
    (1.0f - fu) * (1.0f - fv) * (float)A1y +
    (fu)        * (1.0f - fv) * (float)H1y +
    (1.0f - fu) * (fv)        * (float)A8y +
    (fu)        * (fv)        * (float)H8y;

  xOutAbs = (long)lroundf(x);
  yOutAbs = (long)lroundf(y);
}

void squareCenterSteps(uint8_t file0_7, uint8_t rank1_8, long &cxAbs, long &cyAbs) {
  float u = (float)file0_7 + 0.5f;
  float v = (float)(rank1_8 - 1) + 0.5f;
  boardUVToXY(u, v, cxAbs, cyAbs);

  long xMin,xMax,yMin,yMax;
  getXLimits(xMin,xMax);
  getYLimits(yMin,yMax);
  cxAbs = clampl(cxAbs, xMin + EDGE_STOP_DIST, xMax - EDGE_STOP_DIST);
  cyAbs = clampl(cyAbs, yMin + EDGE_STOP_DIST, yMax - EDGE_STOP_DIST);
}

void deadZoneSlotPos(char side, uint8_t slot, long &xOut, long &yOut) {
  if (slot > 15) slot = 15;

  bool calibrated;
  long basX, basY, hautX, hautY;

  portENTER_CRITICAL(&gMux);
  if (side == 'L') {
    calibrated = g_DZ_L_Calibrated;
    basX  = g_DZ_L_Bas_X;  basY  = g_DZ_L_Bas_Y;
    hautX = g_DZ_L_Haut_X; hautY = g_DZ_L_Haut_Y;
  } else {
    calibrated = g_DZ_R_Calibrated;
    basX  = g_DZ_R_Bas_X;  basY  = g_DZ_R_Bas_Y;
    hautX = g_DZ_R_Haut_X; hautY = g_DZ_R_Haut_Y;
  }
  portEXIT_CRITICAL(&gMux);

  if (!calibrated) {
    // Fallback: single column just outside the board edge, evenly spaced.
    long xMin, xMax, yMin, yMax;
    getXLimits(xMin, xMax);
    getYLimits(yMin, yMax);
    long xEdge   = (side == 'L') ? xMin + EDGE_STOP_DIST + 200 : xMax - EDGE_STOP_DIST - 200;
    long yRange  = yMax - yMin - 2 * EDGE_STOP_DIST;
    long yBase   = yMin + EDGE_STOP_DIST;
    long yStep   = yRange / 15;
    xOut = xEdge;
    yOut = yBase + (long)slot * yStep;
    webLog("[DZ] slotPos side=%c slot=%u FALLBACK (not calibrated) -> (%ld,%ld)",
           side, slot, xOut, yOut);
    return;
  }

  // Linear interpolation: slot 15 = bas, slot 0 = haut
  // t goes from 0 (slot 15) to 1 (slot 0)
  float t = (float)(15 - slot) / 15.0f;
  xOut = basX + (long)lroundf(t * (float)(hautX - basX));
  yOut = basY + (long)lroundf(t * (float)(hautY - basY));
  webLog("[DZ] slotPos side=%c slot=%u CALIBRATED bas=(%ld,%ld) haut=(%ld,%ld) -> (%ld,%ld)",
         side, slot, basX, basY, hautX, hautY, xOut, yOut);
}

void squareEdgeMidpoint(uint8_t f0, uint8_t r1, int dirX, int dirY, long &exAbs, long &eyAbs) {
  float uC = (float)f0 + 0.5f;
  float vC = (float)(r1 - 1) + 0.5f;

  float u = uC;
  float v = vC;

  if (dirX == -1) u = (float)f0;
  if (dirX == +1) u = (float)f0 + 1.0f;

  if (dirY == -1) v = (float)(r1 - 1);
  if (dirY == +1) v = (float)(r1);

  boardUVToXY(u, v, exAbs, eyAbs);

  long cx, cy;
  boardUVToXY(uC, vC, cx, cy);

  long vx = cx - exAbs;
  long vy = cy - eyAbs;
  float len = sqrtf((float)vx * (float)vx + (float)vy * (float)vy);
  if (len > 1.0f) {
    float nx = (float)vx / len;
    float ny = (float)vy / len;
    exAbs = (long)lroundf((float)exAbs + nx * (float)EDGE_MARGIN_STEPS);
    eyAbs = (long)lroundf((float)eyAbs + ny * (float)EDGE_MARGIN_STEPS);
  }

  long xMin,xMax,yMin,yMax;
  getXLimits(xMin,xMax);
  getYLimits(yMin,yMax);
  exAbs = clampl(exAbs, xMin + EDGE_STOP_DIST, xMax - EDGE_STOP_DIST);
  eyAbs = clampl(eyAbs, yMin + EDGE_STOP_DIST, yMax - EDGE_STOP_DIST);
}