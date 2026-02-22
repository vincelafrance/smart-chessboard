#include "BoardMapping.h"
#include "Utils.h"
#include "MotionCoreXY.h"

void boardUpdateFromOrigin(long oxAbs, long oyAbs) {
  long A1cx = oxAbs + OFF_A1C_X, A1cy = oyAbs + OFF_A1C_Y;
  long H1cx = oxAbs + OFF_H1C_X, H1cy = oyAbs + OFF_H1C_Y;
  long A8cx = oxAbs + OFF_A8C_X, A8cy = oyAbs + OFF_A8C_Y;
  long H8cx = oxAbs + OFF_H8C_X, H8cy = oyAbs + OFF_H8C_Y;

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
  g_A1C_X=A1cx; g_A1C_Y=A1cy;
  g_H1C_X=H1cx; g_H1C_Y=H1cy;
  g_A8C_X=A8cx; g_A8C_Y=A8cy;
  g_H8C_X=H8cx; g_H8C_Y=H8cy;

  g_BO_A1_X=BO_A1x; g_BO_A1_Y=BO_A1y;
  g_BO_H1_X=BO_H1x; g_BO_H1_Y=BO_H1y;
  g_BO_A8_X=BO_A8x; g_BO_A8_Y=BO_A8y;
  g_BO_H8_X=BO_H8x; g_BO_H8_Y=BO_H8y;
  portEXIT_CRITICAL(&gMux);

  Serial.print("[BOARD] centerAbs=");
  Serial.print(oxAbs); Serial.print(","); Serial.print(oyAbs);
  Serial.print("  outerA1Abs=");
  Serial.print(BO_A1x); Serial.print(","); Serial.println(BO_A1y);
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