#include "PathPlanner.h"
#include "Utils.h"
#include "MotionCoreXY.h"
#include "Magnet.h"
#include "BoardMapping.h"
#include "DriversUART.h"

static inline float dist2(long ax,long ay,long bx,long by){
  float dx = (float)(ax - bx);
  float dy = (float)(ay - by);
  return dx*dx + dy*dy;
}
static inline float dist(long ax,long ay,long bx,long by){
  return sqrtf(dist2(ax,ay,bx,by));
}

static inline void uvToXY_clamped(float u, float v, long &xAbs, long &yAbs) {
  boardUVToXY(u, v, xAbs, yAbs);
  long xMin,xMax,yMin,yMax;
  getXLimits(xMin,xMax);
  getYLimits(yMin,yMax);
  xAbs = clampl(xAbs, xMin + EDGE_STOP_DIST, xMax - EDGE_STOP_DIST);
  yAbs = clampl(yAbs, yMin + EDGE_STOP_DIST, yMax - EDGE_STOP_DIST);
}

static inline float clampTurnLineV(float v) {
  if (v <= 0.0f) return 1.0f;
  if (v >= 8.0f) return 7.0f;
  return v;
}
static inline float clampTurnLineU(float u) {
  if (u <= 0.0f) return 1.0f;
  if (u >= 8.0f) return 7.0f;
  return u;
}
static inline void pickTurnLinesV(uint8_t tr, float &vA, float &vB) {
  float a = (float)(tr - 1);
  float b = (float)(tr);
  if (a <= 0.0f) { vA = 1.0f; vB = 2.0f; return; }
  if (b >= 8.0f) { vA = 7.0f; vB = 6.0f; return; }
  vA = a; vB = b;
}
static inline void pickTurnLinesU(uint8_t tf, float &uA, float &uB) {
  float a = (float)(tf);
  float b = (float)(tf + 1.0f);
  if (a <= 0.0f) { uA = 1.0f; uB = 2.0f; return; }
  if (b >= 8.0f) { uA = 7.0f; uB = 6.0f; return; }
  uA = a; uB = b;
}

static inline float pickBestTurnLineV(float uFixed, float v0, float vA, float vB, float uEntry, float vEntryMid,
                                      long fromExitX,long fromExitY, long &turn1x,long &turn1y, long &turn2x,long &turn2y)
{
  (void)v0;
  long t1ax,t1ay,t2ax,t2ay, eMidAx,eMidAy;
  uvToXY_clamped(uFixed, vA, t1ax,t1ay);
  uvToXY_clamped(uEntry, vA, t2ax,t2ay);
  uvToXY_clamped(uEntry, vEntryMid, eMidAx,eMidAy);
  float LA = dist(fromExitX,fromExitY, t1ax,t1ay) + dist(t1ax,t1ay, t2ax,t2ay) + dist(t2ax,t2ay, eMidAx,eMidAy);

  long t1bx,t1by,t2bx,t2by, eMidBx,eMidBy;
  uvToXY_clamped(uFixed, vB, t1bx,t1by);
  uvToXY_clamped(uEntry, vB, t2bx,t2by);
  uvToXY_clamped(uEntry, vEntryMid, eMidBx,eMidBy);
  float LB = dist(fromExitX,fromExitY, t1bx,t1by) + dist(t1bx,t1by, t2bx,t2by) + dist(t2bx,t2by, eMidBx,eMidBy);

  if (LA <= LB) { turn1x=t1ax; turn1y=t1ay; turn2x=t2ax; turn2y=t2ay; return LA; }
  else          { turn1x=t1bx; turn1y=t1by; turn2x=t2bx; turn2y=t2by; return LB; }
}

static inline float pickBestTurnLineU(float vFixed, float u0, float uA, float uB, float vEntry, float uEntryMid,
                                      long fromExitX,long fromExitY, long &turn1x,long &turn1y, long &turn2x,long &turn2y)
{
  (void)u0;
  long t1ax,t1ay,t2ax,t2ay, eMidAx,eMidAy;
  uvToXY_clamped(uA, vFixed, t1ax,t1ay);
  uvToXY_clamped(uA, vEntry, t2ax,t2ay);
  uvToXY_clamped(uEntryMid, vEntry, eMidAx,eMidAy);
  float LA = dist(fromExitX,fromExitY, t1ax,t1ay) + dist(t1ax,t1ay, t2ax,t2ay) + dist(t2ax,t2ay, eMidAx,eMidAy);

  long t1bx,t1by,t2bx,t2by, eMidBx,eMidBy;
  uvToXY_clamped(uB, vFixed, t1bx,t1by);
  uvToXY_clamped(uB, vEntry, t2bx,t2by);
  uvToXY_clamped(uEntryMid, vEntry, eMidBx,eMidBy);
  float LB = dist(fromExitX,fromExitY, t1bx,t1by) + dist(t1bx,t1by, t2bx,t2by) + dist(t2bx,t2by, eMidBx,eMidBy);

  if (LA <= LB) { turn1x=t1ax; turn1y=t1ay; turn2x=t2ax; turn2y=t2ay; return LA; }
  else          { turn1x=t1bx; turn1y=t1by; turn2x=t2bx; turn2y=t2by; return LB; }
}

static inline bool isAdjacent(uint8_t ff, uint8_t fr, uint8_t tf, uint8_t tr) {
  int df = (int)tf - (int)ff;
  int dr = (int)tr - (int)fr;
  if (df == 0 && dr == 0) return false;
  return (abs(df) <= 1 && abs(dr) <= 1);
}


static inline void midpointClamped(long ax,long ay,long bx,long by,long &mx,long &my){
  mx = (ax + bx) / 2;
  my = (ay + by) / 2;
  long xMin,xMax,yMin,yMax;
  getXLimits(xMin,xMax);
  getYLimits(yMin,yMax);
  mx = clampl(mx, xMin + EDGE_STOP_DIST, xMax - EDGE_STOP_DIST);
  my = clampl(my, yMin + EDGE_STOP_DIST, yMax - EDGE_STOP_DIST);
}

static inline void markMagPickDrop(Waypoint *wps, uint8_t n, uint8_t pickIdx, uint8_t dropIdx){
  if(!wps || n==0) return;
  for(uint8_t i=0;i<n;i++) if(wps[i].mag != 0 && wps[i].mag != 1) wps[i].mag = -1;
  if(pickIdx < n) wps[pickIdx].mag = 1;
  if(dropIdx < n) wps[dropIdx].mag = 0;
}
static inline void appendPostDropNeighbor(Waypoint *wps, uint8_t &n, uint8_t tf, uint8_t tr){
  (void)wps;
  (void)n;
  (void)tf;
  (void)tr;
}
static inline void squareInternalPoint(uint8_t f0, uint8_t r1, float uOffset, float vOffset, long &xAbs, long &yAbs) {
  float u = (float)f0 + clampf(uOffset, 0.15f, 0.85f);
  float v = (float)(r1 - 1) + clampf(vOffset, 0.15f, 0.85f);
  uvToXY_clamped(u, v, xAbs, yAbs);
}

void beginMoveSeq(const Waypoint *wps, uint8_t n) {
  if (!wps || n == 0) return;
  if (n > MAX_WAYPOINTS) n = MAX_WAYPOINTS;

  // Select the per-group current profile before the first step, based on the
  // overall move vector (start -> final waypoint).  Done outside the critical
  // section because selectCurrentsForMove only reads shared position under
  // its own brief lock.
  {
    long fromX, fromY;
    portENTER_CRITICAL(&gMux);
    fromX = g_xAbs;
    fromY = g_yAbs;
    portEXIT_CRITICAL(&gMux);
    selectCurrentsForMove(fromX, fromY, wps[n - 1].x, wps[n - 1].y);
  }

  portENTER_CRITICAL(&gMux);
  for (uint8_t i = 0; i < n; i++) g_waypoints[i] = wps[i];
  g_wpCount = n;
  g_wpIndex = 0;
  g_pathActive = true;

  g_recenter = false;
  g_vx_xy = 0;
  g_vy_xy = 0;

  g_pathTargetX = g_waypoints[0].x;
  g_pathTargetY = g_waypoints[0].y;

  g_autoMagnetPath = true;
  g_lastCmdMs = millis();
  portEXIT_CRITICAL(&gMux);

  magnetSet(false);

  Serial.print("[PATH] begin seq, n=");
  Serial.println(n);
}

void abortPath() {
  portENTER_CRITICAL(&gMux);
  g_pathActive = false;
  g_wpCount = 0;
  g_wpIndex = 0;
  g_autoMagnetPath = false;
  portEXIT_CRITICAL(&gMux);
}

void planSquareMoveDirect(uint8_t ff, uint8_t fr, uint8_t tf, uint8_t tr) {
  if (ff == tf && fr == tr) return;

  long fx, fy, tx, ty;
  squareCenterSteps(ff, fr, fx, fy);
  squareCenterSteps(tf, tr, tx, ty);

  Waypoint wps[2];
  uint8_t n = 0;
  wps[n++] = { fx, fy, -1 };
  wps[n++] = { tx, ty, -1 };

  long xMin,xMax,yMin,yMax;
  getXLimits(xMin,xMax);
  getYLimits(yMin,yMax);
  for (uint8_t i=0;i<n;i++){
    wps[i].x = clampl(wps[i].x, xMin + EDGE_STOP_DIST, xMax - EDGE_STOP_DIST);
    wps[i].y = clampl(wps[i].y, yMin + EDGE_STOP_DIST, yMax - EDGE_STOP_DIST);
  }

  markMagPickDrop(wps, n, 0, (uint8_t)(n-1));
  appendPostDropNeighbor(wps, n, tf, tr);
  beginMoveSeq(wps, n);

  Serial.print("[PATH] DIRECT center ");
  Serial.print((char)('a' + ff)); Serial.print(fr);
  Serial.print(" -> ");
  Serial.print((char)('a' + tf)); Serial.println(tr);
}

static inline void planWiggleSameSquare(uint8_t ff, uint8_t fr) {
  long xNow, yNow;
  portENTER_CRITICAL(&gMux);
  xNow = g_xAbs;
  yNow = g_yAbs;
  portEXIT_CRITICAL(&gMux);

  long fx, fy;
  squareCenterSteps(ff, fr, fx, fy);

  long eLx,eLy,eRx,eRy,eDx,eDy,eUx,eUy;
  squareEdgeMidpoint(ff, fr, -1, 0, eLx, eLy);
  squareEdgeMidpoint(ff, fr, +1, 0, eRx, eRy);
  squareEdgeMidpoint(ff, fr, 0, -1, eDx, eDy);
  squareEdgeMidpoint(ff, fr, 0, +1, eUx, eUy);

  long fromEntryX = eLx, fromEntryY = eLy;
  float best = dist2(xNow,yNow,eLx,eLy);
  float d;
  d = dist2(xNow,yNow,eRx,eRy); if (d < best) { best = d; fromEntryX=eRx; fromEntryY=eRy; }
  d = dist2(xNow,yNow,eDx,eDy); if (d < best) { best = d; fromEntryX=eDx; fromEntryY=eDy; }
  d = dist2(xNow,yNow,eUx,eUy); if (d < best) { best = d; fromEntryX=eUx; fromEntryY=eUy; }

  long p1x,p1y,p2x,p2y;
  squareInternalPoint(ff, fr, 0.75f, 0.50f, p1x, p1y);
  squareInternalPoint(ff, fr, 0.25f, 0.50f, p2x, p2y);

  Waypoint wps[12];
  uint8_t n = 0;

  wps[n++] = { fromEntryX, fromEntryY , -1 };
  wps[n++] = { fx, fy , -1 };
  wps[n++] = { p1x, p1y , -1 };
  wps[n++] = { p2x, p2y , -1 };
  wps[n++] = { p1x, p1y , -1 };
  wps[n++] = { p2x, p2y , -1 };
  wps[n++] = { fx, fy , -1 };

  markMagPickDrop(wps, n, 1, (uint8_t)(n-1));

  markMagPickDrop(wps, n, 1, (uint8_t)(n-1));

    beginMoveSeq(wps, n);

  Serial.print("[PATH] WIGGLE on ");
  Serial.print((char)('a' + ff)); Serial.println(fr);
}

void planSquareMove(uint8_t ff, uint8_t fr, uint8_t tf, uint8_t tr) {
  if (ff == tf && fr == tr) { planWiggleSameSquare(ff, fr); return; }


// ===== Castling (roque) physical sequence =====
// Detect by king moving 2 files on same rank: e-file (ff=4) to g/c, rank 1 or 8.
if (fr == tr && ff == 4 && (fr == 1 || fr == 8) && (int)abs((int)tf - (int)ff) == 2) {
  const uint8_t rank = fr;

  // Determine rook start/end squares in 0-7 file coordinates
  uint8_t rookF0, rookT0, kingT0;
  kingT0 = tf;

  if (tf == 6) { // e -> g (kingside)
    rookF0 = 7;  // h
    rookT0 = 5;  // f
  } else {       // e -> c (queenside)
    rookF0 = 0;  // a
    rookT0 = 3;  // d
  }

  long rookSX, rookSY, rookTX, rookTY, kingSX, kingSY, kingTX, kingTY;
  squareCenterSteps(rookF0, rank, rookSX, rookSY);
  squareCenterSteps(rookT0, rank, rookTX, rookTY);
  squareCenterSteps(ff,    rank, kingSX, kingSY);
  squareCenterSteps(kingT0,rank, kingTX, kingTY);

  // Intersection point: midpoint between rook final and king final (between adjacent squares)
  long ix, iy;
  midpointClamped(rookTX, rookTY, kingTX, kingTY, ix, iy);

  // Build one combined waypoint sequence with waypoint-driven magnet toggles:
  // 1) Go to rook start, pick rook
  // 2) Move rook to intersection, drop rook
  // 3) Go to king start, pick king
  // 4) Move king to final, drop king
  // 5) Go to intersection, pick rook
  // 6) Move rook to final, drop rook
  Waypoint wps[12];
  uint8_t n = 0;

  wps[n++] = { rookSX, rookSY,  1 };   // pick rook
  wps[n++] = { ix,     iy,      0 };   // drop rook at intersection
  wps[n++] = { kingSX, kingSY,  1 };   // pick king
  wps[n++] = { kingTX, kingTY,  0 };   // drop king
  wps[n++] = { ix,     iy,      1 };   // pick rook again
  wps[n++] = { rookTX, rookTY,  0 };   // drop rook at final

  beginMoveSeq(wps, n);

  Serial.print("[PATH] CASTLING rank "); Serial.print(rank);
  Serial.print(" king e"); Serial.print(rank);
  Serial.print(" -> "); Serial.print((char)('a' + tf)); Serial.print(rank);
  Serial.print(" rook "); Serial.print((char)('a' + rookF0)); Serial.print(rank);
  Serial.print(" -> "); Serial.print((char)('a' + rookT0)); Serial.println(rank);
  return;
}

  long xNow, yNow;
  portENTER_CRITICAL(&gMux);
  xNow = g_xAbs;
  yNow = g_yAbs;
  portEXIT_CRITICAL(&gMux);

  long fx, fy, tx, ty;
  squareCenterSteps(ff, fr, fx, fy);
  squareCenterSteps(tf, tr, tx, ty);

  long eLx,eLy,eRx,eRy,eDx,eDy,eUx,eUy;
  squareEdgeMidpoint(ff, fr, -1, 0, eLx, eLy);
  squareEdgeMidpoint(ff, fr, +1, 0, eRx, eRy);
  squareEdgeMidpoint(ff, fr, 0, -1, eDx, eDy);
  squareEdgeMidpoint(ff, fr, 0, +1, eUx, eUy);

  long fromEntryX = eLx, fromEntryY = eLy;
  float best = dist2(xNow,yNow,eLx,eLy);
  float d;
  d = dist2(xNow,yNow,eRx,eRy); if (d < best) { best = d; fromEntryX=eRx; fromEntryY=eRy; }
  d = dist2(xNow,yNow,eDx,eDy); if (d < best) { best = d; fromEntryX=eDx; fromEntryY=eDy; }
  d = dist2(xNow,yNow,eUx,eUy); if (d < best) { best = d; fromEntryX=eUx; fromEntryY=eUy; }

  if (isAdjacent(ff, fr, tf, tr)) {
    Waypoint wps[4];
    uint8_t n = 0;
    wps[n++] = { fromEntryX, fromEntryY , -1 };
    wps[n++] = { fx, fy , -1 };
    wps[n++] = { tx, ty , -1 };
      markMagPickDrop(wps, n, 1, (uint8_t)(n-1));
      appendPostDropNeighbor(wps, n, tf, tr);
      beginMoveSeq(wps, n);

    Serial.print("[PATH] ADJ center->center ");
    Serial.print((char)('a' + ff)); Serial.print(fr);
    Serial.print(" -> ");
    Serial.print((char)('a' + tf)); Serial.println(tr);
    return;
  }


  // ===== Diagonal moves (bishop / diagonal queen / king) =====
  // Follow centers of each intermediate diagonal square (e.g., f1->c4: e2,d3).
  {
    int df = (int)tf - (int)ff;
    int dr = (int)tr - (int)fr;
    int adf = abs(df);
    int adr = abs(dr);
    if (adf == adr && adf > 0) {
      Waypoint wps[MAX_WAYPOINTS];
      uint8_t n = 0;

      int stepF = (df > 0) ? 1 : -1;
      int stepR = (dr > 0) ? 1 : -1;

      long sx, sy;
      squareCenterSteps(ff, fr, sx, sy);
      wps[n++] = { sx, sy, -1 };

      for (int i = 1; i <= adf && n < MAX_WAYPOINTS; i++) {
        squareCenterSteps(ff + i * stepF, fr + i * stepR, sx, sy);
        wps[n++] = { sx, sy, -1 };
      }

      long xMin,xMax,yMin,yMax;
      getXLimits(xMin,xMax);
      getYLimits(yMin,yMax);

      for (uint8_t i=0;i<n;i++){
        wps[i].x = clampl(wps[i].x, xMin + EDGE_STOP_DIST, xMax - EDGE_STOP_DIST);
        wps[i].y = clampl(wps[i].y, yMin + EDGE_STOP_DIST, yMax - EDGE_STOP_DIST);
      }

      markMagPickDrop(wps, n, 0, (uint8_t)(n-1));
      appendPostDropNeighbor(wps, n, tf, tr);
      beginMoveSeq(wps, n);

      Serial.print("[PATH] DIAG centers ");
      Serial.print((char)('a' + ff)); Serial.print(fr);
      Serial.print(" -> ");
      Serial.print((char)('a' + tf)); Serial.println(tr);
      return;
    }
  }

  int dirFile = 0, dirRank = 0;
  if (tf > ff) dirFile = +1;
  if (tf < ff) dirFile = -1;
  if (tr > fr) dirRank = +1;
  if (tr < fr) dirRank = -1;

  long fromExitX, fromExitY;
  float fromExit_u, fromExit_v;
  if (dirFile != 0) {
    squareEdgeMidpoint(ff, fr, dirFile, 0, fromExitX, fromExitY);
    fromExit_u = (dirFile > 0) ? (float)ff + 1.0f : (float)ff;
    fromExit_v = (float)(fr - 1) + 0.5f;
  } else {
    squareEdgeMidpoint(ff, fr, 0, dirRank, fromExitX, fromExitY);
    fromExit_u = (float)ff + 0.5f;
    fromExit_v = (dirRank > 0) ? (float)(fr) : (float)(fr - 1);
  }

  long toEnterFromLeftX,toEnterFromLeftY;
  long toEnterFromRightX,toEnterFromRightY;
  long toEnterFromDownX,toEnterFromDownY;
  long toEnterFromUpX,toEnterFromUpY;
  squareEdgeMidpoint(tf, tr, -1, 0, toEnterFromLeftX,  toEnterFromLeftY);
  squareEdgeMidpoint(tf, tr, +1, 0, toEnterFromRightX, toEnterFromRightY);
  squareEdgeMidpoint(tf, tr, 0, -1, toEnterFromDownX,  toEnterFromDownY);
  squareEdgeMidpoint(tf, tr, 0, +1, toEnterFromUpX,    toEnterFromUpY);

  bool comeFromLeft = (tf >= ff);
  float uEntryH = comeFromLeft ? (float)tf : (float)tf + 1.0f;
  float vEntryMidH = (float)(tr - 1) + 0.5f;
  uEntryH = clampTurnLineU(uEntryH);

  float vTurnA, vTurnB;
  pickTurnLinesV(tr, vTurnA, vTurnB);

  long h_turn1x,h_turn1y,h_turn2x,h_turn2y;
  float LH = pickBestTurnLineV(fromExit_u, fromExit_v, vTurnA, vTurnB, uEntryH, vEntryMidH,
                               fromExitX,fromExitY, h_turn1x,h_turn1y, h_turn2x,h_turn2y);

  long h_entryX = comeFromLeft ? toEnterFromLeftX : toEnterFromRightX;
  long h_entryY = comeFromLeft ? toEnterFromLeftY : toEnterFromRightY;

  bool comeFromDown = (tr >= fr);
  float vEntryV = comeFromDown ? (float)(tr - 1) : (float)(tr);
  float uEntryMidV = (float)tf + 0.5f;
  vEntryV = clampTurnLineV(vEntryV);

  float uTurnA, uTurnB;
  pickTurnLinesU(tf, uTurnA, uTurnB);

  long v_turn1x,v_turn1y,v_turn2x,v_turn2y;
  float LV = pickBestTurnLineU(fromExit_v, fromExit_u, uTurnA, uTurnB, vEntryV, uEntryMidV,
                               fromExitX,fromExitY, v_turn1x,v_turn1y, v_turn2x,v_turn2y);

  long v_entryX = comeFromDown ? toEnterFromDownX : toEnterFromUpX;
  long v_entryY = comeFromDown ? toEnterFromDownY : toEnterFromUpY;

  LH += dist(h_entryX,h_entryY, tx,ty);
  LV += dist(v_entryX,v_entryY, tx,ty);

  bool useH = (LH <= LV);

  Waypoint wps[12];
  uint8_t n = 0;

  wps[n++] = { fromEntryX, fromEntryY , -1 };
  wps[n++] = { fx, fy , -1 };
  wps[n++] = { fromExitX, fromExitY , -1 };

  if (useH) {
    wps[n++] = { h_turn1x, h_turn1y , -1 };
    wps[n++] = { h_turn2x, h_turn2y , -1 };
    wps[n++] = { h_entryX, h_entryY , -1 };
  } else {
    wps[n++] = { v_turn1x, v_turn1y , -1 };
    wps[n++] = { v_turn2x, v_turn2y , -1 };
    wps[n++] = { v_entryX, v_entryY , -1 };
  }

  wps[n++] = { tx, ty , -1 };

  long xMin,xMax,yMin,yMax;
  getXLimits(xMin,xMax);
  getYLimits(yMin,yMax);
  for (uint8_t i=0;i<n;i++){
    wps[i].x = clampl(wps[i].x, xMin + EDGE_STOP_DIST, xMax - EDGE_STOP_DIST);
    wps[i].y = clampl(wps[i].y, yMin + EDGE_STOP_DIST, yMax - EDGE_STOP_DIST);
  }

  markMagPickDrop(wps, n, 1, (uint8_t)(n-1));
  appendPostDropNeighbor(wps, n, tf, tr);

  beginMoveSeq(wps, n);

  Serial.print("[PATH] FROM ");
  Serial.print((char)('a' + ff)); Serial.print(fr);
  Serial.print(" -> TO ");
  Serial.print((char)('a' + tf)); Serial.println(tr);
}