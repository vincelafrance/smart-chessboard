#pragma once
#include "Globals.h"

void boardUpdateFromOrigin(long oxAbs, long oyAbs);
// Returns the physical position for dead-zone slot [0..15].
// side = 'L' (whites) or 'R' (blacks). Slot 15 = bas, slot 0 = haut.
// Falls back to a computed position if not yet calibrated.
void deadZoneSlotPos(char side, uint8_t slot, long &xOut, long &yOut);
void boardUpdateFromCorners();   // recompute BO_ boundaries from the 4 independently-stored corner centres
void boardUVToXY(float u, float v, long &xOutAbs, long &yOutAbs);
void squareCenterSteps(uint8_t file0_7, uint8_t rank1_8, long &cxAbs, long &cyAbs);
void squareEdgeMidpoint(uint8_t f0, uint8_t r1, int dirX, int dirY, long &exAbs, long &eyAbs);