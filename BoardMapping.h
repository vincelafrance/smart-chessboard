#pragma once
#include "Globals.h"

void boardUpdateFromOrigin(long oxAbs, long oyAbs);
void boardUVToXY(float u, float v, long &xOutAbs, long &yOutAbs);
void squareCenterSteps(uint8_t file0_7, uint8_t rank1_8, long &cxAbs, long &cyAbs);
void squareEdgeMidpoint(uint8_t f0, uint8_t r1, int dirX, int dirY, long &exAbs, long &eyAbs);