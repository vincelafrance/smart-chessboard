#pragma once
#include "Globals.h"

void beginMoveSeq(const Waypoint *wps, uint8_t n);
void abortPath();

void planSquareMove(uint8_t ff, uint8_t fr, uint8_t tf, uint8_t tr);
void planSquareMoveDirect(uint8_t ff, uint8_t fr, uint8_t tf, uint8_t tr);