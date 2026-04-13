#pragma once
#include "Globals.h"

// ============================================================
// ChessTestRun.h
// Automated test sequence that exercises every chess piece
// movement type (pion, cavalier, fou, tour, reine, roi, roque,
// capture, dead zone delivery) using the same path planner and
// magnet system as a real game.
//
// Runs as a dedicated FreeRTOS task.  Respects g_testRunAbortReq
// for graceful abort via the "stop" WebSocket command.
// ============================================================

// Called once at boot (after autoTuneInit).
void chessTestRunInit();

// Start the test sequence.  Returns false if busy.
bool chessTestRunStart();

// Request graceful abort.
void chessTestRunStop();
