#pragma once
#include <stdint.h>

// Hardware step-pulse generator — ESP32 esp_timer ISR dispatch.
//
// Two independent timers (one per motor) fire at exactly the requested period
// regardless of FreeRTOS scheduling, WiFi preemptions or task jitter.
// The step task calls stepGenSetA/B every ~1 ms to update the desired period
// and direction; the ISRs pick up the new values on the next callback.
//
// period_us = 0  →  motor stopped (timer keeps polling at idle rate so it
//                   can restart quickly when a non-zero period is set).
// dir            →  +1 forward, -1 reverse (ignored when period_us = 0).

void stepGenInit();
void stepGenSetA(uint32_t period_us, int8_t dir);
void stepGenSetB(uint32_t period_us, int8_t dir);
