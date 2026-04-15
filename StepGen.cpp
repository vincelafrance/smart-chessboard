#include "StepGen.h"
#include "Globals.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

// When period_us == 0 the ISR reschedules itself at this idle interval so it
// picks up a new period quickly when motion resumes.  200 µs ≈ 5000 polls/s —
// negligible CPU load (~0.2 %) while keeping start latency under one tick.
static const uint32_t IDLE_POLL_US = 200;

struct MotorState {
  esp_timer_handle_t timer;
  gpio_num_t         stepPin;
  gpio_num_t         dirPin;
  volatile long*     pPos;       // pointer to g_Apos or g_Bpos
  volatile uint32_t  period_us;  // 0 = stopped; written by task, read by ISR
  volatile int8_t    dir;        // +1 or -1;   written by task, read by ISR
};

static MotorState s_A, s_B;

// ---------------------------------------------------------------------------
// Timer callback — runs in the esp_timer task at priority 22, much higher
// than the step task (priority 3) or WiFi (priority 23).  Jitter is a few
// µs vs hundreds of µs with the old software polling loop.
// ESP_TIMER_ISR (true ISR, sub-µs jitter) requires ESP-IDF ≥4.4; fall back
// to ESP_TIMER_TASK which is still vastly better than a FreeRTOS task loop.
// ---------------------------------------------------------------------------
static void IRAM_ATTR motor_cb(void* arg) {
  MotorState* m = (MotorState*)arg;
  uint32_t p = m->period_us;

  if (p == 0) {
    // Motor stopped — keep polling so we restart quickly.
    esp_timer_start_once(m->timer, IDLE_POLL_US);
    return;
  }

  int8_t d = m->dir;

  // 1. Set direction.
  //    TMC2209 requires ≥20 ns between DIR stable and STEP rising edge.
  //    The subsequent instructions provide well over this margin.
  gpio_set_level(m->dirPin, d > 0 ? 1 : 0);

  // 2. Step pulse — TMC2209 requires ≥1.9 µs high time.
  gpio_set_level(m->stepPin, 1);
  esp_rom_delay_us(2);
  gpio_set_level(m->stepPin, 0);

  // 3. Re-arm immediately after the pulse for minimal inter-step jitter.
  //    Re-read period to pick up any update from the velocity task.
  uint32_t nextP = m->period_us;
  esp_timer_start_once(m->timer, nextP > 0 ? nextP : IDLE_POLL_US);

  // 4. Update position counter — use task-safe critical section since
  //    ESP_TIMER_TASK runs in a FreeRTOS task context, not an ISR.
  portENTER_CRITICAL(&gMux);
  *(m->pPos) += (d > 0 ? 1 : -1);
  portEXIT_CRITICAL(&gMux);
}

static void createMotor(MotorState* m, const char* name) {
  esp_timer_create_args_t args = {};
  args.callback              = motor_cb;
  args.arg                   = m;
  args.dispatch_method       = ESP_TIMER_TASK;  // high-priority timer task (priority 22)
  args.name                  = name;
  args.skip_unhandled_events = true;  // never burst-fire if callback runs long
  esp_timer_create(&args, &m->timer);
  esp_timer_start_once(m->timer, IDLE_POLL_US);  // start in idle-polling mode
}

void stepGenInit() {
  s_A = { nullptr, (gpio_num_t)LEFT_STEP,  (gpio_num_t)LEFT_DIR,  &g_Apos, 0, +1 };
  s_B = { nullptr, (gpio_num_t)RIGHT_STEP, (gpio_num_t)RIGHT_DIR, &g_Bpos, 0, +1 };
  createMotor(&s_A, "stepA");
  createMotor(&s_B, "stepB");
}

// Called by the step task every ~1 ms to update desired motor parameters.
// Both writes are to volatile variables — the ISR picks them up on its next
// callback without any mutex (32-bit and 8-bit writes are atomic on Xtensa).
void stepGenSetA(uint32_t period_us, int8_t dir) {
  s_A.dir       = dir;
  s_A.period_us = period_us;
}

void stepGenSetB(uint32_t period_us, int8_t dir) {
  s_B.dir       = dir;
  s_B.period_us = period_us;
}
