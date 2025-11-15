#include "SimpleGPIO.h"
#include "SimpleADC.h"
#include "SimplePWM.h"
#include "SimpleTimer.h"
#include "Hbridge.h"
#include "QuadratureEncoder.h"
#include "PID_CAYETANO.h"
#include "Viscometer.h"
#include <stdio.h>
#include "esp_timer.h"
#include "esp_task_wdt.h"

SimpleTimer timer;
Viscometer visco1;

uint64_t dt = 20000;

// Make this GLOBAL instead of local to app_main()
static TimerConfig Motor_Timer{
    .timer = LEDC_TIMER_0,
    .frequency = 1000,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

static void IRAM_ATTR timerinterrupt(void *arg)
{
    timer.setInterrupt();
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();

    timer.setup(timerinterrupt, "MainTimer");

    uint8_t Encoder_PINs[2] = {16, 17};
    uint8_t Motor_Pins[2] = {14, 27};
    uint8_t motor_ch[2] = {0, 1};

    visco1.setup(Motor_Pins, motor_ch, Encoder_PINs, &Motor_Timer, dt);

    timer.startPeriodic(dt);

    while (1)
    {
        if (timer.interruptAvailable())
        {
            visco1.measure();
        }
    }
}