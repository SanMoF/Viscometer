#include "SimpleGPIO.h"
#include "SimpleADC.h"
#include "SimplePWM.h"
#include "SimpleTimer.h"
#include "Hbridge.h"
#include "QuadratureEncoder.h"
#include "PID_CAYETANO.h"
#include "Viscometer.h"

// Timer ISR
SimpleTimer timer;
HBridge Motor;
PID_CAYETANO PID;
QuadratureEncoder Encoder;
uint64_t dt = 20000;
static void IRAM_ATTR timerinterrupt(void *arg)
{
    timer.setInterrupt();
}

extern "C" void app_main()
{
    //PINS
    uint8_t Encoder_PINs[2] = {16,17};
    uint8_t Motor_Pins[2] = {32,33};
    uint8_t motor_ch [2] = {0,1};

    // Disable watchdog
    esp_task_wdt_deinit();
    float gains[3] = {0.1f,1.0f,0.0f};
    float error, u, ref, current;
    // Timer setup
    PID.setup(gains,dt);
    timer.setup(timerinterrupt, "MainTimer");
    timer.startPeriodic(dt);
    Encoder.setup(Encoder_PINs,0.34);
    Motor.setup(Motor_Pins, motor_ch);
    
    while (1)
    {
        if (timer.interruptAvailable())
        {   
            
        } // timer tick
    } // while(1)
}