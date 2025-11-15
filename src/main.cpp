#include "definitons.h"


static void IRAM_ATTR timerinterrupt(void *arg)
{
    timer.setInterrupt();
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();
    timer.setup(timerinterrupt, "MainTimer");
    visco1.setup(Motor_Pins, motor_ch, Encoder_PINs, &Motor_Timer, dt,ADC_PIN );

    timer.startPeriodic(dt);
    float speed = 0.0f;
    while (1)
    {
        if (timer.interruptAvailable())
        {
            ViscometerReading visc_read = visco1.measure();
            printf("Speed: %.2f RPM, ADC: %.2f\n", visc_read.rpm, visc_read.viscosity);

        }
    }
}