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
    Stepper_Up.setup(STEPPER_UP_PWM_PIN,STEPPER_UP_DIR_PIN,Stepper_UP_CH,&PWM_STEPPER_UP_TIMER,STEPPER_DEGREES_PER_STEP);
    Stepper_Rot.setup(STEPPER_ROT_PWM_PIN, STEPPER_ROT_DIR_PIN,Stepper_ROT_CH, &PWM_STEPPER_ROT_TIMER, STEPPER_DEGREES_PER_STEP);
    timer.startPeriodic(dt);
    while (1)
    {
        if (timer.interruptAvailable())
        {
            ViscometerReading visc_read = visco1.measure();
            printf("Speed: %.2f RPM, ADC: %.2f\n", visc_read.rpm, visc_read.viscosity);

        }
    }
}