#include "definitons.h"

static void IRAM_ATTR timerinterrupt(void *arg)
{
    timer.setInterrupt();
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();
    timer.setup(timerinterrupt, "MainTimer");
    visco1.setup(Motor_Pins, motor_ch, Encoder_PINs, &Motor_Timer, dt, ADC_PIN);
    Stepper_Up.setup(STEPPER_UP_PWM_PIN, STEPPER_UP_DIR_PIN, Stepper_UP_CH, &PWM_STEPPER_UP_TIMER, STEPPER_DEGREES_PER_STEP);
    Stepper_Rot.setup(STEPPER_ROT_PWM_PIN, STEPPER_ROT_DIR_PIN, Stepper_ROT_CH, &PWM_STEPPER_ROT_TIMER, STEPPER_DEGREES_PER_STEP);
    timer.startPeriodic(dt);
    // Useful variables
    int len = 0;
    int mode = 0;
    float ref = 0.0f;      // incoming reference (mode-dependent)
    char Buffer[64] = {0}; // ensure this exists (or use the one in your header)
    const float DEGREE_DEADBAND = 1.0f;
    const float MIN_FREQ = 100.0f;
    const float MAX_FREQ = 2000.0f;
    while (1)
    {
        if (timer.interruptAvailable())
        {

            len = UART.available();
            if (len)
            {
                UART.read(Buffer, len);
                sscanf(Buffer, "%d,%f", &mode, &ref);
            }
            if (mode == 0)
            {
                ViscometerReading visc_read = visco1.measure();
                printf("Speed: %.2f RPM, ADC: %.2f\n", visc_read.rpm, visc_read.viscosity);
            }
            else if (mode == 1)
            {
                visco1.setTargetSpeed(100.0f);
            }
            else if (mode == 2)
            {
                // Mode 2: Move Stepper_Up to ref degrees
                int32_t current_steps_up = Stepper_Up.getPosition();
                float current_degrees_up = (current_steps_up * 360.0f) / (float)STEPPER_DEGREES_PER_STEP;
                float error_up = ref - current_degrees_up;

                if ((error_up > DEGREE_DEADBAND) || (error_up < -DEGREE_DEADBAND))
                {
                    float u = PID_STEPPER.computedU(error_up);
                    Stepper_Up.moveDegrees(error_up, (uint32_t)u);
                    printf("Mode 2 - Stepper UP -> Set: %.2f°, Pos: %.2f°, Err: %.2f°, Freq: %.0f Hz\n",
                           ref, current_degrees_up, error_up, u);
                }
                else
                {
                    static int print_counter_up = 0;
                    if ((print_counter_up++ % 10) == 0)
                        printf("Mode 2 - Stepper UP at target: %.2f° (setpoint %.2f°)\n", current_degrees_up, ref);
                }
            }
        }
    }
}