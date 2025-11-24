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

    while (1)
    {
        if (timer.interruptAvailable())
        {

            len = UART.available();
            if (len)
            {
                UART.read(Buffer, len);
                // Scan and change mode swithc between automatic or manual
                // sscanf(Buffer, "%d,%f", &mode, &ref);
            }

            switch (Current_state)
            {
            case POWER_OFF:

            break;
            case POWER_ON:
                break;
            case HOMING:
                break;
            case READ_COLOR_TAG:
                Color_sensor.readRaw(C, R, G, B);

                break;
            case INDICATE_COLOR_LED:
                break;
            case MOVE_TO_MEASURE_POS:
                int32_t current_steps_up = Stepper_Up.getPosition();
                float current_degrees_up = (current_steps_up * 360.0f) / (float)STEPPER_DEGREES_PER_STEP;
                float error_up = ref - current_degrees_up;

                if ((error_up > DEGREE_DEADBAND) || (error_up < -DEGREE_DEADBAND))
                {
                    float u = PID_STEPPER.computedU(error_up);
                    Stepper_Up.moveDegrees(error_up, (uint32_t)u);
                }
                break;
                break;
            case LOWER_SPINDLE:
                // Mode 2: Move Stepper_Up to ref degrees
                int32_t current_steps_up = Stepper_Up.getPosition();
                float current_degrees_up = (current_steps_up * 360.0f) / (float)STEPPER_DEGREES_PER_STEP;
                float error_up = 0 - current_degrees_up;

                if ((error_up > DEGREE_DEADBAND) || (error_up < -DEGREE_DEADBAND))
                {
                    float u = PID_STEPPER.computedU(error_up);
                    Stepper_Up.moveDegrees(error_up, (uint32_t)u);
                }
                break;
            case RAISE_SPINDLE:
                // Mode 2: Move Stepper_Up to ref degrees
                int32_t current_steps_up = Stepper_Up.getPosition();
                float current_degrees_up = (current_steps_up * 360.0f) / (float)STEPPER_DEGREES_PER_STEP;
                float error_up = ref - current_degrees_up;

                if ((error_up > DEGREE_DEADBAND) || (error_up < -DEGREE_DEADBAND))
                {
                    float u = PID_STEPPER.computedU(error_up);
                    Stepper_Up.moveDegrees(error_up, (uint32_t)u);
                }
                break;
            case MEASURE_VISCOSITY:
                ViscometerReading visc_read = visco1.measure();
                break;
            case EVALUATE_RESULT:
                
                break;
            case DOSE_WATER:

                break;
            case STIR_HIGH_RPM:
                visco1.setTargetSpeed(100.0f);

                break;
            case STIR_HOLD_DELAY:
                break;
            case RE_MEASURE:
                break;
            case ACCEPT_SAMPLE:
                break;
            case REJECT_SAMPLE:
                break;
            case MOVE_TO_CLEAN_POS:
                break;
            case CLEANING_RINSE:
                break;
            case EMERGENCY_STOP:

                break;

            default:
                break;
            }
        }
    }
}
