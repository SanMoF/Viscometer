#include "definitons.h"

static void IRAM_ATTR timerinterrupt(void *arg)
{
    timer.setInterrupt();
}

// incremental Step_W_PID — uses dt (global) as timer period (microseconds)
static void Step_W_PID(
    Stepper &stp,
    PID_CAYETANO &pid,
    float setpoint_deg,
    uint32_t steps_per_rev,
    float deadband_deg = 1.0f,
    float min_freq = 200.0f,
    float max_freq = 2000.0f)
{
    // degrees per step for this stepper
    const float deg_per_step = 360.0f / (float)steps_per_rev;

    // read current position and error
    int32_t pos_steps = stp.getPosition();
    float pos_deg = (pos_steps * 360.0f) / (float)steps_per_rev;
    float error = setpoint_deg - pos_deg;

    // deadband -> hard stop + reset accumulator + reset PID integral
    if (fabsf(error) <= deadband_deg)
    {
        stp.forceStop();
        // reset accumulator for this function instance
        // We keep a static accumulator for a single stepper usage in tests.
        static float frac_acc = 0.0f;
        frac_acc = 0.0f;
        // If your PID has reset method, call it here (preferred)
        // pid.resetIntegral(); // uncomment if available
        return;
    }

    // PID output => desired frequency (Hz)
    float u = pid.computedU(error);

    // safe guard frequency magnitude
    float freq = fabsf(u);
    if (freq < min_freq) freq = min_freq;
    if (freq > max_freq) freq = max_freq;

    // convert dt (global) microseconds -> seconds
    float dt_s = ((float)dt) / 1e6f; // dt is defined in definitions.h

    // steps to issue this tick (float)
    float steps_float = freq * dt_s;

    // static fractional accumulator (single accumulator for test stepper)
    static float fractional_steps_acc = 0.0f;
    fractional_steps_acc += steps_float;

    // integer steps to issue now
    int32_t steps_to_issue = (int32_t)floorf(fractional_steps_acc);
    if (steps_to_issue <= 0)
    {
        // no full steps this tick; nothing to command yet
        return;
    }

    // build degrees to move this tick (preserve sign from error)
    float degrees_to_move = (float)steps_to_issue * deg_per_step;
    if (error < 0.0f) degrees_to_move = -degrees_to_move;

    // command the incremental movement at the desired frequency
    stp.moveDegrees(degrees_to_move, (uint32_t)freq);

    // subtract the integer steps we just issued
    fractional_steps_acc -= (float)steps_to_issue;
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
            }

            switch (Current_state)
            {
            case POWER_OFF:
                (void)0;
                break;

            case POWER_ON:
                (void)0;
                break;

            case HOMING:
                (void)0;
                break;

            case READ_COLOR_TAG:
                Color_sensor.readRaw(C, R, G, B);
                break;

            case INDICATE_COLOR_LED:
                (void)0;
                break;

            case MOVE_TO_MEASURE_POS:
                (void)0;
                break;

            case LOWER_SPINDLE:
                Step_W_PID(Stepper_Up, PID_STEPPER, 500, 400);
                break;

            case RAISE_SPINDLE:
                Step_W_PID(Stepper_Up, PID_STEPPER, 0, 400);
                break;

            case MEASURE_VISCOSITY:
                (void)0;
                {
                    ViscometerReading visc_read = visco1.measure();
                }
                break;

            case EVALUATE_RESULT:
                (void)0;
                break;

            case DOSE_WATER:
                (void)0;
                break;

            case STIR_HIGH_RPM:
                visco1.setTargetSpeed(100.0f);
                break;

            case STIR_HOLD_DELAY:
                (void)0;
                break;

            case RE_MEASURE:
                (void)0;
                break;

            case ACCEPT_SAMPLE:
                (void)0;
                break;

            case REJECT_SAMPLE:
                (void)0;
                break;

            case MOVE_TO_CLEAN_POS:
                (void)0;
                break;

            case CLEANING_RINSE:
                (void)0;
                break;

            case EMERGENCY_STOP:
                (void)0;
                break;

            case CHECK_ADJUSTMENT_LOOP:
                (void)0;
                break;

            case MAINTENANCE_MODE:
                (void)0;
                break;

            case CALIBRATE_SENSORS:
                (void)0;
                break;

            default:
                (void)0;
                break;
            }
        }
    }
}
