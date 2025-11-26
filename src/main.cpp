// main.cpp - state machine test with incremental Step_W_PID
#include "definitons.h"
#include <math.h>
#include <inttypes.h>

static void IRAM_ATTR timerinterrupt(void *arg)
{
    timer.setInterrupt();
}

// ---------- Per-stepper fractional accumulators ----------
float frac_acc_up = 0.0f;
float frac_acc_rot = 0.0f; // unused here but provided for symmetry

// ---------- Incremental Step_W_PID ----------
// Issues integer steps-per-tick based on frequency from PID and preserves fractional remainder.
// Returns true while still moving, false when target reached (deadband).
static bool Step_W_PID(
    Stepper &stp,
    PID_CAYETANO &pid,
    float setpoint_deg,
    float &frac_acc,        // per-stepper fractional accumulator (must persist across ticks)
    uint32_t steps_per_rev, // steps per revolution (integer)
    float deadband_deg = DEGREE_DEADBAND,
    float min_freq = MIN_FREQ,
    float max_freq = MAX_FREQ)
{
    // degrees per step
    const float deg_per_step = 360.0f / (float)steps_per_rev;

    // read current position (steps) and convert to degrees
    int32_t pos_steps = stp.getPosition();
    float pos_deg = (pos_steps * 360.0f) / (float)steps_per_rev;

    // error in degrees
    float error = setpoint_deg - pos_deg;

    // If within deadband -> hard stop and clear accumulator
    if (fabsf(error) <= deadband_deg)
    {
        stp.forceStop();
        frac_acc = 0.0f;
        // If PID offers an integral reset, call it here to avoid windup:
        // pid.resetIntegral();
        return false; // at target (not moving)
    }

    // PID output interpreted as desired frequency (Hz)
    float u = pid.computedU(error);

    // magnitude of frequency and clamp
    float freq = fabsf(u);
    if (freq < min_freq)
        freq = min_freq;
    if (freq > max_freq)
        freq = max_freq;

    // Convert global dt to seconds. dt is defined in definitions.h (you used microseconds before)
    // If your dt is microseconds (as in earlier code), convert accordingly:
    float dt_s = ((float)dt) / 1e6f;

    // Floating number of steps that should be produced this tick
    float steps_float = freq * dt_s;

    // Accumulate fractional steps
    frac_acc += steps_float;

    // Issue only integer full steps this tick
    int32_t steps_to_issue = (int32_t)floorf(frac_acc);
    if (steps_to_issue <= 0)
    {
        // Nothing to do this tick, still moving
        return true;
    }

    // Convert steps to degrees for this tick, keep sign according to error
    float degrees_to_move = (float)steps_to_issue * deg_per_step;
    if (error < 0.0f)
        degrees_to_move = -degrees_to_move;

    // Command movement for this small chunk
    stp.moveDegrees(degrees_to_move, (uint32_t)freq);

    // Subtract issued integer steps from accumulator
    frac_acc -= (float)steps_to_issue;

    return true; // still moving
}

// ============================================================================
// app_main
// ============================================================================
extern "C" void app_main(void)
{
    esp_task_wdt_deinit();

    // compute steps-per-revolution from STEPPER_DEGREES_PER_STEP (1.8 deg/step -> 200 steps/rev)
    // set up timer and peripherals
    timer.setup(timerinterrupt, "MainTimer");
    visco1.setup(Motor_Pins, motor_ch, Encoder_PINs, &Motor_Timer, dt, ADC_PIN);

    // Stepper setup: pass steps-per-rev (integer)
    Stepper_Up.setup(STEPPER_UP_PWM_PIN, STEPPER_UP_DIR_PIN, Stepper_UP_CH, &PWM_STEPPER_UP_TIMER, STEPS_PER_REV);
    Stepper_Rot.setup(STEPPER_ROT_PWM_PIN, STEPPER_ROT_DIR_PIN, Stepper_ROT_CH, &PWM_STEPPER_ROT_TIMER, STEPS_PER_REV);

    timer.startPeriodic(dt);

    // Ensure PID for stepper is set up (use globals from definitions.h)
    PID_STEPPER.setup(PID_GAINS, (int)(dt / 1000000)); // PID.setup expects sample time in ms; convert if dt in us

    while (1)
    {
        if (!timer.interruptAvailable())
        {
            // nothing to do until next tick
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Safe UART read (non-blocking)
        len = UART.available();
        if (len > 0)
        {
        }

        // Example: emergency stop check (you can wire this to a safety input if desired)
        // if (estop_gpio.get() == 1) { Stepper_Up.forceStop(); Stepper_Rot.forceStop(); Current_state = EMERGENCY_STOP; }

        // State machine
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
        {
            // fixed setpoint for lower spindle
            float target = 2 * 360 / 0.8; // same as before
            bool moving = Step_W_PID(Stepper_Up, PID_STEPPER, target, frac_acc_up, STEPS_PER_REV, DEGREE_DEADBAND, MIN_FREQ, MAX_FREQ);

            if (!moving)
            {
                printf("LOWER_SPINDLE reached target\n");
                Current_state = RAISE_SPINDLE; // move to next state only when target reached
            }
            break;
        }

        case RAISE_SPINDLE:
        {
            float target = 0; // back to home
            bool moving = Step_W_PID(Stepper_Up, PID_STEPPER, target, frac_acc_up, STEPS_PER_REV, DEGREE_DEADBAND, MIN_FREQ, MAX_FREQ);

            if (!moving)
            {
                printf("RAISE_SPINDLE reached home\n");
                // you can move to the next state or loop back
                Current_state = POWER_ON; // or whichever state comes next
            }
            break;
        }

        case MEASURE_VISCOSITY:
        {
            (void)0;
            ViscometerReading visc_read = visco1.measure();
            break;
        }

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
        } // end switch
    } // end while
} // end app_main
