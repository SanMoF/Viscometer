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
float frac_acc_rot = 0.0f; // unused here but kept for symmetry

// ---------- Persistent timestamps (microseconds) ----------
uint64_t visc_start_time = 0; // when viscometer measurement started
uint64_t pump_start_time = 0; // when pump dosing started

// ---------- Incremental Step_W_PID ----------
// Issues integer steps-per-tick based on frequency from PID and preserves fractional remainder.
// Returns true while still moving, false when target reached (deadband).
bool Step_W_PID(
    Stepper &stp,
    PID_CAYETANO &pid,
    float setpoint_deg,
    float &frac_acc,        // per-stepper fractional accumulator (must persist across ticks)
    uint32_t steps_per_rev, // steps per revolution (integer)
    float deadband_deg = DEGREE_DEADBAND,
    float min_freq = MIN_FREQ,
    float max_freq = MAX_FREQ)
{
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
        return false; // at target (not moving)
    }

    // PID output interpreted as desired frequency (Hz)
    float u = pid.computedU(error);
    printf("U: %f\n", u);

    // magnitude of frequency and clamp
    float freq = fabsf(u);
    if (freq < min_freq)
        freq = min_freq;
    if (freq > max_freq)
        freq = max_freq;

    // dt is global (microseconds) from definitions.h — convert to seconds
    float dt_s = ((float)dt);

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
    printf("Frecuency: %f\n", freq);
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

    // compute integer steps-per-rev from STEPPER_DEGREES_PER_STEP
    const uint32_t STEPS_PER_REV = (uint32_t)lroundf(360.0f / 0.225);

    // setup timer and peripherals
    timer.setup(timerinterrupt, "MainTimer");
    visco1.setup(Motor_Pins, motor_ch, Encoder_PINs, &Motor_Timer, dt, ADC_PIN);

    // Stepper & pump setup
    Stepper_Up.setup(STEPPER_UP_PWM_PIN, STEPPER_UP_DIR_PIN, Stepper_UP_CH, &PWM_STEPPER_UP_TIMER, STEPS_PER_REV);
    Stepper_Rot.setup(STEPPER_ROT_PWM_PIN, STEPPER_ROT_DIR_PIN, Stepper_ROT_CH, &PWM_STEPPER_ROT_TIMER, STEPS_PER_REV);

    // If you have a Pump object declared in definitons.h:
    Pump.setup(Pump_PIns, pump_ch, &Motor_Timer);
    US_Sensor.setup(echo, trig, trig_CH, US_Timer);

    timer.startPeriodic(dt);

    // Setup PID with dt directly in microseconds
    PID_STEPPER.setup(PID_GAINS, dt); // dt is already in microseconds!
    PID_STEPPER.setULimit(MAX_FREQ);  // Set to 1000 Hz (or whatever MAX_FREQ is)

    // start test: lower spindle first
    Current_state = DETECTION;
    ref = 0.0f;

    while (1)
    {
        if (!timer.interruptAvailable())
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Non-blocking UART read
        len = UART.available();
        if (len > 0)
        {
            if (len > (int)sizeof(Buffer) - 1)
                len = sizeof(Buffer) - 1;
            int r = UART.read(Buffer, len);
            if (r > 0)
                Buffer[r] = '\0';
            // optional: parse commands: e.g., sscanf(Buffer, "%d,%f", &mode, &ref);
        }

        uint64_t elapsed_visc_us = 0;
        uint64_t elapsed_pump_us = 0;

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
        case DETECTION:
        {
            static uint32_t last_measure = 0;
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

            if (now - last_measure >= 100)
            {
                float distance = US_Sensor.getDistance() / 10.0f;

                // Simple debug output (can't access private _echo_time)
                printf("DETECTION: distance=%.2f cm\n", distance);

                last_measure = now;

                if (distance > 0.1f && distance < 10.0f)
                {
                    printf("✓ Object detected at %.2f cm! Moving to LOWER_SPINDLE\n", distance);
                    Current_state = LOWER_SPINDLE;
                }
                else if (distance == 0.0f)
                {
                    printf("  WARNING: No echo received (distance = 0)\n");
                }
                else
                {
                    printf("  Waiting... (need distance < 10 cm)\n");
                }
            }
            break;  
        }
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
            // example target (keep same conversion you used earlier)
            float target = -4 * 1500 / 3.3; //-3*20000/2.6;
            bool moving = Step_W_PID(Stepper_Up, PID_STEPPER, target, frac_acc_up, STEPS_PER_REV, DEGREE_DEADBAND, MIN_FREQ, MAX_FREQ);

            if (!moving)
            {
                // reached lower position -> start viscometer measurement
                visc_start_time = esp_timer_get_time();
                visco1.setTargetSpeed(100.0f); // spin viscometer
                Current_state = MEASURE_VISCOSITY;
            }
            break;
        }

        case MEASURE_VISCOSITY:
        {
            // measure periodically
            ViscometerReading visc_read = visco1.measure();
            printf("MEASURE_VISCOSITY: rpm=%.1f visc=%.3f\n", visc_read.rpm, visc_read.viscosity);

            // compute elapsed time (use previously declared variable)
            elapsed_visc_us = esp_timer_get_time() - visc_start_time;
            if (elapsed_visc_us >= 5000000) // 5 seconds
            {
                visco1.setTargetSpeed(0.0f);            // stop viscometer
                pump_start_time = esp_timer_get_time(); // start pump timer
                Pump.setSpeed(60.0f);                   // start pump
                Current_state = DOSE_WATER;
            }
            break;
        }

        case DOSE_WATER:
        {
            // check pump elapsed time
            elapsed_pump_us = esp_timer_get_time() - pump_start_time;
            if (elapsed_pump_us >= 2300000) // 5 seconds
            {
                Pump.setSpeed(0.0f);
                Current_state = RAISE_SPINDLE;
            }
            break;
        }

        case RAISE_SPINDLE:
        {
            float target = 0.0f; // back home
            bool moving = Step_W_PID(Stepper_Up, PID_STEPPER, target, frac_acc_up, STEPS_PER_REV, DEGREE_DEADBAND, MIN_FREQ, MAX_FREQ);

            if (!moving)
            {
                printf("RAISE_SPINDLE reached home\n");
                Current_state = POWER_ON; // next state
            }
            break;
        }

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
            // hard stop all actuators
            Stepper_Up.forceStop();
            Stepper_Rot.forceStop();
            Pump.setSpeed(0.0f);
            visco1.setTargetSpeed(0.0f);
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
