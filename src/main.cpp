// main.cpp - standalone test: rotate Stepper_Rot +90deg (persist in test)
#include "definitons.h"
#include <math.h>
#include <inttypes.h>

static void IRAM_ATTR timerinterrupt(void *arg)
{
    timer.setInterrupt();
}

// ---------- Incremental Step_W_PID (fixed dt units) ----------
bool Step_W_PID(
    Stepper &stp,
    PID_CAYETANO &pid,
    float setpoint_deg,
    float &frac_acc,
    uint32_t steps_per_rev,
    float deadband_deg = DEGREE_DEADBAND,
    float min_freq = MIN_FREQ,
    float max_freq = MAX_FREQ)
{
    const float deg_per_step = 360.0f / (float)steps_per_rev;

    int32_t pos_steps = stp.getPosition();
    float pos_deg = (pos_steps * 360.0f) / (float)steps_per_rev;

    float error = setpoint_deg - pos_deg;

    if (fabsf(error) <= deadband_deg)
    {
        stp.forceStop();
        frac_acc = 0.0f;
        return false; // finished
    }

    float u = pid.computedU(error);
    printf("Step_W_PID: pos_deg=%.3f set=%.3f err=%.3f U=%.3f\n", pos_deg, setpoint_deg, error, u);

    float freq = fabsf(u);
    if (freq < min_freq)
        freq = min_freq;
    if (freq > max_freq)
        freq = max_freq;

    // FIX: convert dt (microseconds) to seconds
    float dt_s = ((float)dt) / 1e6f;

    // steps to issue this tick (floating)
    float steps_float = freq * dt_s;

    frac_acc += steps_float;

    int32_t steps_to_issue = (int32_t)floorf(frac_acc);
    if (steps_to_issue <= 0)
    {
        // nothing to issue yet
        return true; // still moving
    }

    float degrees_to_move = (float)steps_to_issue * deg_per_step;
    if (error < 0.0f)
        degrees_to_move = -degrees_to_move;

    printf("Step_W_PID: issuing %ld steps -> %.3f deg @ freq=%.1f Hz\n", steps_to_issue, degrees_to_move, freq);
    stp.moveDegrees(degrees_to_move, (uint32_t)freq);

    frac_acc -= (float)steps_to_issue;

    return true; // still moving
}

// ----------------------------------------------------------------------------
// rotate_stepper_relative
// Non-blocking helper to rotate `stp` by delta_deg relative to current pos.
// Uses pid and the provided fractional accumulator (frac_acc). Returns true when finished.
// ----------------------------------------------------------------------------
bool rotate_stepper_relative(Stepper &stp, PID_CAYETANO &pid,
                             float delta_deg, float &frac_acc,
                             uint32_t steps_per_rev,
                             float deadband_deg = 0.5f,
                             float min_freq = 20.0f, float max_freq = 600.0f)
{
    // persistent state for this helper invocation
    static bool active = false;
    static float target_deg = 0.0f;

    int32_t pos_steps = stp.getPosition();
    float pos_deg = (pos_steps * 360.0f) / (float)steps_per_rev;

    if (!active)
    {
        target_deg = pos_deg + delta_deg;
        active = true;
        frac_acc = 0.0f;
        printf("rotate_stepper_relative: start pos=%.3f -> target=%.3f (delta=%.3f)\n", pos_deg, target_deg, delta_deg);
    }

    bool still_moving = Step_W_PID(stp, pid, target_deg, frac_acc, steps_per_rev, deadband_deg, min_freq, max_freq);

    if (!still_moving)
    {
        active = false;
        printf("rotate_stepper_relative: reached target %.3f deg\n", target_deg);
        return true;
    }
    return false;
}

// ============================================================================
// app_main - standalone rotation test
// ============================================================================
extern "C" void app_main(void)
{
    // install gpio ISR service if needed
    if (gpio_install_isr_service(ESP_INTR_FLAG_IRAM) != ESP_OK)
    {
        printf("gpio_isr_service already installed or returned error (ignored)\n");
    }

    esp_task_wdt_deinit();

    // compute steps per rev from the constant in definitions.h
    const uint32_t STEPS_PER_REV = (uint32_t)lroundf(360.0f / STEPPER_DEGREES_PER_STEP);

    // setup peripherals (uses definitions.h globals)
    timer.setup(timerinterrupt, "MainTimer");
    visco1.setup(Motor_Pins, motor_ch, Encoder_PINs, &Motor_Timer, dt, ADC_PIN);

    Color_sensor.begin(I2C_NUM_0, 0x29); // optional, safe to keep

    Stepper_Up.setup(STEPPER_UP_PWM_PIN, STEPPER_UP_DIR_PIN, Stepper_UP_CH, &PWM_STEPPER_UP_TIMER, STEPS_PER_REV);
    Stepper_Rot.setup(STEPPER_ROT_PWM_PIN, STEPPER_ROT_DIR_PIN, Stepper_ROT_CH, &PWM_STEPPER_ROT_TIMER, 200);

    Pump.setup(Pump_PIns, pump_ch, &Motor_Timer);
    US_Sensor.setup(echo, trig, trig_CH, US_Timer);

    timer.startPeriodic(dt);

    // PID setup (dt in microseconds; PID_CAYETANO expects dt in microseconds per your code)
    PID_STEPPER.setup(PID_GAINS, dt);
    PID_STEPPER.setULimit(MAX_FREQ);

    printf("\n=== ROTATION TEST START ===\n");
    printf("Will rotate Stepper_Rot by +90 degrees (non-blocking). Logs follow.\n\n");

    // Ensure accumulator is cleared
    frac_acc_rot = 0.0f;

    // We'll persist in this test loop forever so you can observe behavior.
    bool rotation_complete = false;

    while (1)
    {
        if (!timer.interruptAvailable())
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // call the rotation helper until it reports completion
        if (!rotation_complete)
        {
            rotation_complete = rotate_stepper_relative(Stepper_Rot, PID_STEPPER,
                                                       -900.0f,        // Approx +90 deg
                                                       frac_acc_rot, // rotation accumulator
                                                       200,
                                                       3.0f,   // deadband (deg)
                                                       20.0f,  // min freq Hz
                                                       600.0f); // max freq Hz

            if (rotation_complete)
            {
                printf("\n=== Rotation finished. Stepper_Rot should be at +90 deg relative to start. ===\n");
            }
        }
        else
        {
            // persist here: print occasional status so you can confirm it's not moving
            static uint64_t last_print = 0;
            uint64_t now_us = esp_timer_get_time();
            if (last_print == 0 || (now_us - last_print) >= 1000000ULL) // every 1s
            {
                int32_t pos_steps = Stepper_Rot.getPosition();
                float pos_deg = (pos_steps * 360.0f) / (float)STEPS_PER_REV;
                printf("Status: Stepper_Rot pos_steps=%ld pos_deg=%.3f\n", (long)pos_steps, pos_deg);
                last_print = now_us;
            }
        }

        // continue loop: non-blocking, no state machine
    }
}
