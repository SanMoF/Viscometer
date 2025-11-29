// main.cpp - Complete state machine with viscosity-based control
#include "definitons.h"
#include <math.h>
#include <inttypes.h>

static void IRAM_ATTR timerinterrupt(void *arg)
{
    timer.setInterrupt();
}

// ---------- Incremental Step_W_PID ----------
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
        return false;
    }

    float u = pid.computedU(error);
    printf("U: %f\n", u);

    float freq = fabsf(u);
    if (freq < min_freq)
        freq = min_freq;
    if (freq > max_freq)
        freq = max_freq;

    float dt_s = ((float)dt);
    float steps_float = freq * dt_s;

    frac_acc += steps_float;

    int32_t steps_to_issue = (int32_t)floorf(frac_acc);
    if (steps_to_issue <= 0)
    {
        return true;
    }

    float degrees_to_move = (float)steps_to_issue * deg_per_step;
    if (error < 0.0f)
        degrees_to_move = -degrees_to_move;
    printf("Frecuency: %f\n", freq);

    stp.moveDegrees(degrees_to_move, (uint32_t)freq);

    frac_acc -= (float)steps_to_issue;

    return true;
}

bool color_sampling_step()
{
    const uint64_t SAMPLE_WINDOW_US = 5000000ULL;  // 5s sampling
    const uint64_t SAMPLE_INTERVAL_US = 100000ULL; // 100ms between samples
    const uint64_t POLL_WAIT_US = 5000000ULL;      // 5s extra wait

    const uint16_t R_black = 195;
    const uint16_t G_black = 154;
    const uint16_t B_black = 81;

    const uint16_t R_white = 3231U;
    const uint16_t G_white = 3097U;
    const uint16_t B_white = 2297U;

    static uint64_t window_start_us = 0;
    static uint64_t last_sample_us = 0;
    static uint64_t window_done_us = 0;
    static uint32_t n_samples = 0;
    static uint64_t sumC = 0;
    static uint64_t sumR = 0;
    static uint64_t sumG = 0;
    static uint64_t sumB = 0;

    uint64_t now_us = esp_timer_get_time();

    if (window_start_us == 0)
    {
        window_start_us = now_us;
        last_sample_us = 0;
        n_samples = 0;
        sumC = sumR = sumG = sumB = 0;
        window_done_us = 0;
        printf("color_sampling_step: sampling window started (5s)\n");
    }

    if (window_done_us == 0)
    {
        if (last_sample_us == 0 || (now_us - last_sample_us) >= SAMPLE_INTERVAL_US)
        {
            Color_sensor.readRaw(C, R, G, B);

            sumC += (uint32_t)C;
            sumR += (uint32_t)R;
            sumG += (uint32_t)G;
            sumB += (uint32_t)B;
            n_samples++;

            last_sample_us = now_us;
            printf("color_sampling_step: sample %lu -> C=%u R=%u G=%u B=%u\n", n_samples, C, R, G, B);
        }

        if ((now_us - window_start_us) >= SAMPLE_WINDOW_US)
        {
            if (n_samples == 0)
            {
                window_start_us = 0;
                last_sample_us = 0;
                return false;
            }

            uint32_t avgC = (uint32_t)((sumC + n_samples / 2) / n_samples);
            uint32_t avgR = (uint32_t)((sumR + n_samples / 2) / n_samples);
            uint32_t avgG = (uint32_t)((sumG + n_samples / 2) / n_samples);
            uint32_t avgB = (uint32_t)((sumB + n_samples / 2) / n_samples);

            printf("color_sampling_step: averaged raw -> C=%lu R=%lu G=%lu B=%lu (n=%lu)\n",
                   avgC, avgR, avgG, avgB, n_samples);

            int denomR = (int)R_white - (int)R_black;
            int denomG = (int)G_white - (int)G_black;
            int denomB = (int)B_white - (int)B_black;

            int r_temp = 0, g_temp = 0, b_temp = 0;
            if (denomR > 0)
            {
                float ratio = ((float)((int)avgR - (int)R_black)) / (float)denomR;
                r_temp = (int)lroundf(ratio * 255.0f);
            }
            if (denomG > 0)
            {
                float ratio = ((float)((int)avgG - (int)G_black)) / (float)denomG;
                g_temp = (int)lroundf(ratio * 255.0f);
            }
            if (denomB > 0)
            {
                float ratio = ((float)((int)avgB - (int)B_black)) / (float)denomB;
                b_temp = (int)lroundf(ratio * 255.0f);
            }

            if (r_temp < 0)
                r_temp = 0;
            else if (r_temp > 255)
                r_temp = 255;
            if (g_temp < 0)
                g_temp = 0;
            else if (g_temp > 255)
                g_temp = 255;
            if (b_temp < 0)
                b_temp = 0;
            else if (b_temp > 255)
                b_temp = 255;

            Rcal_last = (uint8_t)r_temp;
            Gcal_last = (uint8_t)g_temp;
            Bcal_last = (uint8_t)b_temp;

            printf("color_sampling_step: calibrated -> R:%u G:%u B:%u\n", Rcal_last, Gcal_last, Bcal_last);

            window_done_us = now_us;
            last_sample_us = 0;
            n_samples = 0;
            sumC = sumR = sumG = sumB = 0;
        }
        return false;
    }

    if ((now_us - window_done_us) >= POLL_WAIT_US)
    {
        window_start_us = 0;
        last_sample_us = 0;
        window_done_us = 0;
        n_samples = 0;
        sumC = sumR = sumG = sumB = 0;

        return true;
    }

    return false;
}

// ============================================================================
// app_main
// ============================================================================
extern "C" void app_main(void)
{
    if (gpio_install_isr_service(ESP_INTR_FLAG_IRAM) != ESP_OK)
    {
        printf("gpio_isr_service already installed or error\n");
    }

    esp_task_wdt_deinit();

    const uint32_t STEPS_PER_REV = (uint32_t)lroundf(360.0f / 0.225);

    // Setup peripherals
    timer.setup(timerinterrupt, "MainTimer");
    visco1.setup(Motor_Pins, motor_ch, Encoder_PINs, &Motor_Timer, dt, ADC_PIN);
    Color_sensor.begin(I2C_NUM_0, 0x29);

    // Steppers use SHARED timer
    Stepper_Up.setup(STEPPER_UP_PWM_PIN, STEPPER_UP_DIR_PIN, Stepper_UP_CH, &PWM_STEPPER_UP_TIMER, STEPS_PER_REV);
    Stepper_Rot.setup(STEPPER_ROT_PWM_PIN, STEPPER_ROT_DIR_PIN, Stepper_ROT_CH, &PWM_STEPPER_ROT_TIMER, STEPS_PER_REV);

    Pump.setup(Pump_PIns, pump_ch, &Motor_Timer);
    US_Sensor.setup(echo, trig, trig_CH, US_Timer);

    timer.startPeriodic(dt);

    PID_STEPPER.setup(PID_GAINS, dt);
    PID_STEPPER.setULimit(MAX_FREQ);

    Current_state = DETECTION;
    ref = 0.0f;
    adjustment_iterations = 0;

    printf("\n=== VISCOMETER SYSTEM STARTED ===\n");
    printf("Waiting for object detection...\n\n");

    while (1)
    {
        if (!timer.interruptAvailable())
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint64_t elapsed_visc_us = 0;
        uint64_t elapsed_pump_us = 0;
        uint64_t elapsed_stir_us = 0;
        uint64_t elapsed_hold_us = 0;

        switch (Current_state)
        {
        case POWER_OFF:
            break;

        case POWER_ON:
            break;

        case HOMING:
            break;

        case DETECTION:
        {
            static uint32_t last_measure = 0;
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

            if (now - last_measure >= 100)
            {
                float distance = US_Sensor.getDistance() / 10.0f;
                printf("DETECTION: distance=%.2f cm\n", distance);
                last_measure = now;

                if (distance > 0.1f && distance < 10.0f)
                {
                    printf("✓ Object detected at %.2f cm! Moving to READ_COLOR_TAG\n", distance);
                    Current_state = READ_COLOR_TAG;
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
        {
            if (color_sampling_step())
            {
                printf("Color sampling complete, moving to INDICATE_COLOR_LED\n");
                Current_state = INDICATE_COLOR_LED;
            }
            break;
        }

        case INDICATE_COLOR_LED:
        {
            const uint8_t HIGH_TH = 200;
            const float DOM_FACTOR = 1.25f;
            const uint8_t NOISE_FLOOR = 50;

            detected_color = 0;

            if (Rcal_last >= HIGH_TH && Gcal_last >= HIGH_TH && Bcal_last >= HIGH_TH)
            {
                detected_color = 1; // WHITE
            }
            else
            {
                if ((float)Rcal_last > (float)Gcal_last * DOM_FACTOR &&
                    (float)Rcal_last > (float)Bcal_last * DOM_FACTOR &&
                    Rcal_last > NOISE_FLOOR)
                {
                    detected_color = 3; // RED
                }
                else if ((float)Bcal_last > (float)Rcal_last * DOM_FACTOR &&
                         (float)Bcal_last > (float)Gcal_last * DOM_FACTOR &&
                         Bcal_last > NOISE_FLOOR)
                {
                    detected_color = 2; // BLUE
                }
                else
                {
                    detected_color = 0; // UNKNOWN
                }
            }

            if (detected_color == 1)
            {
                printf("INDICATE_COLOR_LED: Detected WHITE - No dilution needed\n");
                target_viscocity = Viscocity_NO_DIl;
            }
            else if (detected_color == 2)
            {
                printf("INDICATE_COLOR_LED: Detected BLUE - 10%% dilution\n");
                target_viscocity = Viscocity_10_DIl;
            }
            else if (detected_color == 3)
            {
                printf("INDICATE_COLOR_LED: Detected RED - 25%% dilution\n");
                target_viscocity = Viscocity_25_DIl;
            }
            else
            {
                printf("INDICATE_COLOR_LED: Detected UNKNOWN - Using default\n");
                target_viscocity = Viscocity_NO_DIl; // Default to no dilution
            }

            printf("Target viscosity set to: %.3f\n", target_viscocity);
            adjustment_iterations = 0; // Reset iteration counter
            Current_state = LOWER_SPINDLE;
            break;
        }

        case MOVE_TO_MEASURE_POS:
            break;

        case LOWER_SPINDLE:
        {
            float target = -4 * 1500 / 3.3;
            bool moving = Step_W_PID(Stepper_Up, PID_STEPPER, target, frac_acc_up,
                                     STEPS_PER_REV, DEGREE_DEADBAND, MIN_FREQ, MAX_FREQ);

            if (!moving)
            {
                visc_start_time = esp_timer_get_time();
                visco1.setTargetSpeed(176.47f); // 30 RPM (180 deg/s)
                printf("Spindle lowered, starting viscometer measurement\n");
                Current_state = MEASURE_VISCOSITY;
            }
            break;
        }

        case MEASURE_VISCOSITY:
        {
            ViscometerReading visc_read = visco1.measure();
            printf("MEASURE_VISCOSITY: rpm=%.1f visc=%.3f target=%.3f\n",
                   visc_read.rpm, visc_read.viscosity, target_viscocity);

            elapsed_visc_us = esp_timer_get_time() - visc_start_time;
            if (elapsed_visc_us >= 5000000) // 5 seconds
            {
                visco1.setTargetSpeed(0.0f); // Stop viscometer

                // Calculate viscosity tolerance (±10%)
                float lower_limit = target_viscocity * 0.9f;
                float upper_limit = target_viscocity * 1.1f;

                printf("Measurement complete: visc=%.3f, range=[%.3f - %.3f]\n",
                       visc_read.viscosity, lower_limit, upper_limit);

                // Check if viscosity is within tolerance
                if (visc_read.viscosity >= lower_limit && visc_read.viscosity <= upper_limit)
                {
                    printf("✓ Viscosity within tolerance - Sample accepted\n");
                    Current_state = ACCEPT_SAMPLE;
                }
                else
                {
                    // Check if we've exceeded max adjustment iterations
                    if (adjustment_iterations >= MAX_ADJUSTMENT_ITERATIONS)
                    {
                        printf("✗ Max adjustments reached - Sample rejected\n");
                        Current_state = REJECT_SAMPLE;
                    }
                    else
                    {
                        printf("⚠ Viscosity out of range - Starting adjustment cycle %d/%d\n",
                               adjustment_iterations + 1, MAX_ADJUSTMENT_ITERATIONS);
                        adjustment_iterations++;
                        pump_start_time = esp_timer_get_time();
                        Pump.setSpeed(60.0f);
                        Current_state = DOSE_WATER;
                    }
                }
            }
            break;
        }

        case DOSE_WATER:
        {
            elapsed_pump_us = esp_timer_get_time() - pump_start_time;
            if (elapsed_pump_us >= 2300000) // 2.3 seconds
            {
                Pump.setSpeed(0.0f);
                printf("Dosing complete, starting high RPM stir\n");
                stir_start_time = esp_timer_get_time();
                visco1.setTargetSpeed(176.47f); // Start stirring at 30 RPM
                Current_state = STIR_HIGH_RPM;
            }
            break;
        }

        case STIR_HIGH_RPM:
        {
            // Keep motor running and monitor
            visco1.measure(); // Keep PID running

            elapsed_stir_us = esp_timer_get_time() - stir_start_time;
            if (elapsed_stir_us >= 10000000) // 10 seconds
            {
                visco1.setTargetSpeed(0.0f); // Stop stirring
                printf("High RPM stir complete, starting hold delay\n");
                hold_start_time = esp_timer_get_time();
                Current_state = STIR_HOLD_DELAY;
            }
            break;
        }

        case STIR_HOLD_DELAY:
        {
            elapsed_hold_us = esp_timer_get_time() - hold_start_time;
            if (elapsed_hold_us >= 30000000) // 30 seconds
            {
                printf("Hold delay complete, re-measuring viscosity (spindle stays down)\n");
                visc_start_time = esp_timer_get_time();
                visco1.setTargetSpeed(176.47f);    // Start measurement immediately
                Current_state = MEASURE_VISCOSITY; // Go back to measure
            }
            break;
        }

        case RE_MEASURE:
        {
            // This state is not used - we go directly from STIR_HOLD_DELAY to MEASURE_VISCOSITY
            // Keeping it here for future use if needed
            break;
        }

        case ACCEPT_SAMPLE:
        {
            printf("=== SAMPLE ACCEPTED ===\n");
            // TODO: Send acceptance signal to Arduino/LabVIEW
            Current_state = RAISE_SPINDLE;
            break;
        }

        case REJECT_SAMPLE:
        {
            printf("=== SAMPLE REJECTED ===\n");
            // TODO: Send rejection signal to Arduino/LabVIEW
            Current_state = RAISE_SPINDLE;
            break;
        }

        case RAISE_SPINDLE:
        {
            float target = 0.0f;
            bool moving = Step_W_PID(Stepper_Up, PID_STEPPER, target, frac_acc_up,
                                     STEPS_PER_REV, DEGREE_DEADBAND, MIN_FREQ, MAX_FREQ);

            if (!moving)
            {
                printf("RAISE_SPINDLE reached home\n");
                printf("Cycle complete, returning to DETECTION\n\n");
                Current_state = DETECTION; // Ready for next sample
            }
            break;
        }

        case MOVE_TO_CLEAN_POS:
            break;

        case CLEANING_RINSE:
            break;

        case EMERGENCY_STOP:
        {
            Stepper_Up.forceStop();
            Stepper_Rot.forceStop();
            Pump.setSpeed(0.0f);
            visco1.setTargetSpeed(0.0f);
            printf("!!! EMERGENCY STOP ACTIVATED !!!\n");
            break;
        }

        case CHECK_ADJUSTMENT_LOOP:
            break;

        case MAINTENANCE_MODE:
            break;

        case CALIBRATE_SENSORS:
            break;

        default:
            break;
        }
    }
}