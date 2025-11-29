// main.cpp - state machine test with incremental Step_W_PID
#include "definitons.h"
#include <math.h>
#include <inttypes.h>

static void IRAM_ATTR timerinterrupt(void *arg)
{
    timer.setInterrupt();
}

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
// ----------------------------------------------------------------------------
// color_sampling_step()
// Non-blocking sampling + averaging + calibration + 5s polling wait.
// Returns true when sampling + polling finished (Rcal_last/Gcal_last/Bcal_last ready).
// Call repeatedly from your state machine while in READ_COLOR_TAG.
// ----------------------------------------------------------------------------
bool color_sampling_step()
{
    // Config (ajusta si quieres)
    const uint64_t SAMPLE_WINDOW_US = 5000000ULL;  // 5 s sampling window
    const uint64_t SAMPLE_INTERVAL_US = 100000ULL; // 100 ms between samples
    const uint64_t POLL_WAIT_US = 5000000ULL;      // 5 s extra wait (user requested)

    // calibration endpoints (usa tus valores medidos si cambian)
    const uint32_t C_black = 455U;
    const uint16_t R_black = 195;
    const uint16_t G_black = 154;
    const uint16_t B_black = 81;

    const uint32_t C_white = 7086U;
    const uint16_t R_white = 3231U;
    const uint16_t G_white = 3097U;
    const uint16_t B_white = 2297U;

    // estado estático (persiste entre llamadas)
    static uint64_t window_start_us = 0;
    static uint64_t last_sample_us = 0;
    static uint64_t window_done_us = 0; // time when sampling finished
    static uint32_t n_samples = 0;
    static uint64_t sumC = 0;
    static uint64_t sumR = 0;
    static uint64_t sumG = 0;
    static uint64_t sumB = 0;

    uint64_t now_us = esp_timer_get_time();

    // Si no iniciamos la ventana, arrancamos
    if (window_start_us == 0)
    {
        window_start_us = now_us;
        last_sample_us = 0;
        n_samples = 0;
        sumC = sumR = sumG = sumB = 0;
        window_done_us = 0;
        printf("color_sampling_step: sampling window started (%.3f s)\n", SAMPLE_WINDOW_US / 1e6f);
    }

    // Si aún no hemos terminado la ventana de muestreo:
    if (window_done_us == 0)
    {
        // muestreo periódico
        if (last_sample_us == 0 || (now_us - last_sample_us) >= SAMPLE_INTERVAL_US)
        {
            // lee raw (globales C,R,G,B son usados en tu proyecto)
            Color_sensor.readRaw(C, R, G, B);

            sumC += (uint32_t)C;
            sumR += (uint32_t)R;
            sumG += (uint32_t)G;
            sumB += (uint32_t)B;
            n_samples++;

            last_sample_us = now_us;
            printf("color_sampling_step: sample %lu -> C=%u R=%u G=%u B=%u\n", n_samples, C, R, G, B);
        }

        // comprobar final de ventana
        if ((now_us - window_start_us) >= SAMPLE_WINDOW_US)
        {
            if (n_samples == 0)
            {
                // no se tomaron muestras; reiniciamos la ventana para reintentar
                window_start_us = 0;
                last_sample_us = 0;
                return false;
            }

            // promedios
            uint32_t avgC = (uint32_t)((sumC + n_samples / 2) / n_samples);
            uint32_t avgR = (uint32_t)((sumR + n_samples / 2) / n_samples);
            uint32_t avgG = (uint32_t)((sumG + n_samples / 2) / n_samples);
            uint32_t avgB = (uint32_t)((sumB + n_samples / 2) / n_samples);

            printf("color_sampling_step: averaged raw -> C=%lu R=%lu G=%lu B=%lu (n=%lu)\n",
                   avgC, avgR, avgG, avgB, n_samples);

            // calibración a 0..255 usando endpoints
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

            // clamp
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

            // guardar resultados en globals (listas para otros estados)
            Rcal_last = (uint8_t)r_temp;
            Gcal_last = (uint8_t)g_temp;
            Bcal_last = (uint8_t)b_temp;

            printf("color_sampling_step: calibrated -> R:%u G:%u B:%u\n", Rcal_last, Gcal_last, Bcal_last);

            // marcar tiempo de finadlización de muestreo para comenzar el polling
            window_done_us = now_us;

            // liberamos variables de acumulación (pero mantenemos window_done_us)
            last_sample_us = 0;
            n_samples = 0;
            sumC = sumR = sumG = sumB = 0;
        }
        // aún en muestreo: no terminado
        return false;
    }

    // Si llegamos aquí window_done_us != 0 => estamos en periodo de polling (espera 5s)
    if ((now_us - window_done_us) >= POLL_WAIT_US)
    {
        // resetear todo el estado para la próxima vez y retornar finished=true
        window_start_us = 0;
        last_sample_us = 0;
        window_done_us = 0;
        n_samples = 0;
        sumC = sumR = sumG = sumB = 0;

        // Indica que el proceso completo terminó (puedes avanzar de estado)
        return true;
    }

    // todavía esperando polling
    return false;
}

// ============================================================================
// app_main
// ============================================================================
extern "C" void app_main(void)
{
    // app_main (al inicio, una vez)
    if (gpio_install_isr_service(ESP_INTR_FLAG_IRAM) != ESP_OK)
    {
        // Si ya está instalado, gpio_install_isr_service devuelve error; ignorar o loguear
        printf("gpio_isr_service already installed or error\n");
    }

    esp_task_wdt_deinit();

    // compute integer steps-per-rev from STEPPER_DEGREES_PER_STEP
    const uint32_t STEPS_PER_REV = (uint32_t)lroundf(360.0f / 0.225);

    // setup timer and peripherals
    timer.setup(timerinterrupt, "MainTimer");
    visco1.setup(Motor_Pins, motor_ch, Encoder_PINs, &Motor_Timer, dt, ADC_PIN);

    Color_sensor.begin(I2C_NUM_0, 0x29);

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
    Current_state = READ_COLOR_TAG;
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
        {
     
            if (color_sampling_step())
            {
                Current_state = INDICATE_COLOR_LED;
            }
            break;
        }

        case INDICATE_COLOR_LED:
        {
            // Clasificación usando solo ifs y umbrales razonables
            // Umbrales iniciales: ajustar en campo si es necesario
            const uint8_t HIGH_TH = 200;    // si los tres > HIGH_TH -> blanco
            const float DOM_FACTOR = 1.25f; // dominante si 25% mayor
            const uint8_t NOISE_FLOOR = 50; // mínimo para considerarlo significativo

            // inicialmente unknown (0)
            detected_color = 0;

            // WHITE: los tres canales altos
            if (Rcal_last >= HIGH_TH && Gcal_last >= HIGH_TH && Bcal_last >= HIGH_TH)
            {
                detected_color = 1; // WHITE
            }
            else
            {
                // RED: rojo dominante y por encima de ruido
                if ((float)Rcal_last > (float)Gcal_last * DOM_FACTOR &&
                    (float)Rcal_last > (float)Bcal_last * DOM_FACTOR &&
                    Rcal_last > NOISE_FLOOR)
                {
                    detected_color = 3; // RED
                }
                // BLUE: azul dominante y por encima de ruido
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

            // Imprimir resultado usando el string indicado
            if (detected_color == 1)
            {
                printf("INDICATE_COLOR_LED: Detected WHITE\n");
            }
            else if (detected_color == 2)
            {
                printf("INDICATE_COLOR_LED: Detected BLUE\n");
            }
            else if (detected_color == 3)
            {
                printf("INDICATE_COLOR_LED: Detected RED\n");
            }
            else
            {
                printf("INDICATE_COLOR_LED: Detected UNKNOWN\n");
            }

            // avanzar al siguiente estado
            Current_state = LOWER_SPINDLE;
            break;
        }

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
