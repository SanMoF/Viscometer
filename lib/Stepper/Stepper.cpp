#include "Stepper.h"

Stepper::Stepper()
{
    current_angle = 0.0f;
    target_angle = 0.0f;
    current_direction = 0;
    current_frequency = 0.0f;
    degrees_per_step = 1.8f;
    max_frequency = 650.0f;
    min_frequency = 5.0f;
    deadband = 0.9f;
    dt_sec = 0.01f;
}

void Stepper::setup(uint8_t dir_gpio, uint8_t step_gpio,
                    uint8_t pwm_channel, TimerConfig *timer_config,
                    float deg_per_step, uint64_t dt_us)
{
    degrees_per_step = deg_per_step;
    deadband = deg_per_step * 0.5f;
    dt_sec = (float)dt_us / 1000000.0f;

    dir_pin.setup(dir_gpio, GPIO);
    pwm_step.setup(step_gpio, pwm_channel, timer_config);

    current_angle = 0.0f;
    target_angle = 0.0f;

    // Set PWM to fixed frequency
    pwm_step.setFrequency(650.0f);
    pwm_step.setDuty(0.0f);
}

void Stepper::moveDegrees(float degrees)
{
    target_angle = current_angle + degrees;
    update();
}

void Stepper::update()
{
    float error = target_angle - current_angle;
    float abs_error = (error >= 0.0f) ? error : -error;

    if (abs_error > deadband)
    {
        if (error > 0.0f)
        {
            current_direction = 1;
            dir_pin.set(1);
        }
        else
        {
            current_direction = -1;
            dir_pin.set(0);
        }

        float frequency = abs_error * 5.0f;

        if (frequency > max_frequency)
            frequency = max_frequency;
        if (frequency < min_frequency)
            frequency = min_frequency;

        current_frequency = frequency;
        float duty = (frequency / max_frequency) * 100.0f;
        if (duty > 100.0f)
            duty = 100.0f;
        if (duty < 15.0f)
            duty = 15.0f;

        pwm_step.setDuty(duty);

        // Update position estimate
        float steps_per_tick = frequency * dt_sec;
        float degrees_per_tick = steps_per_tick * degrees_per_step;

        if (current_direction == 1)
        {
            current_angle += degrees_per_tick;
        }
        else
        {
            current_angle -= degrees_per_tick;
        }
    }
    else
    {
        // Stop
        current_direction = 0;
        current_frequency = 0.0f;
        pwm_step.setDuty(0.0f);
        current_angle = target_angle;
    }
}
float Stepper::getPosition()
{
    return current_angle;
}
