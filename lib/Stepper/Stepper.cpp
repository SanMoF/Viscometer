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
    max_acceleration = 1000.0f;
    deadband = 0.9f;
    dt_sec = 0.01f;
    kp = 5.0f;
}

void Stepper::setup(uint8_t dir_gpio, uint8_t step_gpio,
                    uint8_t pwm_channel, TimerConfig *timer_config,
                    float deg_per_step, uint64_t dt_us,
                    float proportional_gain, float max_freq,
                    float min_freq, float max_accel)
{
    degrees_per_step = deg_per_step;
    deadband = deg_per_step * 1.0f;
    dt_sec = (float)dt_us / 1000000.0f;
    
    kp = proportional_gain;
    max_frequency = max_freq;
    min_frequency = min_freq;
    max_acceleration = max_accel;

    dir_pin.setup(dir_gpio, GPIO);
    pwm_step.setup(step_gpio, pwm_channel, timer_config);

    current_angle = 0.0f;
    target_angle = 0.0f;
    current_frequency = 0.0f;

    // Set PWM to fixed 50% duty cycle
    pwm_step.setDuty(50.0f);
}

void Stepper::moveDegrees(float degrees)
{
    target_angle = current_angle + degrees;
}

void Stepper::moveToAngle(float angle)
{
    target_angle = angle;
}

void Stepper::update()
{
    float error = target_angle - current_angle;
    float abs_error = fabsf(error);

    if (abs_error > deadband)
    {
        // Set direction based on error sign
        current_direction = (error > 0.0f) ? 1 : -1;
        dir_pin.set(current_direction == 1);

        // Calculate desired frequency using proportional control
        float desired_frequency = abs_error * kp;
        
        // Constrain frequency to operating range
        if (desired_frequency > max_frequency)
            desired_frequency = max_frequency;
        if (desired_frequency < min_frequency)
            desired_frequency = min_frequency;

        // Apply acceleration limiting
        float max_freq_change = max_acceleration * dt_sec;
        float freq_diff = desired_frequency - current_frequency;
        
        if (freq_diff > max_freq_change)
        {
            current_frequency += max_freq_change;
        }
        else if (freq_diff < -max_freq_change)
        {
            current_frequency -= max_freq_change;
        }
        else
        {
            current_frequency = desired_frequency;
        }

        // Set PWM frequency (this controls step rate)
        pwm_step.setFrequency(current_frequency);

        // Update position estimate based on actual frequency
        float steps_per_tick = current_frequency * dt_sec;
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
        // Stop motor - reached target within deadband
        current_direction = 0;
        current_frequency = 0.0f;
        pwm_step.setFrequency(0.0f);
        current_angle = target_angle;
    }
}

float Stepper::getPosition()
{
    return current_angle;
}

void Stepper::stop()
{
    target_angle = current_angle;
    current_frequency = 0.0f;
    current_direction = 0;
    pwm_step.setFrequency(0.0f);
}