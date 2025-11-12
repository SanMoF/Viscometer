// ============================================================================
// Stepper.cpp
// ============================================================================
#include "Stepper.h"
#include <stdio.h>

Stepper::Stepper()
    : _timer_config(nullptr),
      _step_pin(0),
      _dir_pin(0),
      _pwm_channel(0),
      _current_frequency(0),
      _direction(true),
      _calculated_position(0),
      _last_update_time(0),
      _steps_per_revolution(200),
      _target_position(0),
      _is_moving(false)
{
}

Stepper::~Stepper()
{
    _stepperPWM.setDuty(0);
}

void Stepper::setup(uint8_t step_pin, uint8_t dir_pin, uint8_t pwm_channel, TimerConfig *timer_config, uint32_t steps_per_rev)
{
    degrees_per_step = deg_per_step;
    deadband = deg_per_step;
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

void Stepper::moveDegrees(float degrees, uint32_t frequency)
{
    // Update position before starting new move
    if (_is_moving)
    {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t elapsed_ms = current_time - _last_update_time;
        int32_t steps_moved = (_current_frequency * elapsed_ms) / 1000;
        
        if (_direction)
            _calculated_position += steps_moved;
        else
            _calculated_position -= steps_moved;
    }
    
    // Convert degrees to steps
    int32_t steps = (int32_t)((degrees / 360.0f) * _steps_per_revolution);
    _target_position = _calculated_position + steps;
    
    // Set direction
    _direction = (steps >= 0);
    _dirGPIO.set(_direction ? 1 : 0);
    
    // Set frequency
    _current_frequency = frequency;
    _stepperPWM.setFrequency(frequency);
    
    // Start movement
    _stepperPWM.setDuty(50.0f);
    _is_moving = true;
    _last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    printf("Moving %.2f degrees (%ld steps) at %ld Hz, target pos: %ld\n", 
           degrees, steps, frequency, _target_position);
}

int32_t Stepper::getPosition()
{
    if (_is_moving)
    {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t elapsed_ms = current_time - _last_update_time;
        
        int32_t steps_moved = (_current_frequency * elapsed_ms) / 1000;
        
        int32_t new_position;
        if (_direction)
            new_position = _calculated_position + steps_moved;
        else
            new_position = _calculated_position - steps_moved;
        
        // Check if target reached
        if ((_direction && new_position >= _target_position) ||
            (!_direction && new_position <= _target_position))
        {
            _calculated_position = _target_position;
            _stepperPWM.setDuty(0);
            _is_moving = false;
            printf("Target reached. Position: %ld\n", _calculated_position);
        }
        else
        {
            _calculated_position = new_position;
            _last_update_time = current_time;
        }
    }
    
    return _calculated_position;
}
float Stepper::encoder(){
    
}