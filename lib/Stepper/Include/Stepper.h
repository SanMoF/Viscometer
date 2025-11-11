// Stepper.h
#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include "cmath"
class Stepper
{
private:
    SimpleGPIO dir_pin;
    SimplePWM pwm_step;
    
    float degrees_per_step;
    float max_frequency;
    float min_frequency;
    float max_acceleration;
    float deadband;
    float kp;
    
    float current_angle;
    float target_angle;
    int current_direction;
    float current_frequency;
    float dt_sec;
    
public:
    Stepper();
    
    void setup(uint8_t dir_gpio, uint8_t step_gpio,
               uint8_t pwm_channel, TimerConfig* timer_config,
               float deg_per_step, uint64_t dt_us,
               float proportional_gain = 5.0f,
               float max_freq = 650.0f,
               float min_freq = 5.0f,
               float max_accel = 1000.0f);
    
    void update();
    void moveDegrees(float degrees);
    void moveToAngle(float angle);
    float getPosition();
    void stop();
};

#endif // __STEPPER_H__