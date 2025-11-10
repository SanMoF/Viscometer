// Stepper.h
#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "SimpleGPIO.h"
#include "SimplePWM.h"

class Stepper
{
private:
    SimpleGPIO dir_pin;
    SimplePWM pwm_step;
    
    float degrees_per_step;
    float max_frequency;
    float min_frequency;
    float deadband;
    
    float current_angle;
    float target_angle;
    int current_direction;
    float current_frequency;
    float dt_sec;
    
    
    public:
    Stepper();
    
    void setup(uint8_t dir_gpio, uint8_t step_gpio,
        uint8_t pwm_channel, TimerConfig* timer_config,
        float deg_per_step, uint64_t dt_us);
        
        void update();
    void moveDegrees(float degrees);
    float getPosition() ;

};

#endif // __STEPPER_H__
