#ifndef _HBRIDGE_H
#define _HBRIDGE_H

#include "SimplePWM.h"

class HBridge
{
public:
    HBridge();
    void setup(uint8_t pwm_pin[], uint8_t pwm_channel[]);
    void setSpeed(float speed);
    void setStop();

private:
    SimplePWM PWM_CLKW;
    SimplePWM PWM_CCLKW;
    TimerConfig motor_pwm_config{// Structure for timer configuration
                                 .timer = LEDC_TIMER_0,
                                 .frequency = 20000, // Frequency in Hz
                                 .bit_resolution = LEDC_TIMER_10_BIT,
                                 .mode = LEDC_LOW_SPEED_MODE};
};

#endif // _HBRIDGE_H