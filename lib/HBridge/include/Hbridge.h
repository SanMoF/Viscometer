#ifndef _HBRIDGE_H
#define _HBRIDGE_H

#include "SimplePWM.h"

class HBridge
{
public:
    HBridge();
    void setup(uint8_t pwm_pin[], uint8_t pwm_channel[], TimerConfig motor_pwm_config);
    void setSpeed(float speed);
    void setStop();

private:
    SimplePWM PWM_CLKW;
    SimplePWM PWM_CCLKW;
    
};

#endif // _HBRIDGE_H