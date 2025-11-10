#ifndef __BDCMOTOR_H__
#define __BDCMOTOR_H__

#include "SimplePWM.h"

class BDCMotor
{
public:
    BDCMotor();
    void setup(const uint8_t in_pin[2], const uint8_t ch[2], TimerConfig motor_timer);
    void setSpeed(float Speed);

private:
    SimplePWM in[2];
};

#endif // __BDCMOTOR_H__+