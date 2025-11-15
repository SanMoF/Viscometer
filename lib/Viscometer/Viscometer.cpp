#include "Viscometer.h"
#include <stdio.h>

Viscometer::Viscometer()
{
    targetSpeed = 180.0f;
    currentSpeed = 0.0f;
}

Viscometer::~Viscometer()
{
    motor.setSpeed(0.0f);
}

void Viscometer::setup(uint8_t motorPins[2],
                       uint8_t motorChannels[2],
                       uint8_t encoderPins[2],
                       TimerConfig* timerCfg, uint64_t dt)
{
    motor.setup(motorPins, motorChannels, timerCfg);
    encoder.setup(encoderPins, 0.36445f);
    float gains[3] = {0.1f, 1.0f, 0.0f};
    pid.setup(gains, (float) dt);
    motor.setSpeed(0.0f);
}


void Viscometer::measure()
{
    currentSpeed = encoder.getSpeed();
    float error = targetSpeed - currentSpeed;
    float u = pid.computedU(error);
    
    motor.setSpeed(u);
}