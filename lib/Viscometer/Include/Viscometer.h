#ifndef __VISCOMETER_H__
#define __VISCOMETER_H__

#include "Hbridge.h"
#include "QuadratureEncoder.h"
#include "PID_CAYETANO.h"
#include <stdint.h>

struct TimerConfig;

class Viscometer
{
public:
    Viscometer();
    ~Viscometer();

    void setup(uint8_t motorPins[2],
               uint8_t motorChannels[2],
               uint8_t encoderPins[2],
               TimerConfig &timerCfg);

    void measure();

private:
    HBridge motor;
    QuadratureEncoder encoder;
    PID_CAYETANO pid;

    float targetSpeed;
    float currentSpeed;
};

#endif