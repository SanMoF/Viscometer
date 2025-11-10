#ifndef __VISCOMETER_H__
#define __VISCOMETER_H__

// Minimal includes for the components we encapsulate
#include "SimpleGPIO.h"
#include "SimpleADC.h"
#include "PID_CAYETANO.h"
#include "SimplePWM.h"
#include "BDCMotor.h"
#include "QuadratureEncoder.h"
#include <stdint.h>

// measurement result returned by a single call to measure()
struct Measurement {
    float rpm;        // instantaneous/converted RPM (uses same 0.17 factor as your original)
    float viscosity;  // viscosity computed from one ADC sample (linear mapping)
};

class Viscometer
{
public:
    Viscometer();
    ~Viscometer();

    // Setup the viscometer hardware. This method configures:
    //  - the BDC motor (spinMotorPins & spinMotorChannels)
    //  - the on-board PWM used by the motor (internal SimplePWM member)
    //  - the quadrature encoder (encoderPins) with encoderDegreesPerEdge
    //  - the ADC input (adcPin)
    //
    // Parameters:
    //   spinMotorPins[2]      : two pins used by your BDC motor driver
    //   spinMotorChannels[2]  : two PWM channels the motor uses
    //   encoderPins[2]        : A and B encoder pins
    //   adcPin                : ADC input pin for viscosity sensor
    //   encoderDegreesPerEdge : degrees per encoder edge (default ~0.36445)
    void setup(const uint8_t spinMotorPins[2],
               const uint8_t spinMotorChannels[2],
               const uint8_t encoderPins[2],
               uint8_t adcPin,
               float encoderDegreesPerEdge = 0.36445f);

    // Single non-blocking measurement step.
    // - reads encoder once, computes PID output and writes it to motor
    // - reads ADC once and converts to viscosity using your linear mapping
    // Returns Measurement { rpm, viscosity } immediately.
    Measurement measure();

    // Start/command stirring. Duty expressed as percent (0..100).
    // Returns instantaneous RPM (from encoder read).
    float stir(float duty_percent = 100.0f);

    // Stop spinning motor immediately.
    void stop();
private:
    SimplePWM      pwmSpin;     // PWM used to drive motor (configured in setup)
    BDCMotor       motor;       // BDC motor wrapper that uses pwmSpin
    QuadratureEncoder encoder;  // local encoder object
    SimpleADC      adc;         // local ADC instance
    PID_CAYETANO   pid;         // PID controller instance

    // configuration
    float encoderDegreesPerEdge;
    float targetSpeed; // used by measure() (default 30.0)
    float gainArr[3];  // local PID gains: {Kp, Ki, Kd}

    // conversion factor used in original code
    static constexpr float RPM_CONV = 0.17f;
};
#endif // __VISCOMETER_H__
