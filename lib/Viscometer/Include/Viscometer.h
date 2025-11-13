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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Forward-declare TimerConfig (defined elsewhere in your code)
struct TimerConfig;

// measurement result returned by a single call to measure()
struct Measurement {
    float rpm;        // instantaneous/converted RPM (uses same RPM_CONV factor)
    float viscosity;  // viscosity computed from one ADC sample (linear mapping)
};

class Viscometer
{
public:
    Viscometer();
    ~Viscometer();

    // Setup the viscometer. TimerConfig passed by reference
    void setup( uint8_t spinMotorPins[2],
                uint8_t spinMotorChannels[2],
                uint8_t encoderPins[2],
               uint8_t adcPin,
                TimerConfig &timerCfg,
               float encoderDegreesPerEdge = 0.36445f);

    // Single measurement (reads encoder + ADC once and returns Measurement)
    Measurement measure();

    // Start a non-blocking stir at 'rpm' for 'duration_ms' milliseconds.
    // Returns true if start succeeded (stirring started), false otherwise.
    bool startStir(float rpm = 120.0f, uint32_t duration_ms = 30000);

    // Immediately stop stirring (non-blocking)
    void stopStir();

    // Polling API: call update frequently (e.g. from your main loop or a periodic task)
    // update() will stop the motor automatically when the duration expires.
    void update();

    // Query whether stir is active
    bool isStirring() const;

    void stop(); // emergency stop for motor (stops everything)

private:
    SimplePWM      pwmSpin;
    BDCMotor       motor;       // BDC motor wrapper that uses pwmSpin
    QuadratureEncoder encoder;  // local encoder object
    SimpleADC      adc;         // local ADC instance
    PID_CAYETANO   pid;         // PID controller instance

    float encoderDegreesPerEdge;
    float targetSpeed; // used by measure() (defaults to a measurement speed ~25 RPM)
    float gainArr[3];  // local PID gains: {Kp, Ki, Kd}

    // conversion factor used in original code
    static constexpr float RPM_CONV = 0.17f;

    // Stir state (non-blocking)
    bool stirring;
    TickType_t stirEndTick; // tick at which stirring should stop
    float stirDuty;         // last computed duty used during stirring

    // Motor mapping params (simple placeholder — replace with calibration)
    float maxExpectedRPM; // used to convert rpm -> duty (0..100)
    float rpmToDuty(float rpm) const;
};

#endif // __VISCOMETER_H__
