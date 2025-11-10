#include "Viscometer.h"

// No stdio/string used — purely embedded-oriented

// choose a local timer config for the motor PWM (keeps class self-contained).
// These values match the original sketch's PWM_Spin config: 2000 Hz, 14-bit.
// If your platform's LEDC identifiers are needed, replace with the appropriate values.
static TimerConfig defaultMotorTimer {
    .timer = LEDC_TIMER_2,
    .frequency = 2000,
    .bit_resolution = LEDC_TIMER_14_BIT,
    .mode = LEDC_LOW_SPEED_MODE
};

Viscometer::Viscometer()
{
    // default PID gains similar to previous examples but conservative
    gainArr[0] = 1.0f;   // Kp
    gainArr[1] = 0.0f; // Ki
    gainArr[2] = 0.0f;     // Kd

    encoderDegreesPerEdge = 0.36445f;
    targetSpeed = 180.0f; // target used by measure() (same semantics as prior code)
}

Viscometer::~Viscometer()
{
    // ensure motor stopped
    stop();
}

void Viscometer::setup(const uint8_t spinMotorPins[2],
                       const uint8_t spinMotorChannels[2],
                       const uint8_t encoderPins[2],
                       uint8_t adcPin,
                       float encoderDegreesPerEdge_,TimerConfig Timer_1)
{
    // store configuration
    encoderDegreesPerEdge = encoderDegreesPerEdge_;

    // Configure the PWM object we own for the spin motor.
    // Use the first motor pin as the PWM output pin when calling pwmSpin.setup.
    // channel is chosen as spinMotorChannels[0] for the pwm driver.
    // If your PWM API requires different mapping, adapt accordingly.
    

    // Setup the BDC motor instance with our pwmSpin
    motor.setup(spinMotorPins, spinMotorChannels, Timer_1);

    // Setup encoder (local instance)
    encoder.setup(encoderPins, encoderDegreesPerEdge);

    // Setup ADC
    adc.setup(adcPin);

    // Setup PID with local gain array. Period argument kept similar to previous code
    pid.setup(gainArr, 10000);

    // Make sure motor is stopped
    motor.setSpeed(0.0f);
}

Measurement Viscometer::measure()
{
    Measurement m;
    float enc_speed = encoder.getSpeed(); 
    float rpm = enc_speed * RPM_CONV;

    float error = targetSpeed - enc_speed;
    float u = pid.computedU(error);
    motor.setSpeed(u);

    // 3) single ADC read and convert to viscosity with the original formula
    float raw = adc.read(ADC_READ_RAW);
    float mu_rel = 0.7688f * raw - 848.81f;

    // Prepare result
    m.rpm = rpm;
    m.viscosity = mu_rel;

    return m;
}

void Viscometer::stir()
{
    
    motor.setSpeed(100.0f);

}

void Viscometer::stop()
{
    motor.setSpeed(0.0f);
}

