#include "Viscometer.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static TimerConfig defaultMotorTimer {
    .timer = LEDC_TIMER_2,
    .frequency = 2000,
    .bit_resolution = LEDC_TIMER_14_BIT,
    .mode = LEDC_LOW_SPEED_MODE
};

Viscometer::Viscometer()
{
    // default PID gains (tune as needed)
    gainArr[0] = 1.0f;   // Kp
    gainArr[1] = 0.0f;   // Ki
    gainArr[2] = 0.0f;   // Kd

    encoderDegreesPerEdge = 0.36445f;
    // measurement targetSpeed chosen so measured rpm ~= 25
    targetSpeed = 25.0f / RPM_CONV;

    // non-blocking stir state
    stirring = false;
    stirEndTick = 0;
    stirDuty = 0.0f;

    // placeholder max expected RPM for mapping rpm -> duty
    maxExpectedRPM = 2000.0f; // change to your real motor max
}

Viscometer::~Viscometer()
{
    stop();
}

void Viscometer::setup( uint8_t spinMotorPins[2],
                        uint8_t spinMotorChannels[2],
                       uint8_t encoderPins[2],
                       uint8_t adcPin,
                        TimerConfig &timerCfg,
                       float encoderDegreesPerEdge_)
{
    encoderDegreesPerEdge = encoderDegreesPerEdge_;

    // Configure PWM if needed (uncomment/adapt to your SimplePWM API)
    // pwmSpin.setup(spinMotorPins[0], spinMotorChannels[0], timerCfg);

    motor.setup(spinMotorPins, spinMotorChannels, timerCfg);
    encoder.setup(encoderPins, encoderDegreesPerEdge);
    adc.setup(adcPin);
    pid.setup(gainArr, 10000);

    motor.setSpeed(0.0f);
}

Measurement Viscometer::measure()
{
    Measurement m;
    float enc_speed = encoder.getSpeed();
    float rpm = enc_speed * RPM_CONV;

    // PID control toward targetSpeed (targetSpeed in encoder units)
    float error = targetSpeed - enc_speed;
    float u = pid.computedU(error);
    motor.setSpeed(u);

    float raw = adc.read(ADC_READ_RAW);
    float mu_rel = 0.7688f * raw - 848.81f;

    m.rpm = rpm;
    m.viscosity = mu_rel;
    return m;
}

float Viscometer::rpmToDuty(float rpm) const
{
    // Simple linear mapping from rpm to 0..100 duty%
    // Replace with your calibrated mapping
    float duty = (rpm / maxExpectedRPM) * 100.0f;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;
    return duty;
}

bool Viscometer::startStir(float rpm, uint32_t duration_ms)
{
    if (duration_ms == 0) return false;

    // Compute duty (adapt this if your motor.setSpeed expects another scale)
    float duty = rpmToDuty(rpm);
    motor.setSpeed(duty);

    // Set non-blocking stop time
    TickType_t now = xTaskGetTickCount();
    TickType_t ticks = pdMS_TO_TICKS(duration_ms);
    // Guard against overflow of tick add (TickType_t is unsigned)
    stirEndTick = now + ticks;
    stirring = true;
    stirDuty = duty;
    return true;
}

void Viscometer::stopStir()
{
    if (stirring) {
        motor.setSpeed(0.0f);
        stirring = false;
        stirEndTick = 0;
        stirDuty = 0.0f;
    }
}

void Viscometer::update()
{
    if (!stirring) return;

    TickType_t now = xTaskGetTickCount();
    // If now >= stirEndTick => time to stop
    // Handle typical tick wraparound correctly using subtraction
    if ((now - stirEndTick) < ( (TickType_t)0x80000000 )) {
        // now >= stirEndTick (works with wraparound)
        motor.setSpeed(0.0f);
        stirring = false;
        stirEndTick = 0;
        stirDuty = 0.0f;
        // Optionally: set motor to measurement speed instead of stopping.
        // float measureDuty = rpmToDuty(25.0f); motor.setSpeed(measureDuty);
    }
}

bool Viscometer::isStirring() const
{
    return stirring;
}

void Viscometer::stop()
{
    motor.setSpeed(0.0f);
    stirring = false;
    stirEndTick = 0;
    stirDuty = 0.0f;
}
