// ============================================================================
// definitions.h
// ============================================================================
#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "SimpleTimer.h"
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include "SimpleUART.h"
#include "SimpleADC.h"
#include "Hbridge.h"

#include "Stepper.h"
#include "QuadratureEncoder.h"
#include "PID_CAYETANO.h"
#include "TCS34725.h"
#include "SimpleRGB.h"
#include "Viscometer.h"

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
uint64_t dt = 20000;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
SimpleTimer timer;
Stepper Stepper_Up;
Stepper Stepper_Rot;
Viscometer visco1;
SimpleUART UART(115200);
PID_CAYETANO PID_STEPPER;
float PID_GAINS[3] = {0.1f, 1.0f, 0.0f};
// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Stepper Motor Pins
static const uint8_t STEPPER_UP_DIR_PIN = 33;  // GPIO output
static const uint8_t STEPPER_UP_PWM_PIN = 18;  // PWM capable
static const uint8_t STEPPER_ROT_DIR_PIN = 19; // GPIO output
static const uint8_t STEPPER_ROT_PWM_PIN = 25; // PWM capable

// I2C Pins for Color Sensor (default ESP32 I2C)
static const uint8_t I2C_SDA_PIN = 21; // I2C SDA
static const uint8_t I2C_SCL_PIN = 22; // I2C SCL

uint8_t Encoder_PINs[2] = {16, 17};
uint8_t Motor_Pins[2] = {14, 27};
uint8_t motor_ch[2] = {0, 1};
uint8_t Stepper_UP_CH = 2;
uint8_t Stepper_ROT_CH = 3;
uint8_t ADC_PIN = 34;

// ============================================================================
// PWM TIMER CONFIGURATIONS
// ============================================================================
static TimerConfig PWM_STEPPER_UP_TIMER{
    .timer = LEDC_TIMER_0,
    .frequency = 650,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

static TimerConfig PWM_STEPPER_ROT_TIMER{
    .timer = LEDC_TIMER_1,
    .frequency = 650,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

// Make this GLOBAL instead of local to app_main()
static TimerConfig Motor_Timer{
    .timer = LEDC_TIMER_0,
    .frequency = 1000,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

static const float STEPPER_DEGREES_PER_STEP = 1.8f;
char Buffer[32];

#endif // __DEFINITIONS_H__
