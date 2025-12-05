// ============================================================================
// definitions.h - Updated with E-STOP
// ============================================================================
#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "Buzzer.h"
#include "SimpleTimer.h"
#include "cmath"
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include "SimpleUART.h"
#include "SimpleADC.h"
#include "Hbridge.h"
#include "Ultrasonic.h"

#include "Stepper.h"
#include "QuadratureEncoder.h"
#include "PID_CAYETANO.h"
#include "TCS34725.h"
#include "SimpleRGB.h"
#include "Viscometer.h"

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
uint64_t dt = 10000;

enum ViscometerState
{
    // POWER SEQUENCE
    POWER_OFF = 0,
    POWER_ON,

    // INITIALIZATION / HOMING
    HOMING,
    DETECTION,

    // SAMPLE IDENTIFICATION
    READ_COLOR_TAG,
    INDICATE_COLOR_LED,

    // POSITIONING FOR MEASUREMENT
    MOVE_TO_MEASURE_POS,

    // VERTICAL SPINDLE MOVEMENT
    LOWER_SPINDLE,
    RAISE_SPINDLE,

    // FIRST MEASUREMENT CYCLE
    MEASURE_VISCOSITY,
    EVALUATE_RESULT,

    // DOSING LOOP
    DOSE_WATER,
    STIR_HIGH_RPM,
    STIR_HOLD_DELAY,
    RE_MEASURE,
    CHECK_ADJUSTMENT_LOOP,

    // FINAL CLASSIFICATION
    ACCEPT_SAMPLE,
    REJECT_SAMPLE,

    // CLEANING & DRYING PROCEDURE
    MOVE_TO_CLEAN_POS,
    MOVE_TO_DRY_POS,
    CLEANING_RINSE,

    // SAFETY & MAINTENANCE
    EMERGENCY_STOP,
    MAINTENANCE_MODE,
    CALIBRATE_SENSORS
};

// ============================================================================
// COLOR GLOBALS
// ============================================================================
uint8_t Rcal_last = 0;
uint8_t Gcal_last = 0;
uint8_t Bcal_last = 0;
volatile uint8_t detected_color = 0; // 0=unknown, 1=WHITE, 2=BLUE, 3=RED

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
SimpleTimer timer;
Stepper Stepper_Up;
Stepper Stepper_Rot;
Viscometer visco1;
HBridge Pump;
SimpleUART UART(115200);
PID_CAYETANO PID_STEPPER;
TCS34725 Color_sensor;
Ultrasonic US_Sensor;
SimpleGPIO Failed, STOP_BAND, E_STOP;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
float PID_GAINS[3] = {20.0f, 0.0f, 0.0f};
uint16_t R, G, B, C;
uint32_t STEPS_PER_REV = 400;

float target_viscocity;

// Per-stepper fractional accumulators
float frac_acc_up = 0.0f;
float frac_acc_rot = 0.0f;

// Persistent timestamps (microseconds)
uint64_t visc_start_time = 0;
uint64_t pump_start_time = 0;
uint64_t stir_start_time = 0;
uint64_t hold_start_time = 0;

// Viscosity adjustment tracking
int adjustment_iterations = 0;
const int MAX_ADJUSTMENT_ITERATIONS = 3;

float Viscocity_NO_DIl, Viscocity_10_DIl, Viscocity_25_DIl;

// Utility variables
int len = 0;
int mode = 0;
float ref = 0.0f;
char Buffer[64] = {0};

// PID parameters
const float DEGREE_DEADBAND = 1.0f;
const float MIN_FREQ = 100.0f;
const float MAX_FREQ = 3000.0f;

ViscometerState Current_state = DETECTION;

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Stepper Motor Pins
static const uint8_t STEPPER_UP_DIR_PIN = 33;
static const uint8_t STEPPER_UP_PWM_PIN = 18;
static const uint8_t STEPPER_ROT_DIR_PIN = 19;
static const uint8_t STEPPER_ROT_PWM_PIN = 25;

// I2C Pins for Color Sensor
static const uint8_t I2C_SDA_PIN = 21;
static const uint8_t I2C_SCL_PIN = 22;

// Motor and Encoder Pins
uint8_t Encoder_PINs[2] = {16, 17};
uint8_t Motor_Pins[2] = {14, 27};
uint8_t Pump_PIns[2] = {23, 26};

// Analog and Digital I/O
uint8_t ADC_PIN = 34;
uint8_t PIN_STOP_BAND = 15;
uint8_t PIN_FAILED = 5;

uint8_t BUZZER_PIN = 4; // GPIO 4 for buzzer
uint8_t buzzer_ch = 7;  // PWM channel 7
// EMERGENCY STOP - Choose your GPIO pin
// Common choices: GPIO 4, 32, 35, 36, 39 (make sure it's not used elsewhere)
uint8_t E_STOP_PIN = 32; // ← CHANGE THIS to your actual E-STOP button GPIO

// Ultrasonic Sensor Pins
uint8_t trig = 12;
uint8_t echo = 13;

// ============================================================================
// PWM CHANNELS
// ============================================================================
uint8_t motor_ch[2] = {0, 1};
uint8_t Stepper_UP_CH = 2;
uint8_t Stepper_ROT_CH = 3;
uint8_t pump_ch[2] = {4, 5};
uint8_t trig_CH = 6;

// ============================================================================
// PWM TIMER CONFIGURATIONS
// ============================================================================
static TimerConfig PWM_STEPPER_UP_TIMER{
    .timer = LEDC_TIMER_0,
    .frequency = 650,
    .bit_resolution = LEDC_TIMER_8_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

static TimerConfig PWM_STEPPER_ROT_TIMER{
    .timer = LEDC_TIMER_1,
    .frequency = 650,
    .bit_resolution = LEDC_TIMER_8_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

static TimerConfig Motor_Timer{
    .timer = LEDC_TIMER_2,
    .frequency = 1000,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

static TimerConfig US_Timer{
    .timer = LEDC_TIMER_3,
    .frequency = 50,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_LOW_SPEED_MODE};
static TimerConfig Buzzer_Timer{
    .timer = LEDC_TIMER_0, // Using HIGH SPEED TIMER 0
    .frequency = 1000,     // Default frequency (will change for notes)
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_HIGH_SPEED_MODE // HIGH SPEED MODE for buzzer
};

// Musical note frequencies
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_REST 0

// Mary Had a Little Lamb - Simple and recognizable!
const int MELODY_MARY[] = {
    NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4,  // Ma-ry had a
    NOTE_E4, NOTE_E4, NOTE_E4,           // lit-tle lamb
    NOTE_D4, NOTE_D4, NOTE_D4,           // lit-tle lamb
    NOTE_E4, NOTE_G4, NOTE_G4            // lit-tle lamb
};

const int DURATIONS_MARY[] = {
    300, 300, 300, 300,
    300, 300, 500,
    300, 300, 500,
    300, 300, 500
};

const int LENGTH_MARY = 13;

// Success sound - Ascending scale
const int MELODY_SUCCESS[] = {
    NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5
};

const int DURATIONS_SUCCESS[] = {
    200, 200, 200, 400
};

const int LENGTH_SUCCESS = 4;

// Error sound - Two low beeps
const int MELODY_ERROR[] = {
    NOTE_E4, NOTE_REST, NOTE_E4
};

const int DURATIONS_ERROR[] = {
    300, 100, 300
};

const int LENGTH_ERROR = 3;

// Short beep
const int MELODY_BEEP[] = {
    NOTE_A4
};

const int DURATIONS_BEEP[] = {
    150
};

const int LENGTH_BEEP = 1;
static const float STEPPER_DEGREES_PER_STEP = 1.8f;

#endif // __DEFINITIONS_H__