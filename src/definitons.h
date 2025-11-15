// ============================================================================
// definitions.h
// ============================================================================
#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <stdint.h>
#include "SimpleTimer.h"
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include "Stepper.h"
#include "QuadratureEncoder.h"
#include "PID_CAYETANO.h"
#include "SimpleUART.h"
#include "TCS34725.h"
#include "SimpleRGB.h"
#include "driver/i2c.h"
#include "Viscometer.h"
#include "SimpleGPIO.h"
#include "SimpleADC.h"
#include "SimplePWM.h"
#include "SimpleTimer.h"
#include "Hbridge.h"
#include "QuadratureEncoder.h"
#include "PID_CAYETANO.h"
#include "Viscometer.h"
#include <stdio.h>
#include "esp_timer.h"
#include "esp_task_wdt.h"

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
uint64_t dt = 20000;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
static SimpleTimer timer;
static Stepper Stepper_Up;
static Stepper Stepper_Rot;

Viscometer visco1;

static QuadratureEncoder enco;
static PID_CAYETANO PID;
static SimpleUART UART_MESSAGE(115200);
static TCS34725 Color_sensor;
static SimpleGPIO emg_relay, Buzz;
SimpleRGB RGB;
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
static const float ENCODER_DEGREES_PER_EDGE = 0.36445f;

// ============================================================================
// PID CONFIGURATION
// ============================================================================
static float PID_GAINS[3] = {0.1f, 1.0f, 0.0f}; // Kp, Ki, Kd

// ============================================================================
// COMMUNICATION VARIABLES
// ============================================================================

// Receive buffer
static char rxbuf[128];

// Sensor data (RGB color sensor)
static uint16_t sensor_r = 0;
static uint16_t sensor_g = 0;
static uint16_t sensor_b = 0;
static uint16_t sensor_c = 0;

// Data to send to LabVIEW
struct TelemetryData
{
    int R;
    int G;
    int B;
    float pos_x;
    float pos_y;
    float speed;
};
static TelemetryData telemetry = {0, 0, 0, 0.0f, 0.0f, 0.0f};

// Data received from LabVIEW
struct ControlData
{
    float ref_rpms;
    int emergency_stop;
    int send_water;
    float ref_pos_x;
    float ref_pos_y;
};
static ControlData control = {0.0f, 0, 0, 0.0f, 0.0f};

// State machine
static int current_state = 0;

// ============================================================================
// STATE DEFINITIONS
// ============================================================================
enum SystemState
{
    STATE_ELEVATION = 0,
    STATE_ROTATION = 1,
    STATE_PUMP = 2,
    STATE_VELOCITY_CONTROL = 3,
    STATE_IDLE = 4
};

#endif // __DEFINITIONS_H__
