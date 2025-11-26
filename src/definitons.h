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
#include "cmath"
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
uint64_t dt = 50000;

enum ViscometerState
{

    // -------------------------------------------------------------------------
    // POWER SEQUENCE
    // -------------------------------------------------------------------------
    POWER_OFF = 0,
    // LabVIEW must show: "Power Off"
    // ESP: everything de-energized, relay OFF.
    // Telemetry: relay=0, motors=0, pump=0.

    POWER_ON,
    // LabVIEW shows: "Power On – Running safety checks"
    // ESP: energize power relay, test sensors, verify encoder OK.
    // Telemetry: relay=1

    // -------------------------------------------------------------------------
    // INITIALIZATION / HOMING
    // -------------------------------------------------------------------------
    HOMING,
    // LabVIEW shows: "Homing all axes"
    // ESP: home rotation stepper + vertical screw stepper.
    // Telemetry: motor_pos_rot, motor_pos_z, homing_ok.

    // -------------------------------------------------------------------------
    // SAMPLE IDENTIFICATION
    // -------------------------------------------------------------------------
    READ_COLOR_TAG,
    // LabVIEW: "Reading color tag"
    // ESP: Color sensor measurement → identifies viscosity target range.
    // Telemetry: raw_color, decoded_color, target_visc_min/max.

    INDICATE_COLOR_LED,
    // LabVIEW: "Indicating color"
    // ESP: Send color to the labview
    // Telemetry: color_led=<R/G/B>

    // -------------------------------------------------------------------------
    // POSITIONING FOR MEASUREMENT
    // -------------------------------------------------------------------------
    MOVE_TO_MEASURE_POS,
    // LabVIEW: "Rotating to measurement station"
    // ESP: rotate turret to assigned measurement cup.
    // Telemetry: rot_pos_deg, at_measure_pos=1/0

    // -------------------------------------------------------------------------
    // VERTICAL SPINDLE MOVEMENT
    // -------------------------------------------------------------------------
    LOWER_SPINDLE,
    // LabVIEW: "Lowering spindle"
    // ESP: move Z-stepper downward until contact depth.
    // Telemetry: z_pos_mm, load_sensor_value, in_liquid=1/0

    RAISE_SPINDLE,
    // LabVIEW: "Raising spindle"
    // ESP: return Z-axis to zero height after measurement or cleaning.
    // Telemetry: z_pos_mm

    // -------------------------------------------------------------------------
    // FIRST MEASUREMENT CYCLE
    // -------------------------------------------------------------------------
    MEASURE_VISCOSITY,
    // LabVIEW: "Measuring viscosity"
    // ESP: spin spindle, run PID RPM control, compute viscosity from torque/RPM curve.
    // Telemetry: rpm, torque, viscosity_raw, viscosity_filtered

    EVALUATE_RESULT,
    // LabVIEW: "Evaluating viscosity result"
    // ESP: compare viscosity to target range.
    // Telemetry: visc, target_min, target_max, in_range=1/0

    // -------------------------------------------------------------------------
    // DOSING LOOP
    // -------------------------------------------------------------------------
    DOSE_WATER,
    // LabVIEW: "Dosing water"
    // ESP: run pump for computed milliseconds.
    // Telemetry: pump=1, dose_ms, fluid_temp

    STIR_HIGH_RPM,
    // LabVIEW: "Stirring (High RPM)"
    // ESP: maintain high RPM ≥ 120 for 30 s.
    // Telemetry: rpm, current_motor, timer_remaining

    STIR_HOLD_DELAY,
    // LabVIEW: "Waiting/sta0bilizing mixture"
    // ESP: 60-second wait with periodic checks.
    // Telemetry: timer_remaining

    RE_MEASURE,
    // LabVIEW: "Re-measuring viscosity"
    // ESP: run MEASURE_VISCOSITY again.
    // Telemetry: viscosity_new, rpm, torque

    CHECK_ADJUSTMENT_LOOP,
    // LabVIEW: "Checking if additional dosing is needed"
    // ESP: if (visc out of range AND loops < MAX) → return to DOSE_WATER.
    // Telemetry: remaining_iterations, in_range=1/0

    // -------------------------------------------------------------------------
    // FINAL CLASSIFICATION
    // -------------------------------------------------------------------------
    ACCEPT_SAMPLE,
    // LabVIEW: "Sample Accepted"
    // ESP: pulse I/O line to Arduino for acceptance.
    // Telemetry: accepted=1

    REJECT_SAMPLE,
    // LabVIEW: "Sample Rejected"
    // ESP: pulse line for rejection.
    // Telemetry: rejected=1

    // -------------------------------------------------------------------------
    // CLEANING PROCEDURE
    // -------------------------------------------------------------------------
    MOVE_TO_CLEAN_POS,
    // LabVIEW: "Rotating to cleaning station"
    // ESP: rotate turret to cleaning bath.
    // Telemetry: rot_pos_deg, at_clean_pos

    CLEANING_RINSE,
    // LabVIEW: "Rinsing & Stirring inside water"
    // ESP: submerge spindle, spin at moderate RPM.
    // Telemetry: cleaning_rpm, rinse_time

    EMERGENCY_STOP,
    // LabVIEW: "EMERGENCY STOP"
    // ESP: Immediately turn OFF motors, pump, relay.
    // Telemetry: emergency=1

    MAINTENANCE_MODE,
    // LabVIEW: "Maintenance mode"
    // ESP: low-level control of all actuators for diagnostics.
    // Telemetry: depends on test type

    CALIBRATE_SENSORS
    // LabVIEW: "Calibrating sensors"
    // ESP: calibration for encoder, load cell, color sensor, TOF.
    // Telemetry: calibration_progress, calibration_ok
};

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

// Consts or variable definitions
float PID_GAINS[3] = {180.0f, 0.0f, 0.0f};
uint16_t R, G, B, C;
uint32_t STEPS_PER_REV = 400;


// Useful variables
// per-stepper fractional accumulators (one per stepper you use)

int len = 0;
int mode = 0;
float ref = 0.0f;      // incoming reference (mode-dependent)
char Buffer[64] = {0}; // ensure this exists (or use the one in your header)
const float DEGREE_DEADBAND = 1.0f;
const float MIN_FREQ = 100.0f;
const float MAX_FREQ = 3000.0f;
ViscometerState Current_state = LOWER_SPINDLE;
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
uint8_t Pump_PIns[2] = {23, 26};
uint8_t ADC_PIN = 34;

// PWM CHANNELS
uint8_t motor_ch[2] = {0, 1};
uint8_t Stepper_UP_CH = 2;
uint8_t Stepper_ROT_CH = 3;
uint8_t pump_ch[2] = {4, 5};

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

#endif // __DEFINITIONS_H__
