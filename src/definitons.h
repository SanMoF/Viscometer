// ============================================================================
// definitions.h
// ============================================================================
#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <stdint.h>
#include "SimpleTimer.h"
#include "SimpleGPIO.h"
#include "HBridge.h"
#include "BDCMotor.h"
#include "SimplePWM.h"
#include "Stepper.h"
#include "QuadratureEncoder.h"
#include "PID_CAYETANO.h"
#include "SimpleUART.h"
#include "TCS34725.h"
#include "SimpleRGB.h"
#include "driver/i2c.h"


// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
static const uint32_t dt = 50; // 10ms tick period in microseconds
SimpleGPIO dir, interrupt;
SimplePWM PWM_stepper;


static SimpleTimer timer;
uint8_t dir_PIN = ;
uint8_t PWM_pin = ;
uint8_t interrupt =;

int count;

#endif // __DEFINITIONS_H__

