// ============================================================================
// main.cpp - PID controlled stepper movement with UART control
// ============================================================================
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Stepper.h"
#include "PID_CAYETANO.h"
#include "SimpleUART.h"

// Global objects
Stepper stepper;
PID_CAYETANO PID;
SimpleUART uart(115200);

// Configuration pins
#define STEP_PIN 25
#define DIR_PIN 26
#define PWM_CHANNEL 0

// Timer configuration
TimerConfig timer_config = {
    .timer = LEDC_TIMER_0,
    .frequency = 650,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_LOW_SPEED_MODE,
};

extern "C" void app_main(void)
{
    float gains[3] = {0.1f, 1.0f, 0.0f};  // Kp, Ki, Kd
    printf("\n=== Stepper Motor Controller with PID and UART ===\n\n");
    
    // Setup PID and stepper
    PID.setup(gains, 1000);
    stepper.setup(STEP_PIN, DIR_PIN, PWM_CHANNEL, &timer_config, 400);
    
    // Initial delay
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // PID control variables
    float setpoint = 0.0f;
    float error = 0.0f;
    float u = 0.0f;
    int32_t current_position = 0;
    float current_position_degrees = 0.0f;
    
    printf("Ready! Send degree values via UART (e.g., 90, 180, 0)\n");
    
    // Main loop - PID controlled movement with UART input
    while (1)
    {
        // Check for UART input to change setpoint
        int len = uart.available();
        if (len > 0)
        {
            char buffer[32] = {0};
            uart.read(buffer, len);
            
            float new_setpoint = 0.0f;
            if (sscanf(buffer, "%f", &new_setpoint) == 1)
            {
                setpoint = new_setpoint;
                printf("\n=== New setpoint received: %.2f degrees ===\n", setpoint);
            }
            else
            {
                printf("Invalid input. Please send a number (e.g., 90)\n");
            }
        }
        
        // Get current position in degrees
        current_position = stepper.getPosition();
        current_position_degrees = (current_position * 360.0f) / 400.0f;
        
        // Calculate error
        error = setpoint - current_position_degrees;
        
        // Only move if error is significant (> 1 degree)
        if (error > 1.0f || error < -1.0f)
        {
            // Compute control signal (frequency)
            u = PID.computedU(error);
            
            // Clamp frequency to reasonable range (100 Hz to 2000 Hz)
            if (u < 100.0f) u = 100.0f;
            if (u > 2000.0f) u = 2000.0f;
            
            // Move stepper
            stepper.moveDegrees(error, (uint32_t)u);
            
            printf("Setpoint: %.2f°, Position: %.2f°, Error: %.2f°, Freq: %.0f Hz\n", 
                   setpoint, current_position_degrees, error, u);
        }
        else
        {
            // At target - print status less frequently
            static int counter = 0;
            if (counter++ % 10 == 0)
            {
                printf("At target: %.2f° (setpoint: %.2f°)\n", 
                       current_position_degrees, setpoint);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // PID update rate
    }
}