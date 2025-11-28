// ============================================================================
// Ultrasonic.h - REVERT TO ORIGINAL (that worked)
// ============================================================================
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "SimplePWM.h"
#include "driver/gpio.h"
#include "esp_timer.h"

class Ultrasonic
{
private:
    SimplePWM trigger;
    gpio_num_t _gpio_echo;
    uint64_t _prev_micros = 0;
    uint64_t _echo_time = 0;
    uint8_t _states[2] = {0, 0};

public:
    Ultrasonic();
    ~Ultrasonic();
    
    // BACK TO ORIGINAL - pass by value like before
    void setup(uint8_t gpio_echo, uint8_t gpio_trig, uint8_t ch_trig, TimerConfig timer_config);
    
    float getDistance();
    void handler();
};

#endif // ULTRASONIC_H
