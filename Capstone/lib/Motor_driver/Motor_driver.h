#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "Arduino.h"

class Motor_driver
{
public:
    Motor_driver(uint8_t pwm_pinA, uint8_t dir_pinA, uint8_t pwm_pinB, uint8_t dir_pinB, uint8_t standby_pin);
    void run(float right, float left);
    void twist(float speed, float spin);

private:
    // distance between wheels (cm)
    const float L = 2.32537; 
    uint8_t standby;
    uint8_t pwm_pin_[2];
    uint8_t dir_pin_[2];
};

#endif