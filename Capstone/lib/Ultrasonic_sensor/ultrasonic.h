#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <Arduino.h>

class Ultrasonic
{
public:
    Ultrasonic(uint8_t trig_pin, uint8_t echo_pin);
    float getMeasurement();

private:
    uint8_t trig_pin_;
    uint8_t echo_pin_;
    uint16_t maxDistance;
    uint32_t timeout;
};

#endif