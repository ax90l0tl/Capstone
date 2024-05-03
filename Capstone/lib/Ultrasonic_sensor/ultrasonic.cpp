#include "ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t trig_pin, uint8_t echo_pin)
{
    trig_pin_ = trig_pin;
    echo_pin_ = echo_pin;
    pinMode(trig_pin_, OUTPUT);
    pinMode(echo_pin_, INPUT);
    maxDistance = 200;
    timeout = 100;
}

float Ultrasonic::getMeasurement()
{
    // digitalWrite(trig_pin_, LOW);
    // delayMicroseconds(2);
    digitalWrite(trig_pin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin_, LOW);
    unsigned long duration = pulseIn(echo_pin_, HIGH);
    float distance = ((float)duration * 0.0343) / 2;
    delay(5);
    if (distance <= maxDistance && distance > 0)
    {
        return distance;
    }
    else
    {
        return -1;
    }
}