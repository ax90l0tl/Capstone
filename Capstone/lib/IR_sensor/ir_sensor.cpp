#include "ir_sensor.h"

IR_sensor::IR_sensor(uint8_t pin)
{
    pin_ = pin;
    pinMode(pin_, INPUT);
}

int16_t IR_sensor::getData()
{
    return (analogRead(pin_));
}