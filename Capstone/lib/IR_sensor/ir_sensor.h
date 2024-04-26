#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include "Arduino.h"

class IR_sensor
{
public:
    IR_sensor(uint8_t pin);
    int16_t getData();

private:
    uint8_t pin_;
};

#endif