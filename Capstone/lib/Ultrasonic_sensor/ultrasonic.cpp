#include "ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t trig_pin, uint8_t echo_pin){
    trig_pin_ = trig_pin;
    echo_pin_ = echo_pin;
    pinMode(trig_pin_, OUTPUT);
    pinMode(echo_pin_, INPUT);
    digitalWrite(trig_pin_, LOW); 
}

double Ultrasonic::getMeasurement(){
    digitalWrite(trig_pin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin_, LOW);
    double duration = pulseIn(echo_pin_, HIGH);
    double distance = (duration*0.0343)/2;
    return (distance);
}