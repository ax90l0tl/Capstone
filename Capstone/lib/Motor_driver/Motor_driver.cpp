#include "Motor_driver.h"

Motor_driver::Motor_driver(uint8_t pwm_pinA, uint8_t dir_pinA, uint8_t pwm_pinB, uint8_t dir_pinB, uint8_t standby_pin)
{
    pwm_pin_[0] = pwm_pinA;
    dir_pin_[0] = dir_pinA;
    pinMode(pwm_pin_[0], OUTPUT);
    pinMode(dir_pin_[0], OUTPUT);
    pwm_pin_[1] = pwm_pinB;
    dir_pin_[1] = dir_pinB;
    pinMode(pwm_pin_[1], OUTPUT);
    pinMode(dir_pin_[1], OUTPUT);
    standby = standby_pin;
}

// Order is Right, Left
void Motor_driver::run(float right, float left)
{
    if (right == 0 && left == 0)
    {
        digitalWrite(standby, LOW);
    }
    else
    {
        if (abs(right) > 1 || abs(left) > 1)
        {
            right = right / abs(max(right, left));
            left = left / abs(max(right, left));
        }
        digitalWrite(standby, HIGH);
        float speed[2] = {right, left};
        for (uint8_t i = 0; i < 2; i++)
        {
            if (speed[i] <= 0)
            {
                digitalWrite(dir_pin_[i], LOW);
                analogWrite(pwm_pin_[i], (abs(speed[i]) * 255));
            }
            else
            {
                digitalWrite(dir_pin_[i], HIGH);
                analogWrite(pwm_pin_[i], (abs(speed[i]) * 255));
            }
        }
    }
}
void Motor_driver::twist(float speed, float spin)
{
    speed = constrain(speed, -1.0, 1.0);
    spin = constrain(spin, -1.0, 1.0);
    run((speed + spin)/2, (speed - spin)/ 2);
}