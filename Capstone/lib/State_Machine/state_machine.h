#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <Arduino.h>
#include "Motor_driver.h"
#include "ultrasonic.h"
#include "ir_sensor.h"
#include "MPU6050.h"
#include "PID_v1.h"
#include "FastLED.h"
#include "config.h"

struct data_packet
{
    float distance;
    int16_t ir_array[9] = {};
    float acceleration[3] = {};
    float rotation[3] = {};
};

enum States
{
    STANDBY = 0,
    LINE_FOLLOWING = 1,
    TURN = 2,
    INTERSECTION_PICKUP = 3,
    GRABBING = 4,
    INTERSECTION_DROPOFF = 5,
    DROPOFF = 6,
    FINISH = 7,
};

class StateMachine
{
public:
    StateMachine();
    void state_machine_update(data_packet data, States state, States last_state = STANDBY);
    data_packet getData();
    void printData();
    float *lineFollowing();
    bool turn(double angle, int timeout=10000);

private:
    data_packet last_data;
    int threshold;
    uint8_t intersection_counter = 0;
    double setpoint_line, input_line, output_line;
    double setpoint_turn, input_turn, output_turn;
    PID pid_line = PID(&input_line, &output_line, &setpoint_line, gains_line[0], gains_line[1], gains_line[2], DIRECT);
    PID pid_turn = PID(&input_turn, &output_turn, &setpoint_turn, gains_turn[0], gains_turn[1], gains_turn[2], DIRECT);
    CRGB leds[NUM_LEDS];
    data_packet data;
};

double angleWrap_360(double input);
double angleWrap_180(double input);
double angleWrap_pi(double input);
#endif