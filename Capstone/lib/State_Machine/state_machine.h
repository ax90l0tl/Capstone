#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <Arduino.h>
#include "Motor_driver.h"
#include "ultrasonic.h"
#include "ir_sensor.h"
#include "imu.h"
#include "PID_v1.h"
#include "FastLED.h"
#include "config.h"
#include "Servo.h"

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
    SEARCH = 8,
};

class StateMachine
{
public:
    StateMachine();
    States choose_action(States state, float speed, bool verbose = false);
    States update(States last_state, bool verbose = false);
    void getInstructions();
    data_packet getData(bool verbose = false, bool use_ir = false, bool use_imu = false, bool use_ultrasonic = false);
    
    bool detectLine();
    bool detectLeft();
    bool detectRight();
    bool detectWall();
    bool detectHill();

    void lineFollowing(float speed, bool verbose = false, unsigned long timeout = 500);
    void lineFollowing_gen(float speed);
    bool turn(double angle, double w, bool verbose = false, unsigned long timeout = 5000, float speed = 0);
    void approach_wall(float speed, float threshold, bool verbose = false);
    void leave_wall(float speed, bool verbose = false);
    void grab(bool verbose = false);
    void exit_intersection(float speed, bool verbose = false);
    bool search(float speed, bool verbose = false);
    void find_intersection(float speed, bool verbose = false, unsigned long timeout = 1000);

    bool assignMotor(Motor_driver *motor_driver);
    bool assignIMU(IMU *IMU);
    bool assignIR(IR_sensor *IR_sensor[NUM_IR]);
    bool assignUltrasonic(Ultrasonic *ultrasonic_sensor);
    bool assignServo(Servo *servo[NUM_SERVO]);

private:
    Motor_driver *motor;
    IMU *imu;
    IR_sensor *ir_sensor[NUM_IR];
    Ultrasonic *ultrasonic;
    Servo *servos[NUM_SERVO];
    uint8_t intersections[4] = {0, 1, 5, 3};
    uint8_t T_counter = 0;
    uint8_t L_counter = 0;
    uint8_t intersection_counter = 0;
    float prev_orientation = 0;
    bool climb = false;
    bool picked_up = true;
    bool dropped_off = false;


    double setpoint_line, input_line, output_line;
    double setpoint_turn, input_turn, output_turn;
    PID pid_line = PID(&input_line, &output_line, &setpoint_line, P_GAIN_LINE, I_GAIN_LINE, D_GAIN_LINE, DIRECT);
    PID pid_turn = PID(&input_turn, &output_turn, &setpoint_turn, P_GAIN_TURN, I_GAIN_TURN, D_GAIN_TURN, DIRECT);
    CRGB leds[NUM_LEDS];
    data_packet data;
};

double angleWrap_360(double input);
double angleWrap_180(double input);
double angleWrap_pi(double input);
double get_nearest_angle_multiple(double input);
double get_nearest_angle(double input);
#endif