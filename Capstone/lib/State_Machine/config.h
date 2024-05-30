#ifndef CONFIG_H
#define CONFIG_H
#include "Arduino.h"

#define LED_PIN 4
#define IMU_INT 2
#define NUM_LEDS 1
#define TRIG_PIN 13
#define ECHO_PIN 12
#define MOTOR_PWM_A 5
#define MOTOR_PWM_B 6
#define MOTOR_DIR_A 7
#define MOTOR_DIR_B 8
#define MOTOR_EN 3
#define NUM_IR 9
#define IR_0 A0
#define IR_1 A1
#define IR_2 A2
#define IR_3 A8 // Left
#define IR_4 A9 // Right
#define IR_5 A10
#define IR_6 A11
#define IR_7 A12
#define IR_8 A13
#define NUM_SERVO 3
#define SERVO_0 10
#define SERVO_1 11
#define SERVO_2 19

// State Color Codes
#define RED 0xFF0000 //Line Following
#define BLUE 0x0000FF // Turn
#define GREEN 0x00FF00 // Intersection Pickup
#define PPINK 0xFF00FF // Grabbing
#define YELLOW 0xFFFF00 // Intersection Dropoff
#define PURPLE 0x5E00FF // Dropoff
#define WHITE 0xFFFFFF // Standby

#define THRESHOLD 800
#define WALL_THRESHOLD_CLOSE 8.0
#define WALL_THRESHOLD_FAR 40.0
#define WALL_THRESHOLD 4.0
#define HILL_THRESHOLD 55.0

/*
    INTERSECTION DEFINITION PICKUP

    4--+--5
       |
    2--+--3
       |  
    0--+--1

    INTERSECTION DEFINITION DROPOFF
    
      |
    0_|
      |
    1_|
      |
    2_|
      |
    3_|
      |
    4_|
      |
    5_|
*/


#define SPEED 0.4

#define P_GAIN_TURN 10.0
#define I_GAIN_TURN 4.0
#define D_GAIN_TURN 1.0

// #define P_GAIN_LINE 1.5
// #define I_GAIN_LINE 0.00
// #define D_GAIN_LINE 1.0

#define P_GAIN_LINE 0.5
#define I_GAIN_LINE 0.00
#define D_GAIN_LINE 0.27

#endif