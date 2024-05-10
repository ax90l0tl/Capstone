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
#define IR_0 0
#define IR_1 1
#define IR_2 2
#define IR_3 8
#define IR_4 9
#define IR_5 10
#define IR_6 11
#define IR_7 12
#define IR_8 13

// State Color Codes
#define RED 0xFF0000 //Line Following
#define BLUE 0x0000FF // Turn
#define GREEN 0x00FF00 // Intersection Pickup
#define PINK 0xFF00FF // Grabbing
#define YELLOW 0xFFFF00 // Intersection Dropoff
#define PURPLE 0x5E00FF // Dropoff
#define WHITE 0xFFFFFF // Standby

#define THRESHOLD 700

#define P_GAIN_TURN 0.2
#define I_GAIN_TURN 0.1
#define D_GAIN_TURN 0.005

// #define P_GAIN_LINE 1.5
// #define I_GAIN_LINE 0.00
// #define D_GAIN_LINE 1.0

#define P_GAIN_LINE 10.0
#define I_GAIN_LINE 0.00
#define D_GAIN_LINE 6.67

#endif