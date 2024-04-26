#include <Arduino.h>
#include "Motor_driver.h"
#include "ultrasonic.h"
#include "ir_sensor.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "state_machine.h"
#include "imu.h"
#include "config.h"

Motor_driver *Motors;
Ultrasonic *ultrasonic;
IR_sensor *ir_array[9] = {nullptr};
IMU *imu;
StateMachine *state_machine;

uint8_t intersection_counter = 0;
double setpoint, input, output;
// turn pid
double p = 0.2, i = 0.1, d = 0.005;

// line pid
// double p = 0.01, i = 0.005, d = 0.0005;
int white = 60;
int black = 800;
// PID pid_line = PID(&input, &output, &setpoint, p, i, d, DIRECT);
// PID pid_turn = PID(&input, &output, &setpoint, p, i, d, DIRECT);

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(1000);
  Motors = new Motor_driver(MOTOR_PWM_A, MOTOR_DIR_A, MOTOR_PWM_B, MOTOR_DIR_B, MOTOR_EN);
  ultrasonic = new Ultrasonic(TRIG_PIN, ECHO_PIN);
  uint8_t ir_pins[9] = {IR_0, IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7, IR_8};
  for (uint8_t i = 0; i < 9; i++)
  {
    ir_array[i] = new IR_sensor(ir_pins[i]);
  }

  imu = new IMU(2);
  state_machine = new StateMachine();
}

void loop()
{
  data_packet data;
  // for (uint8_t i = 0; i < 9; i++)
  // {
  //   data.ir_array[i] = ir_array[i]->getData();
  //   Serial.print(data.ir_array[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();
  Serial.println(ultrasonic->getMeasurement());
  // imu->getData();
  // imu->printData();
  // data.rotation[0] = imu->ypr[0];
  // setpoint = 0;
  // input = angleWrap_pi(1.57 - 0);
  // input = data.ir_array[0] - data.ir_array[2];
  // Serial.print("input: ");
  // Serial.println(input);
  // Serial.println(data.rotation[0]);
  // double error = setpoint - input;
  // Serial.print("error: ");
  // Serial.println(error);
  // setpoint = 0;
  // pid_line.Compute();
  // Serial.print("output: ");
  // Serial.println(output);
  // Motors->twist(0.5, output);
}
