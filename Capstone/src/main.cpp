#include <Arduino.h>
#include "Motor_driver.h"
#include "ultrasonic.h"
#include "ir_sensor.h"
#include "state_machine.h"
#include "imu.h"
#include "config.h"

Motor_driver *Motors;
Ultrasonic *ultrasonic;
IR_sensor *ir_array[NUM_IR] = {};
IMU *imu;
StateMachine *state_machine;
Servo *servo[NUM_SERVO] = {};

void (*resetFunc)(void) = 0; // declare reset function @ address 0
States state = STANDBY;
uint8_t trig_pin_ = TRIG_PIN;
uint8_t echo_pin_ = ECHO_PIN;

void setup()
{
  Motors = new Motor_driver(MOTOR_PWM_A, MOTOR_DIR_A, MOTOR_PWM_B, MOTOR_DIR_B, MOTOR_EN);
  ultrasonic = new Ultrasonic(TRIG_PIN, ECHO_PIN);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(1000);
  imu = new IMU(IMU_INT);
  uint8_t ir_pins[9] = {IR_0, IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7, IR_8};
  for (uint8_t i = 0; i < NUM_IR; i++)
  {
    ir_array[i] = new IR_sensor(ir_pins[i]);
  }
  if (!imu->start())
  {
    Serial.println("FAILED TO INITIALIZE IMU");
    delay(1000);
    resetFunc();
  }
  uint8_t servo_pins[3] = {SERVO_0, SERVO_1, SERVO_2};
  for (uint8_t i = 0; i < NUM_SERVO; i++)
  {
    servo[i]->attach(servo_pins[i]);
  }
  state_machine = new StateMachine();
  state_machine->assignMotor(Motors);
  state_machine->assignIMU(imu);
  state_machine->assignUltrasonic(ultrasonic);
  state_machine->assignIR(ir_array);
  state_machine->assignServo(servo);
}

void loop()
{
  state = state_machine->update(state, false);
  // state = state_machine->update(state, true);
  // state_machine->lineFollowing(1.0, true);
  // delay(10);
  // state_machine->getData(true, true, true, true);
  // state_machine->getData(true, false, false, true);
  // Serial.print(state_machine->detectWall());
  // state_machine->turn(-M_PI_2, 1.0, false, 2000);
  // delay(1000);
  // state_machine->exit_intersection(1.0, false);
  // delay(1000);
  // state_machine->approach_wall(0.3);
  // delay(1000);
  // state_machine->leave_wall(0.3);
  // delay(1000);
  // state_machine->turn(0.0, 1.0, false, 2000);
  // delay(1000);
  // state_machine->exit_intersection(1.0, true);
  // data_packet data = state_machine->getData(true, false, true, false);
  // float prev = data.rotation[0];
  // float prev = 0;
  // state_machine->turn(1.57, 1.0);
  // delay(2000);
  // Serial.print("prev" );
  // Serial.println(prev);
  // state_machine->turn(prev, 1.0);
  // state_machine->grab();
  // state_machine->lineFollowing(0.5);
}
