#include <Arduino.h>
#include "Motor_driver.h"
#include "ultrasonic.h"
#include "ir_sensor.h"
#include "state_machine.h"
#include "imu.h"
#include "config.h"

Motor_driver *Motors;
Ultrasonic *ultrasonic;
IR_sensor *ir_array[9] = {};
IMU *imu;
StateMachine *state_machine;

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
  for (uint8_t i = 0; i < 9; i++)
  {
    ir_array[i] = new IR_sensor(ir_pins[i]);
  }
  if (!imu->start())
  {
    Serial.println("FAILED TO INITIALIZE IMU");
    delay(1000);
    resetFunc();
  }
  IR_sensor *ir_ptr = ir_array[0];
  state_machine = new StateMachine();
  state_machine->assignMotor(Motors);
  state_machine->assignIMU(imu);
  state_machine->assignUltrasonic(ultrasonic);
  state_machine->assignIR(ir_ptr);
  pinMode(trig_pin_, OUTPUT);
  pinMode(echo_pin_, INPUT);
  digitalWrite(trig_pin_, LOW);
}

void loop()
{
  state_machine->getData(false);
  state = state_machine->update(state);
  Serial.println(state);
}
