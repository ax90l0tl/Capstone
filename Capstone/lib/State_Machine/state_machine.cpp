#include "state_machine.h"

StateMachine::StateMachine()
{
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
    pid_turn.SetMode(AUTOMATIC);
    pid_turn.SetSampleTime(10);
    pid_turn.SetOutputLimits(-1.0, 1.0);
    pid_line.SetMode(AUTOMATIC);
    pid_line.SetSampleTime(10);
    pid_line.SetOutputLimits(-1.0, 1.0);
}

States StateMachine::update(States state, States last_state)
{
    switch (state)
    {
    case LINE_FOLLOWING:
        leds[0].setColorCode(RED);
        FastLED.show();
        // // See Intersection
        // if (data.ir_array[0] && data.ir_array[1] && data.ir_array[3] > THRESHOLD)
        // {
        //     update(INTERSECTION_PICKUP, state);
        // }
        // // Hit wall
        // if (data.distance < 20)
        // {
        //     update(GRABBING, state);
        // }
        // else
        // {
        lineFollowing(1.0);
        // }
        break;
    case TURN:
        leds[0].setColorCode(BLUE);
        FastLED.show();
        // switch (last_state)
        // {
        // case INTERSECTION_PICKUP:
        //     // depends
        //     break;
        // case INTERSECTION_DROPOFF:
        //     if (intersection_destination == intersection_counter)
        //     {
        //         turn(M_PI_2);
        //     }
        //     else
        //     {
        //         update(LINE_FOLLOWING, DROPOFF);
        //     }
        //     break;
        // case GRABBING:
        //     turn(M_1_PI);
        //     break;
        // default:
        //     turn(M_PI_2);
        //     break;
        // }
        break;
    case INTERSECTION_PICKUP:
        leds[0].setColorCode(GREEN);
        FastLED.show();
        intersection_counter++;
        // update(TURN, state);
        break;
    case GRABBING:
        leds[0].setColorCode(PINK);
        FastLED.show();
        use_ultrasonic = true;
        // update(TURN, state);
        break;
    case INTERSECTION_DROPOFF:
        leds[0].setColorCode(YELLOW);
        FastLED.show();
        intersection_counter++;
        // update(TURN, state);
        break;
    case DROPOFF:
        leds[0].setColorCode(PURPLE);
        FastLED.show();
        use_ultrasonic = true;
        // update(FINISH, state);
        break;
    default:
        leds[0].setColorCode(WHITE);
        FastLED.show();
        if(detectLine()){
            state = update(LINE_FOLLOWING);
        }
        break;
    }
    return state;
}

void StateMachine::getInstructions()
{
    intersection_destination = 3;
}

data_packet StateMachine::getData(bool verbose)
{
    for (uint8_t i = 0; i < 9; i++)
    {
        data.ir_array[i] = ir_sensor[i].getData();
        if (verbose)
        {
            Serial.print(data.ir_array[i]);
            Serial.print(",");
        }
    }
    Serial.println();
    imu->getData();
    for (uint8_t i = 0; i < 3; i++)
    {
        data.rotation[i] = imu->ypr[i];
        if (verbose)
        {
            Serial.print(data.rotation[i]);
            Serial.print(",");
        }
    }
    Serial.println();
    if (use_ultrasonic)
    {
        data.distance = ultrasonic->getMeasurement();
        if (verbose)
        {
            Serial.println(data.distance);
        }
    }
    return data;
}

bool StateMachine::detectLine()
{
    if (data.ir_array[0] > THRESHOLD && data.ir_array[2] > THRESHOLD)
    {
        return true;
    }
    return false;
}
bool StateMachine::detectPickupIntersection()
{
    if (data.ir_array[3] > THRESHOLD && data.ir_array[4] > THRESHOLD)
    {
        return true;
    }
    return false;
}
bool StateMachine::detectDropoffInttersection()
{
    if (data.ir_array[3] < THRESHOLD && data.ir_array[4] > THRESHOLD)
    {
        return true;
    }
    return false;
}

void StateMachine::lineFollowing(double speed)
{
    while (true)
    {
        uint16_t ir_data[9] = {0};
        for (uint8_t i = 0; i < 9; i++)
        {
            ir_data[i] = ir_sensor[i].getData();
        }
        // if (ir_data[3] > THRESHOLD || ir_data[4 > THRESHOLD])
        // {
        //     break;
        // }
        input_line = ir_data[0] - ir_data[2];
        setpoint_line = 0;
        pid_line.Compute();
        motor->twist(speed, output_line);
    }
}

bool StateMachine::turn(double angle, int timeout)
{
    unsigned long current = millis();
    while (abs(output_turn) <= 0.02)
    {
        if (current - millis() > timeout)
        {
            return false;
        }
        input_turn = angleWrap_pi(angle - imu->ypr[0]);
        output_turn = 0;
        setpoint_turn = 0;
        double error = setpoint_turn - input_turn;
        pid_turn.Compute();
        Serial.print("setpoint: ");
        Serial.println(error);
        Serial.print("output: ");
        Serial.println(output_turn);
        motor->twist(0, output_turn);
    }
    return true;
}

bool StateMachine::assignMotor(Motor_driver *motor_driver)
{
    if (motor_driver == nullptr)
    {
        return false;
    }
    motor = motor_driver;
    return true;
}
bool StateMachine::assignIMU(IMU *IMU)
{
    if (IMU == nullptr)
    {
        return false;
    }
    imu = IMU;
    return true;
}
bool StateMachine::assignIR(IR_sensor *IR_sensor)
{
    if (IR_sensor == nullptr)
    {
        return false;
    }
    ir_sensor = IR_sensor;
    return true;
}
bool StateMachine::assignUltrasonic(Ultrasonic *ultrasonic_sensor)
{
    if (ultrasonic == nullptr)
    {
        return false;
    }
    ultrasonic = ultrasonic_sensor;
    return true;
}

double angleWrap_360(double input)
{
    return (input -= 360. * floor(input * (1. / 360.)));
}

double angleWrap_180(double input)
{
    return (input -= ceil(input / 360. - 0.5) * 360.);
}

double angleWrap_pi(double input)
{
    return (atan2(sin(input), cos(input)));
}
