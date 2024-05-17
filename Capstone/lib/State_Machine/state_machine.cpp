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
    data = getData(false, true, true, true);
    switch (state)
    {
    case LINE_FOLLOWING:
        leds[0].setColorCode(RED);
        FastLED.show();
        // See Intersection
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
        // update(FINISH, state);
        break;
    default:
        leds[0].setColorCode(WHITE);
        FastLED.show();
        if (detectLine())
        {
            state = update(LINE_FOLLOWING);
        }
        break;
    }
    return state;
}

States StateMachine::update(States last_state)
{
    data = getData(true, true, false, false);
    if (detectLine())
    {
        if (detectLeft() && detectRight())
        {
            last_state = INTERSECTION_PICKUP;
            leds[0].setColorCode(GREEN);
            FastLED.show();
        }
        else if (detectLeft() || detectRight())
        {
            last_state = INTERSECTION_DROPOFF;
            if (detectLeft())
            {
                leds[0].setColorCode(YELLOW);
                FastLED.show();
                // turn(90);
            }
            else if (detectRight())
            {
                leds[0].setColorCode(YELLOW);
                FastLED.show();
                // turn(-90);
            }
        }
        else
        {
            last_state = LINE_FOLLOWING;
            leds[0].setColorCode(RED);
            FastLED.show();
            lineFollowing(SPEED);
        }
    }
    return last_state;
}

void StateMachine::getInstructions()
{
    intersection_destination = 3;
}

data_packet StateMachine::getData(bool verbose, bool use_ir, bool use_imu, bool use_ultrasonic)
{
    if (use_ir)
    {
        for (uint8_t i = 0; i < NUM_IR; i++)
        {
            data.ir_array[i] = ir_sensor[i]->IR_sensor::getData();
            if (verbose)
            {
                Serial.print(data.ir_array[i]);
                Serial.print(",");
            }
        }
        if (verbose)
        {
            Serial.println();
        }
    }
    if (use_imu)
    {
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
        if (verbose)
        {
            Serial.println();
        }
    }
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
bool StateMachine::detectLeft()
{
    if (data.ir_array[3] > THRESHOLD)
    {
        return (true);
    }
    return (false);
}
bool StateMachine::detectRight()
{
    if (data.ir_array[4] > THRESHOLD)
    {
        return (true);
    }
    return (false);
}
bool StateMachine::detectWall()
{
    if (data.distance < WALL_THRESHOLD)
    {
        return (true);
    }
    return (false);
}

void StateMachine::lineFollowing(double speed)
{
    while (true)
    {
        data = getData(true, true, false, false);
        if (data.ir_array[3] > THRESHOLD || data.ir_array[4] > THRESHOLD)
        {
            break;
        }
        input_line = data.ir_array[0] - data.ir_array[2];
        setpoint_line = 0;
        pid_line.SetTunings(speed * P_GAIN_LINE, speed * I_GAIN_LINE, speed * D_GAIN_LINE);
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
void StateMachine::grab()
{
    delay(1000);
    servos[0]->write(30);
    delay(1000);
    servos[1]->write(0);
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
bool StateMachine::assignIR(IR_sensor *IR_sensor[NUM_IR])
{
    if (IR_sensor == nullptr)
    {
        return false;
    }
    for (uint8_t i = 0; i < NUM_IR; i++)
    {
        ir_sensor[i] = IR_sensor[i];
    }
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
bool StateMachine::assignServo(Servo *servo[NUM_SERVO])
{
    if (servo == nullptr)
    {
        return false;
    }
    for (uint8_t i = 0; i < NUM_SERVO; i++)
    {
        servos[i] = servo[i];
        servos[i]->write(0);
    }
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
