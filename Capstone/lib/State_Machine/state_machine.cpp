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

States StateMachine::choose_action(States state, float speed, bool verbose)
{
    switch (state)
    {
    case LINE_FOLLOWING:
        leds[0].setColorCode(RED);
        FastLED.show();
        lineFollowing(speed);
        break;
    case TURN:
        verbose = false;
        leds[0].setColorCode(BLUE);
        FastLED.show();
        if (detectLeft() && detectRight())
        {
            if (intersections[intersection_counter] % 2 == 0)
            {
                turn(data.rotation[0] + M_PI_2, 1.0, verbose, 2000);
            }
            else
            {
                turn(data.rotation[0] - M_PI_2, 1.0, verbose, 2000);
            }
        }
        else
        {
            if (detectLeft())
            {
                turn(data.rotation[0] + M_PI_2, 1.0, verbose, 2000);
            }
            else if (detectRight())
            {
                turn(data.rotation[0] - M_PI_2, 1.0, verbose, 2000);
            }
        }
        exit_intersection(0.5);
        break;
    case INTERSECTION_PICKUP:
        prev_orientation = 0;
        choose_action(TURN, 1.0);
        leds[0].setColorCode(GREEN);
        FastLED.show();
        exit_intersection(1.0);
        // approach_wall(0.3);
        delay(1000);
        // leave_wall(0.5);
        turn(prev_orientation, 1.0, false, 3000);
        delay(1000);
        exit_intersection(1.0);
        delay(1000);
        break;
    case GRABBING:
        leds[0].setColorCode(PINK);
        FastLED.show();

        break;
    case INTERSECTION_DROPOFF:
        prev_orientation = M_PI;
        choose_action(TURN, 1.0);
        leds[0].setColorCode(YELLOW);
        FastLED.show();
        exit_intersection(speed);
        // approach_wall(0.3);
        delay(1000);
        // leave_wall(0.3);
        Serial.print("prev orient: ");
        Serial.println(prev_orientation);
        turn(prev_orientation, 1.0, verbose, 1000);
        exit_intersection(1.0);
        break;
    case DROPOFF:
        leds[0].setColorCode(PURPLE);
        FastLED.show();
        // approach_wall(0.3);
        break;
    case SEARCH:
        leds[0].setColorCode(WHITE);
        FastLED.show();
        search(0.5);
    default:
        leds[0].setColorCode(WHITE);
        FastLED.show();
        break;
    }
    return state;
}

States StateMachine::update(States last_state, bool verbose)
{
    data = getData(true, true, true, true);
    float speed = SPEED;
    if (detectLine())
    {
        if (detectLeft() && detectRight())
        {
            leds[0].setColorCode(GREEN);
            FastLED.show();
            // if (T_counter == intersections[intersection_counter])
            // {
            last_state = INTERSECTION_PICKUP;
            // T_counter++;
            // intersection_counter++;
            // if (intersection_counter == 2)
            // {
            // intersection_counter = 0;
            // }
            // }
            // else
            // {
            // exit_intersection(0.3);
            // last_state = LINE_FOLLOWING;
            // }
        }
        else if (detectLeft() || detectRight())
        {
            leds[0].setColorCode(YELLOW);
            FastLED.show();
            // Serial.println(L_counter);
            // Serial.println(intersection_counter);
            // if (L_counter == intersections[3])
            // {
            last_state = INTERSECTION_DROPOFF;
            // intersection_counter++;
            // Serial.println(L_counter);
            // Serial.println(intersection_counter);
            // }
            // else
            // {
            // L_counter++;
            // exit_intersection(0.3);
            // last_state = LINE_FOLLOWING;
            // }
        }
        else
        {
            last_state = LINE_FOLLOWING;
        }
    }
    else
    {
        if (detectLeft() || detectRight())
        {
            last_state = TURN;
        }
        else
        {
            last_state = SEARCH;
        }
    }
    // if (detectWall())
    // {
    //     if (last_state == INTERSECTION_PICKUP)
    //     {
    //         last_state = GRABBING;
    //         leds[0].setColorCode(PINK);
    //         FastLED.show();
    //     }
    //     else if (last_state == INTERSECTION_DROPOFF)
    //     {
    //         last_state = DROPOFF;
    //         leds[0].setColorCode(PURPLE);
    //         FastLED.show();
    //     }
    //     else if (last_state == GRABBING || last_state == DROPOFF)
    //     {
    //         last_state = STANDBY;
    //     }
    //     else
    //     {
    //         last_state = LINE_FOLLOWING;
    //         // speed = 1.0;
    //     }
    // }
    choose_action(last_state, speed, verbose);
    return last_state;
}

void StateMachine::getInstructions()
{
    // intersection_destination = 3;
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
    if (data.distance < WALL_THRESHOLD_FAR && abs(cos(data.rotation[0])) < 0.1)
    {
        return (true);
    }
    return (false);
}

void StateMachine::lineFollowing(float speed, bool verbose, unsigned long timeout)
{
    unsigned long current = millis();
    pid_line.SetTunings(speed * P_GAIN_LINE, speed * I_GAIN_LINE, speed * D_GAIN_LINE);
    while (millis() - current < timeout)
    {
        data = getData(verbose, true, false, false);
        if (detectLeft() || detectRight() || !detectLine())
        {
            break;
        }
        lineFollowing_gen(speed);
    }
    motor->twist(0.0, 0.0);
}
void StateMachine::lineFollowing_gen(float speed)
{
    input_line = data.ir_array[2] - data.ir_array[0];
    setpoint_line = 0;
    pid_line.Compute();
    if (speed < 0)
    {
        motor->twist(speed, -output_line);
    }
    else
    {
        motor->twist(speed, output_line);
    }
}
bool StateMachine::turn(double angle, double w, bool verbose, unsigned long timeout, float speed)
{
    unsigned long current = millis();
    pid_turn.SetOutputLimits(-w, w);
    output_turn = 0;
    setpoint_turn = 0;
    double error = 100;
    while (millis() - current < timeout)
    {
        data = getData(verbose, true, true, false);
        input_turn = angleWrap_pi(angle - data.rotation[0]);
        input_turn = angleWrap_pi(angle - data.rotation[0]);
        error = setpoint_turn - input_turn;
        output_turn = 0;
        pid_turn.Compute();
        Serial.print("setpoint: ");
        Serial.println(error);
        Serial.print("output: ");
        Serial.println(output_turn);
        motor->twist(speed, output_turn);
    }
    return true;
}
void StateMachine::approach_wall(float speed, bool verbose)
{
    pid_line.SetTunings(speed * P_GAIN_LINE, speed * I_GAIN_LINE, speed * D_GAIN_LINE);
    data = getData(verbose, false, false, true);
    while (abs(data.distance - WALL_THRESHOLD_CLOSE) > WALL_THRESHOLD)
    {
        data = getData(verbose, true, false, true);
        lineFollowing_gen(speed);
    }
    motor->twist(0.0, 0.0);
}
void StateMachine::leave_wall(float speed, bool verbose)
{
    pid_line.SetTunings(speed * P_GAIN_LINE, speed * I_GAIN_LINE, speed * D_GAIN_LINE);
    data = getData(verbose, true, false, false);
    // while (abs(data.distance - WALL_THRESHOLD_FAR) > 4.0)
    while (!detectLeft() && !detectRight())
    {
        data = getData(verbose, true, false, false);
        lineFollowing_gen(-speed);
    }
    motor->twist(0.0, 0.0);
}
void StateMachine::exit_intersection(float speed, bool verbose)
{
    pid_line.SetTunings(speed * P_GAIN_LINE, speed * I_GAIN_LINE, speed * D_GAIN_LINE);
    data = getData(verbose, true, false, false);
    while (detectLeft() || detectRight())
    {
        data = getData(verbose, true, false, false);
        if (!detectLeft() && !detectRight())
        {
            break;
        }
        lineFollowing_gen(speed);
    }
    motor->twist(0.0, 0.0);
}
void StateMachine::grab(bool verbose)
{
    delay(1000);
    servos[0]->write(30);
    delay(1000);
    servos[1]->write(0);
}
bool StateMachine::search(float speed, bool verbose)
{
    data = getData(verbose, true, false, false);
    while (data.ir_array[1] < THRESHOLD)
    {
        data = getData(verbose, true, false, false);
        motor->twist(0.0, -speed);
    }
    motor->twist(0.0, 0.0);
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
