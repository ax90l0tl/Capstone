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

void StateMachine::state_machine_update(data_packet data, States state, States last_state)
{
    // get current sensor measurements
    last_data = data;
    switch (state)
    {
    case LINE_FOLLOWING:
        leds[0].setColorCode(RED);
        FastLED.show();
        // See Intersection
        if (data.ir_array[0] && data.ir_array[1] && data.ir_array[3] > threshold)
        {
            state_machine_update(INTERSECTION_PICKUP, state);
        }
        // Hit wall
        if (data.distance < 20)
        {
            state_machine_update(GRABBING, state);
        }
        else
        {
            lineFollowing();
        }
        break;
    case TURN:
        leds[0].setColorCode(BLUE);
        FastLED.show();
        switch (last_state)
        {
        case INTERSECTION_PICKUP:
            // depends
            break;
        case INTERSECTION_DROPOFF:
            turn(M_PI_2);
            break;
        case GRABBING:
            turn(M_1_PI);
            break;
        default:
            turn(M_PI_2);
            break;
        }
        break;
    case INTERSECTION_PICKUP:
        leds[0].setColorCode(GREEN);
        FastLED.show();
        intersection_counter++;
        state_machine_update(TURN, state);
        break;
    case GRABBING:
        leds[0].setColorCode(PINK);
        FastLED.show();
        state_machine_update(TURN, state);
        break;
    case INTERSECTION_DROPOFF:
        leds[0].setColorCode(YELLOW);
        FastLED.show();
        state_machine_update(TURN, state);
        break;
    case DROPOFF:
        leds[0].setColorCode(PURPLE);
        FastLED.show();
        state_machine_update(FINISH, state);
        break;
    default:
        leds[0].setColorCode(WHITE);
        FastLED.show();
        state_machine_update(STANDBY);
        break;
    }
}

data_packet getData()
{
    data_packet msg;
    return msg;
}

float *StateMachine::lineFollowing()
{
    data = getData();
    input_line = data.ir_array[0] - data.ir_array[2];
    setpoint_line = 0;
    pid_line.Compute();
    float power[2] = {0, (float)output_line};
    return power;
}

bool StateMachine::turn(double angle, int timeout)
{
    unsigned long current = millis();
    while (abs(output_turn) <= 0.02)
    {
        if(current -millis() > timeout){
            return false;
        }
        input_turn = angleWrap_pi(angle - last_data.rotation[0]);
        output_turn = 0;
        setpoint_turn = 0;
        double error = setpoint_turn - input_turn;
        pid_turn.Compute();
        Serial.print("setpoint: ");
        Serial.println(error);
        Serial.print("output: ");
        Serial.println(output_turn);
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