#include "SolenoidDriver.h"
// Possible pins:
    // pinMode(PD8, OUTPUT);
    // pinMode(PD9, OUTPUT);
    // pinMode(PD10, OUTPUT);
    // pinMode(PD11, OUTPUT);

    // pinMode(PD12, OUTPUT);
    // pinMode(PD13, OUTPUT);
    // pinMode(PD14, OUTPUT);
    // pinMode(PD15, OUTPUT);


bool SolenoidDriver::init()
{
    pinMode(pin,OUTPUT);
    set_state(state);
    actuation.state = state;
    actuation.time = millis();
    return true;
}

bool SolenoidDriver::update()
{
    return true;
}

bool SolenoidDriver::set_state(bool new_state)
{
    state = new_state;
    digitalWrite(pin,state);
    actuation.state = state;
    actuation.time = millis();
    return true;
}

bool SolenoidDriver::get_state()
{
    return state;
}

ActuatorEvent SolenoidDriver::get_event()
{
    return actuation;
}

