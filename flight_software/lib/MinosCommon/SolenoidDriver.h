#pragma once

#include "MinosActuator.h"
#include "pins_arduino.h"



class SolenoidDriver : public MinosActuator
{
    public:

        SolenoidDriver(uint32_t pin_number,bool normal_state) :
            pin(pin_number),
            state(normal_state)
        {};

        bool init();
        bool update();

        bool set_state(bool state);
        bool get_state();
        ActuatorEvent get_event();
    private:
        ActuatorEvent actuation;
        bool state;
        uint32_t pin;
};