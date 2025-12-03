#pragma once

#include "Arduino.h"
#include <cstdint>

typedef struct ActuatorEvent {
    uint32_t time;
    bool state;
    bool transition;
} ActuatorEvent;

class MinosActuator
{
    public:
        virtual bool init() {return false;}
        virtual bool update() {return false;} // no clue what this would do

        virtual bool set_state(bool state) {return false;}
        virtual bool get_state() {return false;}
        virtual ActuatorEvent get_event() {return ActuatorEvent();}


};