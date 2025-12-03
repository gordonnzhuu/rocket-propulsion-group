#pragma once


#include "MinosSensor.h"
#include "pins_arduino.h"
#include "StateMachine.h"

extern FlightStateMachine state_machine;

class StateChannel : public MinosSensor {

    public:
        
        bool init();
        bool update();
        int32_t get_data();
    private:
        int32_t data; // mm or mm/s
        
};