#pragma once


#include "MinosSensor.h"
#include "pins_arduino.h"

class VoltMonitor : public MinosSensor {

    public:
        VoltMonitor(uint32_t pin) :
            channel(pin)
        {};
        
        bool init();
        bool update();
        int32_t get_data();
    private:
        uint32_t channel;
        uint32_t data; // uV
};