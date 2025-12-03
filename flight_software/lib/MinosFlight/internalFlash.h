#pragma once
//#include <FlashStorage_STM32.hpp>
//#include <EEPROM.h>
#include "StateMachine.h"

class internalFlash
{
    public:
        internalFlash():
            addr(0) 
        {};
        bool savedSensors = false;
        bool init();
        bool saveSensors();
        bool setFlightState(flight_fsm_e flightState);
        uint32_t getTimeout(){return configTimeout;}
        bool setTimeout(uint32_t timeout){configTimeout = timeout; return true;}
        uint8_t readFlightSensors(SensorMap& sensors, ActuatorMap& actuators);
    private:
        uint16_t addr = 0;
        uint32_t configTimeout = 0;
};

