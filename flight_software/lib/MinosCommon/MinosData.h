#pragma once

#include "Arduino.h"
#include "minos_util.h"
#include "AdcChannel.h"
#include <cstdint>

#define CONFIG_LENGTH 100
typedef std::map<uint32_t, ActuatorEvent> AcuatorEvents;

class MinosData
{
    public:
        virtual bool init() {return false;}
        virtual bool write(uint8_t dataBuffer[], uint16_t dataLength) {return false;}
        virtual bool getConfig(SensorMap& SensorMap, ActuatorMap& actuators, AD7193 Adc0, AD7193 Adc1) {return false;};
        virtual bool writeConfig(SensorMap& SensorMap, ActuatorMap& actuators) {return false;};

        virtual bool writeDataAll(SensorMap& SensorMap, ActuatorMap& actuators) {return false;};
        virtual bool writeActuators(ActuatorMap& actuators) {return false;};
        virtual uint8_t read() {return 0;}
    private:
       // SPIClass spi3 =  SPIClass(PC12, PC11, PC10);
};