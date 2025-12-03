#pragma once

#include "MinosData.h"
#include "SPI.h"
#include "../SST25VF_BURPG/SST25VF.h"

class Flash : public MinosData
{
    public:
        Flash(uint32_t m_cs_pin):
            cs_pin(m_cs_pin) {};
        bool init();
        bool write(uint8_t dataBuffer[], uint16_t dataLength);
        bool getConfig(SensorMap& SensorMap, ActuatorMap& actuators, AD7193 Adc0, AD7193 Adc1);
        bool writeConfig(SensorMap& SensorMap, ActuatorMap& actuators);

        bool writeDataAll(SensorMap& SensorMap, ActuatorMap& actuators);
        bool readDataLine(char dataBuffer[]);
        bool clearFlash();

        uint32_t getDataLength();
        uint8_t read(uint16_t addr);

    private:
        uint32_t cs_pin;
        uint32_t waddress = 0;
        uint32_t raddress = 0;
        uint32_t data_l;
        bool data_flag = false;
        uint32_t flag_length = 0;
        SST25VF flash;
        char idbuffer[16];
    };