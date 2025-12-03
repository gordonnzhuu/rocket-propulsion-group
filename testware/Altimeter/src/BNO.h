#pragma once


#include "MinosSensor.h"
#include "pins_arduino.h"
#include "BNO055.h"

class BNO : public MinosSensor {
    public:
        BNO(TwoWire i2c) :
            bno_i2c(i2c)
        {};
        bool init(TwoWire two = two);
        bool update();
        
        uint32_t get_data();

        uint32_t get_data(char axis);

        BNO055_measurment_data_t get_data_struct();

        void set_config(uint8_t *config);

        bool get_config(uint8_t *config);
        
    private:
        uint8_t addr = 0x28;
        uint8_t config[22];
        BNO055_measurment_data_t data;
        TwoWire bno_i2c;

};