#pragma once

#include "Adafruit_BMP3XX.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include "MinosSensor.h"


class BMP388 : public MinosSensor {
    public:
    BMP388(TwoWire i2c) :
        bmp_i2c(i2c)
        {};  
        bool init(TwoWire two = two);
        bool update();
        
        uint32_t get_data();
        
        
    private:
        uint8_t addr = 0x77;
        float altitude;
        TwoWire bmp_i2c;
};