#pragma once

#include "Arduino.h"
#include <cstdint>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>

static TwoWire two(PB7, PB6);

class MinosSensor
{
    public:
        virtual bool init() {
            __NOP();
            return false;
        }
        virtual bool update() {return false;}

        virtual uint32_t get_data() {return 0;}
};