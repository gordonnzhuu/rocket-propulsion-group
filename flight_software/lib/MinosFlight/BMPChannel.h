#pragma once


#include "MinosSensor.h"
#include "pins_arduino.h"
#include "BMP388.h"

extern BMP388 bmp;

class BMPChannel : public MinosSensor {

    public:
        BMPChannel(BMP388::channel_config_t channel) :
            channel(channel)
        {};
        
        bool init();
        bool update();
        int32_t get_data();
    private:
        BMP388::channel_config_t channel;
        uint32_t data; // cm/s^2
};