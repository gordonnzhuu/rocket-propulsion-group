#pragma once


#include "MinosSensor.h"
#include "pins_arduino.h"
#include "BNO.h"

extern BNO bno;

class BNOChannel : public MinosSensor {

    public:
        BNOChannel(BNO::channel_config_t channel) :
            channel(channel)
        {};
        
        bool init();
        bool update();
        int32_t get_data();
    private:
        BNO::channel_config_t channel;
        uint32_t data; // cm/s^2
};