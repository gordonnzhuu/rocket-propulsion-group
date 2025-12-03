#pragma once

#include "MinosSensor.h"
#include "pins_arduino.h"
#include "GPS.h"

extern GPS gps;

class GPSChannel : public MinosSensor {

    public:
        GPSChannel(GPS::channel_config_t channel) :
            channel(channel)
        {};
        
        bool init();
        bool update();
        int32_t get_data();
    private:
        GPS::channel_config_t channel;
        uint32_t data;
};