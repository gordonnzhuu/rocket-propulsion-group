#pragma once


#include "MinosSensor.h"
#include "pins_arduino.h"
#include "AlphaBetaFilter.h"
#include "ApogeeDetector.h"

extern ApogeeDetector apogee_detector;

class GNCChannel : public MinosSensor {

    public:
        enum channel_config_t {
            ALT_VEL
        };

        GNCChannel(GNCChannel::channel_config_t channel) :
            channel(channel)
        {};
        
        bool init();
        bool update();
        int32_t get_data();
    private:
        GNCChannel::channel_config_t channel;
        int32_t data; // mm or mm/s
};