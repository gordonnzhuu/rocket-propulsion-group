#pragma once
#include "MinosSensor.h"

class LoopTimer : public MinosSensor {
    public:
        LoopTimer(double _alpha) :
            alpha(_alpha)
        {};
        
        bool init();
        bool update();
        int32_t get_data();
    private:
        double alpha;
        uint32_t average_loop_time;
        uint32_t last_loop_time;
};