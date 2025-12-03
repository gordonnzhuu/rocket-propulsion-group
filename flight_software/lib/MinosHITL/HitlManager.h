#pragma once

#include <vector>
#include <CSV_Parser.h>
#include "SDcard.h"

class HitlManager {
    public:    
        HitlManager():
            hitlParser(hitl_format)
        {};
 
        enum hitl_mode_t {
            MODE_PAUSED,
            MODE_REAL_TIME,
        };

        bool init();
        bool update();

        void set_hitl_mode(hitl_mode_t mode);
        bool set_hitl_time(double new_time);

        uint32_t get_current_time();
        double get_current_alt();

        hitl_mode_t get_hitl_mode( ) { return current_mode; }

    private:
        const char* hitl_format = "fffff";
        CSV_Parser hitlParser;

        hitl_mode_t current_mode = MODE_PAUSED;
        uint32_t init_time = 0;
        uint32_t current_index = 0;

        std::vector<uint32_t> time_vec;
        std::vector<float> alt_vec;
};