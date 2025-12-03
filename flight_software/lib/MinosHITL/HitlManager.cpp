#include "HitlManager.h"

extern SDcard sdcard;

char feedRowParser() {
  return sdcard.getNextHitlChar();
}
bool rowParserFinished() {
  return sdcard.checkHitlAvailible();
}

bool HitlManager::init(){

    while (hitlParser.parseRow()) {
        time_vec.push_back(
            (uint32_t)(((float*)hitlParser["Time (s)"])[0] * 1000)
        );
       
        alt_vec.push_back(
           ((float*)hitlParser["Altitude AGL (m)"])[0]
        );
    }

    init_time = millis();
    volatile uint32_t tmp = time_vec[6];
    __NOP();
    return true;
}

bool HitlManager::update() {
    if (current_mode == MODE_REAL_TIME) {
        while (time_vec[current_index + 1] <= (millis() - init_time) && current_index < time_vec.size()) {
            current_index++;
        }
    }

    return true;
}

void HitlManager::set_hitl_mode(hitl_mode_t new_mode) {
    current_mode = new_mode;

    if (current_mode == MODE_REAL_TIME) {
        current_index = 0;
        init_time = millis();
    }
}

double HitlManager::get_current_alt() {
    double tmp;
    if (current_mode == MODE_PAUSED) {
        tmp = 0.0;
    }else{
        tmp = alt_vec[current_index];
    }
    return tmp;
}