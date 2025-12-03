#include "LoopTimer.h"

bool LoopTimer::init() {
    last_loop_time = micros();
    average_loop_time = 10; //baseline of 10ms
    return true;
}

bool LoopTimer::update() {
    uint32_t current_time = micros();
    uint32_t dt = current_time - last_loop_time;
    last_loop_time = current_time;

    //if dt is suddenly huge, it's an overflow, ignore it
    //expect to occur once every 70 minutes or so
    if (dt >= 0xff000000) return false;

    //exponential moving average
    average_loop_time = (uint32_t)(
        alpha * dt + (1 - alpha)*average_loop_time
    );

    return true;
}

int32_t LoopTimer::get_data() {
    return average_loop_time;
}