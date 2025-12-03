#include "GNCChannel.h"

bool GNCChannel::init() {
    return true;
}

bool GNCChannel::update() {

    // actual sensor poll handled by BNO hardware class.
    if (channel == GNCChannel::ALT_VEL)
        data = apogee_detector.get_da_dt();
    else
        return false;
    return true;
}

int32_t GNCChannel::get_data() {
    return data;
}
