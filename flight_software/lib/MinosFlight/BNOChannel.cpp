#include "BNOChannel.h"

bool BNOChannel::init() {
    return bno.getinitialized();
}

bool BNOChannel::update() {
    // actual sensor poll handled by BNO hardware class.
    data = bno.get_data(channel);
    
    return true;
}

int32_t BNOChannel::get_data() {
    return data;
}