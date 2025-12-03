#include "BMPChannel.h"

bool BMPChannel::init() {
    return bmp.getinitialized();
}

bool BMPChannel::update() {

    // actual sensor poll handled by BNO hardware class.
    data = bmp.get_data(channel);
    
    return true;
}

int32_t BMPChannel::get_data() {
    return data;
}