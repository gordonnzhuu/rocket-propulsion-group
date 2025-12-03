#include "GPSChannel.h"

bool GPSChannel::init() {
    return gps.getinitialized();
}

bool GPSChannel::update() {
    // actual sensor poll handled by GPS hardware class.
    data = gps.get_data(channel);
    
    return true;
}

int32_t GPSChannel::get_data() {
    return data;
}