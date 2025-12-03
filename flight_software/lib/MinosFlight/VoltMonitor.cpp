#include "VoltMonitor.h"

bool VoltMonitor::init() {
    pinMode(channel, INPUT_ANALOG);
    return true;
}

bool VoltMonitor::update() {

    // actual sensor poll handled by BNO hardware class.
    data = (uint32_t) ((float) (analogRead(channel))*3.3/1023*1000) ;
    
    return true;
}

int32_t VoltMonitor::get_data() {
    return data; //uV
}
