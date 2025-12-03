#include "StateChannel.h"

bool StateChannel::init() {
    return true;
}

bool StateChannel::update() {

    // actual sensor poll handled by BNO hardware class.
    data = state_machine.get_state();
    return true;
}

int32_t StateChannel::get_data() {
    return data;
}
