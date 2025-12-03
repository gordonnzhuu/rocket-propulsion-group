#include "ApogeeDetector.h"

bool ApogeeDetector::init(double v0) {
    filter.init_state(
        get_da_dt(),
        v0
    );

    return true;
}

bool ApogeeDetector::update() {
    double da_dt = get_da_dt();

    filter.update(
        da_dt,
        double(alt_vec[0].time - alt_vec[1].time) / 1000.0
    );

    volatile double filtered_da_dt = filter.get_x();

    if (millis() > 2000 && !apogee_detected && filtered_da_dt < v_min) {
        apogee_detected = true;
    }
    
    return true;
}

void ApogeeDetector::add_measurement(double measurement) {
    alt_measurement_t new_measurement;
    new_measurement.altitude = measurement;
    new_measurement.time = millis();

    alt_vec.insert(alt_vec.begin(), new_measurement);

    // keep max size at K
    if (alt_vec.size() > k) {
        alt_vec.pop_back();
    }
}

double ApogeeDetector::get_da_dt() {
    // If there is nothing to take a derivative of.
    if (alt_vec.size() < 1)
        return 0.0;

    // Check for float div-by-zero
    if (alt_vec.front().time - alt_vec.back().time == 0)
        return 0.0;
    
    double dt = ((double)((alt_vec.front().time - alt_vec.back().time)))/1000.0;
    double da = (alt_vec.front().altitude - alt_vec.back().altitude);
    double da_dt = da / dt;

    return da_dt;
}

double ApogeeDetector::get_filtered_da_dt() {
    return filter.get_x();
}

bool ApogeeDetector::poll_apogee() {
    return apogee_detected;
}