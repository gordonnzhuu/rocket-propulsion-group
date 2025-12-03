#include "AlphaBetaFilter.h"

void AlphaBetaFilter::init_state(double x0, double v0) {
    x = x0;
    v = v0;
}

double AlphaBetaFilter::update(double new_x, double dt) {
    if (dt == 0) {
        return x;
    }

    double xk = x + dt * v;
    double vk = v;

    double rk = new_x - xk;
    x = xk + (alpha * rk);
    v = vk + (beta / dt)*rk;

    return x;
}

double AlphaBetaFilter::get_x() {
    return x;
}

double AlphaBetaFilter::get_v() {
    return v;
}