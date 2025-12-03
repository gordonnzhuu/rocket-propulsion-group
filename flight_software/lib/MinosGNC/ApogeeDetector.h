#pragma once
#include <vector>

#include "minos_util.h"
#include "AlphaBetaFilter.h"

#define APOGEE_ALPHA 0.1
#define APOGEE_BETA 0.001
#define APOGEE_K 20

typedef struct {
    double altitude;
    uint32_t time;
} alt_measurement_t;

class ApogeeDetector
{
    public:
        ApogeeDetector(
            double _alpha,
            double _beta,
            double _k,
            double _v_min
        ):  
            alpha(_alpha),
            beta(_beta),
            k(_k),
            v_min(_v_min),
            filter(alpha, beta)
        {};

        bool init(double v0);
        bool update();

        bool poll_apogee();
        void add_measurement(double measurement);
        double get_da_dt();
        double get_filtered_da_dt();

    private:
        double alpha;
        double beta;
        double k;
        double v_min;

        bool apogee_detected = false;

        std::vector<alt_measurement_t> alt_vec;

        AlphaBetaFilter filter;
};
