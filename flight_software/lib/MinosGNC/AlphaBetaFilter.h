#pragma once

class AlphaBetaFilter {
    public:
        AlphaBetaFilter(double _alpha, double _beta):
            alpha(_alpha),
            beta(_beta)
        {};

        void init_state(double x0, double v0);
        double update(double new_x, double dt);

        double get_x();
        double get_v();
    private:
        double alpha;
        double beta;

        double x;
        double v;
};
