//
// Created by Cain on 2022/12/7.
//

#ifndef CONTROL_ANGULAR_VELOCITY_LOOP_SHAPING_HPP
#define CONTROL_ANGULAR_VELOCITY_LOOP_SHAPING_HPP

#include "siso_base.hpp"

namespace control {
    using namespace matrix;

    class AngularVelocityLoopShaping : public SISO<2> {
    public:
        explicit AngularVelocityLoopShaping(float u_min=-1.f, float u_max=1.f) : SISO<2>(u_min, u_max) {};

        float operator()(const float &ref, const float &obs) override {
            Vector2f y(ref, ref - obs);

            // x_ = Ax + Bu
            Vector<float, 2> x_prev;
            x_prev = _x;

            _x(0) = +(9.148400e-01f)*x_prev(0)+(-4.798691e-03f)*y(0)+(-1.621462e-04f)*y(1);
            _x(1) = +(1.000000e+00f)*x_prev(1)+(-2.505323e-18f)*y(0)+(2.541108e-04f)*y(1);
            _u_raw = +(3.829680e+00f)*x_prev(0)+(1.123817e-01f)*x_prev(1)+(2.143448e-01f)*y(0)+(3.585452e-02f)*y(1);

            clip();

            return _u_clip;
        };

        void reset_integral() override {
            _x(1) = 0.f;
        }
    };
}

#endif //CONTROL_ANGULAR_VELOCITY_LOOP_SHAPING_H