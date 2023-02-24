//
// Created by Cain on 2023/1/9.
//

#ifndef ECL_ANGULAR_VELOCITY_H_INFINITY_HPP
#define ECL_ANGULAR_VELOCITY_H_INFINITY_HPP

#include "siso_base.hpp"

namespace control {
    using namespace matrix;

    class AngularVelocityHInfinity : public SISO<9> {
    public:
        explicit AngularVelocityHInfinity(float u_min=-1.f, float u_max=1.f) : SISO<9>(u_min, u_max) {};

        float operator()(const float &ref, const float &obs) override {
            Vector2f y(ref, ref - obs);

            // x_ = Ax + Bu
            Vector<float, 9> x_prev;
            x_prev = _x;

            _x(0) = +(8.767949e-01f)*x_prev(0)+(6.292332e-02f)*x_prev(1)+(6.304253e-04f)*y(0)+(4.083787e-04f)*y(1);
            _x(1) = +(-6.292332e-02f)*x_prev(0)+(8.767949e-01f)*x_prev(1)+(1.942860e-04f)*y(0)+(-3.186859e-05f)*y(1);
            _x(2) = +(9.696436e-01f)*x_prev(2)+(6.076778e-02f)*x_prev(3)+(1.291924e-03f)*y(0)+(1.061685e-03f)*y(1);
            _x(3) = +(-6.076778e-02f)*x_prev(2)+(9.696436e-01f)*x_prev(3)+(-3.297218e-03f)*y(0)+(-9.878509e-04f)*y(1);
            _x(4) = +(9.427804e-01f)*x_prev(4)+(2.115931e-09f)*x_prev(5)+(-1.347396e-03f)*y(0)+(7.838706e-18f)*y(1);
            _x(5) = +(-2.115931e-09f)*x_prev(4)+(9.427804e-01f)*x_prev(5)+(8.526948e+03f)*y(0)+(-1.056356e-10f)*y(1);
            _x(6) = +(9.641783e-01f)*x_prev(6)+(-2.998020e-06f)*y(0)+(-5.258832e-08f)*y(1);
            _x(7) = +(9.937625e-01f)*x_prev(7)+(1.590997e-04f)*y(0)+(2.754263e-04f)*y(1);
            _x(8) = +(1.000000e+00f)*x_prev(8)+(2.211961e-04f)*y(1);
            _u_raw = +(3.753590e+00f)*x_prev(0)+(1.258466e-01f)*x_prev(1)+(1.662830e+00f)*x_prev(2)+(-1.750723e+00f)*x_prev(3)+(2.148980e+00f)*x_prev(4)+(-5.137981e-09f)*x_prev(5)+(1.764761e+00f)*x_prev(6)+(3.138093e-01f)*x_prev(7)+(3.922030e-02f)*x_prev(8)+(3.827971e-03f)*y(0)+(2.633689e-03f)*y(1);

            clip();

            return _u_clip;
        };

        void reset_integral() override {
            _x(8) = 0.f;
        }
    };
}

#endif //ECL_ANGULAR_VELOCITY_H_INFINITY_HPP
