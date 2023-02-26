//
// Created by Cain on 2023/2/24.
//

#ifndef ECL_AHRS_H
#define ECL_AHRS_H

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace ahrs {
    class AHRS {
    public:
        explicit AHRS(float dt);

        void feed_aux_meas(matrix::Vector3f &meas_nav, matrix::Vector3f &meas_body, float k=K1);
        void feed_imu(matrix::Vector3f &delta_ang, matrix::Vector3f &delta_vel, float dt);
        void feed_vel(matrix::Vector3f &vel);
        void fit();

        void reset();

        matrix::Quatf &q() { return _q; }
        matrix::Dcmf &r() { return _r; }
        matrix::Vector3f &v() { return _v; }
        matrix::Vector3f &delta_ang_bias() { return _delta_ang_bias; }

    protected:
        matrix::Dcmf _r;
        matrix::Quatf _q;
        matrix::Vector3f _v;
        matrix::Vector3f _delta_ang_bias;

        float _dt;
        matrix::AxisAnglef _delta_angle;
        matrix::Vector3f _delta_vel;
        matrix::Vector3f _error_delta_angle;
        matrix::Vector3f _error_vel;

    private:
        static constexpr float K0 {0.1f};
        static constexpr float K1 {0.05f};
        static constexpr float K2 {4.f};
        static constexpr float G {9.8f};

        static void quaternion_from_axis_angle(matrix::Quatf &q, const matrix::Vector3f &axis_angle) {
            /*
            a = n * θ
            q = [cos(θ/2), n*sin(θ/2)]
            */

            const float a_norm_square = axis_angle.norm_squared();
            if (a_norm_square < 1e-12f) {
                q(0) = sqrtf(1.f - 0.25f * a_norm_square);
                q(1) = 0.5f * axis_angle(0);
                q(2) = 0.5f * axis_angle(1);
                q(3) = 0.5f * axis_angle(2);
            } else {
                const float a_norm = sqrtf(a_norm_square);
                const matrix::Vector3f a_unit = axis_angle / a_norm;
                const float half_theta = 0.5f * a_norm;
                const float c = cosf(half_theta), s = sinf(half_theta);

                q(0) = c;
                q(1) = s * a_unit(0);
                q(2) = s * a_unit(1);
                q(3) = s * a_unit(2);
            }
        }
    };
}

#endif //ECL_AHRS_H
