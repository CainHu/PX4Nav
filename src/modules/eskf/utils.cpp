//
// Created by Cain on 2023/1/3.
//

#include "utils.h"

namespace eskf {
    void rotation_from_axis_angle(Dcmf &r, const Vector3f &axis_angle) {
        /* Rodrigues's Formula:
        * a = n * θ
        * R = cosθ*I + (1 - cosθ)*n*n' + sinθ*n^
        * */
        const float a_norm_square = axis_angle.norm_squared();
        if (a_norm_square < 1e-18f) {
            r(0, 0) = 1.f;
            r(0, 1) = -axis_angle(2);
            r(0, 2) = axis_angle(1);
            r(1, 0) = axis_angle(2);
            r(1, 1) = 1.f;
            r(1, 2) = -axis_angle(0);
            r(2, 0) = -axis_angle(1);
            r(2, 1) = axis_angle(0);
            r(2, 2) = 1.f;
        } else {
            const float a_norm = sqrtf(a_norm_square);
            const Vector3f a_unit = axis_angle / a_norm;
            const float theta = a_norm;
            const float cos_theta = cosf(theta), sin_theta = sinf(theta);
            const float tmp = 1.f - cos_theta;

            const float xx = a_unit(0) * a_unit(0) * tmp;
            const float xy = a_unit(0) * a_unit(1) * tmp;
            const float xz = a_unit(0) * a_unit(2) * tmp;
            const float yy = a_unit(1) * a_unit(1) * tmp;
            const float yz = a_unit(1) * a_unit(2) * tmp;
            const float zz = a_unit(2) * a_unit(2) * tmp;

            const float sx = sin_theta * a_unit(0);
            const float sy = sin_theta * a_unit(1);
            const float sz = sin_theta * a_unit(2);

            r(0, 0) = cos_theta + xx;
            r(0, 1) = xy - sz;
            r(0, 2) = xz + sy;
            r(1, 0) = xy + sz;
            r(1, 1) = cos_theta + yy;
            r(1, 2) = yz - sx;
            r(2, 0) = xz - sy;
            r(2, 1) = yz + sx;
            r(2, 2) = cos_theta + zz;
        }
    }

    void quaternion_from_axis_angle(Quatf &q, const Vector3f &axis_angle) {
        /*
        a = n * θ
        q = [cos(θ/2), n*sin(θ/2)]
        */

        const float a_norm_square = axis_angle.norm_squared();
        if (a_norm_square < 1e-18f) {
            q(0) = sqrtf(1.f - 0.25f * a_norm_square);
            q(1) = 0.5f * axis_angle(0);
            q(2) = 0.5f * axis_angle(1);
            q(3) = 0.5f * axis_angle(2);
        } else {
            const float a_norm = sqrtf(a_norm_square);
            const Vector3f a_unit = axis_angle / a_norm;
            const float half_theta = 0.5f * a_norm;
            const float c = cosf(half_theta), s = sinf(half_theta);

            q(0) = c;
            q(1) = s * a_unit(0);
            q(2) = s * a_unit(1);
            q(3) = s * a_unit(2);
        }
    }
}
