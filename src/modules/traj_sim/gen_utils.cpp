#include "gen_utils.h"

namespace generator {
    using namespace std;
    using namespace matrix;

    Matrix3d askew(const Vector3d &vec) {
        Matrix3d mat {};

        mat(0, 1) = -vec(2);
        mat(0, 2) = vec(1);
        mat(1, 0) = vec(2);
        mat(1, 2) = -vec(0);
        mat(2, 0) = -vec(1);
        mat(2, 1) = vec(0);

        return mat;       
    }

    Matrix3d euler2rot(const Vector3d &euler) {
        double cx = cosf(euler(0)), sx = sinf(euler(0));
        double cy = cosf(euler(1)), sy = sinf(euler(1));
        double cz = cosf(euler(2)), sz = sinf(euler(2));

        Matrix3d rot;
        rot(0, 0) = cy*cz;
        rot(0, 1) = cz*sx*sy - cx*sz;
        rot(0, 2) = cx*cz*sy + sx*sz;
        rot(1, 0) = cy*sz;
        rot(1, 1) = cx*cz + sx*sy*sz;
        rot(1, 2) = -(cz*sx) + cx*sy*sz;
        rot(2, 0) = -sy;
        rot(2, 1) = cy*sx;
        rot(2, 2) = cx*cy;

        return rot;
    }

    Vector3d rot2euler(const Matrix3d &rot) {
        Vector3d euler;

        double sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
        if (sy < 1e-6) {
            euler(0) = atan2(-rot(1, 2), rot(1, 1));
            euler(1) = atan2(-rot(2, 0), sy);
            euler(2) = 0.;
        } else {
            euler(0) = atan2(rot(2, 1), rot(2, 2));
            euler(1) = atan2(-rot(2, 0), sy);
            euler(2) = atan2(rot(1, 0), rot(0, 0));
        }

        return euler;
    }

    Quatd euler2quat(const Vector3d &euler) {
        Quatd quat;

        double cx = cos(0.5 * euler(0)), sx = sin(0.5 * euler(0));
        double cy = cos(0.5 * euler(1)), sy = sin(0.5 * euler(1));
        double cz = cos(0.5 * euler(2)), sz = sin(0.5 * euler(2));

        quat(0) = cz * cy * cx + sz * sy * sx;
        quat(1) = cz * cy * sx - sz * sy * cx;
        quat(2) = sz * cy * sx + cz * sy * cx;
        quat(3) = sz * cy * cx - cz * sy * sx;

        return quat;
    }

    Vector3d quat2euler(const Quatd &quat) {
        Vector3d euler;

        euler(0) = atan2(2. * (quat(0) * quat(1) + quat(2) * quat(3)), 1. - 2. * (quat(1) * quat(1) + quat(2) * quat(2)));
        euler(1) = asin(2. * (quat(0) * quat(2) - quat(1) * quat(3)));
        euler(2) = atan2(2. * (quat(0) * quat(3) + quat(1) * quat(2)), 1. - 2. * (quat(2) * quat(2) + quat(3) * quat(3)));

        return euler;
    }

    Matrix3d quat2rot(const Quatd &quat) {
        Matrix3d rot;

        const double q00 = quat(0) * quat(0), q01 = quat(0) * quat(1), q02 = quat(0) * quat(2), q03 = quat(0) * quat(3);
        const double q11 = quat(1) * quat(1), q12 = quat(1) * quat(2), q13 = quat(1) * quat(3);
        const double q22 = quat(2) * quat(2), q23 = quat(2) * quat(3);
        const double q33 = quat(3) * quat(3);

        rot(0, 0) = q00 + q11 - q22 - q33;
        rot(0, 1) = 2. * (q12 - q03);
        rot(0, 2) = 2. * (q02 + q13);
        rot(1, 0) = 2. * (q12 + q03);
        rot(1, 1) = q00 - q11 + q22 - q33;
        rot(1, 2) = 2. * (q23 - q01);
        rot(2, 0) = 2. * (q13 - q02);
        rot(2, 1) = 2. * (q01 + q23);
        rot(2, 2) = q00 - q11 - q22 + q33;

        return rot;       
    }

    Quatd rot2quat(const Matrix3d &rot) {
        Quatd quat;

        if (rot(0, 0) >= rot(1, 1) + rot(2, 2)) {
            quat(1) = 0.5 * sqrt(1. + rot(0, 0) - rot(1, 1) - rot(2, 2));
            quat(0) = (rot(2, 1) - rot(1, 2)) / (4. * quat(1));
            quat(2) = (rot(0, 1) + rot(1, 0)) / (4. * quat(1));
            quat(3) = (rot(0, 2) + rot(2, 0)) / (4. * quat(1));
        } else if (rot(1, 1) >= rot(0, 0) + rot(2, 2)) {
            quat(2) = 0.5 * sqrt(1. - rot(0, 0) + rot(1, 1) - rot(2, 2));
            quat(0) = (rot(0, 2) - rot(2, 0)) / (4. * quat(2));
            quat(1) = (rot(0, 1) + rot(1, 0)) / (4. * quat(2));
            quat(3) = (rot(1, 2) + rot(2, 1)) / (4. * quat(2));
        } else if (rot(2, 2) >= rot(0, 0) + rot(1, 1)) {
            quat(3) = 0.5 * sqrt(1. - rot(0, 0) - rot(1, 1) + rot(2, 2));
            quat(0) = (rot(1, 0) - rot(0, 1)) / (4. * quat(3));
            quat(1) = (rot(0, 2) + rot(2, 0)) / (4. * quat(3));
            quat(2) = (rot(1, 2) + rot(2, 1)) / (4. * quat(3));
        } else {
            quat(0) = 0.5 * sqrt(1. + rot(0, 0) + rot(1, 1) + rot(2, 2));
            quat(1) = (rot(2, 1) - rot(1, 2)) / (4. * quat(0));
            quat(2) = (rot(0, 2) - rot(2, 0)) / (4. * quat(0));
            quat(3) = (rot(1, 0) - rot(0, 1)) / (4. * quat(0));
        }

        return quat;
    }

    Matrix3d rv2rot(const Vector3d &rv) {
        double a, b;
        const double rv2 = rv.norm_squared();

        if (rv2 < 1e-8f) {
            a = 1 - rv2 * (1. / 6. - rv2 / 120.);
            b = 0.5 - rv2 * (1. / 24. - rv2 / 720.);
        } else {
            double norm = sqrt(rv2);
            a = sin(norm) / norm;
            b = (1. - cos(norm)) / rv2;
        }

        Matrix3d rv_hat = askew(rv);
        Matrix3d i;
        i.setIdentity();

        return i + a * rv_hat + b * rv_hat * rv_hat;
    }

    Quatd rv2quat(const Vector3d &rv) {
        double q0, q1, q2, q3, s;
        const double rv2 = rv.norm_squared();

        if (rv2 < 1e-8f) {
            q0 = 1. - rv2 * (1. / 8. - rv2 / 384.);
            s = 1. / 2. - rv2 * (1. / 48. - rv2 / 3840.);
        } else {
            double norm = sqrt(rv2);
            q0 = cos(0.5 * norm);
            s = sin(0.5 * norm) / norm;
        }

        q1 = s * rv(0);
        q2 = s * rv(1);
        q3 = s * rv(2);

        return {q0, q1, q2, q3};
    }

    Vector3d quat2rv(Quatd& quat) {
        if (quat(0) < 0) {
            quat = -quat;
        }

        double b;
        double nm_half = acos(quat(0));
        if (nm_half > 1e-12f) {
            b = 2. * nm_half / sin(nm_half);
        } else {
            b = 2.;
        }

        return {b * quat(1), b * quat(2), b * quat(3)};
    }
}
