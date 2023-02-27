//
// Created by Cain on 2023/2/24.
//

#include "ahrs.h"

namespace ahrs {
    AHRS::AHRS(float dt) : _dt(dt) {
        reset();
    }

    void AHRS::feed_imu(matrix::Vector3f &delta_ang, matrix::Vector3f &delta_vel, float dt) {
        _dt = dt;

        _delta_t += dt;
        _delta_vel += matrix::Dcmf(_delta_q) * delta_vel;
        _delta_angle_correct = delta_ang + _delta_ang_offset;

        matrix::Quatf dq;
        quaternion_from_axis_angle(dq, _delta_angle_correct);
        _delta_q = _delta_q * dq;
        _delta_q.normalized();

        _q = _q * dq;
        _q.normalized();
        _r = _q;

        _v += _r * delta_vel;
        _v(2) += G * _dt;
    }

    void AHRS::feed_vel(matrix::Vector3f &vel) {
        float scale = G * G * _delta_t;
        _error_delta_angle += (K1 / scale) * (_delta_vel % (_r.transpose() * (vel - _v)));
        _error_vel += K2 * (vel - _v);
    }

    void AHRS::feed_aux_meas(matrix::Vector3f &meas_nav, matrix::Vector3f &meas_body, float k) {
        _error_delta_angle += k * (meas_body.unit() % (_r.transpose() * meas_nav.unit())) * _delta_t;
    }

    void AHRS::fit() {
        matrix::Quatf dq;
        quaternion_from_axis_angle(dq, _error_delta_angle);
        _q = _q * dq;
        _q.normalized();
        _r = _q;

        _v += _error_vel * _dt;

        _delta_ang_offset += K0 * _error_delta_angle * _dt;

        _delta_t = 0.f;
        _delta_q.setIdentity();
        _delta_vel.setZero();

        _error_vel.setZero();
        _error_delta_angle.setZero();
    }

    void AHRS::reset() {
        _r.setIdentity();
        _q.setIdentity();
        _v.setZero();
        _delta_ang_offset.setZero();
        _delta_angle_correct.setZero();

        _delta_t = 0.f;
        _delta_q.setIdentity();
        _delta_vel.setZero();

        _error_delta_angle.setZero();
        _error_vel.setZero();
    }
}