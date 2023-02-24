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
        _delta_vel = delta_vel;
        _delta_angle = delta_ang + _delta_ang_bias;

        matrix::Quatf dq;
        quaternion_from_axis_angle(dq, _delta_angle);
        _q = _q * dq;
        _q.normalized();
        _r = _q;

        _v += _r * _delta_vel;
        _v(2) += G * _dt;
    }

    void AHRS::feed_vel(matrix::Vector3f &vel) {
        _error_delta_angle += 0.05f * (_delta_vel % (_r.transpose() * (vel - _v)));
        _error_vel += 4.f * (vel - _v);
    }

    void AHRS::feed_aux_meas(matrix::Vector3f &meas_nav, matrix::Vector3f &meas_body, float k) {
        _error_delta_angle += 0.5f * (meas_body % (_r.transpose() * meas_nav)) * _dt;
    }

    void AHRS::fit() {
        matrix::Quatf dq;
        quaternion_from_axis_angle(dq, _error_delta_angle);
        _q = _q * dq;
        _q.normalized();
        _r = _q;

        _v += _error_vel * _dt;

        _delta_ang_bias += 0.1f * _error_delta_angle * _dt;

        _error_vel.setZero();
        _error_delta_angle.setZero();
    }

    void AHRS::reset() {
        _r.setIdentity();
        _q.setIdentity();
        _v.setZero();
        _delta_ang_bias.setZero();

        _delta_angle.setZero();
        _delta_vel.setZero();
        _error_delta_angle.setZero();
        _error_vel.setZero();
    }
}