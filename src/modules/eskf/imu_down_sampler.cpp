//
// Created by Cain on 2023/1/6.
//

#include "imu_down_sampler.h"
#include <iostream>

namespace eskf {
    void ImuDownSampler::reset() {
        _imu_down_sampled = {};
        _delta_q.setIdentity();
        _accumulated_samples = 0;

        // 约束下采样的采样周期
        float target_dt_s = math::constrain(_target_dt_us, dt_us_min, dt_us_max) * 1e-6f;

        _required_samples = math::max((int)roundf(target_dt_s / _delta_ang_dt_avg), 1);

        _target_dt_s = float(_required_samples) * _delta_ang_dt_avg;

        // minimum delta angle dt (in addition to number of samples)
        _min_dt_s = math::max(_delta_ang_dt_avg * (float(_required_samples) - 1.f), _delta_ang_dt_avg * 0.5f);
    }

    bool ImuDownSampler::update(const ImuSample &imu_sample_new) {
        _delta_ang_dt_avg = 0.9f * _delta_ang_dt_avg + 0.1f * imu_sample_new.delta_ang_dt;

        _imu_down_sampled.time_us = imu_sample_new.time_us;
        _imu_down_sampled.delta_ang_dt += imu_sample_new.delta_ang_dt;
        _imu_down_sampled.delta_vel_dt += imu_sample_new.delta_vel_dt;
        _imu_down_sampled.delta_ang_clipping[0] |= imu_sample_new.delta_ang_clipping[0];
        _imu_down_sampled.delta_ang_clipping[1] |= imu_sample_new.delta_ang_clipping[1];
        _imu_down_sampled.delta_ang_clipping[2] |= imu_sample_new.delta_ang_clipping[2];
        _imu_down_sampled.delta_vel_clipping[0] |= imu_sample_new.delta_vel_clipping[0];
        _imu_down_sampled.delta_vel_clipping[1] |= imu_sample_new.delta_vel_clipping[1];
        _imu_down_sampled.delta_vel_clipping[2] |= imu_sample_new.delta_vel_clipping[2];

//        std::cout << "_imu_down_sampled.delta_ang_dt = " << _imu_down_sampled.delta_ang_dt << std::endl;

        const Dcmf dR(_delta_q);
        const Vector3f dv = dR * imu_sample_new.delta_vel;
        _delta_pos += (_delta_vel + 0.5f * dv) * imu_sample_new.delta_vel_dt;
        _delta_vel += dv;

        const Quatf dq(AxisAnglef(imu_sample_new.delta_ang));
        _delta_q = _delta_q * dq;
        _delta_q.normalize();

        ++_accumulated_samples;

//        std::cout << _accumulated_samples << std::endl;

        // _accumulated_samples >= _required_samples && _imu_down_sampled.delta_ang_dt > _min_dt_s = true
        // 发生在imu采样率突然变快的时候, 此时很早就有_accumulated_samples >= _required_samples
        // 但是_imu_down_sampled.delta_ang_dt > _min_dt_s随后才发生
        if ((_accumulated_samples >= _required_samples && _imu_down_sampled.delta_ang_dt > _min_dt_s)
            || (_imu_down_sampled.delta_ang_dt > _target_dt_s)) {

            _imu_down_sampled.delta_ang = AxisAnglef(_delta_q);
            return true;
        }

        return false;
    }
}