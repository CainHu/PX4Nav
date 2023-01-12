//
// Created by Cain on 2023/1/6.
//

#ifndef ECL_IMU_DOWN_SAMPLER_H
#define ECL_IMU_DOWN_SAMPLER_H

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include "common.h"

namespace eskf {
    class ImuDownSampler {
    public:
        static constexpr int32_t dt_us_min {1000};
        static constexpr int32_t dt_us_max {100000};

        explicit ImuDownSampler(int32_t &target_dt_us) : _delta_vel(_imu_down_sampled.delta_vel), _target_dt_us(target_dt_us) {
            reset();
        };

        ImuSample get_down_sampled_imu_and_trigger_reset()
        {
            ImuSample imu {_imu_down_sampled};
            reset();
            return imu;
        }

        void reset();
        bool update(const ImuSample &imu_sample_new);

        const ImuSample &get_down_sampled() const { return _imu_down_sampled; };
        const Quatf &get_delta_q() const { return _delta_q; }
        const Vector3f &get_delta_vel() const { return _delta_vel; }
        const Vector3f &get_delta_pos() const { return _delta_pos; }
        float get_delta_ang_dt_avg() const { return _delta_ang_dt_avg; }
    private:
        Quatf _delta_q;
        Vector3f &_delta_vel;
        Vector3f _delta_pos;

        ImuSample _imu_down_sampled;

        int _accumulated_samples {0};
        int _required_samples {1};

        int32_t &_target_dt_us;

        float _target_dt_s{0.010f};
        float _min_dt_s{0.005f};

        float _delta_ang_dt_avg{0.005f};
    };
}

#endif //ECL_IMU_DOWN_SAMPLER_H
