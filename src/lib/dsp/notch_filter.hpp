//
// Created by Cain on 2023/3/10.
//

#ifndef DSP_NOTCH_FILTER_HPP
#define DSP_NOTCH_FILTER_HPP

#include "biquadratic.hpp"
#include <cfloat>

#define PX4_ISFINITE std::isfinite

namespace digital_signal_processing {
    class NotchFilter {
    public:
        /*!
         * 构造函数
         * @param fs 采样频率(Hz)
         * @param cutoff 陷波中心频率(Hz)
         * @param bandwidth -3dB对应的宽度(Hz)
         * @param depth 陷波的深度(dB)
         */
        NotchFilter(float fs, float cutoff, float bandwidth, float depth) {
            set_fs(fs);
            set_cutoff(cutoff);
            set_bandwidth(bandwidth);
            set_depth(depth);

            update_filter_parameters();
        }

        float operator()(float u) {
            return _k * _bi_quadratic(u);
        }

        void set_fs(float fs) {
            assert(fs > 0.f);
            _fs = fs;
        }

        void set_cutoff(float cutoff) {
            assert(cutoff > 0.f);
            assert(cutoff < 0.5f * _fs);
            _cutoff = cutoff;
        }

        void set_bandwidth(float bandwidth) {
            assert(bandwidth > 0.f);
            assert(0.5f * bandwidth < _cutoff);
            _bandwidth = bandwidth;
        }

        void set_depth(float depth) {
            assert(depth < -3.f);
            _depth = depth;
        }

        void update_filter_parameters() {
            float depth = powf(10.f, _depth / 20.f);
            float zeta2 = _bandwidth / (2.f * _cutoff) / sqrtf(1.f - depth * depth);
            float zeta1 = depth * zeta2;

            float rho = tanf(M_PI_F * _cutoff / _fs);
            float rho2 = rho * rho;
            float b0 = 1.f + 2.f * zeta1 * rho + rho2;
            float b1 = 2.f * (rho2 - 1.f);
            float b2 = 1.f - 2.f * zeta1 * rho + rho2;
            float a0 = 1.f + 2.f * zeta2 * rho + rho2;
            float a1 = b1;
            float a2 = 1.f - 2.f * zeta2 * rho + rho2;

            _bi_quadratic.set_parameters(a1 / a0, a2 / a0, 1.f, b1 / b0, b2 / b0);
            _k = b0 / a0;
        }

        void reset_filter_state() {
            _bi_quadratic.reset();
        }

        void reset_filter_by_input(float u=0.f) {
            _bi_quadratic.reset_by_input(u);
        }

        void reset_filter_by_output(float y=0.f) {
            _bi_quadratic.reset_by_output(y / _k);
        }

    private:
        float _fs {800.f};
        float _cutoff {18.9954f};
        float _bandwidth {15.0847f};
        float _depth {-17.1061f};

        float _k{1.f};
        BiQuadratic _bi_quadratic;
    };
}

#endif //DSP_NOTCH_FILTER_HPP
