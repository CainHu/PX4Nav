//
// Created by Cain on 2023/2/5.
//

#ifndef DSP_SECOND_ORDER_SYSTEM_HPP
#define DSP_SECOND_ORDER_SYSTEM_HPP

#include "biquadratic.hpp"

namespace digital_signal_processing {
    class SecondOrderSystem {
    public:
        /*!
         * 二阶震荡阻尼系统
         * 传递函数: H(s) = 1 / ((s/wn)^2 + 2*ζ*(s/wn) + 1)
         * @param fs - 采样频率 (Hz)(Hz)
         * @param cutoff - 截止频率: 2*pi*wn (Hz)
         * @param damping_ratio - ζ
         * @param prewarp - 畸变频率: fw (Hz)
         */
        SecondOrderSystem(float fs, float cutoff, float damping_ratio, float prewarp=NAN) {
            set_parameters(fs, cutoff, damping_ratio, prewarp);
        }

        float operator()(float u) {
            return _bi_quadratic(u);
        }

        void set_parameters(float fs, float cutoff, float damping_ratio, float prewarp=NAN) {
            assert(fs > 0.f);
            assert(cutoff > 0.f);
            assert(damping_ratio >=0.f);

            _fs = fs;
            _cutoff = cutoff;
            _damp = damping_ratio;

            update_parameters(prewarp);
        }

        float get_fs() const { return _fs; }
        float get_cutoff() const { return _cutoff; }
        float get_wn() const { return 2.f * M_PI_F * _cutoff; }
        float get_damp() const { return _damp; }
        float get_rho() const { return _rho; }
        float ger_prewarp() const { return _prewarp; }

        void reset(float state_0=0.f, float state_1=0.f) {
            _bi_quadratic.reset(state_0, state_1);
        }

        bool reset_by_input(float u=0.f) {
            return _bi_quadratic.reset_by_input(u);
        }

        bool reset_by_output(float y=0.f) {
            return _bi_quadratic.reset_by_output(y);
        }

    private:
        float _fs {800.f};      ///< 采样频率 (Hz)
        float _cutoff {40.f};   ///< 截止频率: 2*pi*wn (Hz)
        float _damp {0.7f};     ///< 阻尼比: ζ
        float _rho {1.f};       ///< fc/fw * tan(pi*fw)
        float _prewarp {1.f};   ///< 畸变频率 :fw (Hz)

        BiQuadratic _bi_quadratic;

        static constexpr float EPS {1e-2f};

        void update_parameters(float prewarp) {
            if (!PX4_ISFINITE(prewarp) || prewarp < 0.f) {
                prewarp = _cutoff;
            }

            assert(prewarp < 0.5f * _fs);
            _prewarp = prewarp;

            if (prewarp < EPS) {
                // 使用泰勒展开近似: tan(x) = x + x^3/3 +2*x^5/15 +17*x^7/315 +62*x^9/2835 + O(x^11)
                float tmp0 = M_PI_F / _fs;  // 1
                float tmp1 = tmp0 * tmp0;   // 2
                float tmp2 = tmp0 * tmp1;   // 3
                float tmp3 = tmp2 * tmp1;   // 5
                float tmp4 = tmp3 * tmp1;   // 7
                float tmp5 = tmp4 * tmp1;   // 9
                float tmp6 = _prewarp * _prewarp;  // 2
                float tmp7 = tmp6 * tmp6;          // 4
                float tmp8 = tmp7 * tmp6;          // 6
                float tmp9 = tmp8 * tmp6;          // 8
                _rho = _cutoff * (tmp0 + 1.f/3.f * tmp2 * tmp6 + 2.f/15.f * tmp3 * tmp7 + 17.f/315.f * tmp4 * tmp8 + 62.f/2835.f * tmp5 * tmp9);
            } else {
                _rho = _cutoff / _prewarp * tanf(M_PI_F * _prewarp / _fs);
            }

            float tmp = 2.f * _damp * _rho;
            float rho2 = _rho * _rho;
            float a0 = 1.f + tmp + rho2;
            float a1 = 2.f * (rho2 - 1.f) / a0;
            float a2 = (1.f - tmp + rho2) / a0;
            float b0 = rho2 / a0;
            float b1 = 2.f * b0;
            float b2 = b0;

            _bi_quadratic.set_parameters(a1, a2, b0, b1, b2);
        }
    };
}

#endif //DSP_SECOND_ORDER_SYSTEM_HPP
