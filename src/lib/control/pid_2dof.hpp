//
// Created by Cain on 2023/1/30.
//

#ifndef CONTROL_PID_2DOF_HPP
#define CONTROL_PID_2DOF_HPP

#include <lib/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/dsp/second_order_system.hpp>

#define PID_2DOF_ADAPTIVE_INTEGRAL
#define PID_2DOF_LPF_FILTER

#define PX4_ISFINITE std::isfinite


namespace control {
    class PID2Dof {
    public:
        /*!
         * 传递函数: kp * (b * r - y) + ki / s * (r - y) + kd * s / (1 / (2*pi*d_cutoff) + 1) * (c * r - y)
         * @param fs - 传感器采样频率(Hz)
         * @param kp - 比例系数
         * @param ki - 积分系数
         * @param kd - 微分系数
         * @param d_cutoff - 微分相关低通滤波的截止频率(Hz)
         * @param lpf_cutoff - 量测量的二阶低通滤波的截止频率(Hz)
         * @param lpf_damp - 量测量的二阶低通滤波的阻尼比
         * @param b - 比例的前馈系数
         * @param c - 微分的前馈系数
         * @param u_min - 最小输出控制量
         * @param u_max - 最大输出控制量
         * @param prewarp - 双线性变换的预畸变频率(Hz)
         */
        PID2Dof(float fs, float kp, float ki, float kd, float d_cutoff,
#ifdef PID_2DOF_LPF_FILTER
                float lpf_cutoff, float lpf_damp,
#endif
                float b = 1.f, float c = 0.f, float u_min=-1.f, float u_max=1.f, float prewarp=NAN)
                : _fs(fs), _kp(kp), _ki(ki), _kd(kd), _d_cutoff(d_cutoff),
#ifdef PID_2DOF_LPF_FILTER
                  _sos(fs, lpf_cutoff, lpf_damp, prewarp),
#endif
                  _b(b), _c(c), _u_min(u_min), _u_max(u_max), _prewarp(prewarp) {
            assert(fs > 0.f);

            if (PX4_ISFINITE(prewarp) && prewarp >= 0.f) {
                assert(prewarp < 0.5f * fs);
            } else {
                _prewarp = NAN;
            }

            set_d_cutoff_and_prewarp(d_cutoff, prewarp);
            set_i_prewarp(prewarp);
        }

        float operator()(float ref, float meas) {
#ifdef PID_2DOF_LPF_FILTER
            meas = _sos(meas);
#endif
            float u_p = _b * ref - meas, u_i = ref - meas, u_d = _c * ref - meas;

            // P
            _p_term = _kp * u_p;

            // I
            if (_u_raw > _u_max) {
                u_i = math::min(u_i, 0.f);
            }
            if (_u_raw < _u_min) {
                u_i = math::max(u_i, 0.f);
            }
            _i_term = _i_state;
#ifdef PID_2DOF_ADAPTIVE_INTEGRAL
            float i_factor = u_i / INTEGRAL_INPUT_MAX;
            i_factor = math::max(0.f, 1.f - i_factor * i_factor);
            _i_state = i_factor * _ki * u_i + _i_state;
#else
            _i_state = _ki * u_i + _i_state;
#endif
            _i_term += _i_state;

            // D
            _d_term = _d_state;
            _d_state = (_kd * u_d + (1.f - _d_rho) * _d_state) / (1.f + _d_rho);
            _d_term = _d_state - _d_term;

            _u_raw = _p_term + _i_term + _d_term;
            _u_clip = math::constrain(_u_raw, _u_min, _u_max);

            return _u_clip;
        }

        float get_kp() const { return _kp; }
        float get_ki() const { return _ki / _i_scale; }
        float get_kd() const { return _kd / _d_scale; }
        float get_b() const { return _b; }
        float get_c() const { return _c; }
        float get_u_min() const { return _u_min; }
        float get_u_max() const { return _u_max; }
        float get_u_clip() const { return _u_clip; }
        float get_u_raw() const { return _u_raw; }
        float get_p_term() const { return _p_term; }
        float get_i_term() const { return _i_term; }
        float get_d_term() const { return _d_term; }
        float get_i_state() const { return _i_state; }
        float get_d_state() const { return _d_state; }
        float get_i_scale() const { return _i_scale; }
        float get_d_scale() const { return _d_scale; }
        float get_prewarp() const { return _prewarp; }
        float get_i_prewarp() const { return _i_prewarp; }
        float get_d_prewarp() const { return _d_prewarp; }
#ifdef PID_2DOF_LPF_FILTER
        float get_lpf_prewarp() const { return _sos.ger_prewarp(); }
        digital_signal_processing::SecondOrderSystem &get_sos() { return _sos; };
#endif

        void set_u_min(float u_min) { _u_min = u_min; }
        void set_u_max(float u_max) { _u_max = u_max; }
        void set_kp(float kp) { _kp = kp; }
        void set_ki(float ki) { _ki = ki * _i_scale; }
        void set_kd(float kd) { _kd = kd * _d_scale; }
        void set_b(float b) { _b = b; }
        void set_c(float c) { _c = c; }

        void set_d_cutoff_and_prewarp(float cutoff, float prewarp=NAN) {
            assert(cutoff < 0.5f * _fs);
            _d_cutoff = cutoff;

            if (!PX4_ISFINITE(prewarp) || prewarp < 0.f) {
                if (PX4_ISFINITE(_prewarp) && _prewarp >= 0.f) {
                    prewarp = _prewarp;
                }
            }

            if (PX4_ISFINITE(prewarp) && prewarp >= 0.f) {
                assert(prewarp < 0.5f * _fs);
                _d_prewarp = prewarp;
            } else {
                _d_prewarp = _d_cutoff;
            }

            _kd /= _d_scale;

            if (_d_prewarp < EPS) {
                // 使用泰勒展开近似: tan(x) = x + x^3/3 +2*x^5/15 +17*x^7/315 +62*x^9/2835 + O(x^11)
                float tmp0 = M_PI_F / _fs;  // 1
                float tmp1 = tmp0 * tmp0;   // 2
                float tmp2 = tmp0 * tmp1;   // 3
                float tmp3 = tmp2 * tmp1;   // 5
                float tmp4 = tmp3 * tmp1;   // 7
                float tmp5 = tmp4 * tmp1;   // 9
                float tmp6 = _d_prewarp * _d_prewarp;  // 2
                float tmp7 = tmp6 * tmp6;              // 4
                float tmp8 = tmp7 * tmp6;              // 6
                float tmp9 = tmp8 * tmp6;              // 8
                _d_rho = _d_cutoff * (tmp0 + 1.f / 3.f * tmp2 * tmp6 + 2.f / 15.f * tmp3 * tmp7 + 17.f / 315.f * tmp4 * tmp8 + 62.f / 2835.f * tmp5 * tmp9);
            } else {
                _d_rho = _d_cutoff / _d_prewarp * tanf(M_PI_F * _d_prewarp / _fs);
            }

            _d_scale = 2.f * M_PI_F * _d_cutoff;
            _kd *= _d_scale;
        }

        void set_i_prewarp(float prewarp) {
            if (!PX4_ISFINITE(prewarp) || prewarp < 0.f) {
                if (PX4_ISFINITE(_prewarp) && _prewarp >= 0.f) {
                    prewarp = _prewarp;
                }
            }

            if (PX4_ISFINITE(prewarp) && prewarp >= 0.f) {
                assert(prewarp < 0.5f * _fs);
                _i_prewarp = prewarp;
            } else {
                _i_prewarp = 0.f;
            }

            _ki /= _i_scale;

            if (_i_prewarp < EPS) {
                // 使用泰勒展开近似: tan(x) = x + x^3/3 +2*x^5/15 +17*x^7/315 +62*x^9/2835 + O(x^11)
                float tmp0 = M_PI_F / _fs;  // 1
                float tmp1 = tmp0 * tmp0;   // 2
                float tmp2 = tmp0 * tmp1;   // 3
                float tmp3 = tmp2 * tmp1;   // 5
                float tmp4 = tmp3 * tmp1;   // 7
                float tmp5 = tmp4 * tmp1;   // 9
                float tmp6 = _i_prewarp * _i_prewarp;  // 2
                float tmp7 = tmp6 * tmp6;              // 4
                float tmp8 = tmp7 * tmp6;              // 6
                float tmp9 = tmp8 * tmp6;              // 8
                _i_scale = (tmp0 + 1.f / 3.f * tmp2 * tmp6 + 2.f / 15.f * tmp3 * tmp7 + 17.f / 315.f * tmp4 * tmp8 + 62.f / 2835.f * tmp5 * tmp9) / (2.f * M_PI_F);
            } else {
                _i_scale = tanf(M_PI_F * _i_prewarp / _fs) / (2.f * M_PI_F * _i_prewarp);
            }

            _ki *= _i_scale;
        }

#ifdef PID_2DOF_LPF_FILTER
        void set_lpf_cutoff_damp_and_prewarp(float cutoff, float damp, float prewarp=NAN) {
            if (!PX4_ISFINITE(prewarp) || prewarp < 0.f) {
                if (PX4_ISFINITE(_prewarp) && _prewarp >= 0.f) {
                    prewarp = _prewarp;
                }
            }

            _sos.set_parameters(_fs, cutoff, damp, prewarp);
        }
#endif

        void set_prewarp(float prewarp) {
            if (PX4_ISFINITE(prewarp) && prewarp >= 0.f) {
                assert(prewarp < 0.5f * _fs);
                _prewarp = prewarp;
            } else {
                _prewarp = NAN;
            }

            set_i_prewarp(_prewarp);
            set_d_cutoff_and_prewarp(_d_cutoff, _prewarp);
#ifdef PID_2DOF_LPF_FILTER
            set_lpf_cutoff_damp_and_prewarp(_sos.get_cutoff(), _sos.get_damp(), _prewarp);
#endif
        }

        void reset_integral() {
            _i_state = 0.f;
            _i_term = 0.f;
        }

        void reset_differential() {
            _d_state = 0.f;
            _d_term = 0.f;
        }

#ifdef PID_2DOF_LPF_FILTER
        void reset_lpf() {
            _sos.reset();
        }
#endif

        void reset() {
            _u_clip = 0.f;
            _u_raw = 0.f;
            _p_term = 0.f;
            _i_term = 0.f;
            _d_term = 0.f;
            _i_state = 0.f;
            _d_state = 0.f;
#ifdef PID_2DOF_LPF_FILTER
            _sos.reset();
#endif
        }

    protected:
        float _fs;
        float _kp, _ki, _kd;
        float _d_cutoff, _d_rho{1.f};
#ifdef PID_2DOF_LPF_FILTER
        digital_signal_processing::SecondOrderSystem _sos;
#endif
        float _b, _c;
        float _u_min, _u_max;
        float _u_clip{0.f}, _u_raw{0.f};
        float _i_scale{1.f}, _d_scale{1.f};
        float _p_term{0.f}, _i_term{0.f}, _d_term{0.f};
        float _i_state{0.f}, _d_state{0.f};
        float _prewarp{NAN}, _i_prewarp{0.f}, _d_prewarp{NAN};

        static constexpr float EPS {1e-2f};
#ifdef PID_2DOF_ADAPTIVE_INTEGRAL
        static constexpr float INTEGRAL_INPUT_MAX {2.f * M_PI_F};
#endif
    };
}

#endif //CONTROL_PID_2DOF_HPP
