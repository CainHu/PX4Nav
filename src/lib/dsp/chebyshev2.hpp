//
// Created by Cain on 2023/1/15.
//

#ifndef DSP_CHEBYSHEV2_HPP
#define DSP_CHEBYSHEV2_HPP

#include "biquadratic.hpp"
#include "biprimary.hpp"
#include <cfloat>

namespace digital_signal_processing {
    template<uint8_t N>
    class ChebyshevII {
    public:
        /*!
         * 计算二阶滤波器的参数, 初始化滤波器系数, 初始化滤波器状态
         * @param fs - 采样频率 (Hz)
         * @param cutoff - 阻带截止频率, 对应幅值为1/(1+eps^2) (Hz)
         * @param a_stop - 阻带边缘值 (dB)
         */
        ChebyshevII(float fs, float cutoff, float a_stop=80.f) {
            assert(N > 0);

            if (N_FOS) {
                _bi_primary = new BiPrimary;
            }
            if (N_SOS) {
                _bi_quadratic = new BiQuadratic [N_SOS];
                _a1 = new float [N_SOS];
                _a2 = new float [N_SOS];
                _b0 = new float [N_SOS];
            }

            // SOS极点参数: s^2 + a1*s + a2, SOS零点参数: b0*s^2 + 0*s + 1,
            float delta = powf(10.f, -a_stop / 20.f);
            float beta = powf((1.f + sqrtf(1.f - delta * delta)) / delta, 1.f / N);
//            float eps = 1.f / sqrtf(powf(10.f, a_stop / 10.f) - 1.f);
//            float beta = powf((sqrtf(1.f + eps * eps) + 1.f) / eps, 1.f / N);
            float r1 = (beta * beta + 1.f) / (2.f * beta);
            float r2 = (beta * beta - 1.f) / (2.f * beta);
            float theta, x, y, norm2;
            for (uint8_t i = 0; i < N_SOS; ++i) {
                theta = float(2 * i + 1) * M_PI_F / float(2 * N);
                float cos_theta = cosf(theta);
                x = -r2 * sinf(theta);
                y = r1 * cos_theta;
                norm2 = x * x + y * y;
                _a1[i] = -2.f * x / norm2;
                _a2[i] = 1.f / norm2;
                _b0[i] = cos_theta * cos_theta;
            }

            // FOS极点参数: s + a
            _a_fos = 1.f / r2;

            set_fs_and_cutoff(fs, cutoff);
        }

        /*!
         * 调用滤波器
         * @param u - 滤波器输入
         * @return - 滤波器输出
         */
        float operator()(float u) {
            // SOS
            for (uint8_t i = 0; i < N_SOS; ++i) {
                u = _bi_quadratic[i](u);
            }

            // FOS
            if (N_FOS) {
                u = (*_bi_primary)(u);
            }

            u *= _k;

            return u;
        }

        /*!
         * 设置采样频率与截止频率
         * @param fs - 设置采样
         * @param cutoff - 阻带截止频率
         */
        void set_fs_and_cutoff(float fs, float cutoff) {
            assert(fs > 0.f);   // 频率必须大于0
            _fs = fs;

            bool cutoff_finite = std::isfinite(cutoff);
            if (cutoff_finite) {
                assert(cutoff > 0.f); // 频率必须大于0
                assert(cutoff < 0.5f * _fs);  // 必须小于采样率的一半
            }
            _cutoff = cutoff;
            _cutoff_finite = cutoff_finite;

            update_filter_parameters();
        }

        /*!
         * 设置采样频率
         * @param fs - 采样频率
         */
        void set_fs(float fs) {
            assert(fs > 0.f);   // 频率必须大于0
            _fs = fs;

            update_filter_parameters();
        }

        /*!
         * 设置截止频率
         * @param cutoff - 阻带截止频率
         */
        void set_cutoff(float cutoff) {
            bool cutoff_p_finite = std::isfinite(cutoff);
            if (cutoff_p_finite) {
                assert(cutoff > 0.f); // 频率必须大于0
                assert(cutoff < 0.5f * _fs);  // 必须小于采样率的一半
            }
            _cutoff = cutoff;
            _cutoff_finite = cutoff_p_finite;

            update_filter_parameters();
        }

        /*!
         * 重置滤波器的状态
         */
        void reset_filter_state() {
            // SOS
            for (uint8_t i = 0; i < N_SOS; ++i) {
                _bi_quadratic[i].reset();
            }

            // FOS
            if (N_FOS) {
                _bi_primary->reset();
            }
        }

        void reset_filter_state_by_input(float u=0.f) {
            // SOS
            for (uint8_t i = 0; i < N_SOS; ++i) {
                if (_bi_quadratic[i].reset_by_input(u)) {
                    u *= (_bi_quadratic[i].b0() + _bi_quadratic[i].b1() + _bi_quadratic[i].b2()) / (1.f + _bi_quadratic[i].a1() + _bi_quadratic[i].a2());
                } else {
                    u = 0.f;
                }
            }

            // FOS
            if (N_FOS) {
                _bi_primary->reset_by_input(u);
            }
        }

        void reset_filter_state_by_output(float y=0.f) {
            y = y / _k;
            if (!PX4_ISFINITE(y)) {
                y = 0.f;
            }

            // SOS
            for (uint8_t i = 0; i < N_SOS; ++i) {
                if (_bi_quadratic[i].reset_by_output(y)) {
                    y *= (1.f + _bi_quadratic[i].a1() + _bi_quadratic[i].a2()) / (_bi_quadratic[i].b0() + _bi_quadratic[i].b1() + _bi_quadratic[i].b2());
                } else {
                    y = 0.f;
                }
            }

            // FOS
            if (N_FOS) {
                _bi_primary->reset_by_output(y);
            }
        }

        ~ChebyshevII() {
            delete _bi_primary;
            delete [] _bi_quadratic;
            delete [] _a1;
            delete [] _a2;
            delete [] _b0;
        }

        static constexpr uint8_t N_SOS {N / 2};   ///< 二阶滤波器极点对数 = 二阶滤波器个数
        static constexpr uint8_t N_FOS {N % 2};   ///< 一阶滤波器极点数 = 一阶滤波器个数

    private:
        float _k {1.f};     ///< 滤波器增益
        BiPrimary *_bi_primary {nullptr};   ///< 二阶滤波器: (b0*(s/w_z)^2 + 0*(s/w_z) + 1) / ((s/w_p)^2 + a1*(s/w_p) + a2)
        BiQuadratic *_bi_quadratic {nullptr};   ///< 一阶滤波器 ((s/w_z) + 1) / ((s/w_p) + a)

        float *_a1 {nullptr}, *_a2 {nullptr};     ///< 二阶滤波器极点参数: s^2 + a1*s + a2
        float *_b0 {nullptr};     ///< 二阶滤波器零点参数: b0*s^2 + 0*s + 1
        float _a_fos {1.f};     ///< 一阶滤波器极点参数: s + a

        float _fs {800.f};          ///< 采样频率
        float _cutoff {63.f};     ///< 阻带截止频率Ωs: |H(Ωs)|^2 = δ^2
        bool _cutoff_finite {true};

        /*!
         * 根据_fs, _cutoff, 重新设置滤波器的系数
         */
        void update_filter_parameters() {
            // 计算 tan(π*fd / fs) -> rho
            float rho , rho2;
            if (_cutoff_finite && (N_FOS || N_SOS)) {
                rho = tanf(M_PI_F * _cutoff / _fs);
                rho2 = rho * rho;
            } else {
                rho = 1.f;
                rho2 = 1.f;
            }

            // 计算二阶滤波器与一阶滤波器的参数
            _k = 1.f;

            if (_cutoff_finite) {
                // SOS极点与零点
                for (uint8_t i = 0; i < N_SOS; ++i) {
                    float tmp0 = _a1[i] * rho;
                    float tmp1 = _a2[i] * rho2;
                    float a0 = 1.f + tmp0 + tmp1;
                    float a1 = 2.f * (tmp1 - 1.f) / a0;
                    float a2 = (1.f - tmp0 + tmp1) / a0;
                    float b0 = (rho2 + _b0[i]);
                    float b1 = 2.f * (rho2 - _b0[i]) / b0;
                    _k *= _a2[i] * b0 / a0;
                    _bi_quadratic[i].set_parameters(a1, a2, 1.f, b1, 1.f);

                }

                // FOS极点
                if (N_FOS) {
                    float tmp = _a_fos * rho;
                    float a0 = tmp + 1.f;
                    float a1 = (tmp - 1.f) / a0;
                    _k *= tmp / a0;
                    _bi_primary->set_parameters(a1, 1.f, 1.f);
                }
            } else {
                for (uint8_t i = 0; i < N_SOS; ++i) {
                    _bi_quadratic[i].set_parameters(2.f, 1.f, 1.f, 2.f, 1.f);
                }
                if (N_FOS) {
                    _bi_primary->set_parameters(1.f, 1.f, 1.f);
                }
            }
        };
    };
}

#endif //DSP_CHEBYSHEV2_HPP
