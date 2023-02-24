//
// Created by Cain on 2023/1/15.
//

#ifndef DSP_CHEBYSHEV1_HPP
#define DSP_CHEBYSHEV1_HPP

#include "biquadratic.hpp"
#include "biprimary.hpp"
#include <cfloat>

namespace digital_signal_processing {
    template<uint8_t NP, uint8_t NZ=0>
    class ChebyshevI {
    public:
        /*!
         * 计算二阶滤波器的参数, 初始化滤波器系数, 初始化滤波器状态
         * @param fs - 采样频率 (Hz)
         * @param cutoff_p - 极点通带截止频率, 对应幅值为1/(1+eps^2) (Hz)
         * @param ripple_p - 极点通带纹波, 同时也是通带边缘值 (dB)
         * @param cutoff_z - 零点通带截止频率, 对应幅值为1/(1+eps^2) (Hz)
         * @param ripple_z - 零点通带纹波, 同时也是通带边缘值 (dB)
         */
        ChebyshevI(float fs, float cutoff_p, float ripple_p=0.5, float cutoff_z=NAN, float ripple_z=0.5) {
            assert(NP > 0);
            assert(NP >= NZ);   // 必须为因果系统

            if (NP_FOS) {
                _bi_primary = new BiPrimary;
            }
            if (NP_SOS) {
                _bi_quadratic = new BiQuadratic [NP_SOS];
                _a1 = new float [NP_SOS];
                _a2 = new float [NP_SOS];
            }
            if (NZ_SOS) {
                _b1 = new float [NZ_SOS];
                _b2 = new float [NZ_SOS];
            }

            // SOS极点参数: s^2 + a1*s + a2
            _eps_p = sqrtf(powf(10.f, ripple_p / 10.f) - 1.f);
            float beta = powf((sqrtf(1.f + _eps_p * _eps_p) + 1.f) / _eps_p, 1.f / NP);
            float r1 = (beta * beta + 1.f) / (2.f * beta);
            float r2 = (beta * beta - 1.f) / (2.f * beta);
            float theta, x, y;
            for (uint8_t i = 0; i < NP_SOS; ++i) {
                theta = float(2 * i + 1) * M_PI_F / float(2 * NP);
                x = -r2 * sinf(theta);
                y = r1 * cosf(theta);
                _a1[i] = -2.f * x;
                _a2[i] = x * x + y * y;
            }

            // FOS极点参数: s + a
            _a_fos = r2;

            // SOS零点参数: s^2 + b1*s + b2
            _eps_z = sqrtf(powf(10.f, ripple_z / 10.f) - 1.f);
            beta = powf((sqrtf(1.f + _eps_z * _eps_z) + 1.f) / _eps_z, 1.f / NP);
            r1 = (beta * beta + 1.f) / (2.f * beta);
            r2 = (beta * beta - 1.f) / (2.f * beta);
            for (uint8_t i = 0; i < NZ_SOS; ++i) {
                x = -r2 * sinf(theta);
                y = r1 * cosf(theta);
                _b1[i] = -2.f * x;
                _b2[i] = x * x + y * y;
            }

            // FOS零点参数: s + b
            _b_fos = r2;

            set_fs_and_cutoff(fs, cutoff_p, cutoff_z);
        }

        /*!
         * 调用滤波器
         * @param u - 滤波器输入
         * @return - 滤波器输出
         */
        float operator()(float u) {
            // SOS
            for (uint8_t i = 0; i < NP_SOS; ++i) {
                u = _bi_quadratic[i](u);
            }

            // FOS
            if (NP_FOS) {
                u = (*_bi_primary)(u);
            }

            u *= _k;

            return u;
        }

        /*!
         * 设置采样频率与截止频率
         * @param fs - 设置采样
         * @param cutoff_p - 极点的截止频率
         * @param cutoff_z - 零点的截止频率
         */
        void set_fs_and_cutoff(float fs, float cutoff_p, float cutoff_z=NAN) {
            assert(fs > 0.f);   // 频率必须大于0
            _fs = fs;

            bool cutoff_p_finite = std::isfinite(cutoff_p);
            if (cutoff_p_finite) {
                assert(cutoff_p > 0.f); // 频率必须大于0
                assert(cutoff_p < 0.5f * _fs);  // 必须小于采样率的一半
            }
            _cutoff_p = cutoff_p;
            _cutoff_p_finite = cutoff_p_finite;

            bool cutoff_z_finite = std::isfinite(cutoff_z);
            if (cutoff_z_finite) {
                assert(cutoff_z > 0.f); // 频率必须大于0
                assert(cutoff_z < 0.5f * _fs);  // 必须小于采样率的一半
            }
            _cutoff_z = cutoff_z;
            _cutoff_z_finite = cutoff_z_finite;

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
         * @param cutoff_p - 极点的截止频率
         * @param cutoff_z - 零点的截止频率
         */
        void set_cutoff(float cutoff_p, float cutoff_z=NAN) {
            bool cutoff_p_finite = std::isfinite(cutoff_p);
            if (cutoff_p_finite) {
                assert(cutoff_p > 0.f); // 频率必须大于0
                assert(cutoff_p < 0.5f * _fs);  // 必须小于采样率的一半
            }
            _cutoff_p = cutoff_p;
            _cutoff_p_finite = cutoff_p_finite;

            bool cutoff_z_finite = std::isfinite(cutoff_z);
            if (cutoff_z_finite) {
                assert(cutoff_z > 0.f); // 频率必须大于0
                assert(cutoff_z < 0.5f * _fs);  // 必须小于采样率的一半
            }
            _cutoff_z = cutoff_z;
            _cutoff_z_finite = cutoff_z_finite;

            update_filter_parameters();
        }

        /*!
         * 重置滤波器的状态
         */
        void reset_filter_state() {
            // SOS
            for (uint8_t i = 0; i < NP_SOS; ++i) {
                _bi_quadratic[i].reset();
            }

            // FOS
            if (NP_FOS) {
                _bi_primary->reset();
            }
        }

        void reset_filter_state_by_input(float u=0.f) {
            // SOS
            for (uint8_t i = 0; i < NP_SOS; ++i) {
                if (_bi_quadratic[i].reset_by_input(u)) {
                    u *= (_bi_quadratic[i].b0() + _bi_quadratic[i].b1() + _bi_quadratic[i].b2()) / (1.f + _bi_quadratic[i].a1() + _bi_quadratic[i].a2());
                } else {
                    u = 0.f;
                }
            }

            // FOS
            if (NP_FOS) {
                _bi_primary->reset_by_input(u);
            }
        }

        void reset_filter_state_by_output(float y=0.f) {
            y = y / _k;
            if (!PX4_ISFINITE(y)) {
                y = 0.f;
            }

            // SOS
            for (uint8_t i = 0; i < NP_SOS; ++i) {
                if (_bi_quadratic[i].reset_by_output(y)) {
                    y *= (1.f + _bi_quadratic[i].a1() + _bi_quadratic[i].a2()) / (_bi_quadratic[i].b0() + _bi_quadratic[i].b1() + _bi_quadratic[i].b2());
                } else {
                    y = 0.f;
                }
            }

            // FOS
            if (NP_FOS) {
                _bi_primary->reset_by_output(y);
            }
        }

        ~ChebyshevI() {
            delete _bi_primary;
            delete [] _bi_quadratic;
            delete [] _a1;
            delete [] _a2;
            delete [] _b1;
            delete [] _b2;
        }

        static constexpr uint8_t NP_SOS {NP / 2};   ///< 二阶滤波器极点对数 = 二阶滤波器个数
        static constexpr uint8_t NP_FOS {NP % 2};   ///< 一阶滤波器极点数 = 一阶滤波器个数
        static constexpr uint8_t NZ_SOS {NZ / 2};   ///< 二阶滤波器零点对数
        static constexpr uint8_t NZ_FOS {NZ % 2};   ///< 一阶滤波器零点对数

    private:
        float _k {1.f};     ///< 滤波器增益
        BiPrimary *_bi_primary {nullptr};   ///< 二阶滤波器: ((s/w_z)^2 + b1*(s/w_z) + b2) / ((s/w_p)^2 + a1*(s/w_p) + a2)
        BiQuadratic *_bi_quadratic {nullptr};   ///< 一阶滤波器 ((s/w_z) + b) / ((s/w_p) + a)

        float *_a1 {nullptr}, *_a2 {nullptr};     ///< 二阶滤波器极点参数: s^2 + a1*s + a2
        float *_b1 {nullptr}, *_b2 {nullptr};     ///< 二阶滤波器零点参数: s^2 + b1*s + b2
        float _a_fos {1.f};     ///< 一阶滤波器极点参数: s + a
        float _b_fos {1.f};     ///< 一阶滤波器零点参数: s + b

        float _fs {800.f};          ///< 采样频率
        float _eps_p {0.1};          /// 极点通带纹波所对应的eps: |H(Ωp)|^2 = 1 / (1 + eps^2)
        float _eps_z {0.1};          /// 零点通带纹波所对应的eps: |H(Ωp)|^2 = 1 / (1 + eps^2)
        float _cutoff_p {53.f};     ///< 通带截止频率Ωp: |H(Ωp)|^2 = 1 / (1 + eps^2)
        float _cutoff_z {63.f};     ///< 通带截止频率Ωp: |H(Ωp)|^2 = 1 / (1 + eps^2)
        bool _cutoff_p_finite {true};
        bool _cutoff_z_finite {true};

        /*!
         * 根据_fs, _cutoff, _cutoff_z, 重新设置滤波器的系数
         */
        void update_filter_parameters() {
            // 计算 tan(π*fd / fs) -> rho
            float rho_p , rho_p2;
            if (_cutoff_p_finite && (NP_FOS || NP_SOS)) {
                rho_p = tanf(M_PI_F * _cutoff_p / _fs);
                rho_p2 = rho_p * rho_p;
            } else {
                rho_p = 1.f;
                rho_p2 = 1.f;
            }

            float rho_z, rho_z2;
            if (_cutoff_z_finite && (NZ_FOS || NZ_SOS)) {
                rho_z = tanf(M_PI_F * _cutoff_z / _fs);
                rho_z2 = rho_z * rho_z;
            } else {
                rho_z = 1.f;
                rho_z2 = 1.f;
            }

            // 计算二阶滤波器与一阶滤波器的参数
            _k = 1.f;

            if (_cutoff_p_finite) {
                // SOS极点
                for (uint8_t i = 0; i < NP_SOS; ++i) {
                    float tmp0 = _a1[i] * rho_p;
                    float tmp1 = _a2[i] * rho_p2;
                    float a0 = 1.f + tmp0 + tmp1;
                    float a1 = 2.f * (tmp1 - 1.f) / a0;
                    float a2 = (1.f - tmp0 + tmp1) / a0;
                    _k *= tmp1 / a0;
                    _bi_quadratic[i].set_parameters(a1, a2, 1.f, 2.f, 1.f);
                }

                // FOS极点
                if (NP_FOS) {
                    float tmp = _a_fos * rho_p;
                    float a0 = tmp + 1.f;
                    float a1 = (tmp - 1.f) / a0;
                    _k *= tmp / a0;
                    _bi_primary->set_parameters(a1, 1.f, 1.f);
                } else {
                    // NP为偶数时, 需要把dcgain设置为1/(1_eps^2)
                    _k /= sqrtf(1.f + _eps_p * _eps_p);
                }
            } else {
                for (uint8_t i = 0; i < NP_SOS; ++i) {
                    _bi_quadratic[i].set_parameters(2.f, 1.f, 1.f, 2.f, 1.f);
                }
                if (NP_FOS) {
                    _bi_primary->set_parameters(1.f, 1.f, 1.f);
                }
            }

            if (_cutoff_z_finite) {
                // SOS零点
                for (uint8_t i = 0; i < NZ_SOS; ++i) {
                    float tmp0 = _b1[i] * rho_z;
                    float tmp1 = _b2[i] * rho_z2;
                    float a0 = 1.f + tmp0 + tmp1;
                    float a1 = 2.f * (tmp1 - 1.f);
                    float a2 = (1.f - tmp0 + tmp1);
                    _k /= tmp1;
                    _bi_quadratic[i].set_parameters_zero(a0, a1, a2);
                }

                // FOS零点
                if (NZ_FOS) {
                    // 不一定有一阶滤波器, 因为滤波器种类和个数仅由极点数确定, 与零点数无关
                    // 如果没有一阶滤波器, 需要把零点放在二阶滤波器上
                    if (NP_FOS) {
                        float tmp = _b_fos * rho_z;
                        float a0 = tmp + 1.f;
                        float a1 = tmp - 1.f;
                        _k /= tmp;
                        _bi_primary->set_parameters_zero(a0, a1);
                    } else {
                        assert(NZ_SOS + 1 > NP_SOS);
                        float tmp = _b_fos * rho_z;
                        float a0 = tmp + 1.f;
                        float a1 = 2.f * tmp;
                        float a2 = tmp - 1.f;
                        _k /= tmp;
                        _bi_quadratic[NZ_SOS].set_parameters_zero(a0, a1, a2);
                    }
                } else {
                    // NZ为正偶数时, 需要把dcgain设置为1/(1_eps^2)
                    if (NZ_SOS) {
                        _k *= sqrtf(1.f + _eps_z * _eps_z);
                    }
                }
            }
        };
    };
}

#endif //DSP_CHEBYSHEV1_HPP
