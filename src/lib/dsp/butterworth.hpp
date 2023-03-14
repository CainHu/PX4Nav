//
// Created by Cain on 2023/1/13.
//

#ifndef DSP_BUTTERWORTH_HPP
#define DSP_BUTTERWORTH_HPP

#include "biquadratic.hpp"
#include "biprimary.hpp"
#include <cfloat>

namespace digital_signal_processing {
    template<uint8_t NP, uint8_t NZ=0>
    class Butterworth {
    public:
        /*!
         * 计算二阶滤波器的参数, 初始化滤波器系数, 初始化滤波器状态
         * @param fs - 采样频率 (Hz)
         * @param cutoff_p - 极点的截止频率 (Hz)
         * @param cutoff_z - 零点的截止频率 (Hz)
         */
        Butterworth(float fs, float cutoff_p, float cutoff_z=NAN) {
            assert(NP > 0);
            assert(NP >= NZ);   // 必须为因果系统

            if (NP_FOS) {
                _bi_primary = new BiPrimary;
            }
            if (NP_SOS) {
                _bi_quadratic = new BiQuadratic [NP_SOS];
                _a1 = new float [NP_SOS];
            }
            if (NZ_SOS) {
                _b1 = new float [NZ_SOS];
            }

            // SOS极点参数: s^2 + a1*s + 1
            for (uint8_t i = 0; i < NP_SOS; ++i) {
                float theta = float(2 * i + 1) * M_PI_F / float(2 * NP);
                _a1[i] = 2.f * sinf(theta);
            }

            // SOS零点参数: s^2 + b1*s + 1
            for (uint8_t i = 0; i < NZ_SOS; ++i) {
                float theta = float(2 * i + 1) * M_PI_F / float(2 * NZ);
                _b1[i] = 2.f * sinf(theta);
            }

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

            bool cutoff_p_finite = PX4_ISFINITE(cutoff_p);
            if (cutoff_p_finite) {
                assert(cutoff_p > 0.f); // 频率必须大于0
                assert(cutoff_p < 0.5f * _fs);  // 必须小于采样率的一半
            }
            _cutoff_p = cutoff_p;
            _cutoff_p_finite = cutoff_p_finite;

            bool cutoff_z_finite = PX4_ISFINITE(cutoff_z);
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
            bool cutoff_p_finite = PX4_ISFINITE(cutoff_p);
            if (cutoff_p_finite) {
                assert(cutoff_p > 0.f); // 频率必须大于0
                assert(cutoff_p < 0.5f * _fs);  // 必须小于采样率的一半
            }
            _cutoff_p = cutoff_p;
            _cutoff_p_finite = cutoff_p_finite;

            bool cutoff_z_finite = PX4_ISFINITE(cutoff_z);
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

        ~Butterworth() {
            delete _bi_primary;
            delete [] _bi_quadratic;
            delete [] _a1;
            delete [] _b1;
        }

        static constexpr uint8_t NP_SOS {NP / 2};   ///< 二阶滤波器极点对数 = 二阶滤波器个数
        static constexpr uint8_t NP_FOS {NP % 2};   ///< 一阶滤波器极点数 = 一阶滤波器个数
        static constexpr uint8_t NZ_SOS {NZ / 2};   ///< 二阶滤波器零点对数
        static constexpr uint8_t NZ_FOS {NZ % 2};   ///< 一阶滤波器零点对数

    private:
        float _k {1.f};     ///< 滤波器增益
        BiPrimary *_bi_primary {nullptr};   ///< 二阶滤波器: ((s/w_z)^2 + b1*(s/w_z) + 1) / ((s/w_p)^2 + a1*(s/w_p) + 1)
        BiQuadratic *_bi_quadratic {nullptr};   ///< 一阶滤波器 ((s/w_z) + 1) / ((s/w_p) + 1)

        float *_a1 {nullptr};     ///< 二阶滤波器极点参数: s^2 + a1*s + 1
        float *_b1 {nullptr};     ///< 二阶滤波器零点参数: s^2 + b1*s + 1

        float _fs {800.f};          ///< 采样频率
        float _cutoff_p {53.f};     ///< -3dB截止频率
        float _cutoff_z {63.f};     ///< -3dB截止频率
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
                    float tmp = _a1[i] * rho_p;
                    float a0 = 1.f + tmp + rho_p2;
                    float a1 = 2.f * (rho_p2 - 1.f) / a0;
                    float a2 = (1.f - tmp + rho_p2) / a0;
                    _k *= rho_p2 / a0;
                    _bi_quadratic[i].set_parameters(a1, a2, 1.f, 2.f, 1.f);
                }

                // FOS极点
                if (NP_FOS) {
                    float a0 = rho_p + 1.f;
                    float a1 = (rho_p - 1.f) / a0;
                    _k *= rho_p / a0;
                    _bi_primary->set_parameters(a1, 1.f, 1.f);
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
                    float tmp = _b1[i] * rho_z;
                    float a0 = 1.f + tmp + rho_z2;
                    float a1 = 2.f * (rho_z2 - 1.f);
                    float a2 = (1.f - tmp + rho_z2);
                    _k /= rho_z2;
                    _bi_quadratic[i].set_parameters_zero(a0, a1, a2);
                }

                // FOS零点
                if (NZ_FOS) {
                    _k /= rho_z;

                    // 不一定有一阶滤波器, 因为滤波器种类和个数仅由极点数确定, 与零点数无关
                    // 如果没有一阶滤波器, 需要把零点放在二阶滤波器上
                    if (NP_FOS) {
                        float a0 = rho_z + 1.f;
                        float a1 = rho_z - 1.f;
                        _bi_primary->set_parameters_zero(a0, a1);
                    } else {
                        assert(NZ_SOS + 1 > NP_SOS);
                        float a0 = rho_z + 1.f;
                        float a1 = 2.f * rho_z;
                        float a2 = rho_z - 1.f;
                        _bi_quadratic[NZ_SOS].set_parameters_zero(a0, a1, a2);
                    }
                }
            }
        };
    };
}



#endif //DSP_BUTTERWORTH_HPP
