//
// Created by Cain on 2023/1/16.
//

#ifndef DSP_INERTIAL_HPP
#define DSP_INERTIAL_HPP

#include "biprimary.hpp"
#include <cfloat>

namespace digital_signal_processing {
    template<uint8_t NP, uint8_t NZ=0>
    class Inertial {
    public:
        /*!
         * 计算二阶滤波器的参数, 初始化滤波器系数, 初始化滤波器状态
         * @param fs - 采样频率 (Hz)
         * @param cutoff_p - 极点时间常数对应的截止频率 (dB)
         * @param cutoff_z - 零点时间常数对应的截止频率 (dB)
         */
        Inertial(float fs, float cutoff_p, float cutoff_z=NAN) {
            assert(NP > 0);
            assert(NP >= NZ);   // 必须为因果系统

            _bi_primary = new BiPrimary [NP];

            set_fs_and_cutoff(fs, cutoff_p, cutoff_z);
        }

        /*!
         * 调用滤波器
         * @param u - 滤波器输入
         * @return - 滤波器输出
         */
        float operator()(float u) {
            // FOS
            for (uint8_t i = 0; i < NP; ++i) {
                u = _bi_primary[i](u);
            }

            u *= _k;

            return u;
        }

        /*!
         * 设置采样频率与截止频率
         * @param fs - 设置采样
         * @param cutoff_p - 极点时间常数对应的截止频率 (dB)
         * @param cutoff_z - 零点时间常数对应的截止频率 (dB)
         */
        void set_fs_and_cutoff(float fs, float cutoff_p, float cutoff_z) {
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
         * @param cutoff_p - 极点时间常数对应的截止频率 (dB)
         * @param cutoff_z - 零点时间常数对应的截止频率 (dB)
         */
        void set_cutoff(float cutoff_p, float cutoff_z) {
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
            // FOS
            for (uint8_t i = 0; i < NP; ++i) {
                _bi_primary[i].reset();
            }
        }

        void reset_filter_state_by_input(float u=0.f) {
            // FOS
            for (uint8_t i = 0; i < NP; ++i) {
                if (_bi_primary[i].reset_by_input(u)) {
                    u *= (_bi_primary[i].b0() + _bi_primary[i].b1()) / (1.f + _bi_primary[i].a1());
                } else {
                    u = 0.f;
                }
            }
        }

        void reset_filter_state_by_output(float y=0.f) {
            y = y / _k;
            if (!PX4_ISFINITE(y)) {
                y = 0.f;
            }

            // FOS
            for (uint8_t i = 0; i < NP; ++i) {
                if (_bi_primary[i].reset_by_output(y)) {
                    y *= (1.f + _bi_primary[i].a1()) / (_bi_primary[i].b0() + _bi_primary[i].b1());
                } else {
                    y = 0.f;
                }
            }
        }

        ~Inertial() {
            delete [] _bi_primary;
        }

    private:
        float _k {1.f};     ///< 滤波器增益
        BiPrimary *_bi_primary {nullptr};   ///< 一阶滤波器 ((s/w_z) + 1) / ((s/w_p) + 1)

        float _fs {800.f};          ///< 采样频率
        float _cutoff_p {53.f};     ///< 极点时间常数对应的截止频率
        float _cutoff_z {63.f};     ///< 零点时间常数对应的截止频率
        bool _cutoff_p_finite {true};
        bool _cutoff_z_finite {true};

        /*!
         * 根据_fs, _cutoff, _cutoff_z, 重新设置滤波器的系数
         */
        void update_filter_parameters() {
            // 计算 tan(π*fd / fs) -> rho
            float rho_p;
            if (_cutoff_p_finite && NP) {
                rho_p = tanf(M_PI_F * _cutoff_p / _fs);
            } else {
                rho_p = 1.f;
            }

            float rho_z;
            if (_cutoff_z_finite && NZ) {
                rho_z = tanf(M_PI_F * _cutoff_z / _fs);
            } else {
                rho_z = 1.f;
            }

            // 计算二阶滤波器与一阶滤波器的参数
            _k = 1.f;

            if (_cutoff_p_finite) {
                // FOS极点
                for (uint8_t i = 0; i < NP; ++i) {
                    float a0 = rho_p + 1.f;
                    float a1 = (rho_p - 1.f) / a0;
                    _k *= rho_p / a0;
                    _bi_primary[i].set_parameters(a1, 1.f, 1.f);
                }
            } else {
                for (uint8_t i = 0; i < NP; ++i) {
                    _bi_primary[i].set_parameters(1.f, 1.f, 1.f);
                }
            }

            if (_cutoff_z_finite) {
                // FOS零点
                for (uint8_t i = 0; i < NZ; ++i) {
                    float a0 = rho_z + 1.f;
                    float a1 = (rho_z - 1.f) / a0;
                    _k *= a0 / rho_z;
                    _bi_primary[i].set_parameters_zero(1.f, a1);
                }
            }
        };
    };
}

#endif //DSP_INERTIAL_HPP
