//
// Created by Cain on 2023/1/3.
//

#include "eskf.h"
#include <cfloat>
#include <cstring>

namespace eskf {
    void ESKF::initialize() {
        set_dt(1e-6f * float(_params.eskf_update_interval_us));
        reset_state();
        reset_error_state();
        reset_covariance_matrix(0, DIM);
        regular_covariance_to_symmetric<DIM>(0);
//        init_covariance();
        reset_accmulator();
    }

    void ESKF::init_covariance() {
        std::memset(_P, 0, sizeof(_P));

        float var_pos_horz = sq(_params.std_init_pos_horz);
        _P[0][0] = var_pos_horz;
        _P[1][1] = var_pos_horz;
        _P[2][2] = sq(_params.std_init_pos_vert);

        float var_vel_horz = sq(_params.std_init_vel_horz);
        _P[3][3] = var_vel_horz;
        _P[4][4] = var_vel_horz;
        _P[5][5] = _params.std_init_vel_vert;

        _P[6][6] = _params.std_init_ang;
        _P[7][7] = _params.std_init_ang;
        _P[8][8] = _params.std_init_ang;

        float var_delta_ang = sq(_params.std_init_ang * _dt);
        _P[9][9] = var_delta_ang;
        _P[10][10] = var_delta_ang;
        _P[11][11] = var_delta_ang;

        float var_delta_vel = sq(_params.std_init_acc_bias * _dt);
        _P[12][12] = var_delta_vel;
        _P[13][13] = var_delta_vel;
        _P[14][14] = var_delta_vel;

        _P[15][15] = sq(_params.std_init_grav);

        float var_mag_ang = sq(_params.std_init_mag_ang);
        _P[16][16] = sq(_params.std_init_mag_norm);
        _P[17][17] = var_mag_ang;
        _P[18][18] = var_mag_ang;

        float var_mag_bias = sq(_params.std_init_mag_bias);
        _P[19][19] = var_mag_bias;
        _P[20][20] = var_mag_bias;
        _P[21][21] = var_mag_bias;

        float var_wind = sq(_params.std_init_wind);
        _P[22][22] = var_wind;
        _P[23][23] = var_wind;
    }

    void ESKF::predict_state(const ImuSample &imu_sample) {
        /*
         * 计算eskf运行的平均时间间隔
         * 平均运行时间 = 采样时间的低通滤波值
         * 需要把采样时间限制在设定的采样时间的50%和200%之间, 以保证平均运行时间不会有很大的跳变
         * */
        float dt_init = float(_params.eskf_update_interval_us) * 1e-6f;
        float dt_eskf = 0.5f * (imu_sample.delta_ang_dt + imu_sample.delta_ang_dt);
        dt_eskf = math::constrain(dt_eskf, 0.5f * dt_init, 2.0f * dt_init);
        dt_eskf = 0.99f * _dt + 0.01f * dt_eskf;
        set_dt(dt_eskf);    // _dt, _dt2, _dt4

        // Δv_corr = Δv - b_Δv
        _delta_vel_corr = imu_sample.delta_vel - _state.delta_vel_bias;
        // a_corr = Δv_corr / Δt
        _acc_corr = _delta_vel_corr / imu_sample.delta_vel_dt;

        // Δv_nav = Rnb * Δv_corr
        _delta_vel_corr_nav = _Rnb * _delta_vel_corr;
        // a_nav = Δv_nav / Δt
        _acc_nav = _delta_vel_corr_nav / imu_sample.delta_vel_dt;

        // 上一时刻的速度
        const Vector3f v_last = _state.vel;

        // v' = v + Δv
        _state.vel += _delta_vel_corr_nav;
        _state.vel(2) += _state.grav * imu_sample.delta_vel_dt;

        // p' = p + 0.5 * (v' + v) * Δt
        _state.pos += 0.5f * (_state.vel + v_last) * imu_sample.delta_vel_dt;

        // Δθ_corr = Δθ - b_Δθ
        _delta_ang_corr = imu_sample.delta_ang - _state.delta_ang_bias;
        // ω_corr = Δθ_corr / Δt
        _gyro_corr = _delta_ang_corr / imu_sample.delta_ang_dt;

        // Δq = Exp((w - bg) * Δt)
        Quatf delta_q {};
        quaternion_from_axis_angle(delta_q, _delta_ang_corr);

        // q' = q * Δq
        const Quatf q = _state.quat_nominal;
        _state.quat_nominal = q * delta_q;
        _state.quat_nominal.normalize();
        _Rnb = _state.quat_nominal;

        // b_Δθ' = (1 - Δt/τ) * b_Δθ
        _state.delta_ang_bias *= math::constrain(1.f - _dt * _params.gyro_bias_tau_inv, 0.f, 1.f);

        // b_Δv' = (1 - Δt/τ) * b_Δv
        _state.delta_vel_bias *= math::constrain(1.f - _dt * _params.acc_bias_tau_inv, 0.f, 1.f);

        // b_m' = (1 - Δt/τ) * b_m
        _state.mag_bias *= math::constrain(1.f - _dt * _params.mag_bias_tau_inv, 0.f, 1.f);

        _state.wind *= math::constrain(1.f - _dt * _params.wind_tau_inv, 0.f, 1.f);

        // IMU距离地面的高度
        _imu_hgt = -_state.pos(2) - _terrain;
    }

    bool ESKF::posterior_estimate(const float (&HP)[DIM], const float &innov_var, const float &innov) {
        /*
        K = P * H' * (H * P * H' + R)^-1
        P = P - K * H * P

        e = y - h = y - (v + R * (w - bg)^ * dis)
        x = x + K * (y - h)
        */

        // Don't correct error state unless (I - KH) * P >= 0 <=> P - PH'(HPH'+R)^-1HP >= 0
        for (uint8_t i = 0; i < DIM; ++i) {
            if (HP[i] * HP[i] > innov_var * _P[i][i]) {
                return false;
            }
        }

        // K = P * H' * (H * P * H' + R)^-1
        float K[DIM];
        for (uint8_t i = 0; i < 12; ++i) {
            K[i] = HP[i] / innov_var;
        }

        // x = x + K * e
        for (uint8_t i = 0; i < 3; ++i) {
            _error_state.pos(i) += K[i] * innov;
            _error_state.vel(i) += K[i + 3] * innov;
            _error_state.ang(i) += K[i + 6] * innov;
            _error_state.delta_ang_bias(i) += K[i + 9] * innov;
        }

        // P = P - K * H * P
        for (uint8_t i = 0; i < 12; ++i) {
            for (uint8_t j = i; j < 12; ++j) {
                _P[i][j] -= K[i] * HP[j];
            }
        }

        if (_control_status.flags.acc_x_bias) {
            // K = P * H' * (H * P * H' + R)^-1
            K[12] = HP[12] / innov_var;

            // x = x + K * e
            _error_state.delta_vel_bias(0) += K[12] * innov;

            // P = P - K * H * P
            for (uint8_t j = 0; j <= 12; ++j) {
                _P[j][12] -= K[j] * HP[12];
            }
        }

        if (_control_status.flags.acc_y_bias) {
            // K = P * H' * (H * P * H' + R)^-1
            K[13] = HP[13] / innov_var;

            // x = x + K * e
            _error_state.delta_vel_bias(1) += K[13] * innov;

            // P = P - K * H * P
            for (uint8_t j = 0; j <= 13; ++j) {
                _P[j][13] -= K[j] * HP[13];
            }
        }

        if (_control_status.flags.acc_z_bias) {
            // K = P * H' * (H * P * H' + R)^-1
            K[14] = HP[14] / innov_var;

            // x = x + K * e
            _error_state.delta_vel_bias(2) += K[14] * innov;

            // P = P - K * H * P
            for (uint8_t j = 0; j <= 14; ++j) {
                _P[j][14] -= K[j] * HP[14];
            }
        }

        if (_control_status.flags.grav) {
            // K = P * H' * (H * P * H' + R)^-1
            K[15] = HP[15] / innov_var;

            // x = x + K * e
            _error_state.grav += K[15] * innov;

            // P = P - K * H * P
            for (uint8_t j = 0; j <= 15; ++j) {
                _P[j][15] -= K[j] * HP[15];
            }
        }

        if (_control_status.flags.mag_norm) {
            // K = P * H' * (H * P * H' + R)^-1
            K[16] = HP[16] / innov_var;

            // x = x + K * e
            _error_state.mag_norm += K[16] * innov;

            // P = P - K * H * P
            for (uint8_t j = 0; j <= 16; ++j) {
                _P[j][16] -= K[j] * HP[16];
            }
        }

        if (_control_status.flags.mag_ang) {
            // K = P * H' * (H * P * H' + R)^-1
            for (uint8_t i = 17; i < 19; ++i) {
                K[i] = HP[i] / innov_var;
            }

            // x = x + K * e
            for (uint8_t i = 17; i < 19; ++i) {
                _error_state.mag_ang(i - 17) += K[i] * innov;
            }

            // P = P - K * H * P
            for (uint8_t i = 17; i < 19; ++i) {
                for (uint8_t j = 0; j <= i; ++j) {
                    _P[j][i] -= K[j] * HP[i];
                }
            }
        }

        if (_control_status.flags.mag_bias) {
            // K = P * H' * (H * P * H' + R)^-1
            for (uint8_t i = 19; i < 22; ++i) {
                K[i] = HP[i] / innov_var;
            }

            // x = x + K * e
            for (uint8_t i = 19; i < 22; ++i) {
                _error_state.mag_bias(i - 19) += K[i] * innov;
            }

            // P = P - K * H * P
            for (uint8_t i = 19; i < 22; ++i) {
                for (uint8_t j = 0; j <= i; ++j) {
                    _P[j][i] -= K[j] * HP[i];
                }
            }
        }

        if (_control_status.flags.wind) {
            // K = P * H' * (H * P * H' + R)^-1
            for (uint8_t i = 22; i < 24; ++i) {
                K[i] = HP[i] / innov_var;
            }

            // x = x + K * e
            for (uint8_t i = 22; i < 24; ++i) {
                _error_state.wind(i - 22) += K[i] * innov;
            }

            // P = P - K * H * P
            for (uint8_t i = 22; i < 24; ++i) {
                for (uint8_t j = 0; j <= i; ++j) {
                    _P[j][i] -= K[j] * HP[i];
                }
            }
        }

        return true;
    }

    void ESKF::constrain_covariance() {
        auto constrain_max = [&](uint8_t &i, float &var_max) {
            if (_P[i][i] > var_max) {
                float rho = var_max / _P[i][i];

                // 只对上三角进行操作
                for (uint8_t j = 0; j <= i; ++j) {
                    _P[j][i] *= rho;
                }
                for (uint8_t j = i; j < DIM; ++j) {
                    _P[i][j] *= rho;
                }
            }
        };

        float var_delta_ang_bias_min = _params.var_gyro_bias_min * _dt2;
        float var_delta_ang_bias_max = _params.var_gyro_bias_max * _dt2;
        float var_delta_vel_bias_min = _params.var_acc_bias_min * _dt2;
        float var_delta_vel_bias_max = _params.var_acc_bias_max * _dt2;

        // δp, δv, δθ, δΔθ, δΔv, δmag_bias
        for (uint8_t i = 0; i < 3; ++i) {
            uint8_t i_vel = i + 3, i_ang = i + 6, i_gyro_bias = i + 9, i_acc_bias = i + 12, i_mag_bias = 19 + i;

            _P[i][i] = math::max(_P[i][i], _params.var_pos_min);
            _P[i_vel][i_vel] = math::max(_P[i_vel][i_vel], _params.var_vel_min);
            _P[i_ang][i_ang] = math::max(_P[i_ang][i_ang], _params.var_angle_min);
            _P[i_gyro_bias][i_gyro_bias] = math::max(_P[i_gyro_bias][i_gyro_bias], var_delta_ang_bias_min);
            _P[i_acc_bias][i_acc_bias] = math::max(_P[i_acc_bias][i_acc_bias], var_delta_vel_bias_min);
            _P[i_mag_bias][i_mag_bias] = math::max(_P[i_mag_bias][i_mag_bias], _params.var_mag_bias_min);

            constrain_max(i, _params.var_pos_max);
            constrain_max(i_vel, _params.var_vel_max);
            constrain_max(i_ang, _params.var_angle_max);
            constrain_max(i_gyro_bias, var_delta_ang_bias_max);
            constrain_max(i_acc_bias, var_delta_vel_bias_max);
            constrain_max(i_mag_bias, _params.var_mag_bias_min);
        }

        // δg, δmag_norm
        uint8_t i_grav = 15, i_mag_norm = 16;

        _P[i_grav][i_grav] = math::max(_P[i_grav][i_grav], _params.var_grav_min);
        _P[i_mag_norm][i_mag_norm] = math::max(_P[i_mag_norm][i_mag_norm], _params.var_mag_norm_min);

        constrain_max(i_grav, _params.var_grav_max);
        constrain_max(i_mag_norm, _params.var_mag_norm_max);

        // δmag_ang, δw
        for (uint8_t i = 17; i < 19; ++i) {
            uint8_t i_wind = i + 5;

            _P[i][i] = math::max(_P[i][i], _params.var_mag_ang_min);
            _P[i_wind][i_wind] = math::max(_P[i_wind][i_wind], _params.var_wind_min);

            constrain_max(i, _params.var_mag_norm_max);
            constrain_max(i_wind, _params.var_wind_max);
        }

        // 增强非负定特性(只针对上三角的元素)
        for (uint8_t i = 0; i < DIM; ++i) {
            for (uint8_t j = i + 1; j < DIM; ++j) {
                float tmp = _P[i][i] * _P[j][j];
                if (_P[i][j] * _P[i][j] > tmp) {
                    if (_P[i][j] > 0.f) {
                        _P[i][j] = sqrtf(tmp);
                    } else {
                        _P[i][j] = -sqrtf(tmp);
                    }
                }
            }
        }

        regular_covariance_to_symmetric<DIM>(0);
    }

    void ESKF::calculate_dist_bottom_imu(float range, const Vector3f &range_offset_nav) {
        static constexpr float cos_60deg = 0.5f;

        // 如果倾转角度大于60°, 则不进行融合
        if (_Rnb(2, 2) <  cos_60deg) {
            return;
        }

        _dist_bottom_imu = math::max(_params.rng_gnd_clearance, range * _Rnb(2, 2)) + range_offset_nav(2);
    }

    void ESKF::calculate_baro_imu(float baro, const Vector3f &baro_offset_nav) {
        _baro_imu = baro + baro_offset_nav(2);
    }

    void ESKF::correct_terrain(float gps_hgt_imu) {
        _terrain += _params.alpha_terrain * (_dist_bottom_imu - _terrain - gps_hgt_imu);
    }

    void ESKF::correct_baro_bias(float gps_hgt_imu) {
        _baro_bias += _params.alpha_baro_bias * (_baro_imu - _baro_bias - gps_hgt_imu);
    }

}
