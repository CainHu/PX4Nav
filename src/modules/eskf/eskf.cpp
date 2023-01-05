//
// Created by Cain on 2023/1/3.
//

#include "eskf.h"
#include <cfloat>

namespace eskf {
    void ESKF::initialize() {
        set_dt(1e-6f * float(_params.eskf_update_interval_us));
        reset_state();
        reset_error_state();
        reset_covariance_matrix(0, DIM);
        regular_covariance_to_symmetric<DIM>(0);
        reset_accmulator();
    }

    void ESKF::predict_state(const Vector3f &delta_ang, const Vector3f &delta_vel) {
        // Δv_corr = Δv - b_Δv
        _delta_vel_corr = delta_vel - _state.delta_vel_bias;
        // a_corr = Δv_corr / Δt
        _acc_corr = _delta_vel_corr / _dt;

        // Δv_nav = Rnb * Δv_corr
        _delta_vel_corr_nav = _Rnb * _delta_vel_corr;
        // a_nav = Δv_nav / Δt
        _acc_nav = _delta_vel_corr_nav / _dt;

        // 上一时刻的速度
        const Vector3f v_last = _state.vel;

        // v' = v + Δv
        _state.vel += _delta_vel_corr_nav;
        _state.vel(2) += _state.grav * _dt;

        // p' = p + 0.5 * (v' + v) * Δt
        _state.pos += 0.5f * (_state.vel + v_last) * _dt;

        // Δθ_corr = Δθ - b_Δθ
        _delta_ang_corr = delta_ang - _state.delta_ang_bias;
        // ω_corr = Δθ_corr / Δt
        _gyro_corr = _delta_ang_corr / _dt;

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
        _imu_hgt = _terrain_vpos - _state.pos(2);
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
}
