//
// Created by Cain on 2022/12/31.
//

#ifndef ECL_ESKF_H
#define ECL_ESKF_H

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <geo/geo.h>

#include "utils.h"
#include "common.h"

namespace eskf {
    using matrix::AxisAnglef;
    using matrix::Dcmf;
    using matrix::Eulerf;
    using matrix::Matrix3f;
    using matrix::Quatf;
    using matrix::Vector2f;
    using matrix::Vector3f;
    using matrix::wrap_pi;
    using matrix::Vector;
    using matrix::Matrix;

    template <uint8_t DELAYS>
    class ESKFRunner;

    class ESKF {
    public:
        constexpr static uint8_t DIM {24};
        constexpr static uint8_t POS_ID {0};
        constexpr static uint8_t VEL_ID {3};
        constexpr static uint8_t ANG_ID {6};
        constexpr static uint8_t DEL_ANG_ID {9};

        ESKF(Parameters &params) : _params(params) {
            set_dt(1e-6f * float(_params.eskf_update_interval_us));
            reset_process_variances();
            reset_state();
            reset_error_state();
            reset_covariance_matrix(0, DIM);
            regular_covariance_to_symmetric<DIM>(0);
//            init_covariance();
            reset_accmulator();
        }

        void initialize();
        void init_covariance();

        // Priori
        /*!
         * p = p + v * Δt + 0.5 * (R * (Δv - b_Δv) + g * Δt) * Δt
         * v = v + R * (Δv - b_Δv) + g * Δt
         * R = R * Exp((Δθ - b_Δθ))
         * b_Δθ = (1 - Δt/τ) * b_Δθ
         * b_Δv = (1 - Δt/τ) * b_Δv
         * g = g
         * mag_norm = mag_norm
         * mag_ang = mag_ang
         * b_m = (1 - Δt/τ) * b_m
         * wind = wind
         *
         * @param delta_ang - 陀螺仪角度增量
         * @param delta_vel - 加速度计速度增量
         */
        void predict_state(const ImuSample &imu_sample);

        /*!
         * 计算 P = F' * P * F + Q
         * 其中 加速度和角速度的量测噪声根据是否达到限幅进行动态调整
         * @param delta_ang - 陀螺仪角度增量
         * @param delta_vel - 加速度计速度增量
         * @param gyro_clipping - 陀螺仪是否达到限幅
         * @param acc_clipping - 加速度计是否达到限幅
         */
        virtual void predict_covariance(const ImuSample &imu_sample) = 0;

        // Posterior
        virtual uint8_t fuse_pos_horz(const Vector2f &pos, const Vector3f &offset_body, const Vector3f &offset_nav,
                                      const float &gate, const float &noise_std, FuseData<2> &fuse_data) = 0;
        virtual uint8_t fuse_pos_vert(const float &pos, const Vector3f &offset_body, const Vector3f &offset_nav,
                                      const float &gate, const float &noise_std, FuseData<1> &fuse_data) = 0;
        virtual uint8_t fuse_vel_horz(const Vector2f &vel, const Vector3f &offset_body, const Vector3f &offset_nav,
                                      const Vector3f &w_cross_offset_body, const Vector3f &w_cross_offset_nav,
                                      const float &gate, const float &noise_std, FuseData<2> &fuse_data) = 0;
        virtual uint8_t fuse_vel_vert(const float &vel, const Vector3f &offset_body, const Vector3f &offset_nav,
                                      const Vector3f &w_cross_offset_body, const Vector3f &w_cross_offset_nav,
                                      const float &gate, const float &noise_std, FuseData<1> &fuse_data) = 0;
        virtual uint8_t fuse_vel_body_horz(const Vector2f &vel, const Vector3f &offset_body, const Vector3f &offset_nav,
                                           const Vector3f &w_cross_offset_body, const Vector3f &w_cross_offset_nav,
                                           const float &gate, const float &noise_std, FuseData<2> &fuse_data) = 0;
        virtual uint8_t fuse_mag_body(const Vector3f &mag_body, const float &gate, const float &noise_std, FuseData<3> &fuse_data) = 0;
        virtual uint8_t fuse_mag_norm(const float &mag_norm, const float &gate, const float &noise_std, FuseData<1> &fuse_data) = 0;
        virtual uint8_t fuse_mag_ang(const Vector2f &mag_ang, const float &gate, const float &noise_std, FuseData<2> &fuse_data) = 0;
        virtual uint8_t fuse_flow(const Vector2f &flow, const Vector3f &offset_body, const Vector3f &offset_nav, const float &gate, const float &noise_std, FuseData<2> &fuse_data) = 0;
        virtual uint8_t fuse_range(const float &range, const Vector3f &offset_body, const Vector3f &offset_nav, const float &gate, const float &noise_std, FuseData<1> &fuse_data) = 0;

        /*!
         * s = s + δs
         */
        virtual void correct_state() = 0;

        /*!
         * 把协方差矩阵投影到新的state中
         */
        virtual void correct_covariance() = 0;

        // Resetters
        /*!
         * 重置ESKF状态, 地形相关状态, 光流相关状态, 测距仪相关状态
         */
        void reset_state() {
            _state.pos.setZero();
            _state.vel.setZero();
            _state.quat_nominal.setIdentity();
            _state.delta_ang_bias.setZero();
            _state.delta_vel_bias.setZero();
            _state.grav = CONSTANTS_ONE_G;
            _state.mag_norm = _params.mag_norm;
            _state.mag_ang = {_params.mag_inclination, _params.mag_declination};
            _state.mag_bias.setZero();
            _state.wind.setZero();
            _Rnb.setIdentity();

            _terrain_vpos = 0.f;
            _imu_hgt = 0.f;

            _flow_vel_body.setZero();
            _flow_vel_nav.setZero();
            _flow_hgt = 0.f;

            _range = 0.f;
            _range_hgt = 0.f;
        }

        // 重置变分状态
        /*!
         * 重置ESKF的误差状态
         */
        void reset_error_state() {
            _error_state.pos.setZero();
            _error_state.vel.setZero();
            _error_state.ang.setZero();
            _error_state.delta_ang_bias.setZero();
            _error_state.delta_vel_bias.setZero();
            _error_state.grav = 0.f;
            _error_state.mag_norm = 0.f;
            _error_state.mag_ang.setZero();
            _error_state.mag_bias.setZero();
            _error_state.wind.setZero();
        }

        /*!
         * 重置过程噪声协方差矩阵(忽略了陀螺仪和加速度计的噪声)
         */
        void reset_process_variances() {
            _Q[0] = _Q[1] = _Q[2] = _params.pos_proc_noise * _params.pos_proc_noise;
            _Q[3] = _Q[4] = _Q[5] = _params.vel_proc_noise * _params.vel_proc_noise;
            _Q[6] = _Q[7] = _Q[8] = _params.ang_axis_proc_noise * _params.ang_axis_proc_noise;
            _Q[9] = _params.grav_proc_noise * _params.grav_proc_noise;
            _Q[10] = _Q[11] = _Q[12] = _params.ang_axis_proc_noise * _params.ang_axis_proc_noise * _dt2;
            _Q[13] = _Q[14] = _Q[15] = _params.acc_bias_proc_noise * _params.acc_bias_proc_noise * _dt2;
            _Q[16] = _params.mag_norm_proc_noise * _params.mag_norm_proc_noise;
            _Q[17] = _Q[18] = _params.mag_ang_proc_noise * _params.mag_ang_proc_noise;
            _Q[19] = _Q[20] = _Q[21] = _params.mag_bias_proc_noise * _params.mag_bias_proc_noise;
            _Q[22] = _Q[23] = _params.wind_proc_noise * _params.wind_proc_noise;
        }

        /*!
         * 利用diag_cov重置误差状态协方差矩阵
         * @param start_index - 开始的下标
         * @param end_index - 终止的下标
         * @param diag_cov - 给定的对角协方差矩阵
         */
        void reset_covariance_matrix(const uint8_t start_index, const uint8_t end_index, const float diag_cov[DIM]) {
            for (uint8_t i = start_index; i < end_index; ++i) {
                // Diaginal
                _P[i][i] = diag_cov[i] * _dt2;

                // Upper triangular
                for (uint8_t j = start_index; j < i; ++j) {
                    _P[j][i] = 0.f;
                }

                // Columns
                for (uint8_t j = 0; j < start_index; ++j) {
                    _P[j][i] = 0.f;
                }

                // Rows
                for (uint8_t j = end_index; j < DIM; ++j) {
                    _P[i][j] = 0.f;
                }
            }
        }

        /*!
         * 重置变分状态协方差矩阵: 利用过程噪声协防差矩阵Q来重置协方差矩阵P
         * @param start_index - 协方差矩阵开始重置的下标
         * @param end_index  - 协方差矩阵结束重置的下标
         */
        void reset_covariance_matrix(const uint8_t start_index, const uint8_t end_index) {
            reset_covariance_matrix(start_index, end_index, _Q);
        }

        /*!
         * 重置变分状态协方差矩阵: 利用对角矩阵diag_cov来重置协方差矩阵P
         * @tparam N - diag_cov的长度
         * @param start_index - 协方差矩阵开始重置的下标
         * @param diag_cov - 目标的协方差对角矩阵
         */
        template <uint8_t N>
        void reset_covariance_matrix(const uint8_t start_index, const float diag_cov[DIM]) {
            uint8_t end_index = start_index + N;
            for (uint8_t i = start_index; i < end_index; ++i) {
                // Diaginal
                _P[i][i] = diag_cov[i - start_index] * _dt2;

                // Upper triangular
                for (uint8_t j = start_index; j < i; ++j) {
                    _P[j][i] = 0.f;
                }

                // Columns
                for (uint8_t j = 0; j < start_index; ++j) {
                    _P[j][i] = 0.f;
                }

                // Rows
                for (uint8_t j = end_index; j < DIM; ++j) {
                    _P[i][j] = 0.f;
                }
            }
        }

        /*!
         * 把协方差矩阵重置为对角矩阵, 且对角元为diag_cov
         * @tparam N - 从start_index开始, 连续重置多少个协方差对角元
         * @param start_index - 协方差矩阵开始重置的下标
         * @param diag_cov - 重置后的对角元
         */
        template <uint8_t N>
        void reset_covariance_matrix(const unsigned char start_index, const float diag_cov) {
            uint8_t end_index = start_index + N;
            for (uint8_t i = start_index; i < end_index; ++i) {
                // Diaginal
                _P[i][i] = diag_cov * _dt2;

                // Upper triangular
                for (uint8_t j = start_index; j < i; ++j) {
                    _P[j][i] = 0.f;
                }

                // Columns
                for (uint8_t j = 0; j < start_index; ++j) {
                    _P[j][i] = 0.f;
                }

                // Rows
                for (uint8_t j = end_index; j < DIM; ++j) {
                    _P[i][j] = 0.f;
                }
            }
        }

        /*!
         * 重置舍去误差累加器
         * @tparam N - 从start_index开始连续重置多少个
         * @param start_index - 开始重置的下标
         */
        template <uint8_t N>
        void reset_accmulator(const uint8_t start_index) {
            uint8_t end_index = start_index + N;
            for (uint8_t i = start_index; i < end_index; ++i) {
                _accumulator[i] = 0.f;
            }
        }

        /*!
         * 重置舍去误差累加器
         */
        void reset_accmulator() {
            for (float & i : _accumulator) {
                i = 0.f;
            }
        }

        // Switches
        void enable_estimation_acc_x_bias() { _control_status.flags.acc_x_bias = true; };
        void enable_estimation_acc_y_bias() { _control_status.flags.acc_y_bias = true; };
        void enable_estimation_acc_z_bias() { _control_status.flags.acc_z_bias = true; };
        void enable_estimation_acc_bias () {
            enable_estimation_acc_x_bias();
            enable_estimation_acc_y_bias();
            enable_estimation_acc_z_bias();
        }
        void enable_estimation_gravity() { _control_status.flags.grav = true; };
        void enable_estimation_magnet() { _control_status.flags.mag_norm = true; };
        void enable_estimation_declination() { _control_status.flags.mag_ang = true; };
        void enable_estimation_magnet_bias() { _control_status.flags.mag_bias = true; };
        void enable_estimation_wind() { _control_status.flags.wind = true; } ;

        void disable_estimation_acc_x_bias() {
            _control_status.flags.acc_x_bias = false;
            reset_covariance_matrix<1>(12, 0.f);
            reset_accmulator<1>(12);
            regular_covariance_to_symmetric<1>(12);
        };
        void disable_estimation_acc_y_bias() {
            _control_status.flags.acc_y_bias = false;
            reset_covariance_matrix<1>(13, 0.f);
            reset_accmulator<1>(13);
            regular_covariance_to_symmetric<1>(13);
        };
        void disable_estimation_acc_z_bias() {
            _control_status.flags.acc_z_bias = false;
            reset_covariance_matrix<1>(14, 0.f);
            reset_accmulator<1>(14);
            regular_covariance_to_symmetric<1>(14);
        };
        void disable_estimation_acc_bias() {
            disable_estimation_acc_x_bias();
            disable_estimation_acc_y_bias();
            disable_estimation_acc_z_bias();
        }
        void disable_estimation_gravity() {
            _control_status.flags.grav = false;
            reset_covariance_matrix<1>(15, 0.f);
            reset_accmulator<1>(15);
            regular_covariance_to_symmetric<1>(15);
        };
        void disable_estimation_mag_norm() {
            _control_status.flags.mag_norm = false;
            reset_covariance_matrix<1>(16, 0.f);
            reset_accmulator<1>(16);
            regular_covariance_to_symmetric<1>(16);
        };
        void disable_estimation_mag_ang() {
            _control_status.flags.mag_ang = false;
            reset_covariance_matrix<2>(17, 0.f);
            reset_accmulator<2>(17);
            regular_covariance_to_symmetric<2>(17);
        };
        void disable_estimation_magnet_bias() {
            _control_status.flags.mag_bias = false;
            reset_covariance_matrix<3>(19, 0.f);
            reset_accmulator<3>(19);
            regular_covariance_to_symmetric<3>(19);
        };
        void disable_estimation_wind() {
            _control_status.flags.wind = false;
            reset_covariance_matrix<2>(22, 0.f);
            reset_accmulator<2>(22);
            regular_covariance_to_symmetric<2>(22);
        };

        // Getters
        const Vector3f &get_position() const { return _state.pos; };
        Vector2f get_pos_horz() const { return _state.pos.xy(); };
        float get_pos_vert() const { return _state.pos(2); };
        const Vector3f &get_velocity() const { return _state.vel; };
        Vector2f get_vel_horz() const { return _state.vel.xy(); };
        float get_vel_vert() const { return _state.vel(2); }
        const Quatf &get_quaternion() const { return _state.quat_nominal; };
        Vector3f get_gyro_bias() const { return _state.delta_ang_bias / _dt; };
        const Vector3f &get_delta_ang_bias() const { return _state.delta_ang_bias; };
        Vector3f get_acc_bias() const { return _state.delta_ang_bias / _dt; };
        const Vector3f &get_delta_vel_bias() const { return _state.delta_vel_bias; };
        float get_gravity() const { return _state.grav; };
        float get_magnet_norm() const { return _state.mag_norm; };
        const Vector2f &get_magnet_angle() const { return _state.mag_ang; };
        const Vector3f &get_magnet_bias() const { return _state.mag_bias; };
        const Vector2f &get_wind() const { return _state.wind; };
        const StateSample &get_state() const { return _state; };
        const Dcmf &get_Rnb() const { return _Rnb; };
        const Dcmf &get_Ren() const { return _Ren; };
        const Vector3f &get_Mpv() const { return _Mpv; };
        float get_dt() const { return _dt; };
        const ErrorState &get_error_state() const { return _error_state; };
        const float (*get_covariance_matrix() const)[DIM] { return _P; };
        float get_terrain_vpos() const { return _terrain_vpos; };
        float get_imu_hgt() const { return _imu_hgt; };
        const Vector3f &get_delta_ang_corr() const { return _delta_ang_corr; };
        const Vector3f &get_gyro_corr() const { return _gyro_corr; };
        const Vector3f &get_delta_vel_corr() const { return _delta_vel_corr; };
        const Vector3f &get_acc_corr() const { return _acc_corr; };
        const Vector3f &get_delta_vel_nav() const { return _delta_vel_corr_nav; };
        const Vector3f &get_acc_nav() const { return _acc_nav; };
        const Vector2f &get_flow_vel_body() const { return _flow_vel_body; };
        const Vector2f &get_flow_vel_nav() const { return _flow_vel_nav; };
        float get_flow_hgt() const { return _flow_hgt; };
        float get_flow_range() const { return _flow_range; };
        float get_range() const { return _range; };
        float get_range_hgt() const { return _range_hgt; };


        // Setters
        void set_dt(const float dt) { _dt = dt; _dt2 = dt * dt; _dt4 = _dt2 * _dt2; };
        void set_position(const Vector3f &p) { _state.pos = p; };
        void set_pos_horz(const Vector2f &p) { _state.pos.xy() = p; };
        void set_pos_vert(const float &p) { _state.pos(2) = p; };
        void set_velocity(const Vector3f &v) { _state.vel = v; };
        void set_vel_horz(const Vector2f &v) { _state.vel.xy() = v; };
        void set_vel_vert(const float &v) { _state.vel(2) = v; };
        void set_attitude(const Dcmf &R) { _state.quat_nominal = R; _Rnb = R; };
        void set_attitude(const Quatf &q) { _state.quat_nominal = q; _Rnb = q; };
        void set_attitude(const AxisAnglef &axis_ang) { _state.quat_nominal = axis_ang; _Rnb = axis_ang; };
        void set_gyro_bias(const Vector3f &bg) { _state.delta_ang_bias = bg * _dt; };
        void set_delta_ang_bias(const Vector3f &b_delta_ang) { _state.delta_ang_bias = b_delta_ang; };
        void set_acc_bias(const Vector3f &ba) { _state.delta_vel_bias = ba * _dt; };
        void set_delta_vel_bias(const Vector3f &b_delta_vel) { _state.delta_vel_bias = b_delta_vel; };
        void set_gravity(const float g) { _state.grav = g; };
        void set_mag_norm(const float norm) { _state.mag_norm = norm; };
        void set_mag_ang(const Vector2f &ang) { _state.mag_ang = ang; };
        void set_bias_magnet(const Vector3f &b_mag) { _state.mag_bias = b_mag; };
        void set_wind(const Vector2f &w) { _state.wind = w; };
        void set_state(const StateSample &state) { _state = state; };

        void set_proc_std_pos(const float &std) { _Q[0] = _Q[1] = _Q[2] = std * std; };
        void set_proc_std_pos_horz(const float &std) { _Q[0] = _Q[1] = std * std; };
        void set_proc_std_pos_vert(const float &std) { _Q[2] = std * std; };
        void set_proc_std_vel(const float &std) { _Q[3] = _Q[4] = _Q[5] = std * std; };
        void set_proc_std_vel_horz(const float &std) { _Q[3] = _Q[4] = std * std; };
        void set_proc_std_vel_vert(const float &std) { _Q[5] = std * std; };
        void set_proc_std_axis_ang(const float &std) { _Q[6] = _Q[7] = _Q[8] = std * std; };
        void set_proc_std_gyro_bias(const float &std) { _Q[9] = _Q[10] = _Q[11] = std * std; };
        void set_proc_std_acc_bias(const float &std) { _Q[12] = _Q[13] = _Q[14] = std * std; };
        void set_proc_std_grav(const float &std) { _Q[15] = std * std; };
        void set_proc_std_mag_norm(const float &std) { _Q[16] = std * std; };
        void set_proc_std_mag_ang(const float &std) { _Q[17] = _Q[18] = std * std; };
        void set_proc_std_mag_bias(const float &std) { _Q[19] = _Q[20] = _Q[21] = std * std; };
        void set_proc_std_wind(const float &std) { _Q[22] = _Q[23] = std * std; };
        template<uint8_t N>
        void set_proc_std(const float &std) { _Q[N] = std * std; };
        template<uint8_t Start, uint8_t End>
        void set_proc_std(const float q[End - Start]) { for (uint8_t i = Start; i < End; ++i) _Q[i] = q[i - Start]; };

    protected:
        Parameters &_params;       ///< eskf参数
        float _dt;                 ///< eskf更新周期
        float _dt2;                ///< eskf更新周期的平方
        float _dt4;                ///< eskf更新周期的4次方

        /* 24维的ESKF */
        StateSample _state;        ///< eskf状态: [p, v, q, b_ang, b_vel, g, m_norm, m_ang, b_m, w]
        ErrorState _error_state;   ///< eskf变分状态: [δp, δv, δθ, δb_ang, δb_vel, δg, δm_norm, δm_ang, δb_m, δw]
        float _P[DIM][DIM] {};     ///< 协方差矩阵
        float _accumulator[DIM] {};///< 舍去误差累加器
        Dcmf _Rnb;                 ///< body -> nav的旋转矩阵
        Dcmf _Ren;                 ///< nav -> earth的旋转矩阵
        Vector3f _Mpv   ;          ///< nav速度 -> (维度, 经度, 高程)的变化率的变换矩阵
        float _Q[DIM] {};          ///< 过程噪声协方差矩阵Q

        /* 地形ESKF */
        float _terrain_vpos {0.f}; ///< 地形高度
        float _imu_hgt {0.f};      ///< imu距地高度

        /* imu */
        Vector3f _delta_ang_corr;   ///< 纠偏后的速度增量(对应陀螺仪的量测)
        Vector3f _gyro_corr;        ///< 纠偏后的加速度(对应陀螺仪的量测)
        Vector3f _delta_vel_corr;   ///< 纠偏后的角度增量(对应加速度计的量测)
        Vector3f _acc_corr;         ///< 纠偏后的角速度(对应加速度计的量测)
        Vector3f _delta_vel_corr_nav;    ///< 纠偏后的角度增量(导航系下的速度增量)
        Vector3f _acc_nav;          ///< 纠偏后的角速度(导航系下的角速度)

        /* 光流 */
        Vector2f _flow_vel_body;    ///< 光流速度(投影在body系)
        Vector2f _flow_vel_nav;     ///< 光流速度(投影在nav系)
        float _flow_hgt;            ///< 光流距地高度(向上为正)
        float _flow_range;          ///< 光流视场中心到地面的距离

        /* 测距仪 */
        float _range;
        float _range_hgt;           ///< 测距仪距地高度(向上为正)

        filter_control_status_u _control_status {false};
//        innovation_fault_status_u _innovation_fault_status {false};

//        bool _imu_updated {false};

        /*!
         * 计算后验协方差矩阵与误差状态
         * @param HP - H*P
         * @param innov_var - H*P*H' + R
         * @param innov - 误差(量测 - 估计)
         * @return - 是否融合成功, 如果后验协方差矩阵的对角元为正, 则认为成功, 否则认为失败
         */
        bool posterior_estimate(const float (&HP)[DIM], const float &innov_var, const float &innov);

        void constrain_covariance();

        /*!
         * 把上三角关联的行列复制到下三角
         * @tparam N - 从start_index开始连续多少个
         * @param start_index - 开始的下标
         */
        template <uint8_t N>
        void regular_covariance_to_symmetric(unsigned char start_index) {
            unsigned char end_index = start_index + N;
            for (unsigned char i = start_index; i < end_index; ++i) {
                for (unsigned char j = 0; j < i; ++j) {
                    _P[i][j] = _P[j][i];
                }
            }
            for (unsigned char i = end_index; i < DIM; ++i) {
                for (unsigned char j = start_index; j < end_index; ++j) {
                    _P[i][j] = _P[j][i];
                }
            }
        }

        /*!
         * 把上三角的列复制到行
         * @tparam N - 从start_index开始连续多少个
         * @param start_index - 开始的下标
         */
        template <uint8_t N>
        void copy_covariance_cols_to_rows(unsigned char start_index) {
            unsigned char end_index = start_index + N;
            for (unsigned char i = start_index; i < end_index; ++i) {
                for (unsigned char j = 0; j < i; ++j) {
                    _P[i][j] = _P[j][i];
                }
            }
        }

        /*!
         * 把上三角的行复制到列
         * @tparam N - 从start_index开始连续多少个
         * @param start_index - 开始的下标
         */
        template <uint8_t N>
        void copy_covariance_rows_to_cols(unsigned char start_index) {
            unsigned char end_index = start_index + N;
            for (unsigned char i = start_index; i < end_index; ++i) {
                for (unsigned char j = i + 1; j < DIM; ++j) {
                    _P[j][i] = _P[i][j];
                }
            }
        }

        /*!
         * 保存sum_previous + y中, y舍弃掉的部分
         * @param sum_previous
         * @param input
         * @param accumulator - sum_previous + y中, y被舍弃掉的部分
         * @return sum_previous + input
         */
        static float kahan_summation(float sum_previous, float input, float &accumulator) {
            /*
            accumulator中记录了sum_previous + y中, y舍弃掉的部分
            */
            const float y = input - accumulator;
            const float t = sum_previous + y;
            accumulator = (t - sum_previous) - y;
            return t;
        }

        /*!
         * 把过程噪声叠加在协方差矩阵上: P = P + Q
         * @tparam N - 从start_index开始连续多少个
         * @param start_index - 开始的下标
         */
        template <uint8_t N>
        void add_process_variances(unsigned char start_index) {
            unsigned char end_index = start_index + N;
            for (unsigned char i = start_index; i < end_index; ++i) {
                _P[i][i] = kahan_summation(_P[i][i], _Q[i] * _dt2, _accumulator[i]);
            }
        }

    private:
        template<uint8_t DELAYS>
        friend class ESKFRunner;
    };
}

#endif //ECL_ESKF_H
