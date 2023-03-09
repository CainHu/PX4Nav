//
// Created by Cain on 2023/1/6.
//

#ifndef ECL_ESKF_RUNNER_H
#define ECL_ESKF_RUNNER_H

#include "eskf.h"
#include "common.h"
#include "utils.h"
#include "imu_down_sampler.h"
#include <functional>
#include <iostream>
#include <geo/geo.h>
#include <world_magnetic_model/geo_mag_declination.h>


namespace eskf {
    template <uint8_t DELAYS>
    class ESKFRunner {
    public:
        static constexpr uint8_t N_GPS {2}; ///< 总共有几个GPS数据源

        explicit ESKFRunner(ESKF &eskf) : _eskf(eskf), _imu_down_sampler(eskf._params.eskf_update_interval_us) {

        };

        void update();

        bool rough_alignment();

        bool should_collect_gps(const GpsMessage &gps, uint8_t n);

        void set_imu_data(const ImuSample &imu_sample);
        void set_gps_data(const GpsMessage &gps_message, uint8_t i);
        void set_magnet_data(const MagSample &mag_sample);
        void set_baro_date(const BaroSample &baro_sample);
        void set_range_data(const RangeSample &range_sample);
        void set_flow_date(const FlowSample &flow_sample);
        void set_ev_data(const ExtVisionSample &ev_sample);

        void set_in_air_status(bool in_air) {
            if (!in_air) {
                _time_last_on_ground = _imu_sample_last.time_us;

            } else {
                _time_last_in_air = _imu_sample_last.time_us;
            }

            _control_status.flags.in_air = in_air;
        }

        void set_vehicle_at_rest(bool at_rest) { _control_status.flags.vehicle_at_rest = at_rest; }

        const OutputSample &get_output_state() { return _output_state; };

    private:
        template<uint8_t START, uint8_t END>
        void uncorrelate_target_from_other_states();

        /* 控制高度融合的相关独立函数 */
        void check_vertical_acceleration_health();
        void control_height_fusion();

        /* 控制磁力计融合的相关独立函数 */
        void control_mag_fuse();
        void run_in_air_mag_align(const Vector3f &mag_sample);
        void run_on_ground_mag_align();
        bool align_mag_and_reset_eskf();
        void check_mag_field_strength(const MagSample &sample);
        void check_mag_inhibition();
        void check_mag_bias_observability();
        bool other_vector_sources_have_stopped();
        bool no_other_vector_aiding_than_mag() const;
        bool above_mag_anomalies() const;
        bool magnetometer_can_be_used() const;
        bool should_inhibit_mag() const;
        void inhibit_mag_fusion();
        void start_mag_norm_fusion();
        void stop_mag_norm_fusion();
        void start_mag_ang_fusion();
        void stop_mag_ang_fusion();
        void start_mag_bias_fusion();
        void stop_mag_bias_fusion();
        Vector3f get_mag_earth() const;

        /* 光流融合 */
        void control_optical_flow_fusion();

        /* 水平GPS融合 */
        void control_gps_except_hgt_fusion();

        /* 静止底座融合 */
        void control_static_fusion();

        /* 虚假位置融合 */
        void control_fake_pos_fusion();

        void update_pre_integral(const ImuSample &sample);
        void predict_state_from_delay(const ImuSample &sample);

        Vector3f calc_earth_rate_ned(float lat_rad) const {
            return {CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad), 0.f, -CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad)};
        }

        bool is_recent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const {
            return sensor_timestamp + acceptance_interval > _imu_sample_last.time_us;
        }

        bool is_timeout(uint64_t sensor_timestamp, uint64_t timeout_period) const {
            return sensor_timestamp + timeout_period < _imu_sample_last.time_us;
        }

        // TODO: 需要进一步
        bool is_terrain_estimate_valid() {
            return true;
        }

        /*!
         * 通过 _control_status 判断激活了多少个水平辅助传感器
         * @return 激活的水平辅助传感器个数
         */
        int get_number_of_active_horizontal_aiding_sources() const {
            return int(_control_status.flags.gps_horz || _control_status.flags.gps_vel)
                   + int(_control_status.flags.opt_flow);
//                   + int(_control_status.flags.ev_horz)
//                   + int(_control_status.flags.ev_vel)
//                   + int(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta);
        }

        /*!
         * 判断是否有水平辅助传感器已被激活
         * @return 水平辅助传感器是否已被激活
         */
        bool is_horizontal_aiding_active() const {
            return get_number_of_active_horizontal_aiding_sources() > 0;
        }

        /*!
         * 判断除给定的水平辅助传感器外, 是否还存在其他的水平辅助传感器
         * @param aiding_flag - 给定的水平辅助传感器
         * @return 是否还存在其他的水平辅助传感器
         */
        bool is_other_source_of_horizontal_aiding_than(const bool aiding_flag) const {
            const int nb_sources = get_number_of_active_horizontal_aiding_sources();
            return aiding_flag ? nb_sources > 1 : nb_sources > 0;
        }

        /*!
         * 判断是否只激活了给定的水平辅助传感器
         * @param aiding_flag - 给定的水平辅助传感器
         * @return 是否只激活了给定的水平辅助传感器
         */
        bool is_only_active_source_of_horizontal_aiding(const bool aiding_flag) const {
            return aiding_flag && !is_other_source_of_horizontal_aiding_than(aiding_flag);
        }

        bool has_horizontal_aiding_timeout() const {
            return is_timeout(_time_last_pos_horz_fuse, RunnerParameters::HORZ_FUSE_TIMEOUT)
                   && is_timeout(_time_last_vel_horz_fuse, RunnerParameters::HORZ_FUSE_TIMEOUT)
                   && is_timeout(_flow_state.time_last_fuse, RunnerParameters::HORZ_FUSE_TIMEOUT);
        }

        RunnerParameters _params {};    ///< 传感器参数

        // TODO: 临时
    public:
        ESKF &_eskf;    ///< 卡尔曼滤波器
    private:
        bool _rough_alignment_completed {false};
        bool _using_synthetic_position {false};

        /* 融合得到的数据(innov, innov_var, ratio_test) */
        FuseData<2> _gps_pos_horz_fuse_data[N_GPS];
        FuseData<1> _gps_pos_vert_fuse_data[N_GPS];
        FuseData<2> _gps_vel_horz_fuse_data[N_GPS];
        FuseData<1> _gps_vel_vert_fuse_data[N_GPS];
        FuseData<2> _ev_pos_horz_fuse_data;
        FuseData<1> _ev_pos_vert_fuse_data;
        FuseData<2> _ev_vel_horz_fuse_data;
        FuseData<1> _ev_vel_vert_fuse_data;
        FuseData<1> _baro_fuse_data;
        FuseData<1> _range_fuse_data;
        FuseData<2> _flow_fuse_data;
        FuseData<3> _mag_body_fuse_data;
        FuseData<1> _mag_norm_fuse_data;
        FuseData<2> _mag_ang_fuse_data;
        FuseData<3> _static_fuse_data;
        FuseData<2> _fake_fuse_data;

        ImuDownSampler _imu_down_sampler;   ///< imu下采样器

        bool _imu_updated {false};  ///< imu是否更新
        ImuSample _imu_sample_last {};  ///< 最新的imu采样

        float _dt_imu_avg {};   ///< imu平均采样时间

        float _min_obs_interval_us {};  ///< imu_buffer的平均时间间隔

        // 最后一次在空中和在地上
        uint64_t _time_last_in_air {};     ///< 最后一次在空中的时间
        uint64_t _time_last_on_ground {};  ///< 最后一次在地上的时间

        // 加速度计
        float _ang_rate_magnitude_filt {0.f};
        float _accel_magnitude_filt {0.f};
        Vector3f _accel_vec_filt {};
        bool _is_manoeuvre_level_high {false};
        bool _is_manoeuvre_level_low {false};

        // TODO: gps数据的精度
        float _gps_horz_error_norm {10.f};

        ImuSample _imu_sample_delay {};
        GpsSample _gps_sample_delay[N_GPS] {};
        BaroSample _baro_sample_delay {};
        RangeSample _range_sample_delay {};
        FlowSample _flow_sample_delay {};
        ExtVisionSample _ev_sample_delay {};
        MagSample _mag_sample_delay {};

        Vector3f _baro_offset_nav {};
        Vector3f _range_offset_nav {};
        Vector3f _gps_offset_nav[N_GPS] {};
        Vector3f _ev_offset_nav {};

        float _mag_norm {0.460042f};			///< 磁场强度 (Gauss)
        float _mag_inclination {0.604262f}; 	///< 磁倾角 (rad)
        float _mag_declination {-0.0565532f};	///< 磁偏角 (rad)

        OutputSample _output_state {};

        // 用于滞后补偿
        Queue<ImuSample, DELAYS> _imu_buffer;
        Queue<Vector3f, DELAYS> _delta_pos_buffer;
        Queue<GpsSample, DELAYS> _gps_buffer[N_GPS];
        Queue<MagSample, DELAYS> _mag_buffer;
        Queue<BaroSample, DELAYS> _baro_buffer;
        Queue<RangeSample, DELAYS> _range_buffer;
        Queue<FlowSample, DELAYS> _flow_buffer;
        Queue<ExtVisionSample, DELAYS> _ev_buffer;
        Queue<PreIntegralSample, DELAYS> _pre_buffer;

        // 数据标记
        uint64_t _time_last_pos_horz_fuse {0};
        uint64_t _time_last_pos_horz_fuse_attempt {0};
        uint64_t _time_last_pos_vert_fuse {0};
        uint64_t _time_last_pos_vert_fuse_attempt {0};
        uint64_t _time_last_vel_horz_fuse {0};
        uint64_t _time_last_vel_vert_fuse {0};
        uint64_t _time_last_mag_body_fuse {0};

        bool _ev_data_ready {false};
        bool _mag_data_ready {false};

        bool _ev_intermittent {true};
        bool _mag_intermittent;

        bool _non_mag_aiding_running_prev {true};

        uint64_t _time_prev_gps_us[N_GPS];
        uint64_t _time_prev_baro_us {0};
        uint64_t _time_prev_range_us {0};
        uint64_t _time_prev_flow_us {0};
        uint64_t _time_prev_ev_us {0};
        uint64_t _time_prev_mag_us {0};

        uint64_t _time_last_ev_hgt_fuse {0};
        uint64_t _mag_align_last_time {0}; ///< 最近一次磁场对齐的时间

        /* 垂直加速度量测异常检测所用到的变量 */
        uint16_t _clip_counter {0};
        uint64_t _time_bad_vert_accel {0};
        uint64_t _time_good_vert_accel {0};

        /* 磁力计融合所用到的特有变量 */
        uint64_t _mag_fusion_not_inhibited_us {0};
        LowPassFilter3d _mag_body_lpf {200.f, 20.f};
        LowPassFilter3d _gyro_orth_mag_lpf {200.f, 20.f};
        Vector3f _mag_body_lp {};
        uint32_t _mag_body_lpf_counter {0};
        bool _is_mag_fusion_inhibited {true};
        bool _mag_fusion_inhibited_too_long {true};
        bool _mag_bias_observable {false};
        bool _mag_align_req {false};

        uint8_t _imu_counter {0};
        LowPassFilter3d _accel_lpf {200.f, 5.f};
        LowPassFilter3d _gyro_lpf {200.f, 5.f};

        uint8_t _baro_counter {0};
        Butterworth<1> _baro_lpf {200.f, 5.f};

        struct RoughAlignmentState {
            bool imu_filter_init {false};
            bool mag_filter_init {false};

            LowPassFilter3d accel_lpf {200.f, 5.f};
            LowPassFilter3d gyro_lpf {200.f, 5.f};

        } _rough_alignment_state;

        struct GpsState {
            bool ned_origin_initialised {false};
            bool any_hgt_available {false};
            bool any_rtk_hgt_available {false};

            bool checks_passed[N_GPS] {false};
            bool intermittent[N_GPS] {true};
            bool data_ready[N_GPS] {false};
            bool hgt_available[N_GPS] {false};
            bool rtk_hgt_available[N_GPS] {false};
            bool speed_valid[N_GPS]{false};

            uint64_t time_last_hgt_fuse[N_GPS] {};
            uint64_t time_last_pos_horz_fuse[N_GPS] {};
            uint64_t time_last_vel_horz_fuse[N_GPS] {};
            uint64_t time_last_vel_vert_fuse[N_GPS] {};
            uint64_t last_fail_us[N_GPS] {};
            uint64_t last_pass_us[N_GPS] {};
            uint64_t last_origin_us {0};

            Vector3f offset_nav[N_GPS] {};
            Vector3f w_cross_offset_body[N_GPS] {};
            Vector3f w_cross_offset_nav[N_GPS] {};

            float hgt_imu[N_GPS] {};
            Vector2f pos_horz_imu[N_GPS] {};
            Vector3f vel_imu[N_GPS] {};

            float origin_unc_pos_horz {0.f}; // horizontal position uncertainty of the GPS origin
            float origin_unc_pos_vert {0.f}; // vertical position uncertainty of the GPS origin
            MapProjection pos_ref{}; // Contains WGS-84 position latitude and longitude of the ESKF origin
            float alt_ref {0.f};
            float yaw_offset{0.0f};	// Yaw offset angle for dual GPS antennas used for yaw estimation (radians).

            MapProjection pos_prev[N_GPS]{}; // Contains WGS-84 position latitude and longitude of the previous GPS message
            float alt_prev[N_GPS]{0.0f};	// height from the previous GPS message (m)

            Vector3f earth_rate_ned {};

            float mag_declination {NAN};
            float mag_inclination {NAN};
            float mag_strength {NAN};

            float pos_horz_error_quality[N_GPS] {1.f};
            float pos_vert_error_quality[N_GPS] {1.f};
            float vel_error_quality[N_GPS] {1.f};

            Vector3f pos_deriv_filt[N_GPS] {};
            float pos_horz_deriv_filt_norm[N_GPS] {};
            float pos_vert_deriv_filt_norm[N_GPS] {};

            Vector2f vel_horz_filt[N_GPS] {};
            float vel_horz_filt_norm[N_GPS] {};

            float vel_vert_diff_filt[N_GPS] {};

            gps_check_fail_status_u check_fail_status[N_GPS] {0};
        } _gps_state;

        struct BaroState {
            bool faulty {false};
            bool intermittent {true};
            bool data_ready {false};
            bool available {false};

            uint64_t time_last_fuse {0};
            uint64_t time_last_bias_fuse {0};

            Vector3f offset_nav {};
        } _baro_state;

        struct RangeState {
            bool healthy {false};
            bool intermittent {true};
            bool data_ready {false};
            bool available {false};

            uint64_t time_last_fuse {0};
            uint64_t time_last_terr_fuse {0};

            Vector3f offset_nav {};
        } _range_state;

        struct TerrState {
            bool valid {false};
            uint64_t time_last_fuse {0};
        } _terr_state;

        struct FlowState {
            bool data_ready {false};
            bool intermittent {true};
            bool inhibit_flow_use {true};

            uint64_t time_last_fuse {0};
            uint64_t time_bad_motion_us {0};
            uint64_t time_good_motion_us {0};

            float imu_delta_time {0.f};

            Vector3f imu_delta_ang {};

            Vector2f flow_compensated_xy_rad {};
            Vector2f last_known_posNE;
        } _flow_state;

        struct StaticState {
            uint64_t time_last_fuse {0};
        } _static_state;

        struct FakeState {
            uint64_t time_last_fuse {0};
            Vector2f last_known_posNE;
        } _fake_state;

        // TODO: 临时
    public:
        filter_control_status_u _control_status {};
        filter_control_status_u _control_status_prev {};
        innovation_fault_status_u _innovation_fault_status {};
        covariance_fault_status_u _covariance_fault_status {};
        fault_status_u _fault_status {};

    };

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::update() {
        static uint64_t count = 0;
        count++;

        _control_status_prev.value = _control_status.value;

        if (!_rough_alignment_completed) {
            _rough_alignment_completed = rough_alignment();

            if (!_rough_alignment_completed) {
                return;
            }
        }

        if (_imu_updated) { // imu更新了同时意味着imu_buffer非空
            if (_imu_buffer.is_full()) {    // 只有当imu_buffer满了之后才运行eskf, 以保证永远在滞后的时刻进行融合
//                std::cout << count << std::endl;

                // 取最滞后的imu数据
                const ImuSample &imu_sample = _imu_buffer.oldest();
                _imu_sample_delay = imu_sample;

                // 判断eskf中哪些状态需要禁止使用
                const float alpha = math::constrain((_imu_sample_delay.delta_vel_dt / _params.acc_bias_inhibit_tc), 0.0f, 1.0f);
                const float beta = 1.0f - alpha;
                const float dt_inv = 1.f / _imu_sample_delay.delta_vel_dt;
                _ang_rate_magnitude_filt = fmaxf(dt_inv * _imu_sample_delay.delta_ang.norm(), beta * _ang_rate_magnitude_filt);
                _accel_magnitude_filt = fmaxf(dt_inv * _imu_sample_delay.delta_vel.norm(), beta * _accel_magnitude_filt);
                _accel_vec_filt = alpha * dt_inv * _imu_sample_delay.delta_vel + beta * _accel_vec_filt;

                _is_manoeuvre_level_high = _ang_rate_magnitude_filt > _params.acc_bias_inhibit_gyr_lim_max
                                           || _accel_magnitude_filt > _params.acc_bias_inhibit_acc_lim_max;
                _is_manoeuvre_level_low = _ang_rate_magnitude_filt < _params.acc_bias_inhibit_gyr_lim_min
                                          && _accel_magnitude_filt < _params.acc_bias_inhibit_acc_lim_min;

                if (_is_manoeuvre_level_high || _is_manoeuvre_level_low) {
                    _eskf.disable_estimation_acc_bias();
                }

//                std::cout << "step 1" << std::endl;

                // 在滞后时刻进行先验估计
                _eskf.predict_state(imu_sample);
                _eskf.predict_covariance(imu_sample);

//                std::cout << "step 2" << std::endl;

                // 对GPS数据进行处理与校验
                for (uint8_t i = 0; i < N_GPS; ++i) {
//                    std::cout << "_gps_buffer[" << int(i) << "].size() = " << int(_gps_buffer[i].size()) << std::endl;
                    _time_prev_gps_us[i] = _gps_sample_delay[i].time_us;
                    _gps_state.intermittent[i] = !is_recent(_gps_buffer[i].newest().time_us, RunnerParameters::GPS_MAX_INTERVAL);
//                    std::cout << "_gps_buffer[i].newest().time_us = " << _gps_buffer[i].newest().time_us << std::endl;
                    _gps_state.data_ready[i] = _gps_buffer[i].pop_first_older_than(imu_sample.time_us, _gps_sample_delay[i]);
                    if (_gps_state.data_ready[i]) {
                        _gps_state.offset_nav[i] = _eskf._Rnb * _params.gps_pos_body[i];
                        _gps_state.w_cross_offset_body[i] = _eskf._gyro_corr % _params.gps_pos_body[i];
                        _gps_state.w_cross_offset_nav[i] = _eskf._Rnb * _gps_state.w_cross_offset_body[i];

                        _gps_state.pos_horz_imu[i] = _gps_sample_delay[i].pos_horz - _gps_state.offset_nav[i].xy();
                        _gps_state.hgt_imu[i] = _gps_sample_delay[i].hgt + _gps_state.offset_nav[i](2);
                        _gps_state.vel_imu[i] = _gps_sample_delay[i].vel - _gps_state.w_cross_offset_nav[i];

//                        std::cout << _gps_sample_delay[i].pos_horz(0) << ", " << _gps_sample_delay[i].pos_horz(1) << ", " << _gps_sample_delay[i].hgt << std::endl;

                        // 在should_collect_gps()中计算_gps_state.checks_passed
//                        _gps_state.checks_passed[i] = _gps_sample_delay[i].fix_type >= 3;
                    } else {
                        // TODO: 也许还有别的处理
                    }

//                    std::cout << "gps " << int(i) << ", _data_ready = " << _gps_state._data_ready[i] << std::endl;
//                    std::cout << "imu_time = " << imu_sample.time_us << ", gps_time = " << _gps_sample_delay[i].time_us << std::endl;
                }

                // 对气压计数据进行处理与校验
                _time_prev_baro_us = _baro_sample_delay.time_us;
                _baro_state.intermittent = !is_recent(_baro_buffer.newest().time_us, RunnerParameters::BARO_MAX_INTERVAL);
                _baro_state.data_ready = _baro_buffer.pop_first_older_than(imu_sample.time_us, _baro_sample_delay);
                if (_baro_state.data_ready) {
                    //TODO: baro的采样间隔(_baro_sample_delay.time_us - _time_prev_baro_us)需要用于气压计偏移估计
                    _baro_state.offset_nav = _eskf._Rnb * _params.baro_pos_body;

                    // TODO: fault的判断
                    _baro_state.faulty = false;
                }
//                std::cout << "baro, _data_ready = " << _baro_state._data_ready << std::endl;

                // 对测距仪数据进行处理与校验
                _time_prev_range_us = _range_sample_delay.time_us;
                _range_state.intermittent = !is_recent(_range_buffer.newest().time_us, RunnerParameters::RANGE_MAX_INTERVAL);
                _range_state.data_ready = _range_buffer.pop_first_older_than(imu_sample.time_us, _range_sample_delay);
                if (_range_state.data_ready) {
                    // TODO: 判断机身角度
                    _range_state.offset_nav = _eskf._Rnb * _params.range_pos_body;

                    // TODO: 进行healthy的判断
                    _range_state.healthy = true;
                }
//                std::cout << "range, _data_ready = " << _range_state._data_ready << std::endl;

                // 对光流数据进行处理与校验
                _time_prev_flow_us = _flow_sample_delay.time_us;
                _flow_state.intermittent = !is_recent(_flow_buffer.newest().time_us, RunnerParameters::FLOW_MAX_INTERVAL);
                _flow_state.data_ready = _flow_buffer.pop_first_older_than(imu_sample.time_us, _flow_sample_delay);
                if (_flow_state.data_ready) {
//                    _control_status.flags.opt_flow = !_flow_intermittent && _flow_check_passed;
                } else {
//                    _control_status.flags.opt_flow = false;
                }
//                std::cout << "opt-flow, _data_ready = " << _flow_state._data_ready << std::endl;

                // 对外部视觉数据进行处理与校验
                _time_prev_ev_us = _ev_sample_delay.time_us;
                _ev_intermittent = !is_recent(_ev_buffer.newest().time_us, RunnerParameters::EV_MAX_INTERVAL);
                _ev_data_ready = _ev_buffer.pop_first_older_than(imu_sample.time_us, _ev_sample_delay);
                if (_ev_data_ready) {
                    // TODO: something;
//                    _control_status.flags.ev
                }
//                std::cout << "ev, _data_ready = " << _ev_data_ready << std::endl;

                // 对磁力计数据进行处理与校验
                _time_prev_mag_us = _mag_sample_delay.time_us;
                _mag_intermittent = !is_recent(_mag_buffer.newest().time_us, RunnerParameters::MAG_MAX_INTERVAL);
                _mag_data_ready = _mag_buffer.pop_first_older_than(imu_sample.time_us, _mag_sample_delay);
                if (_mag_data_ready) {
                    // TODO: something;
                }
//                std::cout << "mag, _data_ready = " << _mag_data_ready << std::endl;

//                std::cout << "step 3" << std::endl;

                control_mag_fuse();
                control_height_fusion();
                control_optical_flow_fusion();
                control_gps_except_hgt_fusion();

//                std::cout << "step 4" << std::endl;

                // 修正滞后时刻的状态与协方差
                _eskf.correct_state();
                // _eskf.correct_covariance();

//                std::cout << "step 5" << std::endl;
            }

//            // 利用滞后时刻的后验估计值和当前时刻的imu值预测当前时刻状态
//            predict_state_from_delay(_imu_buffer.newest());

            _imu_updated = false;
        }
    }

    template<uint8_t DELAYS>
    bool ESKFRunner<DELAYS>::rough_alignment() {
        auto initialise_tilt = [&](const Vector3f &gyro, const Vector3f &acc) -> bool {
            const float accel_norm = acc.norm();
            const float gyro_norm = gyro.norm();

            if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
                accel_norm > 1.2f * CONSTANTS_ONE_G ||
                gyro_norm > math::radians(15.0f)) {
                return false;
            }

            /*计算四元数:
             * ez = (0, 0, 1)
             * ez_body = (x, y, z)
             * ez_body * ez = z
             * ez_body x ez = (y, -x, 0)
             * */
            const Vector3f ez_body = -acc.normalized();
            Quatf q(1.f + ez_body(2), ez_body(1), -ez_body(0), 0.f);
            q.normalize();
            _eskf.set_attitude(q);

            _control_status.flags.tilt_align = true;

//            Eulerf euler(q);
//            std::cout << "acc: " << acc(0) << ", " << acc(1) << ", " << acc(2) << std::endl;
//            std::cout << "ez_body" << ez_body(0) << ", " << ez_body(1) << ", " << ez_body(2) << std::endl;
//            std::cout << "rough euler: " << euler(0) << ", " << euler(1) << ", " << euler(2) << std::endl;
//            assert(-1 > 0);

            return true;
        };

        auto initialise_head = [&](const Vector3f &mag_body) {
            Vector3f mag_est_unit = _eskf._Rnb * mag_body;
            mag_est_unit.normalize();

            Vector3f mag_true_unit = get_mag_earth();
            mag_true_unit.normalize();

            float norm2 = sq(mag_est_unit(0)) + sq(mag_est_unit(1));
            float sin_psi = mag_true_unit(1) * mag_est_unit(0) - mag_true_unit(0) * mag_est_unit(1);
            float cos_psi = mag_true_unit(0) * mag_est_unit(0) + mag_true_unit(1) * mag_est_unit(1);
            Quatf q(norm2 + cos_psi, 0.f, 0.f, sin_psi);
            q.normalize();

            q = q * _eskf._state.quat_nominal;
            q.normalize();
            _eskf.set_attitude(q);

            _control_status.flags.yaw_align = true;

//            float psi = atan2f(sin_psi, cos_psi);
//            std::cout << "head: " << psi << std::endl;
//            assert(-1 > 0);
        };

        if (_imu_updated) {
            const ImuSample &imu_sample = _imu_buffer.newest();

            Vector3f gyro = imu_sample.delta_ang / imu_sample.delta_ang_dt;
            Vector3f acc = imu_sample.delta_vel / imu_sample.delta_vel_dt;
            if (_imu_counter) {
                gyro = _gyro_lpf(gyro);
                acc = _accel_lpf(acc);
            } else {
                _gyro_lpf.reset_filter_state_by_output(gyro);
                _accel_lpf.reset_filter_state_by_output(gyro);
            }
            ++_imu_counter;

            MagSample mag_sample;
            if (_mag_buffer.pop_first_older_than(imu_sample.time_us, mag_sample)) {
                if (_mag_body_lpf_counter) {
                    _mag_body_lp = _mag_body_lpf(mag_sample.mag);
                } else {
                    _mag_body_lpf.reset_filter_state_by_output(mag_sample.mag);
                    _mag_body_lp = mag_sample.mag;
                }
                ++_mag_body_lpf_counter;
            }

            BaroSample baro_sample;
            if (_baro_buffer.pop_first_older_than(imu_sample.time_us, baro_sample)) {
                if (_baro_counter) {
                    _eskf._baro_bias = _baro_lpf(baro_sample.hgt);
                } else {
                    _baro_lpf.reset_filter_state_by_output(baro_sample.hgt);
                    _eskf._baro_bias = baro_sample.hgt;
                }
                std::cout << _eskf._baro_bias << std::endl;
                ++_baro_counter;
            }

            if (_params.mag_body_fusion_type <= RunnerParameters::MAG_FUSE_TYPE_3D) {
                if (_mag_body_lpf_counter < DELAYS || _mag_body_lpf_counter < 100) {
                    // not enough mag samples accumulated
                    return false;
                }
            }

            if (_baro_counter < DELAYS || _baro_counter < 100) {
                return false;
            }

            if (!initialise_tilt(gyro, acc)) {
                return false;
            }

            if (_params.mag_body_fusion_type <= RunnerParameters::MAG_FUSE_TYPE_3D) {
                initialise_head(_mag_body_lp);
            }

//            _eskf.initialize();

            _time_last_pos_vert_fuse = _imu_sample_last.time_us;
            _time_last_pos_horz_fuse = _imu_sample_last.time_us;
            _time_last_vel_vert_fuse = _imu_sample_last.time_us;
            _time_last_vel_horz_fuse = _imu_sample_last.time_us;
            _time_last_pos_vert_fuse_attempt = _imu_sample_last.time_us;
            _time_last_pos_horz_fuse_attempt = _imu_sample_last.time_us;

            return true;
        } else {
            return false;
        }
    }


    /*!
     * 判断垂直加速度是否健康
     * 检测状态存储于: _fault_status.flags.bad_acc_vertical, _fault_status.flags.bad_acc_clipping,
     *              _time_good_vert_accel, _time_bad_vert_accel
     *
     * _fault_status.flags.bad_acc_vertical = true 最近的1s内, 有50%的时间, 加速度在x/y/z轴出出现过clipping
     * _fault_status.flags.bad_acc_clipping = true 最近的BADACC_PROBATION(us)内, 出现过"垂直加速度异常"
     *
     * "垂直加速度异常": 必要条件为 eskf的高度或垂直速度出现严重下降的现象
     *                若高度和速度不同源, 则认为 垂直加速度异常
     *                若高度和速度同源, 且在1s内出现过加速度计clipp(x/y/z轴, 任意一轴出现都算), 则认为 垂直加速度异常
     *
     * _time_bad_vert_accel: 最近一次"垂直加速度异常"的时间
     * _time_good_vert_accel: 最近一次不"垂直加速度异常"的时间
     * @tparam DELAYS
     */
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::check_vertical_acceleration_health() {
        bool is_inertial_nav_falling = false;
        bool are_vertical_pos_and_vel_independent = false;

        if (is_recent(_time_last_pos_vert_fuse_attempt, 1000000)) {   // 1秒内是否尝试过进行垂直位置融合(不一定真的融合了, 可能数据rejected了)
            if (is_recent(_time_last_vel_vert_fuse, 1000000)) {    // 1秒内是否进行垂直速度融合
                // 如果垂直位置和垂直速度用的不是同样的传感器, 则只要falling, 则一定认为垂直加速度异常
                const bool using_gps_for_both = _control_status.flags.gps_hgt && _control_status.flags.gps_vel;
                const bool using_ev_for_both = _control_status.flags.ev_hgt && _control_status.flags.ev_vel;
                are_vertical_pos_and_vel_independent = !(using_gps_for_both || using_ev_for_both);

                if (_control_status.flags.gps_hgt) {
                    if (_control_status.flags.gps_vel) {
                        for (uint8_t i = 0; i < N_GPS; ++i) {
                            is_inertial_nav_falling |= _gps_pos_vert_fuse_data[i].innov_ratio(0) < -_params.vert_innov_test_lim
                                                       && _gps_vel_vert_fuse_data[i].innov_ratio(0) < 0.f;
                            is_inertial_nav_falling |= _gps_vel_vert_fuse_data[i].innov_ratio(0) < -_params.vert_innov_test_lim
                                                       && _gps_pos_vert_fuse_data[i].innov_ratio(0) < 0.f;
                        }
                    } else if (_control_status.flags.ev_vel){
                        are_vertical_pos_and_vel_independent = true;
                        for (uint8_t i = 0; i < N_GPS; ++i) {
                            is_inertial_nav_falling |= _gps_pos_vert_fuse_data[i].innov_ratio(0) < -_params.vert_innov_test_lim
                                                       && _ev_vel_vert_fuse_data.innov_ratio(0) < 0.f;
                            is_inertial_nav_falling |= _ev_vel_vert_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim
                                                       && _gps_pos_vert_fuse_data[i].innov_ratio(0) < 0.f;
                        }
                    }
                } else if (_control_status.flags.baro_hgt) {
                    if (_control_status.flags.gps_vel) {
                        are_vertical_pos_and_vel_independent = true;
                        for (uint8_t i = 0; i < N_GPS; ++i) {
                            is_inertial_nav_falling |= _baro_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim
                                                       && _gps_vel_vert_fuse_data[i].innov_ratio(0) < 0.f;
                            is_inertial_nav_falling |= _gps_vel_vert_fuse_data[i].innov_ratio(0) < -_params.vert_innov_test_lim
                                                       && _baro_fuse_data.innov_ratio(0) < 0.f;
                        }
                    } else if (_control_status.flags.ev_vel){
                        are_vertical_pos_and_vel_independent = true;
                        is_inertial_nav_falling |= _baro_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim
                                                   && _ev_vel_vert_fuse_data.innov_ratio(0) < 0.f;
                        is_inertial_nav_falling |= _ev_vel_vert_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim
                                                   && _baro_fuse_data.innov_ratio(0) < 0.f;
                    }
                } else if (_control_status.flags.rng_hgt) {
                    if (_control_status.flags.gps_vel) {
                        are_vertical_pos_and_vel_independent = true;
                        for (uint8_t i = 0; i < N_GPS; ++i) {
                            is_inertial_nav_falling |= _baro_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim
                                                       && _gps_vel_vert_fuse_data[i].innov_ratio(0) < 0.f;
                            is_inertial_nav_falling |= _gps_vel_vert_fuse_data[i].innov_ratio(0) < -_params.vert_innov_test_lim
                                                       && _baro_fuse_data.innov_ratio(0) < 0.f;
                        }
                    } else if (_control_status.flags.ev_vel){
                        are_vertical_pos_and_vel_independent = true;
                        is_inertial_nav_falling |= _baro_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim
                                                   && _ev_vel_vert_fuse_data.innov_ratio(0) < 0.f;
                        is_inertial_nav_falling |= _ev_vel_vert_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim
                                                   && _baro_fuse_data.innov_ratio(0) < 0.f;
                    }
                } else if (_control_status.flags.ev_hgt) {
                    if (_control_status.flags.gps_vel) {
                        are_vertical_pos_and_vel_independent = true;
                        for (uint8_t i = 0; i < N_GPS; ++i) {
                            is_inertial_nav_falling |= _ev_pos_vert_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim
                                                       && _gps_vel_vert_fuse_data[i].innov_ratio(0) < 0.f;
                            is_inertial_nav_falling |= _gps_vel_vert_fuse_data[i].innov_ratio(0) < -_params.vert_innov_test_lim
                                                       && _ev_pos_vert_fuse_data.innov_ratio(0) < 0.f;
                        }
                    } else if (_control_status.flags.ev_vel) {
                        is_inertial_nav_falling |= _ev_pos_vert_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim
                                                   && _ev_vel_vert_fuse_data.innov_ratio(0) < 0.f;
                        is_inertial_nav_falling |= _ev_vel_vert_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim
                                                   && _ev_pos_vert_fuse_data.innov_ratio(0) < 0.f;
                    }
                }

            } else {    // 只有高度传感器
                if (_control_status.flags.gps_hgt) {
                    for (uint8_t i = 0; i < N_GPS; ++i) {
                        is_inertial_nav_falling |= _gps_pos_vert_fuse_data[i].innov_ratio(0) < -_params.vert_innov_test_lim;
                    }
                } else if (_control_status.flags.baro_hgt) {
                    is_inertial_nav_falling |= _baro_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim;
                } else if (_control_status.flags.rng_hgt) {
                    is_inertial_nav_falling |= _range_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim;
                } else if (_control_status.flags.ev_hgt) {
                    is_inertial_nav_falling |= _ev_pos_vert_fuse_data.innov_ratio(0) < -_params.vert_innov_test_lim;
                }
            }
        }

        // 检查过去的1秒内, 是否有50%的时间加速度计都处于限幅
        const uint16_t clip_count_limit = 1.f / _eskf._dt;
        const bool is_clipping = _imu_sample_delay.delta_vel_clipping[0] || _imu_sample_delay.delta_vel_clipping[1] || _imu_sample_delay.delta_vel_clipping[2];

        if (is_clipping && _clip_counter < clip_count_limit) {
            ++_clip_counter;
        } else if (_clip_counter > 0) {
            --_clip_counter;
        }
        // 如果过去的1秒内, 有50%的时间加速度计都处于限幅, 则认为加速度计处于clipping状态
        _fault_status.flags.bad_acc_clipping = _clip_counter > clip_count_limit / 2;

        // 过去的1秒内, 是否出现过一次加速度计clipping
        const bool is_clipping_frequently = _clip_counter > 0;

        /*
         * 1. 如果垂直位置和速度融合所用的传感器是不同的, 则只要"eskf的状态处于下降状态"就一定认为是垂直加速度异常
         * 2. 如果垂直位置和速度融合所用的传感器是同源的, 则需要“过去的1秒内出现过clipping”且“eskf的状态处于下降状态”才能认为是垂直加速度异常
         * */
        const bool bad_vert_accel = (are_vertical_pos_and_vel_independent || is_clipping_frequently) && is_inertial_nav_falling;
//        std::cout << "is_inertial_nav_falling = " << is_inertial_nav_falling << std::endl;
//        std::cout << "are_vertical_pos_and_vel_independent = " << are_vertical_pos_and_vel_independent << std::endl;
//        std::cout << "is_clipping_frequently = " << is_clipping_frequently << std::endl;
//        std::cout << "bad_vert_accel = " << bad_vert_accel << std::endl;

        if (bad_vert_accel) {
            _time_bad_vert_accel = _imu_sample_delay.time_us;
        } else {
            _time_good_vert_accel = _imu_sample_delay.time_us;
        }

        // 在最近BADACC_PROBATION(us)时间内, 均没有出现过垂直加速度异常, 才认为垂直加速度正常, 否则是为异常
//        _fault_status.flags.bad_acc_vertical = is_recent(_time_bad_vert_accel, RunnerParameters::BADACC_PROBATION);
        if (_fault_status.flags.bad_acc_vertical) {
            _fault_status.flags.bad_acc_vertical = is_recent(_time_bad_vert_accel, RunnerParameters::BADACC_PROBATION);
        } else {
            _fault_status.flags.bad_acc_vertical = bad_vert_accel;
        }
//        std::cout << "imu_sample_last.time_us = " << _imu_sample_last.time_us << std::endl;
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::control_height_fusion() {
        /*!
         * 检查高度传感器是否正常
         * 只要当前时刻的传感器异常, 则放弃所有历史数据, 清空相应的buffer, 并把*_data_ready标志为false
         */
        auto check_hgt_sensor_available = [&]() {
            _baro_state.available = true;
            _range_state.available = true;
            for (uint8_t i = 0; i < N_GPS; ++i) {
                _gps_state.hgt_available[i] = true;
                _gps_state.rtk_hgt_available[i] = false;
            }

            _gps_state.any_hgt_available = false;
            _gps_state.any_rtk_hgt_available = false;

            /*
             * 如果当前高度传感器数据长时间没有进行融合, 并且垂直加速度正常, 则认为当前高度传感器生发了故障.
             * 若垂直加速度异常, 则必须重置eskf中的垂直位置, 重置操作在 switch_to_*() 中.
             */
            if (!_fault_status.flags.bad_acc_vertical) {
                if (_control_status.flags.gps_hgt) {
                    for (uint8_t i = 0; i < N_GPS; ++i) {
                        _gps_state.hgt_available[i] = is_recent(_gps_state.time_last_hgt_fuse[i], RunnerParameters::HGT_FUSE_TIMEOUT);
                    }
                } else if (_control_status.flags.baro_hgt) {
                    _baro_state.available = is_recent(_baro_state.time_last_fuse, RunnerParameters::HGT_FUSE_TIMEOUT);
                }  else if (_control_status.flags.rng_hgt) {
                    _range_state.available = is_recent(_range_state.time_last_fuse, RunnerParameters::HGT_FUSE_TIMEOUT);
                } else if (_control_status.flags.ev_hgt) {
                    _range_state.available = is_recent(_range_state.time_last_fuse, RunnerParameters::HGT_FUSE_TIMEOUT);
                }
            }

            _baro_state.available = _baro_state.available && !_baro_state.intermittent && !_baro_state.faulty;
            if (!_baro_state.available) {
                _baro_buffer.clear();
                _baro_state.data_ready = false;
            }

            _range_state.available = _range_state.available && !_range_state.intermittent && _range_state.healthy;
            if (!_range_state.available) {
                _range_buffer.clear();
                _range_state.data_ready = false;
            }

            for (uint8_t i = 0; i < N_GPS; ++i) {
//                std::cout << "_gps_state.intermittent[i] = " << _gps_state.intermittent[i] << std::endl;
//                std::cout << "_gps_state.checks_passed[i] = " << _gps_state.checks_passed[i] << std::endl;
                _gps_state.hgt_available[i] = _gps_state.hgt_available[i] && (!_gps_state.intermittent[i] && _gps_state.checks_passed[i]);
                _gps_state.any_hgt_available = _gps_state.any_hgt_available || _gps_state.hgt_available[i];
                if (_gps_state.hgt_available[i]) {
                    if (_gps_state.data_ready[i]) {
                        _gps_state.rtk_hgt_available[i] = _gps_sample_delay[i].fix_type >= 5; // 只有这里可能使_rtk_hgt_available[i] = true
                        _gps_state.any_rtk_hgt_available = _gps_state.any_rtk_hgt_available || _gps_state.rtk_hgt_available[i];
                    }
                } else {
                    _gps_buffer[i].clear();
                    _gps_state.data_ready[i] = false;
                }

//                std::cout << _gps_state.intermittent[i] << std::endl;
            }
        };

        auto conservative_range = [&]() -> float {
            return _control_status.flags.in_air ? _range_sample_delay.rng : _params.rng_gnd_clearance;
        };

        /*!
         * 融合气压计偏移和地形高度, 以gps_hgt为参考
         * 若gps不可用, 但气压计可用, 则使用baro - baro_offset为参考
         * 理论上存在以下关系:
         *  -eskf.z = gps_hgt_imu = baro_imu - baro_offset = range_hgt_imu - terr
         *
         *  其中, gps_hgt_imu, baro_imu, baro_offset, range_hgt_imu向上为正
         *  而, eskf.z, terr向下为正
         */
        auto fuse_baro_bias_and_terrain = [&]() {
            // TODO: 是否应该根据垂直加速度的好坏来做一些处理?
            if (_baro_state.available && _control_status.flags.baro_hgt) {
                /*
                 * 此时, -eskf.z = baro_imu - baro_bias
                 * 即, baro_imu = baro_bias - eskf.z = baro + baro_offset_nav.z
                 * 所以, baro = baro_bias - eskf.z - baro_offset_nav
                 * */
                _eskf.set_baro_imu(_eskf._baro_bias - _eskf._state.pos(2));
                for (uint8_t i = 0; i < N_GPS; ++i) {
//                    std::cout << _gps_state.hgt_available[i] << std::endl;
                    if (_gps_state.hgt_available[i] && _gps_state.data_ready[i]) {
                        _eskf.correct_baro_bias(_gps_state.hgt_imu[i]);
                    }
                }
                if (_range_state.available && _range_state.data_ready) {
                    _eskf.calculate_dist_bottom_imu(conservative_range(), _range_offset_nav);
                    _eskf.correct_terrain(-_eskf._state.pos(2));
                    _terr_state.time_last_fuse = _imu_sample_delay.time_us;
                }
            } else if (_gps_state.any_hgt_available && _control_status.flags.gps_hgt) {
//                std::cout << "use gps to fuse terr and bias" << std::endl;
                /*
                 * 此时, -eskf.z = gps_hgt_imu
                 * */
                if (_baro_state.available && _baro_state.data_ready) {
                    _eskf.calculate_baro_imu(_baro_sample_delay.hgt, _baro_offset_nav);
                    _eskf.correct_baro_bias(-_eskf._state.pos(2));
                }
                if (_range_state.available && _range_state.data_ready) {
                    _eskf.calculate_dist_bottom_imu(conservative_range(), _range_offset_nav);
                    _eskf.correct_terrain(-_eskf._state.pos(2));
                    _terr_state.time_last_fuse = _imu_sample_delay.time_us;
                }
            } else if (_range_state.available && (_control_status.flags.rng_hgt || _control_status.flags.ev_hgt)) {
                /*
                 * 此时, -eskf.z = range_hgt_imu - terr
                 * 即, range_hgt_imu = terr - eskf.z = range_hgt + range_offset_nav.z
                 * 所以, range_hgt = -terr - eskf.z - range_offset_nav
                 * */
                _eskf.set_dist_bottom_imu(_eskf._terrain - _eskf._state.pos(2));
                for (uint8_t i = 0; i < N_GPS; ++i) {
                    if (_gps_state.hgt_available[i] && _gps_state.data_ready[i]) {
                        _eskf.correct_terrain(_gps_state.hgt_imu[i]);
                    }
                }
                if (_baro_state.available && _baro_state.data_ready) {
                    _eskf.calculate_baro_imu(_baro_sample_delay.hgt, _baro_offset_nav);
                    _eskf.correct_baro_bias(-_eskf._state.pos(2));
                }
            } else {
                if (_baro_state.available && _baro_state.data_ready) {
                    _eskf.calculate_baro_imu(_baro_sample_delay.hgt, _baro_offset_nav);
                }
                if (_range_state.available && _range_state.data_ready) {
                    _eskf.calculate_dist_bottom_imu(conservative_range(), _range_offset_nav);
                }
            }
        };

        /*
         * 使用gps来融合高度
         * 若之前不是使用gps或垂直加速度异常, 则强制切换到gps融合模式, 并重置eskf高度状态
         * 否则, 直接使用gps数据进行融合
         */
        auto switch_to_gps_fuse = [&]() {
            if (!_control_status.flags.gps_hgt || _fault_status.flags.bad_acc_vertical) {
                // TODO: prior根据gps精度改变
                float prior = 1.f;
                float sum_gps_hgt_imu = 0.f;
                float sum_prior = 0.f;
                for (uint8_t i = 0; i < N_GPS; ++i) {
                    if (_gps_state.hgt_available[i] && _gps_state.data_ready[i]) {
                        sum_gps_hgt_imu += prior * _gps_sample_delay[i].hgt;
                        sum_prior += prior;

                        _gps_state.time_last_hgt_fuse[i] = _imu_sample_delay.time_us;
                    }
                }
                if (sum_prior > 0.f) {
                    _control_status.flags.baro_hgt = false;
                    _control_status.flags.gps_hgt = true;
                    _control_status.flags.rng_hgt = false;
                    _control_status.flags.ev_hgt = false;

                    _eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
                    _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));

                    _time_last_pos_vert_fuse_attempt = _imu_sample_delay.time_us;
                    _time_last_pos_vert_fuse = _imu_sample_delay.time_us;

//                    std::cout << "reset hgt to gps" << std::endl;
                }

            } else {
                for (uint8_t i = 0; i < N_GPS; ++i) {
                    if (_gps_state.hgt_available[i] && _gps_state.data_ready[i]) {
                        _time_last_pos_vert_fuse_attempt = _imu_sample_delay.time_us;

                        // TODO: 使用_gps_sample_delay中的精度因子替换_eskf._params.gps_pos_vert_noise, 并且noise根据rtk改变
                        uint8_t info = _eskf.fuse_pos_vert(-_gps_sample_delay[i].hgt, _params.gps_pos_body[i], _gps_offset_nav[i],
                                                           _eskf._params.gps_pos_vert_innov_gate, _eskf._params.gps_pos_vert_noise, _gps_pos_vert_fuse_data[i]);

                        if (info) {
                            if (info & (uint8_t)0x00000001b) {
                                _innovation_fault_status.flags.reject_gps_pos_z |= (uint8_t)((uint8_t)0x00000001b << i);

                                std::cout << "reject gps[" << int(i) << "]" << std::endl;
                            }
                            if (info & (uint8_t)0x00000010b) {
                                _covariance_fault_status.flags.unhealthy_gps_pos_z |= (uint8_t)((uint8_t)0x00000001b << i);
                            }
                        } else {
                            _time_last_pos_vert_fuse = _imu_sample_delay.time_us;
                            _gps_state.time_last_hgt_fuse[i] = _imu_sample_delay.time_us;

//                            std::cout << "fuse hgt by gps: " << -_gps_sample_delay[i].hgt << std::endl;
                        }
                    }
                }
            }
        };

        /*
         * 使用baro来融合高度
         * 若之前不是使用baro或垂直加速度异常, 则强制切换到baro融合模式, 并重置eskf高度状态
         * 否则, 直接使用baro数据进行融合
         */
        auto switch_to_baro_fuse = [&]() {
//            std::cout << "_control_status.flags.baro_hgt = " << _control_status.flags.baro_hgt << std::endl;
//            std::cout << "_fault_status.flags.bad_acc_vertical = " << _fault_status.flags.bad_acc_vertical << std::endl;
            if (!_control_status.flags.baro_hgt || _fault_status.flags.bad_acc_vertical) {
                if (_baro_state.data_ready) {
//                    std::cout << "reset baro" << std::endl;
                    _control_status.flags.baro_hgt = true;
                    _control_status.flags.gps_hgt = false;
                    _control_status.flags.rng_hgt = false;
                    _control_status.flags.ev_hgt = false;

                    _eskf.set_pos_vert(_eskf._baro_bias - _eskf._baro_imu);
                    _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.baro_noise));

                    _time_last_pos_vert_fuse_attempt = _imu_sample_delay.time_us;
                    _time_last_pos_vert_fuse = _imu_sample_delay.time_us;
                    _baro_state.time_last_fuse = _imu_sample_delay.time_us;
                }
            } else if (_baro_state.data_ready){
//                std::cout << "fuse baro: " << _baro_sample_delay.hgt << ", " << _eskf._state.pos(2) << std::endl;
                _time_last_pos_vert_fuse_attempt = _imu_sample_delay.time_us;

                uint8_t info = _eskf.fuse_pos_vert(_eskf._baro_bias - _baro_sample_delay.hgt, _params.baro_pos_body, _baro_offset_nav,
                                                   _eskf._params.baro_innov_gate, _eskf._params.baro_noise, _baro_fuse_data);

                if (info) {
                    if (info & (uint8_t)0x00000001b) {
                        _innovation_fault_status.flags.reject_baro = true;
                    }
                    if (info & (uint8_t)0x00000010b) {
                        _covariance_fault_status.flags.unhealthy_baro = true;
                    }
                } else {
                    _time_last_pos_vert_fuse = _imu_sample_delay.time_us;
                    _baro_state.time_last_fuse = _imu_sample_delay.time_us;
                }
            }
        };

        /*
         * 使用range来融合高度
         * 若之前不是使用range或垂直加速度异常, 则强制切换到range融合模式, 并重置eskf高度状态
         * 否则, 直接使用range数据进行融合
         */
        auto switch_to_range_fuse = [&]() {
            if (!_control_status.flags.rng_hgt || _fault_status.flags.bad_acc_vertical) {
                if (_range_state.data_ready) {
                    _control_status.flags.baro_hgt = false;
                    _control_status.flags.gps_hgt = false;
                    _control_status.flags.rng_hgt = true;
                    _control_status.flags.ev_hgt = false;

                    _eskf.set_pos_vert(_eskf._terrain - _eskf._dist_bottom_imu);
                    _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.range_noise));

                    _time_last_pos_vert_fuse_attempt = _imu_sample_delay.time_us;
                    _time_last_pos_vert_fuse = _imu_sample_delay.time_us;
                    _range_state.time_last_fuse = _imu_sample_delay.time_us;
                }
            } else if (_range_state.data_ready) {
                _time_last_pos_vert_fuse_attempt = _imu_sample_delay.time_us;

                uint8_t info = _eskf.fuse_range(conservative_range(), _params.range_pos_body, _range_offset_nav,
                                                _eskf._params.range_innov_gate, _eskf._params.range_noise, _range_fuse_data);

                if (info) {
                    if (info & (uint8_t)0x00000001b) {
                        _innovation_fault_status.flags.reject_range = true;
                    }
                    if (info & (uint8_t)0x00000010b) {
                        _covariance_fault_status.flags.unhealthy_range = true;
                    }
                } else {
                    _time_last_pos_vert_fuse = _imu_sample_delay.time_us;
                    _range_state.time_last_fuse = _imu_sample_delay.time_us;
                }
            }
        };

        /*!
         * 没有传感器可用于融合
         */
        auto switch_to_none = [&]() {
            _control_status.flags.baro_hgt = false;
            _control_status.flags.gps_hgt = false;
            _control_status.flags.rng_hgt = false;
            _control_status.flags.ev_hgt = false;
            _control_status.flags.none_hgt = true;
        };

        /*!
         * 选择最优的高度传感器来进行高度融合
         * 优先级为: RTK > Baro > GPS > Range
         */
        auto switch_to_preferred_senor_fuse = [&] {
            if (_gps_state.any_rtk_hgt_available) {
//                std::cout << "_gps_state.any_rtk_hgt_available: switch_to_gps_fuse" << std::endl;
                switch_to_gps_fuse();
            } else if (_baro_state.available) {
//                std::cout << "_baro_state.available: switch_to_baro_fuse" << std::endl;
                switch_to_baro_fuse();
            } else if (_gps_state.any_hgt_available) {
//                std::cout << "_gps_state.any_hgt_available: switch_to_gps_fuse" << std::endl;
                switch_to_gps_fuse();
            } else if (_range_state.available) {
//                std::cout << "_range_state.available: switch_to_range_fuse" << std::endl;
                switch_to_range_fuse();
            }
        };

        /*!
         * 融合高度
         */
        auto fuse_hgt = [&]() {
            switch (_params.hgt_sensor_type) {
                default:
                    // 默认为自动选择传感器
                case RunnerParameters::HGT_SENSOR_AUTO:
                    switch_to_preferred_senor_fuse();
                    break;
                case RunnerParameters::HGT_SENSOR_BARO:
                    if (_baro_state.available) {
                        switch_to_baro_fuse();
                    } else {
                        switch_to_preferred_senor_fuse();
                    }
                    break;
                case RunnerParameters::HGT_SENSOR_GPS:
                    if (_gps_state.any_hgt_available) {
                        switch_to_gps_fuse();
                    } else {
                        switch_to_preferred_senor_fuse();
                    }
                    break;
                case RunnerParameters::HGT_SENSOR_EV:
                case RunnerParameters::HGT_SENSOR_RANGE:
                    if (_range_state.available) {
                        switch_to_range_fuse();
                    } else {
                        switch_to_preferred_senor_fuse();
                    }
                    break;
            }
        };

        check_vertical_acceleration_health();
        check_hgt_sensor_available();
        fuse_baro_bias_and_terrain();
        fuse_hgt();
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::control_optical_flow_fusion() {
        /*!
         * 检查未起飞前, 飞机的动态对于光流融合来说是否合适
         */
        auto update_on_ground_motion_for_optical_flow_checks = [&]() {
            if (_control_status.flags.in_air) {
                _flow_state.time_bad_motion_us = 0;
                _flow_state.time_good_motion_us = _imu_sample_delay.time_us;
            } else {
                // 在地面时，检查车辆是否被摇晃或移动，因为这种情况可能导致导航出错
                const float accel_norm = _accel_vec_filt.norm();

                const bool motion_is_excessive = (accel_norm > (CONSTANTS_ONE_G * 1.5f)) // 最大重力加速度
                                                 || (accel_norm < (CONSTANTS_ONE_G * 0.5f)) // 最小重力加速度
                                                 || (_ang_rate_magnitude_filt > _params.flow_max_rate) // 光流能接受的最大角速度
                                                 || (_eskf._Rnb(2, 2) < cosf(math::radians(30.0f))); // 倾角可接受

                if (motion_is_excessive) {
                    _flow_state.time_bad_motion_us = _imu_sample_delay.time_us;

                } else {
                    _flow_state.time_good_motion_us = _imu_sample_delay.time_us;
                }
            }
        };

        /*!
         * 利用角速度的累积值, 计算平均角速度
         */
        auto calc_opt_flow_body_rate_comp = [&]() -> bool {
            // 如果imu积分时间过长, 则imu积分数据不可用
            if (_flow_state.imu_delta_time > 1.0f) {
                return false;
            }

            if ((_flow_state.imu_delta_time > FLT_EPSILON)
                && (_flow_sample_delay.dt > FLT_EPSILON)) {
                _flow_sample_delay.gyro_xyz = -_flow_state.imu_delta_ang / _flow_state.imu_delta_time * _flow_sample_delay.dt;
                return true;
            }
            return false;
        };

        /*!
         * 利用光流测得的视轴速度与imu平均角速度, 计算补偿后的视轴速度, 用于计算飞机速度在机体系的投影
         */
        auto calc_opt_flow_compensated_xy_rad = [&]() -> bool {
            // TODO: 在累积时应该考虑姿态变化带来的影响
            // 累积imu数据
            _flow_state.imu_delta_ang += _eskf._delta_ang_corr;
            _flow_state.imu_delta_time += _imu_sample_delay.delta_ang_dt;

            if (_flow_state.data_ready) {
                const bool is_quality_good = (_flow_sample_delay.quality >= _params.flow_qual_min);
                const bool is_magnitude_good = !_flow_sample_delay.flow_xy_rad.longerThan(
                        _flow_sample_delay.dt * _params.flow_max_rate);
                const bool is_tilt_good = (_eskf._Rnb(2, 2) > _params.range_cos_max_tilt);

                const float delta_time_min = fmaxf(0.7f * _flow_state.imu_delta_time, 0.001f);
                const float delta_time_max = fminf(1.3f * _flow_state.imu_delta_time, 0.2f);
                const bool is_delta_time_good = _flow_sample_delay.dt >= delta_time_min && _flow_sample_delay.dt <= delta_time_max;
                const bool is_body_rate_comp_available = calc_opt_flow_body_rate_comp();

                // 清空imu累积数据
                _flow_state.imu_delta_ang.setZero();
                _flow_state.imu_delta_time = 0.0f;

                if (is_quality_good
                    && is_magnitude_good
                    && is_tilt_good
                    && is_body_rate_comp_available
                    && is_delta_time_good) {
                    // 对机体的转动进行补偿
                    _flow_state.flow_compensated_xy_rad = _flow_sample_delay.flow_xy_rad - _flow_sample_delay.gyro_xyz.xy();
//                    std::cout << _flow_state.flow_compensated_xy_rad(0) << ", " << _flow_state.flow_compensated_xy_rad(1) << std::endl;
                    return true;
                }
            }
            return false;
        };

        /*!
         * 检测是否应该禁用光流融合
         * 在以下任意条件满足, 且除了光流外的所有水平辅助传感器均不可用时, 才允许进行光流融合:
         * 1. 飞机未起飞, 且机动较小
         * 2. 飞机已起飞, 且最近有进行过仿地融合
         */
        auto check_inhibit_flow_use = [&]() {
            // 如果已经开启了光流融合模式, 若只要gps融合误差大于0.7, 则认为需要光流融合
            // 如果之前没有开启光流融合模式, 若gps融合误差大于1, 则认为需要光流融合
            const float gps_err_norm_lim = _control_status.flags.opt_flow ? 0.7f : 1.0f;

            // 原则上, 能不使用光流就不使用光流, 在飞机机动较好且最近进行过仿地相关融合和航向已经对齐时, 满足以下条件之一, 才允许光流的使用
            // 1. 飞机处于纯惯导融合
            // 2. 在gps水平融合模式下eskf水平融合误差过大
            // 3. 只有光流能提供水平状态的观测
            bool const is_flow_required = _control_status.flags.inertial_dead_reckoning
                                          || is_only_active_source_of_horizontal_aiding(_control_status.flags.opt_flow)
                                          || (_control_status.flags.gps_horz && (_gps_horz_error_norm > gps_err_norm_lim));

            bool const is_range_data_used = is_recent(_terr_state.time_last_fuse, (uint64_t)10e6)
                                            || is_recent(_range_state.time_last_fuse, (uint64_t)10e6);

            _flow_state.inhibit_flow_use = !_control_status.flags.tilt_align || !is_flow_required || !is_range_data_used;
            _flow_state.inhibit_flow_use |= (_params.mag_body_fusion_type < RunnerParameters::MAG_FUSE_TYPE_NONE) && !_control_status.flags.yaw_align;

            // 若飞机机动较差, 则需要禁止光流的使用
            if (_control_status.flags.in_air) {
                const bool flight_condition_not_ok = _control_status.flags.in_air && !is_terrain_estimate_valid();
                _flow_state.inhibit_flow_use |= flight_condition_not_ok;
            } else {
                const bool preflight_motion_not_ok = !_control_status.flags.in_air
                                                     && (is_timeout(_flow_state.time_good_motion_us, (uint64_t)1E5)
                                                         || is_recent(_flow_state.time_bad_motion_us, (uint64_t)5E6));
                _flow_state.inhibit_flow_use |= preflight_motion_not_ok;
            }
        };

        /*!
         * 计算光流的量测噪声的标准差
         * 根据光流数据的质量来线性缩放量测噪声的标准差
         */
        auto calc_optical_flow_meas_std = [&]() ->float {
            const float std_best = fmaxf(_params.flow_noise, 0.05f);
            const float std_worst = fmaxf(_params.flow_noise_qual_min, 0.05f);

            float flow_qual_range = (255.0f - (float)_params.flow_qual_min);

            // 计算标准差的缩放因子, 质量最高时为1, 质量最差时为0
            float scale = 0.f;
            if (flow_qual_range >= 1.0f) {
                scale = math::constrain(((float)_flow_sample_delay.quality - (float)_params.flow_qual_min) / flow_qual_range, 0.0f, 1.0f);
            }

            // 根据缩放因子线性的调整标准方差
            return std_best * scale + std_worst * (1.0f - scale);
        };

        /*!
         * 重置eskf的水平速度与协方差
         * 如果视轴速度计算成功, 则利用该速度来重置eskf水平速度
         * 否则, eskf重置为0
         */
        auto reset_horizontal_velocity_to_optical_flow = [&] {
//            _information_events.flags.reset_vel_to_flow = true;
//            ECL_INFO("reset velocity to flow");
            // 限制离地高度的最小距离
            const float range_hgt_est = fmaxf(_eskf._terrain - _eskf._state.pos(2) - _range_offset_nav(2), _params.rng_gnd_clearance);

            // 计算从焦点到测距仪中心的绝对距离
            const float range = range_hgt_est / _eskf._Rnb(2, 2);

            if ((range - _params.rng_gnd_clearance) > 0.3f) {
                // 计算投影的机体系的速度(假设地面静止不懂且机体z轴上速度为0)
                Vector3f vel_optflow_body;
                vel_optflow_body(0) = -range * _flow_state.flow_compensated_xy_rad(1) / _flow_sample_delay.dt;
                vel_optflow_body(1) = range * _flow_state.flow_compensated_xy_rad(0) / _flow_sample_delay.dt;
                vel_optflow_body(2) = 0.0f;

                // 速度投影的地球系
                const Vector3f vel_optflow_earth = _eskf._Rnb * vel_optflow_body;

                _eskf.set_vel_horz(vel_optflow_earth.xy());

                _time_last_vel_horz_fuse = _imu_sample_delay.time_us;
                _flow_state.time_last_fuse = _imu_sample_delay.time_us;
            } else {
                _eskf.set_vel_horz(Vector2f(0.f, 0.f));
            }

            // 利用光流的量测噪声重置速度协方差矩阵
            _eskf.reset_covariance_matrix<2>(3, sq(range * calc_optical_flow_meas_std()));
        };

        /*!
         * 重置eskf水平位置与协方差
         * 如果飞机已起飞, 则使用最后知道的水平位置来重置eskf水平位置
         * 否则, 把水平位置重置为0
         */
        auto reset_horizontal_position_to_optical_flow = [&] {
//            _information_events.flags.reset_pos_to_last_known = true;
//            ECL_INFO("reset position to last known position");

            if (_control_status.flags.in_air) {
                _eskf.set_pos_horz(_flow_state.last_known_posNE);
            } else {
                _eskf.set_pos_horz(Vector2f(0.f, 0.f));
            }

            // 假设初值位置的方差为0
            // TODO: 初值位置的方差应该根据eskf的协方差来给定
            _eskf.reset_covariance_matrix<2>(0, 0.f);
        };

        /*!
         * 利用光流数据进行融合
         */
        auto fuse_opt_flow = [&]() {
            if (_flow_state.inhibit_flow_use) {
//                std::cout << "inhibit flow use" << std::endl;
                _control_status.flags.opt_flow = false;
            } else if (_control_status.flags.opt_flow
                       && is_recent(_flow_state.time_last_fuse, RunnerParameters::FLOW_FUSE_TIMEOUT)) {
                // 只有最近进行过地形融合, 且上一次光流融合时间间隔没超过一定限制, 才能进行光流融合
                Vector2f flow_rate = _flow_state.flow_compensated_xy_rad / _flow_sample_delay.dt;
                auto info = _eskf.fuse_flow(flow_rate, _params.range_pos_body, _range_offset_nav,
                                            _eskf._params.flow_innov_gate, calc_optical_flow_meas_std(), _flow_fuse_data);
                if (info) {
                    std::cout << "info = " << int(info) << std::endl;
                    if (info & (uint8_t)0x00000001b) {
                        _innovation_fault_status.flags.reject_optflow_x = true;
                    }
                    if (info & (uint8_t)0x00000010b) {
                        _covariance_fault_status.flags.unhealthy_optflow_x = true;
                    }
                    if (info & (uint8_t)0x00000100b) {
                        _innovation_fault_status.flags.reject_optflow_y = true;
                    }
                    if (info & (uint8_t)0x00001000b) {
                        _covariance_fault_status.flags.unhealthy_optflow_y = true;
                    }
                } else {
                    _time_last_vel_horz_fuse = _imu_sample_delay.time_us;
                    _flow_state.time_last_fuse = _imu_sample_delay.time_us;
                }
                _flow_state.last_known_posNE = _eskf._state.pos.xy();
//                std::cout << "fuse flow" << std::endl;
            } else {
                _control_status.flags.opt_flow = true;
                reset_horizontal_velocity_to_optical_flow();
                reset_horizontal_position_to_optical_flow();
//                std::cout << "reset flow" << std::endl;
            }
        };

        update_on_ground_motion_for_optical_flow_checks();
        if (calc_opt_flow_compensated_xy_rad()) { // 只有补偿的视轴速度计算成功才会进行光流融合的判断
            check_inhibit_flow_use();
            fuse_opt_flow();
        }

    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::control_gps_except_hgt_fusion() {
        auto should_reset_gps_fusion = [&]() -> bool {
            const bool is_reset_required = has_horizontal_aiding_timeout()
                                           || is_timeout(_time_last_pos_horz_fuse, 2 * RunnerParameters::HORZ_FUSE_TIMEOUT);

//            const bool is_recent_takeoff_nav_failure = _control_status.flags.in_air
//                                                       && is_recent(_time_last_on_ground, 30000000)
//                                                       && is_timeout(_time_last_vel_horz_fuse, _params.EKFGSF_reset_delay)
//                                                       && (_time_last_vel_horz_fuse > _time_last_on_ground);

            const bool is_inflight_nav_failure = _control_status.flags.in_air
                                                 && is_timeout(_time_last_vel_horz_fuse, RunnerParameters::HORZ_FUSE_TIMEOUT)
                                                 && is_timeout(_time_last_pos_horz_fuse, RunnerParameters::HORZ_FUSE_TIMEOUT)
                                                 && (_time_last_vel_horz_fuse > _time_last_on_ground)
                                                 && (_time_last_pos_horz_fuse > _time_last_on_ground);

//            return is_reset_required || is_recent_takeoff_nav_failure || is_inflight_nav_failure;
            return is_reset_required || is_inflight_nav_failure;
        };

        auto fuse_pos_horz = [&](uint8_t i) {
            std::cout << "fuse_pos_horz" << std::endl;

            uint8_t info = _eskf.fuse_pos_horz(_gps_sample_delay[i].pos_horz, _params.gps_pos_body[i],
                                               _gps_state.offset_nav[i], _eskf._params.gps_pos_horz_innov_gate,
                                               _eskf._params.gps_pos_horz_noise, _gps_pos_horz_fuse_data[i]);
            if (info) {
                if (info & (uint8_t)0x00000001b) {
                    _innovation_fault_status.flags.reject_gps_pos_x |= (uint8_t)((uint8_t)0x00000001b << i);
                }
                if (info & (uint8_t)0x00000010b) {
                    _covariance_fault_status.flags.unhealthy_gps_pos_x |= (uint8_t)((uint8_t)0x00000001b << i);
                }
                if (info & (uint8_t)0x00000100b) {
                    _innovation_fault_status.flags.reject_gps_pos_y |= (uint8_t)((uint8_t)0x00000001b << i);
                }
                if (info & (uint8_t)0x00001000b) {
                    _covariance_fault_status.flags.unhealthy_gps_pos_y |= (uint8_t)((uint8_t)0x00000001b << i);
                }

                std::cout << "fuse_pos_horz: fault" << std::endl;
            } else {
                _time_last_pos_horz_fuse = _imu_sample_delay.time_us;
                _gps_state.time_last_pos_horz_fuse[i] = _imu_sample_delay.time_us;

                std::cout << "fuse_pos_horz: succeed" << std::endl;
            }
        };

        auto fuse_vel_horz = [&](uint8_t i) {
            uint8_t info = _eskf.fuse_vel_horz(_gps_sample_delay[i].vel.xy(), _params.gps_pos_body[i],
                                               _gps_state.offset_nav[i], _gps_state.w_cross_offset_body[i],
                                               _gps_state.w_cross_offset_nav[i], _eskf._params.gps_vel_horz_innov_gate,
                                               _eskf._params.gps_vel_horz_noise, _gps_vel_horz_fuse_data[i]);
            if (info) {
                if (info & (uint8_t)0x00000001b) {
                    _innovation_fault_status.flags.reject_gps_vel_x |= (uint8_t)((uint8_t)0x00000001b << i);
                }
                if (info & (uint8_t)0x00000010b) {
                    _covariance_fault_status.flags.unhealthy_gps_vel_x |= (uint8_t)((uint8_t)0x00000001b << i);
                }
                if (info & (uint8_t)0x00000100b) {
                    _innovation_fault_status.flags.reject_gps_vel_y |= (uint8_t)((uint8_t)0x00000001b << i);
                }
                if (info & (uint8_t)0x00001000b) {
                    _covariance_fault_status.flags.unhealthy_gps_vel_y |= (uint8_t)((uint8_t)0x00000001b << i);
                }
            } else {
                _time_last_vel_horz_fuse = _imu_sample_delay.time_us;
                _gps_state.time_last_vel_horz_fuse[i] = _imu_sample_delay.time_us;
            }
        };

        auto fuse_vel_vert = [&](uint8_t i) {
            uint8_t info = _eskf.fuse_vel_vert(_gps_sample_delay[i].vel(2), _params.gps_pos_body[i],
                                               _gps_state.offset_nav[i], _gps_state.w_cross_offset_body[i],
                                               _gps_state.w_cross_offset_nav[i], _eskf._params.gps_vel_vert_innov_gate,
                                               _eskf._params.gps_vel_vert_noise, _gps_vel_vert_fuse_data[i]);
            if (info) {
                if (info & (uint8_t)0x00000001b) {
                    _innovation_fault_status.flags.reject_gps_vel_z |= (uint8_t)((uint8_t)0x00000001b << i);
                }
                if (info & (uint8_t)0x00000010b) {
                    _covariance_fault_status.flags.unhealthy_gps_vel_z |= (uint8_t)((uint8_t)0x00000001b << i);
                }
            } else {
                _time_last_vel_vert_fuse = _imu_sample_delay.time_us;
                _gps_state.time_last_vel_vert_fuse[i] = _imu_sample_delay.time_us;
            }
        };

        auto arbitrary_condition = [&](uint8_t i) -> bool {
            // 忽略gps的卫星数
            return true;
        };

        auto loose_condition = [&](uint8_t i) -> bool {
            // gps没有在连续5秒内卫星数都不合格
            return !is_timeout(_gps_state.last_pass_us[i], (uint64_t)5e6);
        };

        auto strict_condition = [&](uint8_t i) -> bool {
            // gps在连续5秒内卫星数都合格
            return is_timeout(_gps_state.last_fail_us[i], (uint64_t)1e6);
        };

        auto reset_gps_fusion = [&](const std::function<bool(uint8_t)>& condition) -> bool {
            std::cout << "reset_gps_fusion" << std::endl;

            float prior = 1.f;
            float sum_prior = 0.f;
            Vector2f sum_pos(0.f, 0.f);
            Vector3f sum_vel(0.f, 0.f, 0.f);

            for (uint8_t i = 0; i < N_GPS; ++i) {
                if (_gps_state.data_ready[i] && condition(i)) {
                    sum_pos += prior * _gps_state.pos_horz_imu[i];
                    sum_vel += prior * _gps_state.vel_imu[i];
                    sum_prior += prior;

                    _gps_state.time_last_pos_horz_fuse[i] = _gps_state.time_last_vel_horz_fuse[i] = _gps_state.time_last_vel_vert_fuse[i] = _imu_sample_delay.time_us;
                }
            }

            if (sum_prior > 0.f) {
                sum_pos /= sum_prior;
                sum_vel /= sum_prior;

                _eskf.set_pos_horz(sum_pos);
                // TODO: 使用gps的精度代替 _eskf._params.gps_pos_horz_noise
                _eskf.reset_covariance_matrix<2>(0, sq(_eskf._params.gps_pos_horz_noise));

                _eskf.set_velocity(sum_vel);
                // TODO: 使用gps的精度代替 _eskf._params.gps_vel_horz_noise 和 _eskf._params.gps_vel_vert_noise
                _eskf.reset_covariance_matrix<3>(3, sq(math::max(_eskf._params.gps_vel_horz_noise, _eskf._params.gps_vel_vert_noise)));

                _time_last_pos_horz_fuse = _time_last_vel_horz_fuse = _time_last_vel_vert_fuse = _imu_sample_delay.time_us;

//                std::cout << "SSSSSSSSSSSSSSSSSSSSSSSSSSSSSsum_pos: " << sum_pos(0) << ", " << sum_pos(1) << std::endl;
//                std::cout << "SSSSSSSSSSSSSSSSSSSSSSSSSSSSSsum_vel" << sum_vel(0) << ", " << sum_vel(1) << ", " << sum_vel(2) << std::endl;

//                assert(-1 > 0);

                return true;
            }

            return false;
        };

        auto start_gps_fusion = [&]() {
            _control_status.flags.gps_horz = true;
            _control_status.flags.gps_vel = true;
        };

        auto stop_gps_fusion = [&]() {
            _control_status.flags.gps_horz = false;
            _control_status.flags.gps_vel = false;
        };

        bool any_gps_data_attempted = false;
        bool each_gps_checks_failing = true;
        const bool mandatory_conditions_passing = _control_status.flags.tilt_align
                                                  && _control_status.flags.yaw_align
                                                  && _gps_state.ned_origin_initialised;

//        std::cout << "mandatory_conditions_passing = " << mandatory_conditions_passing << std::endl;

        // 只有姿态已经对齐且home已经设置后, 才能进行gps融合
        if (mandatory_conditions_passing) {
            if (_control_status.flags.gps_horz || _control_status.flags.gps_vel) {
                for (uint8_t i = 0; i < N_GPS; ++i) {
                    /*
                     * 判断是否连续5秒卫星数都不合格
                     * 如果合格, 则进行进行gps融合
                     * 反之则不进行gps融合
                     * */
                    const bool gps_checks_failing = is_timeout(_gps_state.last_pass_us[i], (uint64_t)5e6);
                    if (_gps_state.data_ready[i] && !gps_checks_failing) {
                        any_gps_data_attempted = true;
                        each_gps_checks_failing = false;
                        if (_control_status.flags.gps_horz) {
                            fuse_pos_horz(i);

//                            std::cout << "fuse_pos_horz()" << std::endl;
                        }
                        if (_control_status.flags.gps_vel) {
                            fuse_vel_horz(i);
                            fuse_vel_vert(i);

//                            std::cout << "fuse_vel_horz() and fuse_vel_vert()" << std::endl;
                        }
                    }
                }

                /*
                 * 如果所有gps的卫星数都连续5秒不合格, 若此时存在除gps外的水平辅助传感器, 则停止gps融合
                 * 若不存在除gps外的水平辅助传感器, 则依然进行gps融合
                 * */
                if (each_gps_checks_failing) {
                    if (is_other_source_of_horizontal_aiding_than(_control_status.flags.gps_horz || _control_status.flags.gps_vel)) {
                        stop_gps_fusion();
                        return;
//                        _warning_events.flags.gps_quality_poor = true;
//                        ECL_WARN("GPS quality poor - stopping use");
                    } else {    // 如果已经没有别的水平辅助传感器了, 即使gps数据再差也要进行融合
                        for (uint8_t i = 0; i < N_GPS; ++i) {
                            if (_gps_state.data_ready[i]) {
                                any_gps_data_attempted = true;
                                if (_control_status.flags.gps_horz) {
                                    fuse_pos_horz(i);

//                                    std::cout << "force to fuse_pos_horz()" << std::endl;
                                }
                                if (_control_status.flags.gps_vel) {
                                    fuse_vel_horz(i);
                                    fuse_vel_vert(i);

//                                    std::cout << "force to fuse_vel_horz() and fuse_vel_vert()" << std::endl;
                                }
                            }
                        }
                    }
                }

                /*
                 * 如果尝试过gps融合(代表没有切换水平辅助传感器且至少一个gps_data_ready为true),
                 * 则判断是否应该重置gps融合.
                 * 否则, 代表切换了水平辅助源或者所有gps_data_ready都为false,
                 * 此时需要判断是否长时间没收到gps数据, 如果是, 则需要停止gps融合
                 * */
                if (any_gps_data_attempted) {
                    if (should_reset_gps_fusion()) {
//                        std::cout << "should_reset_gps_fusion" << std::endl;
                        if (each_gps_checks_failing) {
                            reset_gps_fusion(arbitrary_condition);
                        } else {
                            reset_gps_fusion(loose_condition);
                        }
                    }
                } else {
                    if (is_other_source_of_horizontal_aiding_than(_control_status.flags.gps_horz || _control_status.flags.gps_vel)) {
                        bool timeout_1s = false;
                        for (uint8_t i = 0; i < N_GPS; ++i) {
                            timeout_1s |= is_timeout(_gps_sample_delay[i].time_us, 1e6);
                        }
                        if (timeout_1s) {
                            stop_gps_fusion();
                        }
                    } else {
                        bool timeout_10s = false;
                        for (uint8_t i = 0; i < N_GPS; ++i) {
                            timeout_10s |= is_timeout(_gps_sample_delay[i].time_us, 10e6);
                        }
                        if (timeout_10s) {
                            stop_gps_fusion();
                        }
                    }
                }
            } else {
                /*
                 * 如果存在任意一个gps在连续5秒内卫星数都合格, 则切换到gps融合
                 * */
                if (reset_gps_fusion(strict_condition)) {
                    start_gps_fusion();
                }
            }
        } else {    // 姿态还没对齐或home点还没设置, 则不进行gps融合
            stop_gps_fusion();
        }
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::control_mag_fuse() {
        if (_mag_data_ready) {
            // 对磁力计测得的磁场进行滤波. 在磁场对齐中需要用到
            _mag_body_lp = _mag_body_lpf(_mag_sample_delay.mag);
            ++_mag_body_lpf_counter;
//            std::cout << _mag_sample_delay.mag(0) << ", " << _mag_sample_delay.mag(1) << ", " << _mag_sample_delay.mag(2) << std::endl;
//            std::cout << _mag_body_lp(0) << ", " << _mag_body_lp(1) << ", " << _mag_body_lp(2) << std::endl;

            // 检查磁力计测得的磁场强度是否与GPS给出的地球磁场强度接近
            if (_params.check_mag_strength) {
                check_mag_field_strength(_mag_sample_delay);
            } else {
                _control_status.flags.mag_field_disturbed = false;
            }

            if (_params.mag_body_fusion_type >= RunnerParameters::MAG_FUSE_TYPE_NONE
                || _control_status.flags.mag_fault
                || !_control_status.flags.tilt_align) {

                inhibit_mag_fusion();

                if (no_other_vector_aiding_than_mag()) {
                    // TODO
                }

                return;
            }

            // 如果是从别的传感器切换到磁力计, 则需要使用磁场向量重新对准姿态(存在一定前提条件: 当前磁场数据没有被禁止使用)
            _mag_align_req |= other_vector_sources_have_stopped();

            // 如果磁场没有被对齐过, 则需要对齐磁场
            _mag_align_req |= !_control_status.flags.mag_aligned;
            _mag_align_req |= !_control_status.flags.yaw_align;

            // 检测当前磁场数据是否被禁止使用, 并记录磁场被禁止持续时间, 判断磁场数据是否长期被禁用
            check_mag_inhibition();

            // 如果磁场数据长时间被禁止使用, 则需要重新对齐磁场(存在一定前提条件: 当前磁场数据没有被禁止使用)
            _mag_align_req |= _mag_fusion_inhibited_too_long;

            if (no_other_vector_aiding_than_mag()) {
                std::cout << "no_other_vector_aiding_than_mag" << std::endl;
                // 没有gps信号, 或gps只有单天线可用, 或者没有外部视觉提供姿态, 此时需要使用磁力计修正姿态
                // 在这个过程中需要冻结eskf中地球磁场的状态

                if (_control_status.flags.in_air) {
                    // 飞机第一次飞离磁场异常区域, 则需要重新进行磁场对齐(存在一定前提条件: 当前磁场数据没有被禁止使用), 即使之前已经对齐过
                    _mag_align_req |= above_mag_anomalies();
                    run_in_air_mag_align(_mag_sample_delay.mag);
                } else {
                    run_on_ground_mag_align();
                }

                // 如果磁场没有对齐, 则不进行磁场数据的融合
                if (!_control_status.flags.mag_aligned) {
                    return;
                }

                /* 开启磁场数据融合 */

                // 只有当角速度满足一定条件时, 磁力计偏移才被认为是可观测的. 在大多数场景下都是不可观测的, 例如悬停和近匀速运动
                check_mag_bias_observability();
                if (_mag_bias_observable) {
                    start_mag_bias_fusion();
                } else {
                    stop_mag_bias_fusion();
                }

                // 只有磁力计作为姿态校正传感器时, 不进行地球磁场角度的融合, 否则会影响融合时对δθ的估计
                // 因为地球磁场角度与y轴和z轴的δθ共享同样的误差分配
                stop_mag_ang_fusion();

                // 磁场强度的融合根据用户的选择来开启
                if (_params.mag_earth_fusion_type & RunnerParameters::MAG_EARTH_FUSE_TYPE_NORM) {
                    start_mag_norm_fusion();
                } else {
                    stop_mag_norm_fusion();
                }

                uint8_t info = _eskf.fuse_mag_body(_mag_sample_delay.mag, _eskf._params.mag_body_innov_gate,
                                                   _eskf._params.mag_body_noise, _mag_body_fuse_data);

                if (info) {
                    if (info & (uint8_t)0x00000001b) {
                        _innovation_fault_status.flags.reject_mag_body_x = true;
                    }
                    if (info & (uint8_t)0x00000010b) {
                        _covariance_fault_status.flags.unhealthy_mag_body_x = true;
                    }
                    if (info & (uint8_t)0x00000100b) {
                        _innovation_fault_status.flags.reject_mag_body_y = true;
                    }
                    if (info & (uint8_t)0x00001000b) {
                        _covariance_fault_status.flags.unhealthy_mag_body_y = true;
                    }
                    if (info & (uint8_t)0x00010000b) {
                        _innovation_fault_status.flags.reject_mag_body_z = true;
                    }
                    if (info & (uint8_t)0x00100000b) {
                        _covariance_fault_status.flags.unhealthy_mag_body_z = true;
                    }
                } else {
                    _time_last_mag_body_fuse = _imu_sample_delay.time_us;
                }

            } else {
                // 如果存在别的传感器用于修正姿态, 则在去除新息对eskf姿态校正的情况下, 进行融合磁场状态量
                // 以便切换回磁力计融合姿态时, 有准确的地球磁场与磁力计偏移估计

                /* 开启磁场数据融合 */

                // 只有当角速度满足一定条件时, 磁力计偏移才被认为是可观测的. 在大多数场景下都是不可观测的, 如果悬停和近匀速运动
                check_mag_bias_observability();
                if (_mag_bias_observable) {
                    start_mag_bias_fusion();
                } else {
                    stop_mag_bias_fusion();
                }

                // 如果存在其他的姿态修改传感器, 则可以用磁力计数据融合出地球磁场, 但此融合时要清除新息对δθ的贡献
                if (_params.mag_earth_fusion_type & RunnerParameters::MAG_EARTH_FUSE_TYPE_ANG) {
                    start_mag_ang_fusion();
                } else {
                    stop_mag_ang_fusion();
                }

                // 磁场强度的融合根据用户的选择来开启
                if (_params.mag_earth_fusion_type & RunnerParameters::MAG_EARTH_FUSE_TYPE_NORM) {
                    start_mag_norm_fusion();
                } else {
                    stop_mag_norm_fusion();
                };

                uint8_t info = _eskf.fuse_mag_body(_mag_sample_delay.mag, _eskf._params.mag_body_innov_gate,
                                                   _eskf._params.mag_body_noise, _mag_body_fuse_data, true);
                if (info) {
                    if (info & (uint8_t)0x00000001b) {
                        _innovation_fault_status.flags.reject_mag_body_x = true;
                    }
                    if (info & (uint8_t)0x00000010b) {
                        _covariance_fault_status.flags.unhealthy_mag_body_x = true;
                    }
                    if (info & (uint8_t)0x00000100b) {
                        _innovation_fault_status.flags.reject_mag_body_y = true;
                    }
                    if (info & (uint8_t)0x00001000b) {
                        _covariance_fault_status.flags.unhealthy_mag_body_y = true;
                    }
                    if (info & (uint8_t)0x00010000b) {
                        _innovation_fault_status.flags.reject_mag_body_z = true;
                    }
                    if (info & (uint8_t)0x00100000b) {
                        _covariance_fault_status.flags.unhealthy_mag_body_z = true;
                    }
                } else {
                    _time_last_mag_body_fuse = _imu_sample_delay.time_us;
                }
            }
        }
    }

    /*!
     * 是否禁止使用磁场数据
     * @tparam DELAYS
     * @return 用户选择了室内模式且没有gps信号(事实室内) 或者 磁场强度不稳定, 则不进行磁力计数据融合; 否则不禁用磁场数据
     */
    template<uint8_t DELAYS>
    bool ESKFRunner<DELAYS>::should_inhibit_mag() const {
        const bool user_selected = (_params.mag_body_fusion_type == RunnerParameters::MAG_FUSE_TYPE_INDOOR);

        const bool heading_not_required_for_navigation = !_control_status.flags.gps_horz
                                                         && !_control_status.flags.gps_vel
                                                         && !_control_status.flags.ev_horz
                                                         && !_control_status.flags.ev_vel;

        return (user_selected && heading_not_required_for_navigation) || _control_status.flags.mag_field_disturbed;
    }

    /*!
     * 把磁力计的磁场强度与gps/理论的磁场强度进行比较, 若差异超过一定范围内则认为磁场受到了干扰, 返回true; 否则返回false
     * @tparam DELAYS
     * @param mag_sample - 磁力计量测数据
     */
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::check_mag_field_strength(const MagSample &mag_sample) {
        const float mag_sample_norm = mag_sample.mag.norm();

        constexpr float wmm_gate_size = 0.2f; // +/- Gauss
        _control_status.flags.mag_field_disturbed = !((mag_sample_norm >= _mag_norm - wmm_gate_size) &&
                                                      (mag_sample_norm <= _mag_norm + wmm_gate_size));
    }

    /*!
     * 检测当前磁场数据是否被禁止使用, 并记录磁场梅被禁止持续时间, 判断磁场数据是否长期被禁用
     * @tparam DELAYS
     */
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::check_mag_inhibition() {
        // 检测磁场数据是否被禁止使用, 并记录磁场被禁止持续时间
        _is_mag_fusion_inhibited = should_inhibit_mag();
        if (!_is_mag_fusion_inhibited) {
            _mag_fusion_not_inhibited_us = _imu_sample_delay.time_us;
        }
        if (is_timeout(_mag_fusion_not_inhibited_us, RunnerParameters::MAG_FUSE_TIMEOUT)) {
            _mag_fusion_inhibited_too_long = true;
        }
    }

    /*!
     * 判断磁力计偏移的可观性, 当:
     *      (1) 垂直于磁场的角速度分量的模值持续一段时间大于阈值则认为磁力计偏移可观
     *      (2) 如果不满足上述条件, 则当瞬时角速度垂直分量大于一定阈值且上一时刻为可观时, 依然认为可观, 否则一律认为不可观
     * 一般工况下, 磁力计偏移都不可观, 比如飞机在近悬停和近匀速状态下, 磁力计偏移不可观.
     * @tparam DELAYS
     */
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::check_mag_bias_observability() {
        const Vector3f mag_body_unit = _eskf._Rnb * get_mag_earth().normalized();
        const Vector3f gyro_orth_mag = _eskf._gyro_corr - mag_body_unit * mag_body_unit.dot(_eskf._gyro_corr);
        const float gyro_orth_mag_norm2 = gyro_orth_mag.norm_squared();

        const float gyro_orth_mag_lp_norm2 = _gyro_orth_mag_lpf(gyro_orth_mag).norm_squared();

        // 垂直于磁场的角速度分量的模值持续一段时间大于阈值则认为磁力计偏移可观
        // 如果不满足上述条件, 则当瞬时角速度垂直分量大于一定阈值且上一时刻为可观时, 依然认为可观, 否则一律认为不可观
        if (gyro_orth_mag_lp_norm2 > sq(_params.gyro_orth_mag_gate)) {
            _mag_bias_observable = true;
        } else if (_mag_bias_observable) {
            _mag_bias_observable = gyro_orth_mag_norm2 > sq(0.5f * _params.gyro_orth_mag_gate);
        }
    }

    /*!
     * 判断除了磁力计以外, 是否已经没有别的传感器可以提供向量了
     * @tparam DELAYS
     * @return true-除了磁力计以外, 没有别的传感器可以提供向量; false-有除了磁力计以外的传感器可以提供向量
     */
    template<uint8_t DELAYS>
    bool ESKFRunner<DELAYS>::no_other_vector_aiding_than_mag() const {
        return !(_control_status.flags.gps_horz && N_GPS > 1);
    }

    /*!
     * 是否由别的向量传感器切换到了磁力计
     * @tparam DELAYS
     * @return true-之前没有使用磁力计且现在切换到了磁力计则返回; false-没有切换到磁力计或者之前就在使用磁力计
     */
    template<uint8_t DELAYS>
    bool ESKFRunner<DELAYS>::other_vector_sources_have_stopped() {
        // 如果之前磁力计就在使用,
        bool result = no_other_vector_aiding_than_mag() && _non_mag_aiding_running_prev;

        _non_mag_aiding_running_prev = !no_other_vector_aiding_than_mag();

        return  result;
    }

    /*!
     * 判断飞机是否已经离开磁场异常区域(离开地面一定的距离)
     * @tparam DELAYS
     * @return true-已经离开磁场异常区域; false-没有离开磁场异常区域
     */
    template<uint8_t DELAYS>
    bool ESKFRunner<DELAYS>::above_mag_anomalies() const {
        bool above = false;
        // 在离开地面进行飞行状态时, 需要重新使用磁力计数据来进行对准, 以去除地面对磁场的干扰所带来的姿态不准确
        if (!_control_status.flags.mag_aligned_in_flight) {
            // 当离开地面足够远时, 则认为脱离了磁场异常区域, 此时需要重新对准磁场
            above = (_eskf._terrain - _eskf._state.pos(2)) > _params.mag_anomalies_max_hgt;
        }
        return above;
    }

    /*!
     * 飞机在飞行情况下, 对磁场进行对齐
     * 当受到对齐请求, 且当前磁场数据没有被禁止使用时, 才会进行磁场对齐
     * 但是如果用户没有选择磁力计进行组合导航, 则依然不会进行磁场对齐
     * 成功对齐后, _control_status.flags.mag_aligned = _control_status.flags.mag_aligned_in_flight = true
     * 如果磁场长时间被禁止使用, 则重置标志位 _mag_fusion_inhibited_too_long=false, 则需要重启z轴角速度偏移估计
     * @tparam DELAYS
     * @param mag_sample
     */
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::run_in_air_mag_align(const Vector3f &mag_sample) {
        // (没进行过磁场对齐 || 从别的传感器切换到磁力计 || 磁场数据长期被进行) && (当前磁场数据没有被禁止使用)
        if (_mag_align_req && !_is_mag_fusion_inhibited) {
            bool has_realigned_mag = false;

            if (_control_status.flags.gps_vel && _control_status.flags.fixed_wing) {
                // 固定翼模式下, 由于飞机的运动学模型的限制, 可用使用gps速度来计算出航向, 然后使用该航向进行对准
//                has_realigned_yaw = realign_gps_vel(mag_sample);

            } else if (magnetometer_can_be_used()) {
                // 如果磁力计数据没有异常且磁力计的融合模式被激活, 则进行磁场对齐
                has_realigned_mag = align_mag_and_reset_eskf();
            }

            if (has_realigned_mag) {
                _mag_align_req = false;
                _control_status.flags.yaw_align = true;
                _control_status.flags.mag_aligned = true;
                _control_status.flags.mag_aligned_in_flight = true;

                // 由于在室内飞行时, 停止了eskf中关于z轴角速度偏移的更新
                // 所以磁力计被重新激活且成功进行了磁场对齐后, 需要重新启动eskf的z轴角速度偏移的更新
                if (_mag_fusion_inhibited_too_long) {
                    _mag_fusion_inhibited_too_long = false;
                    _eskf.reset_covariance_matrix<1>(11, sq(_eskf._params.std_init_gyro_bias));
                }
            }
        }
    }

    /*!
     * 飞机在地上没起飞时, 对磁场进行对齐
     * 当受到对齐请求, 且当前磁场数据没有被禁止使用时, 才会进行磁场对齐
     * 但是如果用户没有选择磁力计进行组合导航, 则依然不会进行磁场对齐
     * 成功对齐后, _control_status.flags.mag_aligned = true
     * 如果磁场长时间被禁止使用, 则重置标志位 _mag_fusion_inhibited_too_long=false, 则需要重启z轴角速度偏移估计
     * @tparam DELAYS
     */
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::run_on_ground_mag_align() {
        if (_mag_align_req && !_is_mag_fusion_inhibited) {
            const bool has_realigned_mag = magnetometer_can_be_used() ? align_mag_and_reset_eskf() : false;

            if (has_realigned_mag) {
                _mag_align_req = false;
                _control_status.flags.yaw_align = true;
                _control_status.flags.mag_aligned = true;

                // 由于在室内飞行时, 停止了eskf中关于z轴角速度偏移的更新
                // 所以磁力计被重新激活且成功进行了磁场对齐后, 需要重新启动eskf的z轴角速度偏移的更新
                if (_mag_fusion_inhibited_too_long) {
                    _mag_fusion_inhibited_too_long = false;
                    _eskf.reset_covariance_matrix<1>(11, sq(_eskf._params.std_init_gyro_bias));
                }
            }
        }
    }

    /*!
     * 对齐磁场, 去除eskf中其余状态与轴角的相关性, 重置eskf中与磁场相关的状态的协方差
     * @tparam DELAYS
     * @return
     */
    template<uint8_t DELAYS>
    bool ESKFRunner<DELAYS>::align_mag_and_reset_eskf() {
        // 如果在一个imu周期内已经对齐过了, 则无需在对齐, 减少计算资源的消耗
        if (_imu_sample_delay.time_us == _mag_align_last_time) {
            return true;
        }

        // 如果磁力计的低通滤波器启动不够久, 则不进行对齐, 因为此时并没有稳定的机体坐标下的磁场量测
        if (_mag_body_lpf_counter < 100) {
            return false;
        }

        const Vector3f &mag_init = _mag_body_lp;

        const bool heading_required_for_navigation = _control_status.flags.gps_horz || _control_status.flags.ev_horz;

        if ((_params.mag_body_fusion_type <= RunnerParameters::MAG_FUSE_TYPE_3D) ||
            ((_params.mag_body_fusion_type == RunnerParameters::MAG_FUSE_TYPE_INDOOR) && heading_required_for_navigation)) {
            const Vector3f mag_earth_pred = _eskf._Rnb * mag_init;
            const Vector3f mag_earth_true = get_mag_earth();

//            std::cout << "mag_earth_true: " << mag_earth_true(0) << ", " << mag_earth_true(1) << ", " << mag_earth_true(2) << std::endl;
//            std::cout << "mag_earth_pred: " << mag_init(0) << ", " << mag_init(1) << ", " << mag_init(2) << std::endl;

            // mag_earth_pred到mag_earth_true的旋转
            const Quatf dq(mag_earth_pred, mag_earth_true);
//            std::cout << dq(0) << ", " << dq(1) << ", " << dq(2) << ", " << dq(3) << std::endl;
//            assert(-1 > 0);
            const Quatf q_corr = dq * _eskf._state.quat_nominal;
            _eskf.set_attitude(q_corr);

            // 给eskf的协方差矩阵叠加量测噪声
            const float mag_earth_norm2 = mag_earth_true.norm_squared();
            const float mag_earth_norm4 = mag_earth_norm2 * mag_earth_norm2;
            const float mag_earth_var = sq(_eskf._params.mag_norm_noise);
            for (uint8_t i = 0; i < 3; ++i) {
                for (uint8_t j = i; j < 3; ++j) {
                    _eskf._P[i][j] += mag_earth_var * (mag_earth_norm2 - mag_earth_true(i) * mag_earth_true(j)) / mag_earth_norm4;
                }
            }
            _eskf._P[1][0] = _eskf._P[0][1];
            _eskf._P[2][0] = _eskf._P[0][2];
            _eskf._P[2][1] = _eskf._P[1][2];

            // 去除其余状态与轴角的相关性
            uncorrelate_target_from_other_states<0, 3>();

            // 使用修正后的旋转矩阵计算滤波后的机体系量测磁场在地球坐标系下的投影
            const Vector3f mag_earth_corr = _eskf._Rnb * mag_init;
            _eskf.set_mag_earth(mag_earth_corr);

            // 重置与磁场相关的协方差
            _eskf.reset_covariance_matrix<1>(16, sq(_eskf._params.mag_norm_noise));
            _eskf.reset_covariance_matrix<2>(17, sq(_eskf._params.mag_ang_noise));
            _eskf.reset_covariance_matrix<3>(19, sq(_eskf._params.mag_body_noise));

            // 记录最近一次磁场对齐的时间
            _mag_align_last_time = _imu_sample_delay.time_us;

            _time_last_mag_body_fuse = _imu_sample_delay.time_us;

            return true;

        } else if (_params.mag_body_fusion_type == RunnerParameters::MAG_FUSE_TYPE_INDOOR) {
            // we are operating temporarily without knowing the earth frame yaw angle
            return true;

        } else {
            // there is no magnetic yaw observation
            return false;
        }
    }

    /*!
     * 判断磁力计数据是否能用
     * @tparam DELAYS
     * @return - 如果磁力计融合被激活且磁力计的磁场强度与量测的地球磁场强度差异不大, 则认为磁力计数据可用, 否则认为不可用
     */
    template<uint8_t DELAYS>
    bool ESKFRunner<DELAYS>::magnetometer_can_be_used() const {
        return !_control_status.flags.mag_field_disturbed && (_params.mag_body_fusion_type != RunnerParameters::MAG_FUSE_TYPE_NONE);
    }

    /*!
     * 获取地球磁场
     * @tparam DELAYS
     * @return - 如果在飞机进入飞行状态后已经进行过磁场对齐了, 则返回eskf中的地球磁场, 否则返回量测得到的地球磁场
     */
    template<uint8_t DELAYS>
    Vector3f ESKFRunner<DELAYS>::get_mag_earth() const {
        if (_control_status.flags.mag_aligned_in_flight) {
            // 如果在飞机进入飞行状态后已经进行过磁场对齐了, 则返回eskf中的地球磁场
            return _eskf.get_magnet_earth();

        } else {
            // 否则返回量测得到的地球磁场
            float inc = PX4_ISFINITE(_gps_state.mag_inclination) ? _gps_state.mag_inclination : _mag_inclination;
            float dec = PX4_ISFINITE(_gps_state.mag_declination) ? _gps_state.mag_declination : _mag_declination;
            float norm = PX4_ISFINITE(_gps_state.mag_strength) ? _gps_state.mag_strength : _mag_norm;
            const float cos_y = cosf(inc);
//            std::cout << _gps_state.mag_inclination << ", " << _mag_inclination << std::endl;
//            std::cout << _gps_state.mag_declination << ", " << _mag_declination << std::endl;
//            std::cout << _gps_state.mag_strength << ", " << _mag_norm << std::endl;
//            assert(-2 > 0);
            return {_mag_norm * cosf(dec) * cos_y,
                    _mag_norm * sinf(dec) * cos_y,
                    -_mag_norm * sinf(inc)};
        }
    }

//    template<uint8_t DELAYS>
//    void correct_eskf_quat_from_global(const Quatf &dq) {
//        _eskf.quat_normal = dq * _eskf.quat_normal;
//        _eskf.uncorrelate_quat_from_other_states();
//    }
//
//    template<uint8_t DELAYS>
//    void correct_eskf_quat_from_local(const Quatf &dq) {
//        _eskf.quat_normal = _eskf.quat_normal * dq;
//        _eskf.uncorrelate_quat_from_other_states();
//    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::inhibit_mag_fusion() {
        if (!_is_mag_fusion_inhibited) {
            _is_mag_fusion_inhibited = true;
            _control_status.flags.mag_norm = false;
            _control_status.flags.mag_ang = false;
            _control_status.flags.mag_bias = false;
            _eskf.disable_estimation_mag();
        }
    }
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::start_mag_norm_fusion() {
        if (!_control_status.flags.mag_norm) {
            _control_status.flags.mag_norm = true;
            _eskf.enable_estimation_mag_norm();
        }
    }
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::stop_mag_norm_fusion() {
        if (_control_status.flags.mag_norm) {
            _control_status.flags.mag_norm = false;
            _eskf.disable_estimation_mag_norm();
        }
    }
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::start_mag_ang_fusion() {
        if (!_control_status.flags.mag_ang) {
            _control_status.flags.mag_ang = true;
            _eskf.enable_estimation_mag_ang();
        }
    }
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::stop_mag_ang_fusion() {
        if (_control_status.flags.mag_ang) {
            _control_status.flags.mag_ang = false;
            _eskf.disable_estimation_mag_ang();
        }
    }
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::start_mag_bias_fusion() {
        if (!_control_status.flags.mag_bias) {
            _control_status.flags.mag_bias = true;
            _eskf.enable_estimation_mag_bias();
        }
    }
    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::stop_mag_bias_fusion() {
        if (_control_status.flags.mag_bias) {
            _control_status.flags.mag_bias = false;
            _eskf.disable_estimation_mag_bias();
        }
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::control_static_fusion() {
        // Fuse zero velocity at a limited rate (every 200 milliseconds)
        const bool static_update_ready = is_timeout(_static_state.time_last_fuse, (uint64_t)2e5);

        if (static_update_ready) {
            const bool continuing_conditions_passing = _control_status.flags.vehicle_at_rest
                                                       && _control_status_prev.flags.vehicle_at_rest;

            if (continuing_conditions_passing) {
                Vector3f zeros {0.f, 0.f, 0.f};
                float gate = 1e6f;
                float noise = 0.001f;
                _eskf.fuse_vel(zeros, zeros, zeros, zeros, zeros, gate, noise, _static_fuse_data);

                _static_state.time_last_fuse = _imu_sample_last.time_us;
            }
        }

    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::control_fake_pos_fusion() {
        auto reset_fake_pos_fusion = [&] {
            _fake_state.last_known_posNE = _eskf._state.pos.xy();
//            resetHorizontalPositionToLastKnown();
//            resetHorizontalVelocityToZero();
            _fake_state.time_last_fuse = _imu_sample_last.time_us;
        };

        auto start_fake_pos_fusion = [&] {
            if (!_using_synthetic_position) {
                _using_synthetic_position = true;
//                _fuse_hpos_as_odom = false; // TODO: needed?
                reset_fake_pos_fusion();
            }
        };

        auto stop_fake_pos_fusion = [&] {
            _using_synthetic_position = false;
        };

        auto fuse_fake_position = [&] {
            float noise;

            if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
                noise = fmaxf(_eskf._params.pos_horz_noaid_noise, _eskf._params.gps_pos_horz_noise);

            } else if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
                // Accelerate tilt fine alignment by fusing more
                // aggressively when the vehicle is at rest
                noise = 0.01f;

            } else {
                noise = 0.5f;
            }

            const float fake_pos_innov_gate = 3.f;
            Vector3f zeros {0.f, 0.f, 0.f};
            uint8_t info = _eskf.fuse_pos_horz(_fake_state.last_known_posNE, zeros, zeros, fake_pos_innov_gate, noise, _fake_fuse_data);
            if (info == 0) {
                _fake_state.time_last_fuse = _imu_sample_last.time_us;
            }
        };


        // If we aren't doing any aiding, fake position measurements at the last known position to constrain drift
        // During intial tilt aligment, fake position is used to perform a "quasi-stationary" leveling of the EKF
        const bool fake_data_ready = is_timeout(_fake_state.time_last_fuse, (uint64_t)2e5); // Fuse fake position at a limited rate

        if (fake_data_ready) {
            const bool continuing_conditions_passing = !is_horizontal_aiding_active();
            const bool starting_conditions_passing = continuing_conditions_passing;

            if (_using_synthetic_position) {
                if (continuing_conditions_passing) {
                    fuse_fake_position();

                    const bool is_fusion_failing = is_timeout(_fake_state.time_last_fuse, (uint64_t)4e5);

                    if (is_fusion_failing) {
                        reset_fake_pos_fusion();
                    }

                } else {
                    stop_fake_pos_fusion();
                }

            } else {
                if (starting_conditions_passing) {
                    start_fake_pos_fusion();

                    if (_control_status.flags.tilt_align) {
//                        // The fake position fusion is not started for initial alignement
//                        _warning_events.flags.stopping_navigation = true;
//                        ECL_WARN("stopping navigation");
                    }
                }
            }
        }
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::predict_state_from_delay(const ImuSample &sample) {
        // 只要有新的imu的值, 就要更新预积分的值
        update_pre_integral(sample);

        // 利用滞后时刻的后验估计与预积分的值来预测当前时刻的状态
        const PreIntegralSample &pre_oldest = _pre_buffer.oldest();
        _output_state.quat_nominal = _eskf._state.quat_nominal * pre_oldest.dr;
        _output_state.quat_nominal.normalize();
//        _output_state.r = _output_state.quat_nominal;

        _output_state.vel = _eskf._state.vel + _eskf._Rnb * pre_oldest.dv;
        _output_state.vel(2) += _eskf._state.grav * pre_oldest.dt;

        _output_state.pos = _eskf._state.pos + _eskf._Rnb * pre_oldest.dp + _eskf._state.vel * pre_oldest.dt;
        _output_state.pos(2) += 0.5f * _eskf._state.grav * pre_oldest.dt * pre_oldest.dt;

        _output_state.time_us = sample.time_us;
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::update_pre_integral(const ImuSample &sample) {
        // 更新预积分的值
        PreIntegralSample pre_sample;

        pre_sample.dt = 0.5f * (sample.delta_vel_dt + sample.delta_ang_dt);

        const Vector3f axis_angle = sample.delta_ang - _eskf._state.delta_ang_bias;
        rotation_from_axis_angle(pre_sample.dr, axis_angle);

        pre_sample.dv = sample.delta_vel - _eskf._state.delta_vel_bias;
        pre_sample.dp = 0.5f * pre_sample.dv * pre_sample.dt;

        _pre_buffer.push(pre_sample);
        for (unsigned char i = 0; i < _pre_buffer.size() - 1; ++i) {
            unsigned char index = (i < _pre_buffer.capacity() - _pre_buffer.oldest_index()) ? (_pre_buffer.oldest_index() + i) : (i - (_pre_buffer.capacity() - _pre_buffer.oldest_index()));

            const Vector3f drdv = _pre_buffer[index].dr * pre_sample.dv;
            _pre_buffer[index].dp += (_pre_buffer[index].dv + 0.5f * drdv) * pre_sample.dt;
            _pre_buffer[index].dv += drdv;
            _pre_buffer[index].dr *= pre_sample.dr;
            _pre_buffer[index].dt += pre_sample.dt;
        }
    }

    template<uint8_t DELAYS>
    template<uint8_t START, uint8_t END>
    void ESKFRunner<DELAYS>::uncorrelate_target_from_other_states() {
        for (uint8_t i = START; i < END; ++i) {
            // Upper triangular
            for (uint8_t j = START; j < i; ++j) {
                _eskf._P[j][i] = 0.f;
                _eskf._P[i][j] = 0.f;
            }

            // Columns
            for (uint8_t j = 0; j < START; ++j) {
                _eskf._P[j][i] = 0.f;
                _eskf._P[i][j] = 0.f;
            }

            // Rows
            for (uint8_t j = END; j < ESKF::DIM; ++j) {
                _eskf._P[i][j] = 0.f;
                _eskf._P[j][i] = 0.f;
            }
        }
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::set_imu_data(const ImuSample &imu_sample) {
        _imu_updated = false;

        if (_imu_buffer.is_empty()) {   // 第一次接收到imu数据
//            std::cout << "set_imu_data: case 1" << std::endl;

            _dt_imu_avg = 0.5f * (imu_sample.delta_ang_dt + imu_sample.delta_vel_dt);

            _imu_sample_last = imu_sample;

            // 下采样
            _imu_updated = _imu_down_sampler.update(_imu_sample_last);
//            std::cout << "_imu_updated = " << _imu_updated << std::endl;

            // 把下采样后的imu数据记录到buffer中
            if (_imu_updated) {

                _imu_buffer.push(_imu_down_sampler.get_down_sampled());
                _delta_pos_buffer.push(_imu_down_sampler.get_delta_pos());
                _imu_down_sampler.reset();

                // buffer内的imu数据的平均时间
                _min_obs_interval_us = _dt_imu_avg;

//            setDragData(imu_sample);
            }
        } else {    // buffer里已经有数据了
//            std::cout << "set_imu_data: case 2" << std::endl;

            float dt = 1e-6f * float(imu_sample.time_us - _imu_sample_last.time_us);
//            std::cout << "set_imu_data: dt = " << dt << std::endl;

            if (dt > 0.f) {
                _dt_imu_avg = 0.8f * _dt_imu_avg + 0.2f * dt;

                // 划桨补偿
                _imu_sample_last.delta_vel = imu_sample.delta_vel + 0.5f * imu_sample.delta_ang.cross(imu_sample.delta_vel) + 1.f/12.f * (_imu_sample_last.delta_ang.cross(imu_sample.delta_vel) + _imu_sample_last.delta_vel.cross(imu_sample.delta_ang));
                _imu_sample_last.delta_vel_dt = imu_sample.delta_vel_dt;

                // 圆锥补偿
                _imu_sample_last.delta_ang = imu_sample.delta_ang + 1.f/12.f * _imu_sample_last.delta_ang.cross(imu_sample.delta_ang);
                _imu_sample_last.delta_ang_dt = imu_sample.delta_ang_dt;

                memcpy(_imu_sample_last.delta_ang_clipping, imu_sample.delta_ang_clipping, sizeof(imu_sample.delta_ang_clipping));
                memcpy(_imu_sample_last.delta_vel_clipping, imu_sample.delta_vel_clipping, sizeof(imu_sample.delta_vel_clipping));
                _imu_sample_last.time_us = imu_sample.time_us;

                // 下采样
                _imu_updated = _imu_down_sampler.update(_imu_sample_last);

//                std::cout << "set_imu_data: _imu_updated = " << _imu_updated << std::endl;

                // 把下采样后的imu数据记录到buffer中
                if (_imu_updated) {

                    _imu_buffer.push(_imu_down_sampler.get_down_sampled());
                    _delta_pos_buffer.push(_imu_down_sampler.get_delta_pos());
                    _imu_down_sampler.reset();

                    // buffer内的imu数据的平均时间
                    if (_imu_buffer.size() > 1) {
                        _min_obs_interval_us = (_imu_sample_last.time_us - _imu_buffer.oldest().time_us) / (_imu_buffer.size() - 1);
                    } else {
                        _min_obs_interval_us = _dt_imu_avg;
                    }
//            setDragData(imu_sample);
                }
            } else {    // 数据异常
                return;
            }
        }
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::set_gps_data(const GpsMessage &gps, uint8_t i) {
        if (_imu_buffer.is_empty()) {   // imu_buffer没数据, 则不存储数据
            return;
        }

        // TODO: 若gps check不通过或者should_collect_gps=false, 是否应该不把数据存进buffer中?
        auto push_gps_data_to_buffer = [&]() {
            GpsSample gps_sample;

            gps_sample.time_us = gps.time_us;

            gps_sample.vel = gps.vel_ned;

            _gps_state.speed_valid[i] = gps.vel_ned_valid;
            gps_sample.sacc = gps.sacc;
            gps_sample.hacc = gps.eph;
            gps_sample.vacc = gps.epv;

            gps_sample.yaw = gps.yaw;

            gps_sample.fix_type = gps.fix_type;		// 保存fix_type

            if (PX4_ISFINITE(gps.yaw_offset)) {
                _gps_state.yaw_offset = gps.yaw_offset;

            } else {
                _gps_state.yaw_offset = 0.0f;
            }

            if (should_collect_gps(gps, i)) {
                gps_sample.pos_horz = _gps_state.pos_ref.project((gps.lat / 1.0e7), (gps.lon / 1.0e7));
                gps_sample.hgt = (float)gps.alt * 1e-3f - _gps_state.alt_ref;
//                std::cout << gps_sample.pos_horz(0) << ", " << gps_sample.pos_horz(1) << ", " << gps_sample.hgt << std::endl;
            } else {
                gps_sample.pos_horz = {0.f, 0.f};
                gps_sample.hgt = -_eskf._state.pos(2);
            }

            _gps_buffer[i].push(gps_sample);
        };

//        std::cout << "set_gps_time = " << gps_sample.time_us << std::endl;
        if (_gps_buffer[i].is_empty()) {
            push_gps_data_to_buffer();
        } else {
            // 限制gps_buffer记录数据的采样率, 使其采样率一定低于imu_buffer内的imu数据的平均时间
            if (gps.time_us - _gps_buffer[i].newest().time_us > _min_obs_interval_us) {
                push_gps_data_to_buffer();
            } else {
                return;
            }
        }

        // 修正采样时间
        _gps_buffer[i][_gps_buffer[i].newest_index()].time_us -= static_cast<uint64_t>(_params.gps_delay_ms * 1000) +
                static_cast<uint64_t>(_eskf._dt * 5e5f);
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::set_magnet_data(const MagSample &mag_sample) {
        if (_imu_buffer.is_empty()) {   // imu_buffer没数据, 则不存储数据
            return;
        }

        if (_mag_buffer.is_empty()) {
            _mag_buffer.push(mag_sample);
        } else {
            // 限制mag_buffer记录数据的采样率, 使其采样率一定低于imu_buffer内的imu数据的平均时间
            if (mag_sample.time_us - _mag_buffer.newest().time_us > _min_obs_interval_us) {
                _mag_buffer.push(mag_sample);
            } else {
                return;
            }
        }

        // 修正采样时间
        _mag_buffer[_mag_buffer.newest_index()].time_us -= static_cast<uint64_t>(_params.mag_delay_ms * 1000) +
                                                           static_cast<uint64_t>(_eskf._dt * 5e5f);
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::set_baro_date(const BaroSample &baro_sample) {
        if (_imu_buffer.is_empty()) {
            return;
        }

        if (_baro_buffer.is_empty()) {
            _baro_buffer.push(baro_sample);
        } else {
            // 限制baro_buffer记录数据的采样率, 使其采样率一定低于imu_buffer内的imu数据的平均时间
            if (baro_sample.time_us - _baro_buffer.newest().time_us > _min_obs_interval_us) {
                _baro_buffer.push(baro_sample);
            } else {
                return;
            }
        }

        // 修正采样时间
        _baro_buffer[_baro_buffer.newest_index()].time_us -= static_cast<uint64_t>(_params.baro_delay_ms * 1000) +
                                                             static_cast<uint64_t>(_eskf._dt * 5e5f);
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::set_range_data(const RangeSample &range_sample) {
        if (_imu_buffer.is_empty()) {
            return;
        }

        if (_range_buffer.is_empty()) {
            _range_buffer.push(range_sample);
        } else {
            // 限制range_buffer记录数据的采样率, 使其采样率一定低于imu_buffer内的imu数据的平均时间
            if (range_sample.time_us - _range_buffer.newest().time_us > _min_obs_interval_us) {
                _range_buffer.push(range_sample);
            } else {
                return;
            }
        }

        // 修正采样时间
        _range_buffer[_range_buffer.newest_index()].time_us -= static_cast<uint64_t>(_params.range_delay_ms * 1000) +
                                                               static_cast<uint64_t>(_eskf._dt * 5e5f);
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::set_flow_date(const FlowSample &flow_sample) {
        if (_imu_buffer.is_empty()) {
            return;
        }

        if (_flow_buffer.is_empty()) {
            _flow_buffer.push(flow_sample);
        } else {
            // 限制flow_buffer记录数据的采样率, 使其采样率一定低于imu_buffer内的imu数据的平均时间
            if (flow_sample.time_us - _flow_buffer.newest().time_us > _min_obs_interval_us) {
                _flow_buffer.push(flow_sample);
            } else {
                return;
            }
        }

        // 修正采样时间
        _flow_buffer[_flow_buffer.newest_index()].time_us -= static_cast<uint64_t>(_params.flow_delay_ms * 1000) +
                                                             static_cast<uint64_t>(_eskf._dt * 5e5f);
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::set_ev_data(const ExtVisionSample &ev_sample) {
        if (_imu_buffer.is_empty()) {
            return;
        }

        if (_ev_buffer.is_empty()) {
            _ev_buffer.push(ev_sample);
        } else {
            // 限制ev_buffer记录数据的采样率, 使其采样率一定低于imu_buffer内的imu数据的平均时间
            if (ev_sample.time_us - _ev_buffer.newest().time_us > _min_obs_interval_us) {
                _ev_buffer.push(ev_sample);
            } else {
                return;
            }
        }

        // 修正采样时间
        _ev_buffer[_ev_buffer.newest_index()].time_us -= static_cast<uint64_t>(_params.ev_delay_ms * 1000) +
                                                         static_cast<uint64_t>(_eskf._dt * 5e5f);
    }

    template<uint8_t DELAYS>
    bool ESKFRunner<DELAYS>::should_collect_gps(const GpsMessage &gps, uint8_t n) {
        auto reset_gps_drift_check_filters = [&]() {
            _gps_state.vel_horz_filt[n].setZero();
            _gps_state.pos_deriv_filt[n].setZero();

            _gps_state.pos_horz_deriv_filt_norm[n] = NAN;
            _gps_state.pos_vert_deriv_filt_norm[n] = NAN;
            _gps_state.vel_horz_filt_norm[n] = NAN;
        };

        auto gps_is_good = [&]() -> bool {
            // 3D及以上才能认为check通过
            _gps_state.check_fail_status[n].flags.fix = (gps.fix_type < 3);

            // 检查卫星数
            _gps_state.check_fail_status[n].flags.nsats = (gps.nsats < _params.req_nsats);

            // 位置的精度因子
            _gps_state.check_fail_status[n].flags.pdop = (gps.pdop > _params.req_pdop);

            // 水平位置误差与垂直位置误差
            _gps_state.check_fail_status[n].flags.hacc = (gps.eph > _params.req_hacc);
            _gps_state.check_fail_status[n].flags.vacc = (gps.epv > _params.req_vacc);

            // 速度误差
            _gps_state.check_fail_status[n].flags.sacc = (gps.sacc > _params.req_sacc);

            // 计算位置与速度的误差质量
            _gps_state.pos_horz_error_quality[n] = gps.eph / _params.req_hacc;
            _gps_state.pos_vert_error_quality[n] = gps.epv / _params.req_vacc;
            _gps_state.vel_error_quality[n] = gps.sacc / _params.req_sacc;

            // 一阶低通滤波器的前向欧拉实现
            constexpr float filt_time_const = 10.0f;
            const float dt = math::constrain(float(int64_t(_imu_sample_last.time_us) - int64_t(_gps_state.pos_prev[n].getProjectionReferenceTimestamp())) * 1e-6f, 0.001f, filt_time_const);
            const float filter_coef = dt / filt_time_const;

            // 一下检查只有在静止时才生效
            const double lat = gps.lat * 1.0e-7;
            const double lon = gps.lon * 1.0e-7;
            if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
                // 计算相对于上一次位置的位移
                float delta_pos_n = 0.0f;
                float delta_pos_e = 0.0f;
                if (_gps_state.pos_prev[n].getProjectionReferenceTimestamp() > 0) {
                    _gps_state.pos_prev[n].project(lat, lon, delta_pos_n, delta_pos_e);

                } else {
                    // 初始化
                    _gps_state.pos_prev[n].initReference(lat, lon, _imu_sample_last.time_us);
                    _gps_state.alt_prev[n] = 1e-3f * (float)gps.alt;
                }

                // 计算位置的变化率, 并限制其大小
                const Vector3f vel_limit(_params.req_hdrift, _params.req_hdrift, _params.req_vdrift);
                Vector3f pos_derived(delta_pos_n, delta_pos_e, (_gps_state.alt_prev[n] - 1e-3f * (float)gps.alt));
                pos_derived = matrix::constrain(pos_derived / dt, -10.0f * vel_limit, 10.0f * vel_limit);

                // 进行低通滤波
                _gps_state.pos_deriv_filt[n] = pos_derived * filter_coef + _gps_state.pos_deriv_filt[n] * (1.0f - filter_coef);

                // 检查水平位置的变化率是否过高
                _gps_state.pos_horz_deriv_filt_norm[n] = Vector2f(_gps_state.pos_deriv_filt[n].xy()).norm();
                _gps_state.check_fail_status[n].flags.hdrift = (_gps_state.pos_horz_deriv_filt_norm[n] > _params.req_hdrift);

                // 检查垂直位置的变化率是否过高
                _gps_state.pos_vert_deriv_filt_norm[n] = fabsf(_gps_state.pos_deriv_filt[n](2));
                _gps_state.check_fail_status[n].flags.vdrift = (_gps_state.pos_vert_deriv_filt_norm[n] > _params.req_vdrift);

                // 检查水平速度是否过大
                const Vector2f gps_vel_horz = matrix::constrain(Vector2f(gps.vel_ned.xy()),
                                                                -10.0f * _params.req_hdrift,
                                                                10.0f * _params.req_hdrift);
                _gps_state.vel_horz_filt[n] = gps_vel_horz * filter_coef + _gps_state.vel_horz_filt[n] * (1.0f - filter_coef);
                _gps_state.vel_horz_filt_norm[n] = _gps_state.vel_horz_filt[n].norm();
                _gps_state.check_fail_status[n].flags.hspeed = (_gps_state.vel_horz_filt_norm[n] > _params.req_hdrift);

            } else if (_control_status.flags.in_air) {
                // 以下的量在飞机起飞后将不在判断
                _gps_state.check_fail_status[n].flags.hdrift = false;
                _gps_state.check_fail_status[n].flags.vdrift = false;
                _gps_state.check_fail_status[n].flags.hspeed = false;

                reset_gps_drift_check_filters();

            } else {
                // 飞机没有起飞, 但是飞机非静止状态
                reset_gps_drift_check_filters();
            }

            // 保存当前时刻的gps位置
            _gps_state.pos_prev[n].initReference(lat, lon, _imu_sample_last.time_us);
            _gps_state.alt_prev[n] = 1e-3f * (float)gps.alt;

            // 检查gps的垂直速度与eskf的垂直速度的差异
            const float vz_diff_limit = 10.0f * _params.req_vdrift;
            const float diff_vel_z = math::constrain(gps.vel_ned(2) - _eskf._state.vel(2), -vz_diff_limit, vz_diff_limit);
            _gps_state.vel_vert_diff_filt[n] = diff_vel_z * filter_coef + _gps_state.vel_vert_diff_filt[n] * (1.0f - filter_coef);
            _gps_state.check_fail_status[n].flags.vspeed = (fabsf(_gps_state.vel_vert_diff_filt[n]) > _params.req_vdrift);

            // 假设gps最初的检查不通过
            if (_gps_state.last_fail_us[n] == 0) {
                _gps_state.last_fail_us[n] = _imu_sample_last.time_us;
            }

            // 判断所选择的检查是否失败, 并记录时间
            if (_gps_state.check_fail_status[n].flags.fix ||
                (_gps_state.check_fail_status[n].flags.nsats   && (_params.gps_check_mask & RunnerParameters::CHECK_GPS_NSATS)) ||
                (_gps_state.check_fail_status[n].flags.pdop    && (_params.gps_check_mask & RunnerParameters::CHECK_GPS_PDOP)) ||
                (_gps_state.check_fail_status[n].flags.hacc    && (_params.gps_check_mask & RunnerParameters::CHECK_GPS_HACC)) ||
                (_gps_state.check_fail_status[n].flags.vacc    && (_params.gps_check_mask & RunnerParameters::CHECK_GPS_VACC)) ||
                (_gps_state.check_fail_status[n].flags.sacc    && (_params.gps_check_mask & RunnerParameters::CHECK_GPS_SACC)) ||
                (_gps_state.check_fail_status[n].flags.hdrift  && (_params.gps_check_mask & RunnerParameters::CHECK_GPS_HDRIFT)) ||
                (_gps_state.check_fail_status[n].flags.vdrift  && (_params.gps_check_mask & RunnerParameters::CHECK_GPS_VDRIFT)) ||
                (_gps_state.check_fail_status[n].flags.hspeed  && (_params.gps_check_mask & RunnerParameters::CHECK_GPS_HSPD)) ||
                (_gps_state.check_fail_status[n].flags.vspeed  && (_params.gps_check_mask & RunnerParameters::CHECK_GPS_VSPD))
                ) {
                _gps_state.last_fail_us[n] = _imu_sample_last.time_us;

            } else {
                _gps_state.last_pass_us[n] = _imu_sample_last.time_us;
            }

            // 只有检查失败已经过去一段时间, 才能认为检查成功
            return is_timeout(_gps_state.last_fail_us[n], RunnerParameters::MIN_GPS_HEALTH);
        };

        // 检查gps
        _gps_state.checks_passed[n] = gps_is_good();

        if (_rough_alignment_completed && !_gps_state.ned_origin_initialised && _gps_state.checks_passed[n]) {
            // 如果gps数据检查通过, 则初始化原点
            const double lat = gps.lat * 1.0e-7;
            const double lon = gps.lon * 1.0e-7;

            // 计算eskf估计的天线位置
            const Vector3f offset_nav = _eskf._Rnb * _params.gps_pos_body[n];

            if (!_gps_state.pos_ref.isInitialized()) {
                _gps_state.pos_ref.initReference(lat, lon, _imu_sample_last.time_us);

                // 如果在初始化原点前, 已经开始了融合, 则利用eskf的pos与经纬度计算一个等效的水平原点
                if (is_horizontal_aiding_active()) {
                    double est_lat;
                    double est_lon;
                    _gps_state.pos_ref.reproject(-(_eskf._state.pos(0) + offset_nav(0)), -(_eskf._state.pos(1) + offset_nav(1)), est_lat, est_lon);
                    _gps_state.pos_ref.initReference(est_lat, est_lon, _imu_sample_last.time_us);
                }
            }

            // 利用eskf的pos和高程计算等效的垂直原点, 其中alt_ref - eskf = alt, 则alt_ref = alt + eskf
            _gps_state.alt_ref = 1e-3f * (float)gps.alt + _eskf._state.pos(2) + offset_nav(2);
            _gps_state.ned_origin_initialised = true;

            _gps_state.earth_rate_ned = calc_earth_rate_ned((float)math::radians(_gps_state.pos_ref.getProjectionReferenceLat()));
            _gps_state.last_origin_us = _imu_sample_last.time_us;

            const bool declination_was_valid = PX4_ISFINITE(_gps_state.mag_declination);

            // 根据经纬度查表得到本地的磁场
            _gps_state.mag_declination = get_mag_declination_radians((float)lat, (float)lon);
            _gps_state.mag_inclination = get_mag_inclination_radians((float)lat, (float)lon);
            _gps_state.mag_strength = get_mag_strength_gauss((float)lat, (float)lon);

//            // request a reset of the yaw using the new declination
//            if ((_params.mag_body_fusion_type != MAG_FUSE_TYPE_NONE)
//                && !declination_was_valid) {
//                _mag_yaw_reset_req = true;
//            }

//            std::cout << "lat: " << _gps_state.pos_ref.getProjectionReferenceLat() << std::endl;
//            std::cout << "lon: " << _gps_state.pos_ref.getProjectionReferenceLon() << std::endl;

            // 保存原点的误差
            _gps_state.origin_unc_pos_horz = gps.eph;
            _gps_state.origin_unc_pos_vert = gps.epv;

//            _information_events.flags.gps_checks_passed = true;
//            ECL_INFO("GPS checks passed");

        } else if (!_gps_state.ned_origin_initialised) {
            // 在2D模式下, 当经纬度误差满足一定条件时, 依然可用查表出本地磁场
            if ((gps.fix_type >= 2) && (gps.eph < 1000)) {
                const bool declination_was_valid = PX4_ISFINITE(_gps_state.mag_declination);

                // 当前的经纬度
                const double lat = gps.lat * 1.0e-7;
                const double lon = gps.lon * 1.0e-7;

                // 根据经纬度查表得到本地的磁场
                _gps_state.mag_declination = get_mag_declination_radians((float)lat, (float)lon);
                _gps_state.mag_inclination = get_mag_inclination_radians((float)lat, (float)lon);
                _gps_state.mag_strength = get_mag_strength_gauss((float)lat, (float)lon);

                // request mag yaw reset if there's a mag declination for the first time
                if (_params.mag_body_fusion_type != RunnerParameters::MAG_FUSE_TYPE_NONE) {
                    if (!declination_was_valid && PX4_ISFINITE(_gps_state.mag_declination)) {
                        _mag_align_req = true;
                    }
                }

                _gps_state.earth_rate_ned = calc_earth_rate_ned((float)math::radians(lat));
            }
        }

        // 若模式大于等于3D, 且原点已经被初始化, 则可用收集gps数据
        return _gps_state.ned_origin_initialised && (gps.fix_type >= 3);
    }



//    template<uint8_t DELAYS>
//    void ESKFRunner<DELAYS>::set_airspeed_data(const float &true_airspeed, const float &eas2tas, const unsigned long &time_us) {
//        AirspeedSample sample;
//        sample.true_airspeed = true_airspeed;
//        sample.eas2tas = eas2tas;
//        sample.time_us = time_us;
//
//        _airspeed_buffer.push(sample);
//    }
}

#endif //ECL_ESKF_RUNNER_H
