//
// Created by Cain on 2023/1/6.
//

#ifndef ECL_ESKF_RUNNER_H
#define ECL_ESKF_RUNNER_H

#include "eskf.h"
#include "common.h"
#include "utils.h"
#include "imu_down_sampler.h"

#ifndef SINGLE_FUSE
#define MULTI_FUSE
#endif

namespace eskf {
    template <uint8_t DELAYS>
    class ESKFRunner {
    public:
        static constexpr uint8_t N_GPS {2}; ///< 总共有几个GPS数据源

        explicit ESKFRunner(ESKF &eskf) : _eskf(eskf), _imu_down_sampler(eskf._params.eskf_update_interval_us) {

        };

        void update();

        void set_imu_data(const ImuSample &imu_sample);
        void set_gps_data(const GpsSample &gps_sample);
        void set_magnet_data(const MagSample &mag_sample);
        void set_baro_date(const BaroSample &baro_sample);
        void set_range_data(const RangeSample &range_sample);
        void set_flow_date(const FlowSample &flow_sample);
        void set_ev_data(const ExtVisionSample &ev_sample);

        const OutputSample &get_output_state() { return _output_state; };

    private:
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
         */
        void check_vertical_acceleration_health();

        void control_height_sensor_timeouts();

        void control_mag_fuse();
        void check_mag_field_strength(const MagSample &sample);
        bool is_measured_matching_expected(const float &measured, const float &expected, const float &gate);
        bool no_other_yaw_aiding_than_mag();

        void update_pre_integral(const ImuSample &sample);
        void predict_state_from_delay(const ImuSample &sample);

        bool is_recent(uint64_t &sensor_timestamp, uint64_t &acceptance_interval) const {
            return sensor_timestamp + acceptance_interval > _imu_sample_last.time_us;
        }

        bool is_timeout(uint64_t &sensor_timestamp, uint64_t &timeout_period) const {
            return sensor_timestamp + timeout_period < _imu_sample_last.time_us;
        }

        RunnerParameters _params {};    ///< 传感器参数

        ESKF &_eskf;    ///< 卡尔曼滤波器

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

        ImuDownSampler _imu_down_sampler;   ///< imu下采样器

        bool _imu_updated {false};  ///< imu是否更新
        ImuSample _imu_sample_last {};  ///< 最新的imu采样

        float _dt_imu_avg {};   ///< imu平均采样时间

        float _min_obs_interval_us {};  ///< imu_buffer的平均时间间隔

        // 最后一次在空中和在地上
        uint64_t _time_last_in_air {};     ///< 最后一次在空中的时间
        uint64_t _time_last_in_ground {};  ///< 最后一次在地上的时间

        ImuSample _imu_sample_delay {};
        GpsSample _gps_sample_delay {};
        BaroSample _baro_sample_delay {};
        RangeSample _range_sample_delay {};
        FlowSample _flow_sample_delay {};
        ExtVisionSample _ev_sample_delay {};
        MagSample _mag_sample_delay {};

        OutputSample _output_state {};

        // 用于滞后补偿
        Queue<ImuSample, DELAYS> _imu_buffer;
        Queue<Vector3f, DELAYS> _delta_pos_buffer;
        Queue<GpsSample, DELAYS> _gps_buffer;
        Queue<MagSample, DELAYS> _mag_buffer;
        Queue<BaroSample, DELAYS> _baro_buffer;
        Queue<RangeSample, DELAYS> _range_buffer;
        Queue<FlowSample, DELAYS> _flow_buffer;
        Queue<ExtVisionSample, DELAYS> _ev_buffer;
        Queue<PreIntegralSample, DELAYS> _pre_buffer;

        // 数据标记
        bool _gps_data_ready;
        bool _baro_data_ready;
        bool _range_data_ready;
        bool _flow_data_ready;
        bool _ev_data_ready;
        bool _mag_data_ready;

        bool _gps_intermittent;
        bool _baro_intermittent;
        bool _range_intermittent;
        bool _flow_intermittent;
        bool _ev_intermittent;
        bool _mag_intermittent;

        uint64_t _time_prev_gps_us;
        uint64_t _time_prev_baro_us;
        uint64_t _time_prev_range_us;
        uint64_t _time_prev_flow_us;
        uint64_t _time_prev_ev_us;
        uint64_t _time_prev_mag_us;

        /* 垂直加速度量测异常检测所用到的变量 */
        uint16_t _clip_counter {0};
        uint64_t _time_bad_vert_accel {0};
        uint64_t _time_good_vert_accel {0};


        filter_control_status_u _control_status {};
        fault_status_u _fault_status {};

    };

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::update() {
        if (_imu_updated) { // imu更新了同时意味着imu_buffer非空
            if (_imu_buffer.is_full()) {    // 只有当imu_buffer满了之后才运行eskf, 以保证永远在滞后的时刻进行融合
                // 取最滞后的imu数据
                const ImuSample &imu_sample = _imu_buffer.oldest();
                _imu_sample_delay = imu_sample;

                // 在滞后时刻进行先验估计
                _eskf.predict_state(imu_sample);
                _eskf.predict_covariance(imu_sample);

                // 对GPS数据进行处理与校验
                _time_prev_gps_us = _gps_sample_delay.time_us;
                _gps_intermittent = !is_recent(_gps_buffer.newest().time_us, RunnerParameters::GPS_MAX_INTERVAL);
                _gps_data_ready = _gps_buffer.pop_first_older_than(imu_sample.time_us, _gps_sample_delay);
                if (_gps_data_ready){
                    //TODO: 进行gps数据的计算, 比如offset_nav, offset_cross_gyro
//                    _control_status.flags.gps_hgt = !_gps_intermittent && _gps_check_passed;
//                    _control_status.flags.gps_pos_horz = _control_status.flags.gps_hgt;
//                    _control_status.flags.gps_vel_vert = _control_status.flags.gps_hgt;
//                    _control_status.flags.gps_vel_horz = _control_status.flags.gps_hgt;
                } else {
//                    _control_status.flags.gps_hgt = false;
//                    _control_status.flags.gps_pos_horz = false;
//                    _control_status.flags.gps_vel_vert = false;
//                    _control_status.flags.gps_vel_horz = false;
                }

                // 对气压计数据进行处理与校验
                _time_prev_baro_us = _baro_sample_delay.time_us;
                _baro_intermittent = !is_recent(_baro_buffer.newest().time_us, RunnerParameters::BARO_MAX_INTERVAL);
                _baro_data_ready = _baro_buffer.pop_first_older_than(imu_sample.time_us, _baro_sample_delay);
                if (_baro_data_ready) {
                    //TODO: baro的采样间隔(_baro_sample_delay.time_us - _time_prev_baro_us)需要用于气压计偏移估计
//                    _control_status.flags.baro_hgt = !_baro_intermittent && _baro_check_passed;
                } else {
//                    _control_status.flags.baro_hgt = false;
                }

                // 对测距仪数据进行处理与校验
                _time_prev_range_us = _range_sample_delay.time_us;
                _range_intermittent = !is_recent(_range_buffer.newest().time_us, RunnerParameters::RANGE_MAX_INTERVAL);
                _range_data_ready = _range_buffer.pop_first_older_than(imu_sample.time_us, _range_sample_delay);
                if (_range_data_ready) {
                    // TODO: something
//                    _control_status.flags.rng_hgt = !_range_intermittent && _range_check_passed;
                } else {
//                    _control_status.flags.rng_hgt = false;
                }

                // 对光流数据进行处理与校验
                _time_prev_flow_us = _flow_sample_delay.time_us;
                _flow_intermittent = !is_recent(_flow_buffer.newest().time_us, RunnerParameters::FLOW_MAX_INTERVAL);
                _flow_data_ready = _flow_buffer.pop_first_older_than(imu_sample.time_us, _flow_sample_delay);
                if (_flow_data_ready) {
//                    _control_status.flags.opt_flow = !_flow_intermittent && _flow_check_passed;
                } else {
//                    _control_status.flags.opt_flow = false;
                }

                // 对外部视觉数据进行处理与校验
                _time_prev_ev_us = _ev_sample_delay.time_us;
                _ev_intermittent = !is_recent(_ev_buffer.newest().time_us, RunnerParameters::EV_MAX_INTERVAL);
                _ev_data_ready = _ev_buffer.pop_first_older_than(imu_sample.time_us, _ev_sample_delay);
                if (_ev_data_ready) {
                    // TODO: something;
//                    _control_status.flags.ev
                }

                // 对磁力计数据进行处理与校验
                _time_prev_mag_us = _mag_sample_delay.time_us;
                _mag_intermittent = !is_recent(_mag_buffer.newest().time_us, RunnerParameters::MAG_MAX_INTERVAL);
                _mag_data_ready = _mag_buffer.pop_first_older_than(imu_sample.time_us, _mag_sample_delay);
                if (_mag_data_ready) {
                    // TODO: something;
                }



                // 修正滞后时刻的状态与协方差
                _eskf.correct_state();
                // _eskf.correct_covariance();
            }

            // 利用滞后时刻的后验估计值和当前时刻的imu值预测当前时刻状态
            predict_state_from_delay(_imu_buffer.newest());

            _imu_updated = false;
        }
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::check_vertical_acceleration_health() {
        bool is_inertial_nav_falling = false;
        bool are_vertical_pos_and_vel_independent = false;

        if (is_recent(_vert_pos_fuse_attempt_time_us, 1000000)) {   // 1秒内是否尝试过进行垂直位置融合(不一定真的融合了, 可能数据rejected了)
            if (isRecent(_vert_vel_fuse_time_us, 1000000)) {    // 1秒内是否进行垂直速度融合
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

        if (bad_vert_accel) {
            _time_bad_vert_accel = _imu_sample_delay.time_us;
        } else {
            _time_good_vert_accel = _imu_sample_delay.time_us;
        }

        // 在最近BADACC_PROBATION(us)时间内, 均没有出现过垂直加速度异常, 才认为垂直加速度正常, 否则是为异常
        _fault_status.flags.bad_acc_vertical = is_recent(_time_bad_vert_accel, RunnerParameters::BADACC_PROBATION);
//        if (_fault_status.flags.bad_acc_vertical) {
//            _fault_status.flags.bad_acc_vertical = is_recent(_time_bad_vert_accel, _params.BADACC_PROBATION);
//        } else {
//            _fault_status.flags.bad_acc_vertical = bad_vert_accel;
//        }
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::control_height_sensor_timeouts() {
        /*
         * Handle the case where we have not fused height measurements recently and
         * uncertainty exceeds the max allowable. Reset using the best available height
         * measurement source, continue using it after the reset and declare the current
         * source failed if we have switched.
        */

        check_vertical_acceleration_health();

        const bool hgt_fusion_timeout = is_timeout(_time_last_gps_hgt_fuse, RunnerParameters::HGT_FUSE_TIMEOUT) &&
                                        is_timeout(_time_last_baro_fuse, RunnerParameters::HGT_FUSE_TIMEOUT) &&
                                        is_timeout(_time_last_range_fuse, RunnerParameters::HGT_FUSE_TIMEOUT) &&
                                        is_timeout(_time_last_ev_hgh_fuse, RunnerParameters::HGT_FUSE_TIMEOUT);

        /*
         * 若长时间没进行过高度数据的融合或者垂直加速度的量测异常(量测异常会导致eskf的高度推算产生错误), 则需要重置eskf中的高度状态
         * */
        if (hgt_fusion_timeout || _fault_status.flags.bad_acc_vertical) {
            const char *failing_height_source = nullptr;
            const char *new_height_source = nullptr;

            if (_control_status.flags.baro_hgt) {
                /*
                 * 若气压计的高度源异常, 则需要切换高度源
                 * 若气压计的高度源没有异常:
                 *      1. 若加速度质量差, 则不切换高度源, 继续用气压计的高度来重置
                 *      2. 若加速度质量好, 且gps卫星足够多, 则切换到gps高度源
                 * */
                bool reset_to_gps = false;
                if (!_gps_intermittent) {
                    reset_to_gps = (_gps_checks_passed && !_fault_status.flags.bad_acc_vertical) || _baro_hgt_faulty || _baro_hgt_intermittent;
                }

                if (reset_to_gps) {
                    // 如果把_baro_hgt_faulty设置为true, 则再也无法切回气压计
//                    _baro_hgt_faulty = true;

                    start_gps_hgt_fusion();

                    failing_height_source = "baro";
                    new_height_source = "gps";

                } else if (!_baro_hgt_faulty && !_baro_hgt_intermittent) {
                    reset_height_to_baro();

                    failing_height_source = "baro";
                    new_height_source = "baro";
                }

            } else if (_control_status.flags.gps_hgt) {
                /*
                 * 若GPS的高度源异常, 则需要切换高度源
                 * 若GPS的高度源没有异常:
                 *      1. 若加速度质量差, 则不切换高度源, 继续用GPS的高度来重置
                 *      2. 若加速度质量好, 且gps卫星数不够, 则切换到气压计高度源
                 * */
                bool reset_to_baro = false;
                if (!_baro_hgt_faulty && !_baro_hgt_intermittent) {
                    reset_to_baro = (!_fault_status.flags.bad_acc_vertical && !_gps_checks_passed) || _gps_intermittent;
                }

                if (reset_to_baro) {
                    start_baro_hgt_fusion();

                    failing_height_source = "gps";
                    new_height_source = "baro";

                } else if (!_gps_intermittent) {
                    reset_height_to_gps();

                    failing_height_source = "gps";
                    new_height_source = "gps";
                }

            } else if (_control_status.flags.rng_hgt) {
                /*
                 * 若RNG的高度源异常, 则需要切换高度源
                 * 若RNG的高度源没有异常, 则无论加速度好还是环, 都用RNG的高度来重置
                 * */
                if (_range_sensor.is_healthy()) {
                    reset_height_to_rng();

                    failing_height_source = "rng";
                    new_height_source = "rng";

                } else if (!_baro_hgt_faulty && !_baro_hgt_intermittent) {
                    start_baro_hgt_fusion();

                    failing_height_source = "rng";
                    new_height_source = "baro";
                }

            } else if (_control_status.flags.ev_hgt) {
                /*
                 * 若EV的高度源异常, 则需要切换高度源
                 * 若EV的高度源没有异常, 则无论加速度好还是环, 都用RNG的高度来重置
                 * */

                if (!_ev_intermittent) {
                    reset_height_to_ev();

                    failing_height_source = "ev";
                    new_height_source = "ev";

                } else if (_range_sensor.is_healthy()) {
                    // Fallback to rangefinder data if available
                    start_rng_hgt_fusion();

                    failing_height_source = "ev";
                    new_height_source = "rng";

                } else if (!_baro_hgt_faulty && !_baro_hgt_intermittent) {
                    start_baro_hgt_fusion();

                    failing_height_source = "ev";
                    new_height_source = "baro";
                }
            }

            if (failing_height_source && new_height_source) {
//                _warning_events.flags.height_sensor_timeout = true;
//                ECL_WARN("%s hgt timeout - reset to %s", failing_height_source, new_height_source);
            }

            // Also reset the vertical velocity
            if (_control_status.flags.gps_vel && !_gps_intermittent && _gps_checks_passed
                && !is_timeout(_gps_sample_delay.time_us, RunnerParameters::HGT_FUSE_TIMEOUT)) {
                reset_vertical_velocity_to_gps(_gps_sample_delayed);
            } else if (_control_status.flags.ev_vel && !_ev_intermittent
                       && !is_timeout(_ev_sample_delay.time_us, RunnerParameters::HGT_FUSE_TIMEOUT)) {
                reset_vertical_velocity_to_ev(_ev_sample_delay);
            } else {
                reset_vertical_velocity_to_zero();
            }

        }
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::control_mag_fuse() {
        if (_mag_data_ready) {
            if (_params.check_mag_strength) {
                check_mag_field_strength(_mag_sample_delay);
            } else {
                _control_status.flags.mag_field_disturbed = false;
            }

            _is_yaw_fusion_inhibited = false;
            if (no_other_yaw_aiding_than_mag()) {
                // 如果存在出了磁场力计以外的航向数据信息, 则不使用地球磁场数据融合,
                // 因为在此情况下已经保证所有状态可观, 无论是否使用磁力计数据融合
                stop_mag_earth_fusion();

                // 如果不使用磁力计, 则需要停止磁力计偏移的融合
                if (_control_status.flags.mag_body) {
                    start_mag_body_fusion();
                } else {
                    stop_mag_body_fusion();
                }

            } else {
                // 若没有除了磁场力计以外的航向数据信息, 为了保证所有状态可观, 则必须使用地球磁场数据融合

                if (_control_status.flags.mag_field_disturbed) {
                    // 若磁力计量测的模值与地球磁场的模值差异很大, 则说明此时受到了强磁干扰, 不能够使用地球磁场数据融合
                    // 此时z轴旋转不可观
                    stop_mag_earth_fusion();
                    _is_yaw_fusion_inhibited = true;
                } else {
                    // 使用地球磁场数据融合, 此时若使用磁力计数据进行融合, 则所有状态可观
                    start_mag_earth_fusion();
                }

                if (_control_status.flags.mag_body) {
                    start_mag_body_fusion();
                } else {
                    // 如果不使用磁力计, 则需要停止磁力计偏移的融合, 则z轴旋转与z轴角速度偏移一定不可观
                    stop_mag_body_fusion();
                    _is_yaw_fusion_inhibited = true;
                }
            }
        }
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::check_mag_field_strength(const MagSample &mag_sample) {
        const float mag_sample_length = mag_sample.mag.length();
        if (PX4_ISFINITE(_mag_strength_gps)) {
            constexpr float wmm_gate_size = 0.2f; // +/- Gauss
            _control_status.flags.mag_field_disturbed = !is_measured_matching_expected(mag_sample_length, _mag_strength_gps, wmm_gate_size);
        } else {
            constexpr float average_earth_mag_field_strength = 0.45f; // Gauss
            constexpr float average_earth_mag_gate_size = 0.40f; // +/- Gauss
            _control_status.flags.mag_field_disturbed = !is_measured_matching_expected(mag_sample_length, average_earth_mag_field_strength, average_earth_mag_gate_size);
        }
    }

    template<uint8_t DELAYS>
    bool ESKFRunner<DELAYS>::no_other_yaw_aiding_than_mag() {
        return !(_control_status.flags.gps_horz && _n_gps > 1);
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::predict_state_from_delay(const ImuSample &sample) {
        // 只要有新的imu的值, 就要更新预积分的值
        update_pre_integral(sample);

        // 利用滞后时刻的后验估计与预积分的值来预测当前时刻的状态
        const PreIntegralSample &pre_oldest = _pre_buffer.oldest();
        _output_state.q = _eskf._q * pre_oldest.dr;
        _output_state.q.normalize();
        _output_state.r = _output_state.q;

        _output_state.v = _eskf._v + _eskf._rot * pre_oldest.dv;
        _output_state.v[2] += _eskf._g * pre_oldest.dt;

        _output_state.p = _eskf._p + _eskf._rot * pre_oldest.dp + _eskf._v * pre_oldest.dt;
        _output_state.p[2] += 0.5f * _eskf._g * pre_oldest.dt * pre_oldest.dt;

        _output_state.time_us = sample.time_us;
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::update_pre_integral(const ImuSample &sample) {
        // 更新预积分的值
        PreIntegralSample pre_sample;

        pre_sample.dt = 0.5f * (sample.delta_vel_dt + sample.delta_ang_dt);

        const Vector3f axis_angle = sample.delta_ang - _eskf._bg * sample.delta_ang_dt;
        rotation_from_axis_angle(pre_sample.dr, axis_angle);

        pre_sample.dv = sample.delta_vel - _eskf._ba * sample.delta_vel_dt;
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
    void ESKFRunner<DELAYS>::set_imu_data(const ImuSample &imu_sample) {
        _imu_updated = false;

        if (_imu_buffer.is_empty()) {   // 第一次接收到imu数据
            _dt_imu_avg = 0.5f * (imu_sample.delta_ang_dt + imu_sample.delta_vel_dt);

            _imu_sample_last = imu_sample;

            // 下采样
            _imu_updated = _imu_down_sampler.update(_imu_sample_last);

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
            float dt = 1e-6f * float(imu_sample.time_us - _imu_sample_last.time_us);
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

                // 把下采样后的imu数据记录到buffer中
                if (_imu_updated) {

                    _imu_buffer.push(_imu_down_sampler.get_down_sampled());
                    _delta_pos_buffer.push(_imu_down_sampler.get_delta_pos());
                    _imu_down_sampler.reset();

                    // buffer内的imu数据的平均时间
                    _min_obs_interval_us = (_imu_sample_last.time_us - _imu_buffer.get_oldest().time_us) / (_imu_buffer.size() - 1);

//            setDragData(imu_sample);
                }
            } else {    // 数据异常
                return;
            }
        }
    }

    template<uint8_t DELAYS>
    void ESKFRunner<DELAYS>::set_gps_data(const GpsSample &gps_sample) {
        if (_imu_buffer.is_empty()) {   // imu_buffer没数据, 则不存储数据
            return;
        }

        if (_gps_buffer.is_empty()) {
            _gps_buffer.push(gps_sample);
        } else {
            // 限制gps_buffer记录数据的采样率, 使其采样率一定低于imu_buffer内的imu数据的平均时间
            if (gps_sample.time_us - _gps_buffer.newest().time_us > _min_obs_interval_us) {
                _gps_buffer.push(gps_sample);
            } else {
                return;
            }
        }

        // 修正采样时间
        _gps_buffer[_gps_buffer.newest_index()].time_us -= static_cast<uint64_t>(_params.gps_delay_ms * 1000) +
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
