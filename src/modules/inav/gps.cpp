//
// Created by Cain on 2023/3/6.
//

#include "gps.h"
#include "inav.h"

namespace inav {
    float GPS::_alt_ref = 0.f;
    Vector2f GPS::_horz_ref {};

    GPS::GPS(INAV *inav, uint8_t buffer_size) : Sensor(inav, buffer_size), _hgt_aiding_interface(this),
                                                _horz_aiding_interface(this), _vel_vert_aiding_interface(this), _vel_horz_aiding_interface(this) {

    }

    void GPS::update() {
        cout << "GPS:update" << endl;

        // 把数据加入队列
        cout << "push data into deque" << endl;

        // 从队列取delayed数据
        cout << "pop delayed data from deque" << endl;

//        _rtk = true;
        _rtk = _sample_delay.fix_type >= 5;

        if (_checks_passed) {
            _time_last_pass = _inav->_time_current;
        } else {
            _time_last_fail = _inav->_time_current;
        }
        _checks_passing = _inav->is_timeout(_time_last_fail, (uint64_t)5e6);
        _checks_failing = _inav->is_timeout(_time_last_pass, (uint64_t)5e6);

//        _inav->_control_status.flags.tilt_align = true;
//        _inav->_control_status.flags.yaw_align = true;
//        _ned_origin_initialised = true;
//        _checks_passing = true;
//        _checks_failing = false;

        // 异常检测
        _hgt_aiding_interface.anomaly_detection();
        _horz_aiding_interface.anomaly_detection();
        _vel_vert_aiding_interface.anomaly_detection();
        _vel_horz_aiding_interface.anomaly_detection();
    }


    void GpsHgtAidingInterface::fuse() {
        cout << "fuse gps hgt data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
            //        auto *sample_delay = (GpsSample *)(gps->_sample_delay);
            // use   _sample_delay->hgt  to fuse
            auto &sample_delay = gps->_sample_delay;
//            gps->_inav->_eskf.fuse_pos_vert(-sample_delay->hgt, gps->_offset_body, gps->_offset_nav, _reject_gate, _meas_noise, fuse_data);
        }

    }

    void GpsHgtAidingInterface::reset() {
        cout << "reset gps hgt data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));

            _reset_req = false;
        }
    }

    void GpsHgtAidingInterface::check_reset_req() {
        auto gps = (GPS *)_sensor;
        if (!_anomaly) {
            _reset_req |= gps->_inav->_fault_status.flags.bad_acc_vertical;
        }
    }

    void GpsHgtAidingInterface::anomaly_detection() {
        cout << "anomaly detection of gps hgt" << endl;
        auto gps = (GPS *)_sensor;

        if (!gps->_inav->_fault_status.flags.bad_acc_vertical && _actived) {
            _anomaly = gps->_inav->is_recent(_time_last_fuse, RunnerParameters::HGT_FUSE_TIMEOUT);

            if (gps->_inav->is_recent(_time_last_fuse, RunnerParameters::HGT_FUSE_TIMEOUT)) {
                cout << "gps->_inav->is_recent(_time_last_fuse, RunnerParameters::HGT_FUSE_TIMEOUT)" << endl;
            }
        }

//        gps->_checks_passed = true;
        _anomaly = gps->_intermittent || !gps->_checks_passed;

        if (gps->_intermittent || !gps->_checks_passed) {
            cout << "gps->_intermittent || !gps->_checks_passed" << endl;
        }

        if (_anomaly) {
            cout << "gps hgt is anomaly" << endl;
//            _buffer.clear();
            gps->_data_ready = false;
        }
    }


    void GpsHorzAidingInterface::fuse() {
        cout << "fuse gps horz data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
            //        auto *sample_delay = (GpsSample *)(gps->_sample_delay);
            // use   _sample_delay->hgt  to fuse
            auto &sample_delay = gps->_sample_delay;
//            gps->_inav->_eskf.fuse_pos_vert(-sample_delay->hgt, gps->_offset_body, gps->_offset_nav, _reject_gate, _meas_noise, fuse_data);
        }
    }

    void GpsHorzAidingInterface::reset() {
        cout << "reset gps horz data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));

            _reset_req = false;
        }
    }

    void GpsHorzAidingInterface::check_reset_req() {
        auto gps = (GPS *)_sensor;
        if (!_anomaly) {
            _reset_req |= gps->_inav->is_timeout(_time_last_fuse, RunnerParameters::HORZ_FUSE_TIMEOUT);
        }
    }

    void GpsHorzAidingInterface::anomaly_detection() {
        cout << "anomaly detection of gps horz" << endl;
        auto gps = (GPS *)_sensor;
        auto inav = gps->_inav;

        const bool initialised = inav->_control_status.flags.tilt_align
                                 && inav->_control_status.flags.yaw_align
                                 && gps->_ned_origin_initialised;

        if (initialised) {
            if (_actived) {
                const bool is_other_source_of_horizontal_aiding_than_gps = gps->_inav->is_other_source_of_horizontal_aiding_than(GPS_HORZ);
                _anomaly = gps->_checks_failing && is_other_source_of_horizontal_aiding_than_gps;

                _anomaly |= gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_horizontal_aiding_than_gps;
                _anomaly |= gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)10e6);

                if (gps->_checks_failing && is_other_source_of_horizontal_aiding_than_gps) {
                    std::cout << "gps->_checks_failing && is_other_source_of_horizontal_aiding_than_gps" << std::endl;
                }
                if (gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_horizontal_aiding_than_gps) {
                    std::cout << "gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_horizontal_aiding_than_gps" << std::endl;
                }
                if (gps->_intermittent) {
                    std::cout << "gps->_intermittent" << std::endl;
                }
            } else {
                _anomaly = gps->_checks_failing || !gps->_checks_passing;

                if (_anomaly) {
                    std::cout << "gps->_checks_failing || !gps->_checks_passing" << std::endl;
                }
            }
        } else {
            _anomaly = true;

            std::cout << "not be initialised" << std::endl;
        }

        if (_anomaly) {
//            _buffer.clear();
            gps->_data_ready = false;

            cout << "gps horz is anomaly" << endl;
        }

    }


    void GpsVelVertAidingInterface::fuse() {
        cout << "fuse gps vel vert data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
            //        auto *sample_delay = (GpsSample *)(gps->_sample_delay);
            // use   _sample_delay->hgt  to fuse
            auto &sample_delay = gps->_sample_delay;
//            gps->_inav->_eskf.fuse_pos_vert(-sample_delay->hgt, gps->_offset_body, gps->_offset_nav, _reject_gate, _meas_noise, fuse_data);
        }
    }

    void GpsVelVertAidingInterface::reset() {
        cout << "reset gps vel vert data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));

            _reset_req = false;
        }
    }

    void GpsVelVertAidingInterface::check_reset_req() {
        auto gps = (GPS *)_sensor;
        if (!_anomaly) {
            _reset_req |= gps->_inav->is_timeout(_time_last_fuse, 5e10);
        }
    }

    void GpsVelVertAidingInterface::anomaly_detection() {
        cout << "anomaly detection of gps vel vert" << endl;
        auto gps = (GPS *)_sensor;
        auto inav = gps->_inav;

        if (inav->_control_status.flags.tilt_align) {
            if (_actived) {
                const bool is_other_source_of_vertical_velocity_aiding_than_gps = gps->_inav->is_other_source_of_vertical_velocity_aiding_than(GPS_VEL_VERT);
                _anomaly = gps->_checks_failing && is_other_source_of_vertical_velocity_aiding_than_gps;

                _anomaly |= gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_vertical_velocity_aiding_than_gps;
                _anomaly |= gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)10e6);

                if (gps->_checks_failing && is_other_source_of_vertical_velocity_aiding_than_gps) {
                    std::cout << "gps->_checks_failing && is_other_source_of_vertical_velocity_aiding_than_gps" << std::endl;
                }
                if (gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_vertical_velocity_aiding_than_gps) {
                    std::cout << "gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_vertical_velocity_aiding_than_gps" << std::endl;
                }
                if (gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)10e6)) {
                    std::cout << "gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)10e6)" << std::endl;
                }
            } else {
                _anomaly = gps->_checks_failing || !gps->_checks_passing;

                if (_anomaly) {
                    std::cout << "gps->_checks_failing || !gps->_checks_passing" << std::endl;
                }
            }
        } else {
            _anomaly = true;

            std::cout << "not be initialised" << std::endl;
        }
    }


    void GpsVelHorzAidingInterface::fuse() {
        cout << "fuse gps vel horz data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
            //        auto *sample_delay = (GpsSample *)(gps->_sample_delay);
            // use   _sample_delay->hgt  to fuse
            auto &sample_delay = gps->_sample_delay;
//            gps->_inav->_eskf.fuse_pos_vert(-sample_delay->hgt, gps->_offset_body, gps->_offset_nav, _reject_gate, _meas_noise, fuse_data);
        }
    }

    void GpsVelHorzAidingInterface::reset() {
        cout << "reset gps vel horz data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));

            _reset_req = false;
        }
    }

    void GpsVelHorzAidingInterface::check_reset_req() {
        auto gps = (GPS *)_sensor;
        if (!_anomaly) {
            _reset_req |= gps->_inav->is_timeout(_time_last_fuse, 5e10);
        }
    }

    void GpsVelHorzAidingInterface::anomaly_detection() {
        cout << "anomaly detection of gps heading" << endl;
        auto gps = (GPS *)_sensor;
        auto inav = gps->_inav;

        if (gps->_brother) {
            if (inav->_control_status.flags.tilt_align) {
                if (_actived) {
                    const bool is_other_source_of_heading_aiding_than_gps = inav->is_other_source_of_heading_aiding_than(GPS_HEAD);
                    _anomaly = is_other_source_of_heading_aiding_than_gps && gps->_checks_failing;

                    _anomaly |= gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_heading_aiding_than_gps;
                    _anomaly |= gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)10e6);

                    if (gps->_checks_failing && is_other_source_of_heading_aiding_than_gps) {
                        std::cout << "gps->_checks_failing && is_other_source_of_heading_aiding_than_gps" << std::endl;
                    }
                    if (gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_heading_aiding_than_gps) {
                        std::cout << "gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_heading_aiding_than_gps" << std::endl;
                    }
                    if (gps->_intermittent) {
                        std::cout << "gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)10e6)" << std::endl;
                    }
                } else {
                    _anomaly = gps->_checks_failing || !gps->_checks_passing;

                    if (_anomaly) {
                        std::cout << "gps->_checks_failing || !gps->_checks_passing" << std::endl;
                    }
                }
            } else {
                _anomaly = true;

                std::cout << "tilt not be initialised" << std::endl;
            }
        } else {
            _anomaly = true;

            std::cout << "single gps" << std::endl;
        }
    }


    void GpsHeadingAidingInterface::fuse() {
        cout << "fuse gps heading data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
            //        auto *sample_delay = (GpsSample *)(gps->_sample_delay);
            // use   _sample_delay->hgt  to fuse
            auto &sample_delay = gps->_sample_delay;
//            gps->_inav->_eskf.fuse_pos_vert(-sample_delay->hgt, gps->_offset_body, gps->_offset_nav, _reject_gate, _meas_noise, fuse_data);
        }
    }

    void GpsHeadingAidingInterface::reset() {
        cout << "fuse gps heading data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
            //        auto *sample_delay = (GpsSample *)(gps->_sample_delay);
            // use   _sample_delay->hgt  to fuse
            auto &sample_delay = gps->_sample_delay;
//            gps->_inav->_eskf.fuse_pos_vert(-sample_delay->hgt, gps->_offset_body, gps->_offset_nav, _reject_gate, _meas_noise, fuse_data);

            _reset_req = false;
        }
    }

    void GpsHeadingAidingInterface::anomaly_detection() {
        cout << "anomaly detection of gps vel horz" << endl;
        auto gps = (GPS *)_sensor;
        auto inav = gps->_inav;

        if (inav->_control_status.flags.tilt_align) {
            if (_actived) {
                const bool is_other_source_of_horizontal_velocity_aiding_than_gps = gps->_inav->is_other_source_of_horizontal_velocity_aiding_than(GPS_VEL_HORZ);
                _anomaly = is_other_source_of_horizontal_velocity_aiding_than_gps && gps->_checks_failing;

                _anomaly |= gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_horizontal_velocity_aiding_than_gps;
                _anomaly |= gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)10e6);

                if (gps->_checks_failing && is_other_source_of_horizontal_velocity_aiding_than_gps) {
                    std::cout << "gps->_checks_failing && is_other_source_of_horizontal_velocity_aiding_than_gps" << std::endl;
                }
                if (gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_horizontal_velocity_aiding_than_gps) {
                    std::cout << "gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_horizontal_velocity_aiding_than_gps" << std::endl;
                }
                if (gps->_intermittent) {
                    std::cout << "gps->_inav->is_timeout(gps->_buffer.newest().time_us, (uint64_t)10e6)" << std::endl;
                }
            } else {
                _anomaly = gps->_checks_failing || !gps->_checks_passing;

                if (_anomaly) {
                    std::cout << "gps->_checks_failing || !gps->_checks_passing" << std::endl;
                }
            }
        } else {
            _anomaly = true;

            std::cout << "not be initialised" << std::endl;
        }
    }

    void GpsHeadingAidingInterface::check_reset_req() {
        auto gps = (GPS *)_sensor;
        if (!_anomaly) {
            _reset_req |= gps->_inav->is_timeout(_time_last_fuse, 5e10);
        }
    }
}