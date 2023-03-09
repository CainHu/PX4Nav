//
// Created by Cain on 2023/3/6.
//

#include "gps.h"
#include "inav.h"

namespace inav {
    GPS::GPS(INAV *inav, uint8_t buffer_size) : Sensor(inav), _hgt_aux_interface(this), _horz_aux_interface(this) {
        _buffer = new GpsSample[buffer_size];
        _sample_delay = new GpsSample;
    }

    void GPS::update() {
        cout << "GPS:update" << endl;

        // 把数据加入队列
        cout << "push data into deque" << endl;

        // 从队列取delayed数据
        cout << "pop delayed data from deque" << endl;

        _rtk = true;
//        _rtk = ((GpsSample *)_sample_delay)->fix_type >= 5;

        // 异常检测
        _hgt_aux_interface.anomaly_detection();
    }

    void GPSHgtAuxInterface::fuse() {
        cout << "fuse gps hgt data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
            //        auto *sample_delay = (GpsSample *)(gps->_sample_delay);
            // use   _sample_delay->hgt  to fuse
            auto *sample_delay = (GpsSample *)(gps->_sample_delay);
//            gps->_inav->_eskf.fuse_pos_vert(-sample_delay->hgt, gps->_offset_body, gps->_offset_nav, reject_gate, meas_noise, fuse_data);
        }

    }

    void GPSHgtAuxInterface::reset() {
        cout << "reset gps hgt data to eskf" << endl;
        auto gps = (GPS *)_sensor;

        if (gps->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void GPSHgtAuxInterface::anomaly_detection() {
        cout << "anomaly detection of gps hgt" << endl;
        auto gps = (GPS *)_sensor;

        if (!gps->_inav->_fault_status.flags.bad_acc_vertical && _actived) {
            _anomaly = gps->_inav->is_recent(time_last_fuse, RunnerParameters::HGT_FUSE_TIMEOUT);
        }

//        gps->_checks_passed = true;
        if (gps->_intermittent || !gps->_checks_passed) {
            _anomaly = true;
        }

        if (_anomaly) {
            cout << "anomaly" << endl;
//            _buffer.clear();
            gps->_data_ready = false;
        } else {
            if (gps->_inav->_fault_status.flags.bad_acc_vertical) {
                _actived = false;   // 为了reset
            }
        }
    }


    void GPSHorzAuxInterface::fuse() {

    }

    void GPSHorzAuxInterface::reset() {

    }

    void GPSHorzAuxInterface::anomaly_detection() {
        cout << "anomaly detection of gps horz" << endl;
        auto gps = (GPS *)_sensor;
        auto inav = gps->_inav;

        const bool mandatory_conditions_passing = inav->_control_status.flags.tilt_align
                                                  && inav->_control_status.flags.yaw_align
                                                  && gps->_ned_origin_initialised;
        _anomaly = mandatory_conditions_passing;

        if (_actived) {
            _anomaly |= gps->_inav->is_recent(time_last_fuse, RunnerParameters::HGT_FUSE_TIMEOUT);
        }

        if (gps->_intermittent || !gps->_checks_passed) {
            _anomaly = true;
        }

        if (_anomaly) {
            cout << "anomaly" << endl;
//            _buffer.clear();
            gps->_data_ready = false;
        }

//        bool any_gps_data_attempted = false;
//        bool each_gps_checks_failing = true;
//        const bool mandatory_conditions_passing = inav->_control_status.flags.tilt_align
//                                                  && inav->_control_status.flags.yaw_align
//                                                  && gps->_ned_origin_initialised;
//
//        if (mandatory_conditions_passing) {
//            if (_actived) {
//                _anomaly = inav->is_timeout(gps->_time_last_pass, (uint64_t)5e6);
//            } else {
//                _anomaly = inav->is_recent(gps->_time_last_fail, (uint64_t)1e6);
//            }
//        } else {
//            _anomaly = true;
//        }
    }
}