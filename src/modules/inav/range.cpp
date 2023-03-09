//
// Created by Cain on 2023/3/6.
//

#include "range.h"
#include "inav.h"

namespace inav {
    Range::Range(INAV *inav, uint8_t buffer_size) : Sensor(inav), _hgt_aux_interface(this) {
        _buffer = new RangeSample[buffer_size];
        _sample_delay = new RangeSample;
    }

    void Range::update() {
        cout << "Range:update" << endl;

        // 把数据加入队列
        cout << "push data into deque" << endl;

        // 从队列取delayed数据
        cout << "pop delayed data from deque" << endl;

        // 异常检测
        _hgt_aux_interface.anomaly_detection();
    }

    void RangeHgtAuxInterface::fuse() {
        cout << "fuse range hgt data to eskf" << endl;
        auto range = (Range *)_sensor;

        if (range->_data_ready) {
            //        auto *sample_delay = (GpsSample *)(gps->_sample_delay);
            // use   _sample_delay->hgt  to fuse
            auto *sample_delay = (GpsSample *)(range->_sample_delay);
//            range->_inav._eskf.fuse_pos_vert(-sample_delay->hgt, range->_offset_body, range->_offset_nav, reject_gate, meas_noise, fuse_data);
        }

    }

    void RangeHgtAuxInterface::reset() {
        cout << "reset range hgt data to eskf" << endl;
        auto range = (Range *)_sensor;

        if (range->_data_ready) {
//            gps->_inav._eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void RangeHgtAuxInterface::anomaly_detection() {
        cout << "anomaly detection of range hgt" << endl;
        auto range = (Range *)_sensor;

        if (!range->_inav->_fault_status.flags.bad_acc_vertical && _actived) {
            _anomaly = range->_inav->is_recent(time_last_fuse, RunnerParameters::HGT_FUSE_TIMEOUT);
        }

        if (range->_intermittent || !range->_healthy) {
            _anomaly = true;
        }

        if (_anomaly) {
            cout << "anomaly" << endl;
//            _buffer.clear();
            range->_data_ready = false;
        } else {
            if (range->_inav->_fault_status.flags.bad_acc_vertical) {
                _actived = false;
            }
        }
    }



}