//
// Created by Cain on 2023/3/6.
//

#include "baro.h"
#include "inav.h"

namespace inav {
    Baro::Baro(INAV *inav, uint8_t buffer_size) : Sensor(inav), _hgt_aux_interface(this) {
        _buffer = new BaroSample[buffer_size];
        _sample_delay = new BaroSample;
    }

    void Baro::update() {
        cout << "Baro:update" << endl;

        // 把数据加入队列
        cout << "push data into deque" << endl;

        // 从队列取delayed数据
        cout << "pop delayed data from deque" << endl;

        // 异常检测
        _hgt_aux_interface.anomaly_detection();
    }

    void inav::BaroHgtAuxInterface::fuse() {
        cout << "fuse baro hgt data to eskf" << endl;
    }

    void inav::BaroHgtAuxInterface::reset() {
        cout << "reset baro hgt data to eskf" << endl;
    }

    void inav::BaroHgtAuxInterface::anomaly_detection() {
        cout << "anomaly detection of baro hgt" << endl;
        auto baro = (Baro *)_sensor;

        if (!baro->_inav->_fault_status.flags.bad_acc_vertical && _actived) {
            _anomaly = baro->_inav->is_recent(time_last_fuse, RunnerParameters::HGT_FUSE_TIMEOUT);
        }

//        baro->_fault = true;
        if (baro->_intermittent || baro->_fault) {
            _anomaly = true;
        }

        if (_anomaly) {
            cout << "anomaly" << endl;
//            _buffer.clear();
            baro->_data_ready = false;
        } else {
            if (baro->_inav->_fault_status.flags.bad_acc_vertical) {
                _actived = false;
            }
        }
    }
}
