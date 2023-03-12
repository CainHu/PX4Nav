//
// Created by Cain on 2023/3/9.
//

#include "ex_vision.h"
#include "inav.h"

namespace inav {
    inav::ExVision::ExVision(inav::INAV *inav, uint8_t buffer_size) : Sensor(inav, buffer_size),
                                                                      _horz_aiding_interface(this), _vel_vert_aiding_interface(this), _vel_horz_aiding_interface(this) {

    }

    void inav::ExVision::update() {
        cout << "EV:update" << endl;

        // 把数据加入队列
        cout << "push data into deque" << endl;

        // 从队列取delayed数据
        cout << "pop delayed data from deque" << endl;

        _horz_aiding_interface.anomaly_detection();
        _vel_vert_aiding_interface.anomaly_detection();
        _vel_horz_aiding_interface.anomaly_detection();
    }


    void ExVisionHorzAidingInterface::fuse() {
        cout << "fuse ev horz data to eskf" << endl;
        auto ev = (ExVision *)_sensor;

        if (ev->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void ExVisionHorzAidingInterface::reset() {
        cout << "reset ev horz data to eskf" << endl;
        auto ev = (ExVision *)_sensor;

        if (ev->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void ExVisionHorzAidingInterface::anomaly_detection() {
        cout << "anomaly detection of ev horz" << endl;
        _anomaly = true;

        if (_anomaly) {
            cout << "anomaly" << endl;
        }
    }


    void ExVisionVelVertAidingInterface::fuse() {
        cout << "fuse ev vel vert data to eskf" << endl;
        auto ev = (ExVision *)_sensor;

        if (ev->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void ExVisionVelVertAidingInterface::reset() {
        cout << "reset ev vel vert data to eskf" << endl;
        auto ev = (ExVision *)_sensor;

        if (ev->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void ExVisionVelVertAidingInterface::anomaly_detection() {
        cout << "anomaly detection of ev vel vert" << endl;
        _anomaly = true;

        if (_anomaly) {
            cout << "anomaly" << endl;
        }
    }


    void ExVisionVelHorzAidingInterface::fuse() {
        cout << "fuse ev vel horz data to eskf" << endl;
        auto ev = (ExVision *)_sensor;

        if (ev->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void ExVisionVelHorzAidingInterface::reset() {
        cout << "reset ev vel horz data to eskf" << endl;
        auto ev = (ExVision *)_sensor;

        if (ev->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void ExVisionVelHorzAidingInterface::anomaly_detection() {
        cout << "anomaly detection of ev vel horz" << endl;
        _anomaly = true;

        if (_anomaly) {
            cout << "anomaly" << endl;
        }
    }
}
