//
// Created by Cain on 2023/3/11.
//

#include "opt_flow.h"

namespace inav {
    OptFlow::OptFlow(INAV *inav, uint8_t buffer_size) : Sensor(inav, buffer_size), _vel_horz_aiding_interface(this) {

    }

    void OptFlow::update() {
        cout << "OF:update" << endl;

        // 把数据加入队列
        cout << "push data into deque" << endl;

        // 从队列取delayed数据
        cout << "pop delayed data from deque" << endl;

        _vel_horz_aiding_interface.anomaly_detection();
    }


    void OptFlowVelHorzAidingInterface::fuse() {
        cout << "fuse of vel horz data to eskf" << endl;
        auto of = (OptFlow *)_sensor;

        if (of->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void OptFlowVelHorzAidingInterface::reset() {
        cout << "reset of vel horz data to eskf" << endl;
        auto of = (OptFlow *)_sensor;

        if (of->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void OptFlowVelHorzAidingInterface::anomaly_detection() {
        cout << "anomaly detection of of vel horz" << endl;
        _anomaly = true;

        if (_anomaly) {
            cout << "anomaly" << endl;
        }
    }
}