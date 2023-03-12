//
// Created by Cain on 2023/3/6.
//

#include "magnet.h"
#include "inav.h"

namespace inav {
    Magnet::Magnet(INAV *inav, uint8_t buffer_size) : Sensor(inav, buffer_size), _heading_aiding_interface(this), _vector_aiding_interface(this) {

    }

    void Magnet::update() {
        cout << "Mag:update" << endl;

        // 把数据加入队列
        cout << "push data into deque" << endl;

        // 从队列取delayed数据
        cout << "pop delayed data from deque" << endl;

        // 异常检测
        _heading_aiding_interface.anomaly_detection();
        _vector_aiding_interface.anomaly_detection();
    }


    void MagHeadingAidingInterface::fuse() {

    }

    void MagHeadingAidingInterface::reset() {

    }

    void MagHeadingAidingInterface::anomaly_detection() {
        cout << "anomaly detection of mag head" << endl;
        auto mag = (Magnet *)_sensor;

        _anomaly = mag->_intermittent || mag->_fault || !mag->_inav->_control_status.flags.tilt_align;

//        const bool user_selected = (_params.mag_fusion_type == MagFuseType::INDOOR);
//
//        const bool heading_not_required_for_navigation = !_control_status.flags.gps;
//
//        return (user_selected && heading_not_required_for_navigation) || _control_status.flags.mag_field_disturbed;
    }


    void MagVectorAidingInterface::fuse() {

    }

    void MagVectorAidingInterface::reset() {

    }

    void MagVectorAidingInterface::anomaly_detection() {

    }
}