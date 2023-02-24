//
// Created by Cain on 2023/1/28.
//

#include "hgt_sensor_kf.h"

namespace eskf {
    void HgtSensorKF::update(float imu_hgt) {
        auto update_terr = [&]() {
            _terrain = (1.f - _alpha_terr) * _terrain + _alpha_terr * (imu_hgt - _range_hgt);
        };

        auto update_bias = [&]() {
            _baro_bias = (1.f - _alpha_bias) * _baro_bias + _alpha_bias * (_baro_hgt - imu_hgt);
        };

        switch (_state) {
            case 1:
                update_terr();
                break;
            case 2:
                update_bias();
                break;
            case 3:
                update_terr();
                update_bias();
                break;
            default:
                break;
        }

        _state = 0;
    }

    void HgtSensorKF::reset() {
        _terrain = 0.f;
        _baro_bias = 0.f;
        _range_hgt = 0.f;
        _baro_hgt = 0.f;
        _state = 0.f;
    }
}
