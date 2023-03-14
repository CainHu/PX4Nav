//
// Created by Cain on 2023/3/6.
//

#include "magnet.h"
#include "inav.h"

namespace inav {
    Vector3f Magnet::_mag_earth {0.377973f, -0.0213984f, -0.261375f};

    Magnet::Magnet(INAV *inav, uint8_t buffer_size) : Sensor(inav, buffer_size), _heading_aiding_interface(this), _vector_aiding_interface(this) {

    }

    void Magnet::update() {
        cout << "Mag:update" << endl;

        // 把数据加入队列
        cout << "push data into deque" << endl;

        // 从队列取delayed数据
        cout << "pop delayed data from deque" << endl;

        check_mag_field_strength();
        check_mag_bias_observability();
        check_mag_earth_observability();

        // 异常检测
        _heading_aiding_interface.anomaly_detection();
        _vector_aiding_interface.anomaly_detection();
    }

    void Magnet::check_mag_field_strength() {
        cout << "check_mag_field_strength" << endl;

        if ((_fuse_type <= MAG_FUSE_TYPE_3D) ||
            (_fuse_type == MAG_FUSE_TYPE_INDOOR && _inav->is_other_source_of_heading_aiding_than(MAG_HEAD))) {
            static constexpr float wmm_gate_size2 = 0.2f * 0.2f; // +/- Gauss * Gauss

            const float mag_sample_norm2 = _sample_delay.mag.norm_squared();
            const float mag_earth_norm2 = _mag_earth.norm_squared();
            _mag_field_disturbed = !((mag_earth_norm2 >= mag_sample_norm2 - wmm_gate_size2) &&
                                    (mag_earth_norm2 <= mag_sample_norm2 + wmm_gate_size2));
        }

        _mag_field_disturbed = false;

        cout << "_mag_field_disturbed = " << _mag_field_disturbed << endl;
    }

    void Magnet::check_mag_bias_observability() {
        cout << "check_mag_bias_observability" << endl;

        const auto &eskf = _inav->_eskf;

        // TODO: 滤波不应该放在这里, 应该放在eskf或inav中
        const Vector3f mag_body_unit = eskf.get_Rnb() * _mag_earth;
        const Vector3f gyro_orth_mag = eskf.get_gyro_corr() - mag_body_unit * mag_body_unit.dot(eskf.get_gyro_corr());
        const float gyro_orth_mag_norm2 = gyro_orth_mag.norm_squared();

        const float gyro_orth_mag_lp_norm2 = _gyro_orth_mag_lpf(gyro_orth_mag).norm_squared();

        // 垂直于磁场的角速度分量的模值持续一段时间大于阈值则认为磁力计偏移可观
        // 如果不满足上述条件, 则当瞬时角速度垂直分量大于一定阈值且上一时刻为可观时, 依然认为可观, 否则一律认为不可观
        if (gyro_orth_mag_lp_norm2 > sq(_gyro_orth_mag_gate)) {
            _mag_bias_observable = true;
        } else if (_mag_bias_observable) {
            _mag_bias_observable = gyro_orth_mag_norm2 > sq(0.5f * _gyro_orth_mag_gate);
        }

        cout << "_mag_bias_observable = " << _mag_bias_observable << endl;
    }

    void Magnet::check_mag_earth_observability() {
        cout << "check_mag_earth_observability" << endl;

        const auto &eskf = _inav->_eskf;

        // TODO: 滤波不应该放在这里, 应该放在eskf或inav中
        float acc_norm_lp = _acc_norm_lpf(fabs(eskf.get_acc_corr().norm() - CONSTANTS_ONE_G));

        if (_inav->is_other_source_of_heading_aiding_than(MAG_HEAD)) {
            _mag_earth_observable = true;
        } else {
            // 只有速度的变化足够时, 加速度偏移才可观, 此时姿态可观, 从而地球磁场可观
            // 迟滞检查，以避免快速切换
            if (_mag_earth_observable) {
                _mag_earth_observable = acc_norm_lp > _mag_acc_gate;

            } else {
                _mag_earth_observable = acc_norm_lp > _mag_acc_gate * 2.f;
            }
        }

        cout << "_mag_earth_observable = " << _mag_earth_observable << endl;
    }

    void Magnet::set_mag_earth(const Vector3f &mag_earth) {
        _mag_earth = mag_earth;

        // 每次更改_mag_earth后, 都需要重新对齐磁场
        _heading_aiding_interface._reset_req = true;
    }


    void MagHeadingAidingInterface::fuse() {
        cout << "fuse mag as heading to eskf" << endl;
        auto mag = (Magnet *)_sensor;

        if (mag->_data_ready) {
            //        auto *sample_delay = (GpsSample *)(gps->_sample_delay);
            // use   _sample_delay->hgt  to fuse
            auto &sample_delay = mag->_sample_delay;
//            gps->_inav->_eskf.fuse_pos_vert(-sample_delay->hgt, gps->_offset_body, gps->_offset_nav, _reject_gate, _meas_noise, fuse_data);
        }
    }

    void MagHeadingAidingInterface::reset() {
        cout << "reset mag as heading to eskf" << endl;
        auto mag = (Magnet *)_sensor;

        if (mag->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));

            _reset_req = false;
            _heading_aligned = true;
            if (mag->_inav->_control_status.flags.in_air) {
                _heading_aligned_in_flight = true;
            }
        }
    }

    void MagHeadingAidingInterface::check_reset_req() {
        auto mag = (Magnet *)_sensor;
        auto inav = mag->_inav;

        _reset_req |= !inav->_control_status.flags.yaw_align;
        _reset_req |= !_actived;

        if (inav->_control_status.flags.in_air && inav->_control_status.flags.yaw_align && _heading_aligned_in_flight) {
            static constexpr float mag_anomalies_max_hgt = 1.5f;
            _reset_req |= (inav->_eskf.get_terrain() - inav->_eskf.get_pos_vert()) > mag_anomalies_max_hgt;
        }
    }

    void MagHeadingAidingInterface::anomaly_detection() {
        cout << "anomaly detection of mag as heading" << endl;
        auto mag = (Magnet *)_sensor;
        auto inav = mag->_inav;

        _anomaly = mag->_intermittent || mag->_fault || !mag->_inav->_control_status.flags.tilt_align || mag->_mag_field_disturbed;

        const bool is_other_source_of_heading_aiding_than_mag = inav->is_other_source_of_heading_aiding_than(MAG_HEAD);
        _anomaly |= (mag->_fuse_type == MAG_FUSE_TYPE_INDOOR) && !is_other_source_of_heading_aiding_than_mag;

        if (_actived) {
            _anomaly |= inav->is_timeout(mag->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_heading_aiding_than_mag;
            _anomaly |= inav->is_timeout(mag->_buffer.newest().time_us, (uint64_t)10e6);
        }
    }


    void MagVectorAidingInterface::fuse() {
        cout << "fuse mag vector to eskf" << endl;
        auto mag = (Magnet *)_sensor;

        if (mag->_data_ready) {
            //        auto *sample_delay = (GpsSample *)(gps->_sample_delay);
            // use   _sample_delay->hgt  to fuse
            auto &sample_delay = mag->_sample_delay;
//            gps->_inav->_eskf.fuse_pos_vert(-sample_delay->hgt, gps->_offset_body, gps->_offset_nav, _reject_gate, _meas_noise, fuse_data);
        }
    }

    void MagVectorAidingInterface::reset() {
        cout << "reset mag vector to eskf" << endl;
        auto mag = (Magnet *)_sensor;

        if (mag->_data_ready) {
//            gps->_inav->_eskf.set_pos_vert(-sum_gps_hgt_imu / sum_prior);
//            _eskf.reset_covariance_matrix<1>(2, sq(_eskf._params.gps_pos_vert_noise));
        }
    }

    void MagVectorAidingInterface::anomaly_detection() {
        cout << "anomaly detection of mag as vector" << endl;
        auto mag = (Magnet *)_sensor;
        auto inav = mag->_inav;

        _anomaly = mag->_intermittent || mag->_fault || !mag->_inav->_control_status.flags.tilt_align || mag->_mag_field_disturbed;

        const bool is_other_source_of_heading_aiding_than_mag = inav->is_other_source_of_heading_aiding_than(MAG_HEAD);
        _anomaly |= (mag->_fuse_type >= MAG_FUSE_TYPE_INDOOR) && !is_other_source_of_heading_aiding_than_mag;

        if (_actived) {
            _anomaly |= inav->is_timeout(mag->_buffer.newest().time_us, (uint64_t)1e6) && is_other_source_of_heading_aiding_than_mag;
            _anomaly |= inav->is_timeout(mag->_buffer.newest().time_us, (uint64_t)10e6);
        }
    }

    void MagVectorAidingInterface::check_reset_req() {

    }
}