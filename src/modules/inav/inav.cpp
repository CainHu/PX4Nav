//
// Created by Cain on 2023/3/6.
//

#include "inav.h"

namespace inav {
    INAV::INAV(ESKF &eskf, uint8_t buffer_size) : _eskf(eskf), _gps(this, buffer_size), _ev(this, buffer_size),
    _baro(this, buffer_size), _range(this, buffer_size), _of(this, buffer_size), _mag(this, buffer_size) {
        _hgt_aiding_queues[RTK_HGT] = &_gps._hgt_aiding_interface;
        _hgt_aiding_queues[BARO_HGT] = &_baro._hgt_aiding_interface;
        _hgt_aiding_queues[GPS_HGT] = &_gps._hgt_aiding_interface;
        _hgt_aiding_queues[RANGE_HGT] = &_range._hgt_aiding_interface;

        _horz_aiding_queues[GPS_HORZ] = &_gps._horz_aiding_interface;
        _horz_aiding_queues[EV_HORZ] = &_ev._horz_aiding_interface;

        _vel_vert_aiding_queues[GPS_VEL_VERT] = &_gps._vel_vert_aiding_interface;
        _vel_vert_aiding_queues[EV_VEL_VERT] = &_ev._vel_vert_aiding_interface;

        _vel_horz_aiding_queues[GPS_VEL_HORZ] = &_gps._vel_horz_aiding_interface;
        _vel_horz_aiding_queues[EV_VEL_HORZ] = &_ev._vel_horz_aiding_interface;
        _vel_horz_aiding_queues[OF_VEL_HORZ] = &_of._vel_horz_aiding_interface;
    }

    void INAV::update() {
        hgt_fuse();
        horz_fuse();
        vel_vert_fuse();
        vel_horz_fuse();
    }

    void INAV::hgt_fuse() {
        if (_gps._rtk) {
            _hgt_aiding_queues[RTK_HGT] = &_gps._hgt_aiding_interface;
        } else {
            _hgt_aiding_queues[RTK_HGT] = nullptr;
        }

        for (auto &hgt_aux : _hgt_aiding_queues) {
            if (hgt_aux && !hgt_aux->_anomaly) {
                if (hgt_aux->_actived) {
                    hgt_aux->fuse();
                } else {
                    hgt_aux->reset();
                    hgt_aux->_actived = true;
                }
                if (hgt_aux->_inhibit_next) {
                    break;
                }
            }
        }
    }

    void INAV::horz_fuse() {
        bool fuse = false;
        for (auto &horz_aux : _horz_aiding_queues) {
            if (horz_aux && !horz_aux->_anomaly) {
                fuse = true;
                if (horz_aux->_actived) {
                    horz_aux->fuse();
                } else {
                    horz_aux->reset();
                    horz_aux->_actived = true;
                }
                if (horz_aux->_inhibit_next) {
                    break;
                }
            }
        }

        if (!fuse) {
            std::cout << "no sensor to be used to fuse horizontal position." << std::endl;
        }
    }

    void INAV::vel_vert_fuse() {
        bool fuse = false;
        for (auto &vel_horz_aux : _vel_vert_aiding_queues) {
            if (vel_horz_aux && !vel_horz_aux->_anomaly) {
                cout << "fuse vertical velocity" << endl;
                fuse = true;
                if (vel_horz_aux->_actived) {
                    vel_horz_aux->fuse();
                } else {
                    vel_horz_aux->reset();
                    vel_horz_aux->_actived = true;
                }
                if (vel_horz_aux->_inhibit_next) {
                    break;
                }
            }
        }

        if (!fuse) {
            std::cout << "no sensor to be used to fuse vertical velocity." << std::endl;
        }
    }

    void INAV::vel_horz_fuse() {
        bool fuse = false;
        for (auto &vel_horz_aux : _vel_horz_aiding_queues) {
            if (vel_horz_aux && !vel_horz_aux->_anomaly) {
                cout << "fuse horizontal velocity" << endl;
                fuse = true;
                if (vel_horz_aux->_actived) {
                    vel_horz_aux->fuse();
                } else {
                    vel_horz_aux->reset();
                    vel_horz_aux->_actived = true;
                }
                if (vel_horz_aux->_inhibit_next) {
                    break;
                }
            }
        }

        if (!fuse) {
            std::cout << "no sensor to be used to fuse horizontal velocity." << std::endl;
        }
    }

    int INAV::get_number_of_active_horizontal_aiding_sources() const {
        int num = 0;
        for (auto horz_aux_queue : _horz_aiding_queues) {
            if (horz_aux_queue && horz_aux_queue->_actived) {
                ++num;
            }
        }
        return num;
    }

    bool INAV::is_horizontal_aiding_active() const {
        return get_number_of_active_horizontal_aiding_sources() > 0;
    }

    bool INAV::is_other_source_of_horizontal_aiding_than(HORZ_AID_SRC src_id) const {
        assert(src_id < NUM_HORZ_AID_SRC);
        const int nb_sources = get_number_of_active_horizontal_aiding_sources();
        return _horz_aiding_queues[src_id] ? nb_sources > 1 : nb_sources > 0;
    }

    bool INAV::is_only_active_source_of_horizontal_aiding(HORZ_AID_SRC src_id) const {
        assert(src_id < NUM_HORZ_AID_SRC);
        return _horz_aiding_queues[src_id] && get_number_of_active_horizontal_aiding_sources() == 1;
    }

    int INAV::get_number_of_active_vertical_velocity_aiding_sources() const {
        int num = 0;
        for (auto vel_vert_aux_queue : _vel_vert_aiding_queues) {
            if (vel_vert_aux_queue && vel_vert_aux_queue->_actived) {
                ++num;
            }
        }
        return num;
    }

    bool INAV::is_vertical_velocity_aiding_active() const {
        return get_number_of_active_vertical_velocity_aiding_sources() > 0;
    }

    bool INAV::is_other_source_of_vertical_velocity_aiding_than(VEL_VERT_AID_SRC src_id) const {
        assert(src_id < NUM_VEL_VERT_AID_SRC);
        const int nb_sources = get_number_of_active_vertical_velocity_aiding_sources();
        return _vel_vert_aiding_queues[src_id] ? nb_sources > 1 : nb_sources > 0;
    }

    bool INAV::is_only_active_source_of_vertical_velocity_aiding(VEL_VERT_AID_SRC src_id) const {
        assert(src_id < NUM_VEL_VERT_AID_SRC);
        return _vel_vert_aiding_queues[src_id] && get_number_of_active_vertical_velocity_aiding_sources() == 1;
    }

    int INAV::get_number_of_active_horizontal_velocity_aiding_sources() const {
        int num = 0;
        for (auto vel_horz_aux_queue : _vel_horz_aiding_queues) {
            if (vel_horz_aux_queue && vel_horz_aux_queue->_actived) {
                ++num;
            }
        }
        return num;
    }

    bool INAV::is_horizontal_velocity_aiding_active() const {
        return get_number_of_active_horizontal_velocity_aiding_sources() > 0;
    }

    bool INAV::is_other_source_of_horizontal_velocity_aiding_than(VEL_HORZ_AID_SRC src_id) const {
        assert(src_id < NUM_VEL_HORZ_AID_SRC);
        const int nb_sources = get_number_of_active_horizontal_velocity_aiding_sources();
        return _vel_horz_aiding_queues[src_id] ? nb_sources > 1 : nb_sources > 0;
    }

    bool INAV::is_only_active_source_of_horizontal_velocity_aiding(VEL_HORZ_AID_SRC src_id) const {
        assert(src_id < NUM_VEL_HORZ_AID_SRC);
        return _vel_horz_aiding_queues[src_id] && get_number_of_active_horizontal_velocity_aiding_sources() == 1;
    }

    int INAV::get_number_of_active_heading_aiding_sources() const {
        int num = 0;
        for (auto heading_aiding_queue : _heading_aiding_queues) {
            if (heading_aiding_queue && heading_aiding_queue->_actived) {
                ++num;
            }
        }
        return num;
    }

    bool INAV::is_heading_aiding_active() const {
        return get_number_of_active_heading_aiding_sources() > 0;
    }

    bool INAV::is_other_source_of_heading_aiding_than(HEAD_AID_SRC src_id) const {
        assert(src_id < NUM_HEAD_AID_SRC);
        const int nb_sources = get_number_of_active_heading_aiding_sources();
        return _heading_aiding_queues[src_id] ? nb_sources > 1 : nb_sources > 0;
    }

    bool INAV::is_only_active_source_of_heading_aiding(HEAD_AID_SRC src_id) const {
        assert(src_id < NUM_HEAD_AID_SRC);
        return _heading_aiding_queues[src_id] && get_number_of_active_heading_aiding_sources() == 1;
    }

    bool INAV::is_recent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const {
        return sensor_timestamp + acceptance_interval > _time_current;
    }

    bool INAV::is_timeout(uint64_t sensor_timestamp, uint64_t timeout_period) const {
        return sensor_timestamp + timeout_period < _time_current;
    }
}