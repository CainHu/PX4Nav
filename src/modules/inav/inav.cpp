//
// Created by Cain on 2023/3/6.
//

#include "inav.h"

namespace inav {
    INAV::INAV(ESKF &eskf, uint8_t buffer_size) : _eskf(eskf), _gps(this, buffer_size), _ev(this, buffer_size), _baro(this, buffer_size), _range(this, buffer_size) {
        _hgt_aux_queues[RTK_HGT] = &_gps._hgt_aux_interface;
        _hgt_aux_queues[BARO_HGT] = &_baro._hgt_aux_interface;
        _hgt_aux_queues[GPS_HGT] = &_gps._hgt_aux_interface;
        _hgt_aux_queues[RANGE_HGT] = &_range._hgt_aux_interface;

        _horz_aux_queues[GPS_HORZ] = &_gps._horz_aux_interface;
        _horz_aux_queues[EV_HORZ] = &_ev._horz_aux_interface;
    }

    void INAV::update() {
        hgt_fuse();
    }

    void INAV::hgt_fuse() {
        if (_gps._rtk) {
            _hgt_aux_queues[RTK_HGT] = &_gps._hgt_aux_interface;
        } else {
            _hgt_aux_queues[RTK_HGT] = nullptr;
        }

        for (auto &hgt_aux : _hgt_aux_queues) {
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
        for (auto &horz_aux : _horz_aux_queues) {
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
            std::cout << "each sensor is anomaly in the case of horizontal position fusion." << std::endl;
            for (auto &horz_aux : _horz_aux_queues) {
                if (horz_aux) {
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
        }

        if (!fuse) {
            std::cout << "no sensor to be used to fuse horizontal position." << std::endl;
        }
    }

    bool INAV::is_recent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const {
        return sensor_timestamp + acceptance_interval > _time_current;
    }

    bool INAV::is_timeout(uint64_t sensor_timestamp, uint64_t timeout_period) const {
        return sensor_timestamp + timeout_period < _time_current;
    }
}