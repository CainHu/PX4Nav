//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_INAV_H
#define ECL_INAV_H

#include <iostream>
#include <modules/eskf/eskf.h>

#include "sensor.h"
#include "imu.h"
#include "magnet.h"
#include "gps.h"
#include "ev.h"
#include "baro.h"
#include "range.h"
#include "aux_interface.h"
#include "hgt_aux_interface.h"

#define INAV_HGT_AUX 8
#define INAV_HORZ_AUX 4

namespace inav {
    using namespace eskf;
    using namespace std;

    enum hgt_prior {
        RTK_HGT = 0,
        BARO_HGT = 1,
        GPS_HGT = 2,
        RANGE_HGT = 3
    };

    enum horz_prior {
        GPS_HORZ = 0,
        EV_HORZ = 1
    };

    enum vel_horz_prior {
        GPS_VEL_HORZ = 0,
        EV_VEL_HORZ = 1,
        OF_VEL_HORZ = 2
    };

    enum vel_vert_prior {
        GPS_VEL_VERT = 0,
        EV_VEL_VERT = 1
    };

    class INAV {
    public:
        INAV(ESKF &eskf, uint8_t buffer_size);
        ~INAV() = default;

        void update();

        // 传感器
        IMU _imu;
//        Magnet _mag;
        GPS _gps;
        ExVision _ev;
        Baro _baro;
        Range _range;

    protected:
        // 融合算法
        ESKF &_eskf;

        //
        filter_control_status_u _control_status {};
        fault_status_u _fault_status {};

        // 时间
        uint64_t _time_current {0};
        uint64_t _time_last_fuse_horz_pos {0};
        uint64_t _time_last_fuse_vert_pos {0};
        uint64_t _time_last_fuse_horz_vel {0};
        uint64_t _time_last_fuse_vert_vel {0};

        // 高度融合优先级队列
        AuxInterface *_hgt_aux_queues[INAV_HGT_AUX]{};

        // 水平位置融合优先级队列
        AuxInterface *_horz_aux_queues[INAV_HORZ_AUX]{};

        void hgt_fuse();
        void horz_fuse();

        // 共用api
        bool is_recent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const;

        bool is_timeout(uint64_t sensor_timestamp, uint64_t timeout_period) const;

    private:
        friend class IMU;
        friend class Magnet;
        friend class GPS;
        friend class ExVision;
        friend class OptFlow;
        friend class Baro;
        friend class Range;

        friend class HgtAuxInterface;
        friend class GPSHgtAuxInterface;
        friend class GPSHorzAuxInterface;
        friend class BaroHgtAuxInterface;
        friend class RangeHgtAuxInterface;
    };

}

#endif //ECL_INAV_H
