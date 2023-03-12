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
#include "ex_vision.h"
#include "baro.h"
#include "range.h"
#include "opt_flow.h"
#include "aiding_interface.h"
#include "hgt_aiding_interface.h"

namespace inav {
    using namespace eskf;
    using namespace std;

    enum hgt_prior {
        RTK_HGT = 0,
        BARO_HGT = 1,
        GPS_HGT = 2,
        RANGE_HGT = 3,

        NUM_HGT_AID_SRC
    };

    enum horz_prior {
        GPS_HORZ = 0,
        EV_HORZ = 1,

        NUM_HORZ_AID_SRC
    };

    enum vel_vert_prior {
        GPS_VEL_VERT = 0,
        EV_VEL_VERT = 1,

        NUM_VEL_VERT_AID_SRC
    };

    enum vel_horz_prior {
        GPS_VEL_HORZ = 0,
        EV_VEL_HORZ = 1,
        OF_VEL_HORZ = 2,

        NUM_VEL_HORZ_AID_SRC
    };

    enum heading_prior {
        RTK_HEAD = 0,
        MAG_HEAD = 1,
        GPS_HEAD = 2,

        NUM_HEAD_AID_SRC
    };

    enum vector_prior {
        RTK_VECTOR = 0,
        MAG_VECTOR = 1,
        GPS_VECTOR = 2,
        GRAV_VECTOR = 3,

        NUM_VECTOR_AID_SRC
    };

    class INAV {
    public:
        INAV(ESKF &eskf, uint8_t buffer_size);
        ~INAV() = default;

        void update();

        // 公共api
        int get_number_of_active_horizontal_aiding_sources() const;
        bool is_horizontal_aiding_active() const;
        bool is_other_source_of_horizontal_aiding_than(const int name_of_queue) const;
        bool is_only_active_source_of_horizontal_aiding(const int name_of_queue) const;

        int get_number_of_active_horizontal_velocity_aiding_sources() const;
        bool is_horizontal_velocity_aiding_active() const;
        bool is_other_source_of_horizontal_velocity_aiding_than(const int name_of_queue) const;
        bool is_only_active_source_of_horizontal_velocity_aiding(const int name_of_queue) const;

        int get_number_of_active_vertical_velocity_aiding_sources() const;
        bool is_vertical_velocity_aiding_active() const;
        bool is_other_source_of_vertical_velocity_aiding_than(const int name_of_queue) const;
        bool is_only_active_source_of_vertical_velocity_aiding(const int name_of_queue) const;

        bool is_recent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const;
        bool is_timeout(uint64_t sensor_timestamp, uint64_t timeout_period) const;

        void hgt_fuse();
        void horz_fuse();
        void vel_vert_fuse();
        void vel_horz_fuse();

        // 传感器
        IMU _imu;
        Magnet _mag;
        GPS _gps;
        ExVision _ev;
        Baro _baro;
        Range _range;
        OptFlow _of;

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
        AidingInterface *_hgt_aiding_queues[NUM_HGT_AID_SRC]{};
        // 水平位置融合优先级队列
        AidingInterface *_horz_aiding_queues[NUM_HORZ_AID_SRC]{};
        // 垂直速度融合优先级队列
        AidingInterface *_vel_vert_aiding_queues[NUM_VEL_VERT_AID_SRC]{};
        // 水平速度融合优先级队列
        AidingInterface *_vel_horz_aiding_queues[NUM_VEL_HORZ_AID_SRC]{};


    private:
        friend class IMU;
        friend class Magnet;
        friend class GPS;
        friend class ExVision;
        friend class OptFlow;
        friend class Baro;
        friend class Range;

        friend class GpsHgtAidingInterface;
        friend class GpsHorzAidingInterface;
        friend class GpsVelVertAidingInterface;
        friend class GpsVelHorzAidingInterface;
        friend class ExVisionHorzAidingInterface;
        friend class ExVisionVelVertAidingInterface;
        friend class ExVisionVelHorzAidingInterface;
        friend class BaroHgtAidingInterface;
        friend class RangeHgtAidingInterface;
        friend class OptFlowVelHorzAidingInterface;
        friend class MagHeadingAidingInterface;
        friend class MagVectorAidingInterface;
    };
}

#endif //ECL_INAV_H
