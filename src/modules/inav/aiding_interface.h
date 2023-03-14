//
// Created by Cain on 2023/3/7.
//

#ifndef ECL_AUXINTERFACE_H
#define ECL_AUXINTERFACE_H

#include "sensor.h"

namespace inav {
    class AidingInterface {
    public:
        explicit AidingInterface(void *sensor) : _sensor(sensor) {};

        void *_sensor;

        bool _anomaly {false};
        bool _actived {false};
        bool _reset_req {false};
        bool _inhibit_next {true};

        float _reject_gate {1.f};
        float _meas_noise {1.f};

        uint64_t _time_last_fuse {0};

        virtual void fuse() = 0;
        virtual void reset() = 0;
        virtual void anomaly_detection() = 0;
        virtual void check_reset_req() = 0;
        void enable_reset_req() { _reset_req = true; }
        void disable_reset_req() { _reset_req = false; }
    };
}

#endif //ECL_AUXINTERFACE_H
