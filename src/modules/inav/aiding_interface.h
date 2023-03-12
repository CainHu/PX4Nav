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
        bool _inhibit_next {true};

        float reject_gate {1.f};
        float meas_noise {1.f};

        uint64_t time_last_fuse {0};

        virtual void fuse() = 0;
        virtual void reset() = 0;
        virtual void anomaly_detection() = 0;

    };
}

#endif //ECL_AUXINTERFACE_H