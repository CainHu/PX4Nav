//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_SENSOR_H
#define ECL_SENSOR_H

#include <iostream>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <modules/eskf/common.h>


namespace inav {
    using namespace std;
    using namespace eskf;
    class INAV;

    class Sensor {
    public:
        explicit Sensor(INAV *inav) : _inav(inav) {};
        virtual void update() = 0;

        INAV *_inav;

//        bool _anomaly {false};
        bool _intermittent {false};
//        bool _actived {false};
        bool _data_ready {false};

        BaseSample *_buffer {};
        BaseSample *_sample_delay {};

        Vector3f _offset_body {};
        Vector3f _offset_nav {};

        Sensor *_next {nullptr};

//        uint64_t time_last_fuse {};
    };


}

#endif //ECL_SENSOR_H
