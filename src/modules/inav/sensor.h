//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_SENSOR_H
#define ECL_SENSOR_H

#include <iostream>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include "utils.h"
#include <modules/eskf/common.h>


namespace inav {
    using namespace std;
    using namespace eskf;
    class INAV;

    template<class T>
    class Sensor {
    public:
        explicit Sensor(INAV *inav, uint8_t buffer_size) : _inav(inav), _buffer(buffer_size) {};
        virtual void update() = 0;

//        bool _anomaly {false};
        bool _intermittent {false};
//        bool _actived {false};
        bool _data_ready {false};

        INAV *_inav;

        Queue<T> _buffer;
        T _sample_delay {};

        Vector3f _offset_body {};
        Vector3f _offset_nav {};

        Sensor *_brother {nullptr};

//        uint64_t _time_last_fuse {};
    };


}

#endif //ECL_SENSOR_H
