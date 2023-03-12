//
// Created by Cain on 2023/3/12.
//

#ifndef INAV_VECTOR_AIDING_INTERFACE_H
#define INAV_VECTOR_AIDING_INTERFACE_H

#include <modules/eskf/common.h>
#include "aiding_interface.h"

namespace inav {
    using namespace eskf;

    class VectorAidingInterface : public AidingInterface {
    public:
        explicit VectorAidingInterface(void *sensor) : AidingInterface(sensor) {};

        FuseData<3> fuse_data;

//        struct HgtAidingInterface *next{};
    };
}

#endif //INAV_VECTOR_AIDING_INTERFACE_H
