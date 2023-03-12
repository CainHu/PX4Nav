//
// Created by Cain on 2023/3/12.
//

#ifndef INAV_HEADING_AIDING_INTERFACE_H
#define INAV_HEADING_AIDING_INTERFACE_H

#include <modules/eskf/common.h>
#include "aiding_interface.h"

namespace inav {
    using namespace eskf;

    class HeadingAidingInterface : public AidingInterface {
    public:
        explicit HeadingAidingInterface(void *sensor) : AidingInterface(sensor) {};

        FuseData<1> fuse_data;

//        struct HgtAidingInterface *next{};
    };
}

#endif //INAV_HEADING_AIDING_INTERFACE_H
