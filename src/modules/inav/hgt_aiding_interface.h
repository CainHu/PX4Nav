//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_HGT_AIDING_INTERFACE_H
#define ECL_HGT_AIDING_INTERFACE_H

#include <modules/eskf/common.h>
#include "aiding_interface.h"

namespace inav {
    using namespace eskf;

    class HgtAidingInterface : public AidingInterface {
    public:
        explicit HgtAidingInterface(void *sensor) : AidingInterface(sensor) {};

        FuseData<1> fuse_data;

//        struct HgtAidingInterface *next{};
    };
}

#endif //ECL_HGT_AIDING_INTERFACE_H
