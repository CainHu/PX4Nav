//
// Created by Cain on 2023/3/8.
//

#ifndef ECL_HORZ_AIDING_INTERFACE_H
#define ECL_HORZ_AIDING_INTERFACE_H

#include <modules/eskf/common.h>
#include "aiding_interface.h"

namespace inav {
    using namespace eskf;

    class HorzAidingInterface : public AidingInterface {
    public:
        explicit HorzAidingInterface(void *sensor) : AidingInterface(sensor) {};

        FuseData<2> fuse_data;
    };
}

#endif //ECL_HORZ_AIDING_INTERFACE_H
