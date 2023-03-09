//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_HGT_AUX_INTERFACE_H
#define ECL_HGT_AUX_INTERFACE_H

#include <modules/eskf/common.h>
#include "aux_interface.h"

namespace inav {
    using namespace eskf;

    class HgtAuxInterface : public AuxInterface {
    public:
        explicit HgtAuxInterface(Sensor *sensor) : AuxInterface(sensor) {};

        FuseData<1> fuse_data;

//        struct HgtAuxInterface *next{};
    };
}

#endif //ECL_HGT_AUX_INTERFACE_H
