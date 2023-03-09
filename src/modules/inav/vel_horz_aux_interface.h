//
// Created by Cain on 2023/3/8.
//

#ifndef ECL_VEL_HORZ_AUX_INTERFACE_H
#define ECL_VEL_HORZ_AUX_INTERFACE_H

#include <modules/eskf/common.h>
#include "aux_interface.h"

namespace inav {
    using namespace eskf;

    class VelHorzAuxInterface : public AuxInterface {
    public:
        explicit VelHorzAuxInterface(Sensor *sensor) : AuxInterface(sensor) {};

        FuseData<2> fuse_data;
    };
}

#endif //ECL_VEL_HORZ_AUX_INTERFACE_H
