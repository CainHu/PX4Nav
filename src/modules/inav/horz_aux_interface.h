//
// Created by Cain on 2023/3/8.
//

#ifndef ECL_HORZ_AUX_INTERFACE_H
#define ECL_HORZ_AUX_INTERFACE_H

#include <modules/eskf/common.h>
#include "aux_interface.h"

namespace inav {
    using namespace eskf;

    class HorzAuxInterface : public AuxInterface {
    public:
        explicit HorzAuxInterface(Sensor *sensor) : AuxInterface(sensor) {};

        FuseData<2> fuse_data;
    };
}

#endif //ECL_HORZ_AUX_INTERFACE_H
