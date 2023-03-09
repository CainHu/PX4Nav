//
// Created by Cain on 2023/3/8.
//

#ifndef ECL_VEL_VERT_AUX_INTERFACE_H
#define ECL_VEL_VERT_AUX_INTERFACE_H

namespace inav {
    using namespace eskf;

    class VelVertAuxInterface : public AuxInterface {
    public:
        explicit VelVertAuxInterface(Sensor *sensor) : AuxInterface(sensor) {};

        FuseData<1> fuse_data;
    };
}

#endif //ECL_VEL_VERT_AUX_INTERFACE_H
