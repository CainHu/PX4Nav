//
// Created by Cain on 2023/3/8.
//

#ifndef ECL_VEL_VERT_AIDING_INTERFACE_H
#define ECL_VEL_VERT_AIDING_INTERFACE_H

namespace inav {
    using namespace eskf;

    class VelVertAidingInterface : public AidingInterface {
    public:
        explicit VelVertAidingInterface(void *sensor) : AidingInterface(sensor) {};

        FuseData<1> fuse_data;
    };
}

#endif //ECL_VEL_VERT_AIDING_INTERFACE_H
