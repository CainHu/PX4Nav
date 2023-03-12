//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_MAGNET_H
#define ECL_MAGNET_H

#include "sensor.h"
#include "heading_aiding_interface.h"
#include "vector_aiding_interface.h"

namespace inav {
    class MagHeadingAidingInterface : public HeadingAidingInterface {
    public:
        explicit MagHeadingAidingInterface(Sensor<BaroSample> *sensor) : HeadingAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;

    protected:

    };

    class MagVectorAidingInterface : public VectorAidingInterface {
    public:
        explicit MagVectorAidingInterface(Sensor<BaroSample> *sensor) : VectorAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;

    protected:

    };

    class Magnet : public Sensor<MagSample> {
    public:
        Magnet(INAV *inav, uint8_t buffer_size);
        void update() override;

        MagHeadingAidingInterface _heading_aiding_interface;
        MagVectorAidingInterface _vector_aiding_interface;
    protected:
        bool _fault {false};

    private:
        friend class MagHeadingAidingInterface;
        friend class MagVectorAidingInterface;
    };
}


#endif //ECL_MAGNET_H
