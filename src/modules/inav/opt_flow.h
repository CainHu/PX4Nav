//
// Created by Cain on 2023/3/11.
//

#ifndef INAV_OF_H
#define INAV_OF_H

#include "sensor.h"
#include "vel_horz_aiding_interface.h"

namespace inav {
    class OptFlowVelHorzAidingInterface: public VelHorzAidingInterface {
    public:
        explicit OptFlowVelHorzAidingInterface(Sensor<FlowSample> *sensor) : VelHorzAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void check_reset_req() override;
        void anomaly_detection() override;
    };


    class OptFlow : public Sensor<FlowSample> {
    public:
        explicit OptFlow(INAV *inav, uint8_t buffer_size);
        void update() override;

        OptFlowVelHorzAidingInterface _vel_horz_aiding_interface;

    protected:

    private:
        friend class OptFlowVelHorzAidingInterface;
        friend class INAV;
    };
}

#endif //INAV_OF_H
