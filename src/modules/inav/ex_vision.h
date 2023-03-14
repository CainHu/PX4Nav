//
// Created by Cain on 2023/3/9.
//

#ifndef ECL_EX_VISION_H
#define ECL_EX_VISION_H

#include "sensor.h"
#include "hgt_aiding_interface.h"
#include "horz_aiding_interface.h"
#include "vel_horz_aiding_interface.h"
#include "vel_vert_aiding_interface.h"

namespace inav {
    class ExVisionHorzAidingInterface : public HorzAidingInterface {
    public:
        explicit ExVisionHorzAidingInterface(Sensor<ExtVisionSample> *sensor) : HorzAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void check_reset_req() override;
        void anomaly_detection() override;

    protected:

    };


    class ExVisionVelVertAidingInterface: public VelVertAidingInterface {
    public:
        explicit ExVisionVelVertAidingInterface(Sensor<ExtVisionSample> *sensor) : VelVertAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void check_reset_req() override;
        void anomaly_detection() override;
    };


    class ExVisionVelHorzAidingInterface: public VelHorzAidingInterface {
    public:
        explicit ExVisionVelHorzAidingInterface(Sensor<ExtVisionSample> *sensor) : VelHorzAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void check_reset_req() override;
        void anomaly_detection() override;
    };


    class ExVision : public Sensor<ExtVisionSample> {
    public:
        explicit ExVision(INAV *inav, uint8_t buffer_size);
        void update() override;

        ExVisionHorzAidingInterface _horz_aiding_interface;
        ExVisionVelVertAidingInterface _vel_vert_aiding_interface;
        ExVisionVelHorzAidingInterface _vel_horz_aiding_interface;

    protected:
        bool _ned_origin_initialised {false};
        bool _checks_passed {false};
        bool _rtk {false};

        uint64_t _time_last_pass {0};
        uint64_t _time_last_fail {0};

    private:
        friend class ExVisionHgtAuxInterface;
        friend class ExVisionHorzAidingInterface;
        friend class ExVisionVelVertAidingInterface;
        friend class ExVisionVelHorzAidingInterface;
        friend class INAV;
    };
}

#endif //ECL_EX_VISION_H
