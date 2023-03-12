//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_GPS_H
#define ECL_GPS_H

#include "sensor.h"
#include "hgt_aiding_interface.h"
#include "horz_aiding_interface.h"
#include "vel_horz_aiding_interface.h"
#include "vel_vert_aiding_interface.h"

namespace inav {
    class GpsHgtAidingInterface : public HgtAidingInterface {
    public:
        explicit GpsHgtAidingInterface(Sensor<GpsSample> *sensor) : HgtAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;

    protected:

    };


    class GpsHorzAidingInterface : public HorzAidingInterface {
    public:
        explicit GpsHorzAidingInterface(Sensor<GpsSample> *sensor) : HorzAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;
    };


    class GpsVelVertAidingInterface : public VelVertAidingInterface {
    public:
        explicit GpsVelVertAidingInterface(Sensor<GpsSample> *sensor) : VelVertAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;
    };


    class GpsVelHorzAidingInterface : public VelHorzAidingInterface {
    public:
        explicit GpsVelHorzAidingInterface(Sensor<GpsSample> *sensor) : VelHorzAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;
    };


    class GPS : public Sensor<GpsSample> {
    public:
        explicit GPS(INAV *inav, uint8_t buffer_size);
        void update() override;

        GpsHgtAidingInterface _hgt_aiding_interface;
        GpsHorzAidingInterface _horz_aiding_interface;
        GpsVelVertAidingInterface _vel_vert_aiding_interface;
        GpsVelHorzAidingInterface _vel_horz_aiding_interface;

    protected:
        bool _ned_origin_initialised {false};
        bool _checks_passed {false};
        bool _checks_passing {false};
        bool _checks_failing {false};
        bool _rtk {false};

        static Vector2f _horz_ref;
        static float _alt_ref;

        uint64_t _time_last_pass {0};
        uint64_t _time_last_fail {0};

    private:
        friend class GpsHgtAidingInterface;
        friend class GpsHorzAidingInterface;
        friend class GpsVelVertAidingInterface;
        friend class GpsVelHorzAidingInterface;
        friend class INAV;
    };

}

#endif //ECL_GPS_H
