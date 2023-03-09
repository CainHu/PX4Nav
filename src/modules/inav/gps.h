//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_GPS_H
#define ECL_GPS_H

#include "sensor.h"
#include "hgt_aux_interface.h"
#include "horz_aux_interface.h"
#include "vel_horz_aux_interface.h"
#include "vel_vert_aux_interface.h"

namespace inav {
    class GPSHgtAuxInterface : public HgtAuxInterface {
    public:
        explicit GPSHgtAuxInterface(Sensor *sensor) : HgtAuxInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;

    protected:

    };

    class GPSHorzAuxInterface : public HorzAuxInterface {
    public:
        explicit GPSHorzAuxInterface(Sensor *sensor) : HorzAuxInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;
    };

    class GPS : public Sensor {
    public:
        explicit GPS(INAV *inav, uint8_t buffer_size);
        void update() override;

        GPSHgtAuxInterface _hgt_aux_interface;
        GPSHorzAuxInterface _horz_aux_interface;

    protected:
        bool _ned_origin_initialised {false};
        bool _checks_passed {false};
        bool _rtk {false};

        uint64_t _time_last_pass {0};
        uint64_t _time_last_fail {0};

    private:
        friend class HgtAuxInterface;
        friend class GPSHgtAuxInterface;
        friend class GPSHorzAuxInterface;
        friend class INAV;
    };



}

#endif //ECL_GPS_H
