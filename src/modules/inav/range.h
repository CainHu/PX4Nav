//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_RANGE_H
#define ECL_RANGE_H

#include "sensor.h"
#include "hgt_aux_interface.h"

namespace inav {
    class RangeHgtAuxInterface : public HgtAuxInterface {
    public:
        explicit RangeHgtAuxInterface(Sensor *sensor) : HgtAuxInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;

    protected:

    };

    class Range : public Sensor {
    public:
        explicit Range(INAV *inav, uint8_t buffer_size);
        void update() override;

        RangeHgtAuxInterface _hgt_aux_interface;

    protected:
        bool _healthy {false};

    private:
        friend class HgtAuxInterface;
        friend class RangeHgtAuxInterface;
    };



}

#endif //ECL_RANGE_H
