//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_RANGE_H
#define ECL_RANGE_H

#include "sensor.h"
#include "hgt_aiding_interface.h"

namespace inav {
    class RangeHgtAidingInterface : public HgtAidingInterface {
    public:
        explicit RangeHgtAidingInterface(Sensor<RangeSample> *sensor) : HgtAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void check_reset_req() override;
        void anomaly_detection() override;

    protected:

    };

    class Range : public Sensor<RangeSample> {
    public:
        explicit Range(INAV *inav, uint8_t buffer_size);
        void update() override;

        RangeHgtAidingInterface _hgt_aiding_interface;

    protected:
        bool _healthy {false};

    private:
        friend class RangeHgtAidingInterface;
    };



}

#endif //ECL_RANGE_H
