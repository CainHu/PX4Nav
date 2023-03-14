//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_BARO_H
#define ECL_BARO_H

#include "sensor.h"
#include "hgt_aiding_interface.h"
#include "magnet.h"


namespace inav {

    class BaroHgtAidingInterface : public HgtAidingInterface {
    public:
        explicit BaroHgtAidingInterface(Sensor<BaroSample> *sensor) : HgtAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void check_reset_req() override;
        void anomaly_detection() override;

    protected:

    };


    class Baro : public Sensor<BaroSample> {
    public:
        explicit Baro(INAV *inav, uint8_t buffer_size);
        void update() override;

        BaroHgtAidingInterface _hgt_aiding_interface;

    protected:
        bool _fault {false};

    private:
        friend class BaroHgtAidingInterface;
    };

}

#endif //ECL_BARO_H
