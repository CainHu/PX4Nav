//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_BARO_H
#define ECL_BARO_H

#include "sensor.h"
#include "hgt_aux_interface.h"

namespace inav {

    class BaroHgtAuxInterface : public HgtAuxInterface {
    public:
        explicit BaroHgtAuxInterface(Sensor *sensor) : HgtAuxInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;

    protected:

    };

    class Baro : public Sensor {
    public:
        explicit Baro(INAV *inav, uint8_t buffer_size);
        void update() override;

        BaroHgtAuxInterface _hgt_aux_interface;

    protected:
        bool _fault {false};

    private:
        friend class HgtAuxInterface;
        friend class BaroHgtAuxInterface;
    };





}

#endif //ECL_BARO_H
