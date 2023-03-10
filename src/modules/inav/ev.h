//
// Created by Cain on 2023/3/9.
//

#ifndef ECL_EV_H
#define ECL_EV_H

#include "sensor.h"
#include "horz_aux_interface.h"

namespace inav {
    class EVHorzAuxInterface : public HorzAuxInterface {
    public:
        explicit EVHorzAuxInterface(Sensor *sensor) : HorzAuxInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void anomaly_detection() override;

    protected:

    };

    class ExVision : public Sensor {
    public:
        explicit ExVision(INAV *inav, uint8_t buffer_size);
        void update() override;

        EVHorzAuxInterface _horz_aux_interface;

    protected:
        bool _ned_origin_initialised {false};
        bool _checks_passed {false};
        bool _rtk {false};

        uint64_t _time_last_pass {0};
        uint64_t _time_last_fail {0};

    private:
        friend class HgtAuxInterface;
        friend class EVHgtAuxInterface;
        friend class EVHorzAuxInterface;
        friend class INAV;
    };
}

#endif //ECL_EV_H
