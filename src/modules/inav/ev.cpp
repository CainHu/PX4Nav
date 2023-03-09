//
// Created by Cain on 2023/3/9.
//

#include "ev.h"
#include "inav.h"

namespace inav {
    inav::ExVision::ExVision(inav::INAV *inav, uint8_t buffer_size) : Sensor(inav), _horz_aux_interface(this) {
        _buffer = new ExtVisionSample[buffer_size];
        _sample_delay = new ExtVisionSample;
    }

    void inav::ExVision::update() {

    }

    void EVHorzAuxInterface::fuse() {

    }

    void EVHorzAuxInterface::reset() {

    }

    void EVHorzAuxInterface::anomaly_detection() {

    }
}
