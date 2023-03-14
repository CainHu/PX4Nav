//
// Created by Cain on 2023/3/6.
//

#ifndef ECL_MAGNET_H
#define ECL_MAGNET_H

#include "sensor.h"
#include "heading_aiding_interface.h"
#include "vector_aiding_interface.h"
#include <lib/dsp/butterworth.hpp>

namespace inav {
    using namespace digital_signal_processing;

    enum MAG_FUSE_TYPE {
        MAG_FUSE_TYPE_AUTO = 0,
        MAG_FUSE_TYPE_HEADING = 1,
        MAG_FUSE_TYPE_3D = 2,
        MAG_FUSE_TYPE_UNUSED = 3,
        MAG_FUSE_TYPE_INDOOR = 4,
        MAG_FUSE_TYPE_NONE = 5
    };


    class MagHeadingAidingInterface : public HeadingAidingInterface {
    public:
        explicit MagHeadingAidingInterface(Sensor<MagSample> *sensor) : HeadingAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void check_reset_req() override;
        void anomaly_detection() override;

        bool _heading_aligned {false};
        bool _heading_aligned_in_flight {false};

    protected:

    };


    class MagVectorAidingInterface : public VectorAidingInterface {
    public:
        explicit MagVectorAidingInterface(Sensor<MagSample> *sensor) : VectorAidingInterface(sensor) {}

        void fuse() override;
        void reset() override;
        void check_reset_req() override;
        void anomaly_detection() override;

    protected:

    };


    class Magnet : public Sensor<MagSample> {
    public:
        Magnet(INAV *inav, uint8_t buffer_size);
        void update() override;
        void check_mag_bias_observability();
        void check_mag_earth_observability();
        void check_mag_field_strength();

        MagHeadingAidingInterface _heading_aiding_interface;
        MagVectorAidingInterface _vector_aiding_interface;
    protected:
        bool _fault {false};
        bool _mag_field_disturbed {false};
        bool _mag_bias_observable {false};
        bool _mag_earth_observable {false};

        MAG_FUSE_TYPE _fuse_type {MAG_FUSE_TYPE_AUTO};

        float _gyro_orth_mag_gate {0.25f};
        float _mag_acc_gate {0.5f};

        LowPassFilter3d _gyro_orth_mag_lpf {200.f, 20.f};
        Butterworth<1> _acc_norm_lpf {200.f, 20.f};

        static Vector3f _mag_earth;

        void set_mag_earth(const Vector3f &mag_earth);

    private:
        friend class MagHeadingAidingInterface;
        friend class MagVectorAidingInterface;
    };
}


#endif //ECL_MAGNET_H
