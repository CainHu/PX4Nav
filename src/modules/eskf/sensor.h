////
//// Created by Cain on 2023/3/2.
////
//
//#ifndef ECL_SENSOR_H
//#define ECL_SENSOR_H
//
//#include <mathlib/mathlib.h>
//#include <matrix/math.hpp>
//#include "common.h"
//#include "utils.h"
//
//namespace eskf {
//    template<class TYPE_SAMPLE, uint8_t DELAYS>
//    class Sensor {
//    public:
//        Sensor() = default;
//
//        bool is_recent(uint64_t time_current, uint64_t sensor_timestamp, uint64_t acceptance_interval) const {
//            return sensor_timestamp + acceptance_interval > time_current;
//        }
//
//        bool is_timeout(uint64_t time_current, uint64_t sensor_timestamp, uint64_t timeout_period) const {
//            return sensor_timestamp + timeout_period < time_current;
//        }
//
//        bool ready {false};
//        bool intermittent {false};
//        bool available {false};
//
//        TYPE_SAMPLE sample_delay {};
//        Queue<TYPE_SAMPLE, DELAYS> buffer;
//
//        matrix::Vector3f offset_body;
//        matrix::Vector3f offset_nav;
//
//        uint64_t time_last_fuse {0};
//
//    };
//}
//
//#endif //ECL_SENSOR_H
