//
// Created by Cain on 2023/1/28.
//

#ifndef ECL_HGT_KF_H
#define ECL_HGT_KF_H

#include "common.h"
#include "utils.h"

namespace eskf {
    using matrix::Dcmf;

    template <uint8_t DELAYS>
    class ESKFRunner;

    class HgtSensorKF {
    public:
        HgtSensorKF(float alpha_terr=0.01f, float alpha_bias=0.01f)
        : _alpha_terr(alpha_terr), _alpha_bias(alpha_bias) {}

        /*!
         * 更新terrain和baro_bias
         * @param imu_hgt - imu的高度, 向上为正
         */
        void update(float imu_hgt);

        void reset();

        float get_terrain() { return _terrain; }
        float get_baro_bias() { return _baro_bias; }

        void set_terrain(float terrain) { _terrain = terrain; }
        void set_baro_bias(float baro_bias) { _baro_bias = baro_bias; }

        /*!
         * 设置气压计折算到imu处的高度(向上为正)
         * @param range_hgt
         */
        void set_range_hgt(float range_hgt) {
            _range_hgt = range_hgt;
            _state |= (uint32_t )1;
        }

        /*!
         * 设置测距仪折算到imu处的对地距离(向上为正)
         * @param baro_hgt
         */
        void set_baro_hgt(float baro_hgt) {
            _baro_hgt = baro_hgt;
            _state |= (uint32_t)2;
        }

        void set_alpha(float alpha_terr, float alpha_bias) {
            _alpha_terr = alpha_terr;
            _alpha_bias = alpha_bias;
        }

    private:
        float _terrain {0.f};          ///< 地面高度, range.hgt + terrain = imu.hgt
        float _baro_bias {0.f};        ///< 气压计偏移, baro - baro.bias = imu.hgt
        float _range_hgt {0.f};
        float _baro_hgt {0.f};

        float _alpha_terr;
        float _alpha_bias;

        uint32_t _state {0};

    private:
        template<uint8_t DELAYS>
        friend class ESKFRunner;
    };
}

#endif //ECL_HGT_KF_H
