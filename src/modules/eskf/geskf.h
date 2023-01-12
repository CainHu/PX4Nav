//
// Created by Cain on 2023/1/3.
//

#ifndef ECL_GESKF_H
#define ECL_GESKF_H

#include "eskf.h"

namespace eskf {
    class GESKF : public ESKF {
    public:
        explicit GESKF(Parameters &params) : ESKF(params) {};

        // Prior
        void predict_covariance(const ImuSample &imu_sample) override;

        // Posterior
        uint8_t fuse_pos_horz(const Vector2f &pos, const Vector3f &offset_body, const Vector3f &offset_nav,
                              const float &gate, const float &noise_std, FuseData<2> &fuse_data) override;
        uint8_t fuse_pos_vert(const float &pos, const Vector3f &offset_body, const Vector3f &offset_nav,
                              const float &gate, const float &noise_std, FuseData<1> &fuse_data) override;
        uint8_t fuse_vel_horz(const Vector2f &vel, const Vector3f &offset_body, const Vector3f &offset_nav,
                              const Vector3f &w_cross_offset_body, const Vector3f &w_cross_offset_nav,
                              const float &gate, const float &noise_std, FuseData<2> &fuse_data) override;
        uint8_t fuse_vel_vert(const float &vel, const Vector3f &offset_body, const Vector3f &offset_nav,
                              const Vector3f &w_cross_offset_body, const Vector3f &w_cross_offset_nav,
                              const float &gate, const float &noise_std, FuseData<1> &fuse_data) override;
        uint8_t fuse_vel_body_horz(const Vector2f &vel, const Vector3f &offset_body, const Vector3f &offset_nav,
                                   const Vector3f &w_cross_offset_body, const Vector3f &w_cross_offset_nav,
                                   const float &gate, const float &noise_std, FuseData<2> &fuse_data) override;
        uint8_t fuse_mag_body(const Vector3f &mag_body, const float &gate, const float &noise_std, FuseData<3> &fuse_data) override;
        uint8_t fuse_mag_norm(const float &mag_norm, const float &gate, const float &noise_std, FuseData<1> &fuse_data) override;
        uint8_t fuse_mag_ang(const Vector2f &mag_ang, const float &gate, const float &noise_std, FuseData<2> &fuse_data) override;
        uint8_t fuse_flow(const Vector2f &flow, const Vector3f &offset_body, const Vector3f &offset_nav, const float &gate, const float &noise_std, FuseData<2> &fuse_data) override;
        uint8_t fuse_range(const float &range, const Vector3f &offset_body, const Vector3f &offset_nav, const float &gate, const float &noise_std, FuseData<1> &fuse_data) override;

        /*!
         * p = p + δp
         * v = v + δv
         * R = (I + δθ) * R
         * b_Δθ = b_Δθ + δb_Δθ
         * b_Δv = b_Δv + δb_Δv
         * g = g + δg
         * mag_norm = mag_norm + δmag_norm
         * mag_ang = mag_ang + δmag_ang
         * bm = bm + δbm
         * w = w + δw
         */
        void correct_state() override;

        void correct_covariance() override;
    };
}

#endif //ECL_GESKF_H
