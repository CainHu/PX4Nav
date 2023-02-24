//
// Created by Cain on 2023/2/5.
//

#ifndef CONTROL_LEADLAG_2DOF_HPP
#define CONTROL_LEADLAG_2DOF_HPP

#include <lib/control/pid_2dof.hpp>

namespace control {
    class LeadLag2Dof {
    public:
        /*!
         * 传递函数: u = -k*(s/z1+1)*(s/z2+1)/(s*(s/p+1))*y + k*(s/z1+1)*(s/z2+1)/(s*(s/p+1))*r
         * @param fs - 采样频率(Hz)
         * @param k - 超前滞后的增益
         * @param z1 - y相关的超前滞后的零点(rad/s)
         * @param z2 - y相关的超前滞后的零点(rad/s)
         * @param p 超前滞后的极点(rad/s)
         * @param lpf_wn - 量测量的二阶低通滤波的截止频率(rad/s)
         * @param lpf_zeta - 量测量的二阶低通滤波的阻尼比
         * @param z3 - r相关的超前滞后的零点(rad/s)
         * @param z4 - r相关的超前滞后的零点(rad/s)
         * @param u_min - 最小输出控制量
         * @param u_max - 最大输出控制量
         * @param prewarp - 双线性变换的预畸变频率(Hz)
         */
        LeadLag2Dof(float fs, float k, float z1, float z2, float p,
#ifdef LPF_FILTER
                    float lpf_wn, float lpf_zeta,
#endif
                    float z3=NAN, float z4=NAN, float u_min=-1.f, float u_max=1.f, float prewarp=NAN) {
#ifdef LPF_FILTER
            assert(2.f * M_PI_F * lpf_wn < 0.5f * fs);
#endif
            assert(k > 0.f);
            assert(p > 0.f);

            if (!std::isfinite(z3) || !std::isfinite(z4)) {
                z3 = p;
                z4 = 1.f / (1.f / z1 + 1.f / z2 - 1.f / p);
            }

            float z1_inv = 1.f / z1, z2_inv = 1.f / z2, z3_inv = 1.f / z3, z4_inv = 1.f / z4, p_inv = 1.f / p;
            float p_inv2 = p_inv * p_inv;
            float tmp1 = z1_inv + z2_inv;
            float tmp2 = z1_inv * z2_inv;
            float tmp3 = z3_inv + z4_inv;
            float tmp4 = z3_inv * z4_inv;
            float tmp5 = tmp1 - p_inv;
            float tmp6 = p_inv2 - tmp1 * p_inv + tmp2;
            float ki = k;
            float kp = k * tmp5;
            float kd = k * tmp6;
            float b = (tmp3 - p_inv) / tmp5;
            float c = (p_inv2 - tmp3 * p_inv + tmp4) / tmp6;
            float d_cutoff = 2.f * M_PI_F * p;

            _pid_2dof_p = new PID2Dof(fs, kp, ki, kd, d_cutoff,
#ifdef LPF_FILTER
                                      2.f * M_PI_F * lpf_wn, lpf_zeta,
#endif
                                      b, c, u_min, u_max, prewarp);
        }

        float operator()(float ref, float meas) {
            return (*_pid_2dof_p)(ref, meas);
        }

        const PID2Dof *get_pid_2dof_p() const { return _pid_2dof_p; }

        void set_lead_lag(float k, float z1, float z2, float p, float z3=NAN, float z4=NAN, float prewrap=NAN) {
            assert(k > 0.f);
            assert(p > 0.f);

            if (!std::isfinite(z3) || !std::isfinite(z4)) {
                z3 = p;
                z4 = 1.f / (1.f / z1 + 1.f / z2 - 1.f / p);
            }

            float z1_inv = 1.f / z1, z2_inv = 1.f / z2, z3_inv = 1.f / z3, z4_inv = 1.f / z4, p_inv = 1.f / p;
            float p_inv2 = p_inv * p_inv;
            float tmp1 = z1_inv + z2_inv;
            float tmp2 = z1_inv * z2_inv;
            float tmp3 = z3_inv + z4_inv;
            float tmp4 = z3_inv * z4_inv;
            float tmp5 = tmp1 - p_inv;
            float tmp6 = p_inv2 - tmp1 * p_inv + tmp2;
            float ki = k;
            float kp = k * tmp5;
            float kd = k * tmp6;
            float b = (tmp3 - p_inv) / tmp5;
            float c = (p_inv2 - tmp3 * p_inv + tmp4) / tmp6;
            float d_cutoff = 2.f * M_PI_F * p;

            _pid_2dof_p->set_kp(kp);
            _pid_2dof_p->set_ki(ki);
            _pid_2dof_p->set_kd(kd);
            _pid_2dof_p->set_b(b);
            _pid_2dof_p->set_c(c);
            _pid_2dof_p->set_i_prewarp(prewrap);
            _pid_2dof_p->set_d_cutoff_and_prewarp(d_cutoff, prewrap);
        }

#ifdef LPF_FILTER
        void set_lpf(float wn, float zeta, float prewrap=NAN) {
            _pid_2dof_p->set_lpf_cutoff_damp_and_prewarp(2.f * M_PI_F * wn, zeta, prewrap);
        }
#endif

    protected:
        PID2Dof *_pid_2dof_p {nullptr};
    };
}

#endif //CONTROL_LEADLAG_2DOF_HPP
