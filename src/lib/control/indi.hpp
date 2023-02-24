//
// Created by Cain on 2023/2/5.
//

#ifndef CONTROL_INDI_HPP
#define CONTROL_INDI_HPP

#include <lib/control/pid_2dof.hpp>

#define ADAPTIVE

#ifdef ADAPTIVE
#define ADAPTIVE_LAMBDA 0.01f
#endif

namespace control {
    class INDI {
    public:
        /*!
         * 传递函数: u = wn^2/Kp * (Tp*s+1)(k*(r-y) - s*y) / (s*(s+2*zeta*wn))
         * @param fs
         * @param Kp
         * @param Tp
         * @param k
         * @param wn
         * @param zeta
         * @param lpf_wn
         * @param lpf_zeta
         * @param u_min
         * @param u_max
         * @param prewarp
         */
        INDI(float fs, float Kp, float Tp, float k, float wn, float zeta,
#ifdef LPF_FILTER
             float lpf_wn, float lpf_zeta,
#endif
             float u_min=-1.f, float u_max=1.f, float prewarp=NAN)
#ifdef LPF_FILTER
             : _Kp(Kp), _Tp(Tp), _k(k), _wn(wn), _zeta(zeta)
#endif
        {
            assert(fs > 0.f);
            assert(Kp > 0.f);
            assert(Tp > 0.f);
            assert(k > 0.f);
            assert(wn > 0.f);
            assert(zeta > 0.f);

            float tmp = wn * wn / Kp;
            float pc = 2.f * zeta * wn;
            float ki = tmp * k / pc;
            float kp = (tmp * (Tp * k + 1.f) - ki) / pc;
            float kd = (tmp * Tp - kp) / pc;

            _pid_2dof_p = new PID2Dof(fs, kp, ki, kd, 2.f * M_PI_F * pc,
#ifdef LPF_FILTER
                                      lpf_wn, lpf_zeta,
#endif
                                      1.f, 0.f, u_min, u_max, prewarp);
        }

        float operator()(float ref, float meas) {
            return (*_pid_2dof_p)(ref, meas);
        }

        const PID2Dof *get_pid_2dof_p() const { return _pid_2dof_p; }

        void set_indi(float Kp, float Tp, float k, float wn, float zeta, float prewrap=NAN) {
            assert(Kp > 0.f);
            assert(Tp > 0.f);
            assert(k > 0.f);
            assert(wn > 0.f);
            assert(zeta > 0.f);

            float tmp = wn * wn / Kp;
            float pc = 2.f * zeta * wn;
            float ki = tmp * k / pc;
            float kp = (tmp * (Tp * k + 1.f) - ki) / pc;
            float kd = (tmp * Tp - kp) / pc;

            _pid_2dof_p->set_kp(kp);
            _pid_2dof_p->set_ki(ki);
            _pid_2dof_p->set_kd(kd);
            _pid_2dof_p->set_i_prewarp(prewrap);
            _pid_2dof_p->set_d_cutoff_and_prewarp(2.f * M_PI_F * pc, prewrap);
        }

#ifdef LPF_FILTER
        void set_lpf(float wn, float zeta, float prewrap=NAN) {
            _pid_2dof_p->set_lpf_cutoff_damp_and_prewarp(2.f * M_PI_F * wn, zeta, prewrap);
        }
#endif

    protected:
#ifdef ADAPTIVE
        float _Kp, _Tp, _k, _wn, _zeta;
#endif
        PID2Dof *_pid_2dof_p {nullptr};
    };
}

#endif //CONTROL_INDI_HPP
