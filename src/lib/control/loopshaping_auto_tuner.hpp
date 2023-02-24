//
// Created by Cain on 2023/2/8.
//

#ifndef CONTROL_LOOPSHAPING_AUTO_TUNER_HPP
#define CONTROL_LOOPSHAPING_AUTO_TUNER_HPP

#include <lib/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

namespace control {
    struct ParamsModel;
    struct ParamsPID;
    struct ParamsLeadLag;
    struct ParamsLpf;
    struct ParamsLoopShaping;
    class LoopShapingAutoTuner;

    std::ostream & operator<<(std::ostream &out, const ParamsPID &pid);
    std::ostream & operator<<(std::ostream &out, ParamsPID &pid);
    std::ostream & operator<<(std::ostream &out, const ParamsLeadLag &leadlag);
    std::ostream & operator<<(std::ostream &out, ParamsLeadLag &leadlag);
    std::ostream & operator<<(std::ostream &out, const ParamsLpf &lpf);
    std::ostream & operator<<(std::ostream &out, ParamsLpf &lpf);
    std::ostream & operator<<(std::ostream &out, const ParamsLoopShaping &loop_shaping);
    std::ostream & operator<<(std::ostream &out, ParamsLoopShaping &loop_shaping);
    std::ostream & operator<<(std::ostream &out, const LoopShapingAutoTuner &tuner);
    std::ostream & operator<<(std::ostream &out, LoopShapingAutoTuner &tuner);

    /*!
     * G(s) = k / (s + p1) / (s + p2) * exp(-Td*s)
     */
    struct ParamsModel {
        float k {0.f};
        float p1 {0.f};
        float p2 {0.f};
        float Td {0.f};

        ParamsModel() {}

        ParamsModel(float k, float p1, float p2, float Td=0.f) {
            assert(Td >= 0.f);

            this->k = k;
            this->p1 = p1;
            this->p2 = p2;
            this->Td = Td;
        }
    };

    /*!
     * C(s) = k * (s/z1 + 1)/s * (s/z2 + 1)/(s/p2 + 1)
     */
    struct ParamsLeadLag {
        float z1 {0.f};
        float z2 {0.f};
        static constexpr float p1 {0.f};
        float p2 {0.f};
        float k {0.f};

        ParamsLeadLag() {};
        ParamsLeadLag(float z1, float z2, float p2, float k) {
            this->z1 = z1;
            this->z2 = z2;
            this->p2 = p2;
            this->k = k;
        }
        ParamsLeadLag(const ParamsPID &params_pid);

        friend std::ostream & operator<<(std::ostream &out, const ParamsLeadLag &params_leadlag);
        friend std::ostream & operator<<(std::ostream &out, ParamsLeadLag &params_leadlag);
    };

    /*!
     * C(s) = kp + ki/s + kd * s/(s/d_cutoff + 1)
     */
    struct ParamsPID {
        float kp {0.f};
        float ki {0.f};
        float kd {0.f};
        float d_cutoff{0.f};

        ParamsPID() {};
        ParamsPID(float kp, float ki, float kd, float d_cutoff) {
            this->kp = kp;
            this->ki = ki;
            this->kd = kd;
            this->d_cutoff = d_cutoff;
        }
        ParamsPID(const ParamsLeadLag &params_leadLag);

        friend std::ostream & operator<<(std::ostream &out, const ParamsPID &params_pid);
        friend std::ostream & operator<<(std::ostream &out, ParamsPID &params_pid);
    };

    ParamsPID::ParamsPID(const ParamsLeadLag &params_leadLag) {
        d_cutoff = params_leadLag.p2;
        ki = params_leadLag.k;
        kp = ki * (1.f / params_leadLag.z1 + 1.f / params_leadLag.z2) - ki / d_cutoff;
        kd = ki / (params_leadLag.z1 * params_leadLag.z2) - kp / d_cutoff;
    }

    ParamsLeadLag::ParamsLeadLag(const ParamsPID &params_pid) {
        k = params_pid.ki;
        p2 = params_pid.d_cutoff;

        float b = -(params_pid.kp + params_pid.ki / params_pid.d_cutoff) / params_pid.ki;
        float c = (params_pid.kp / params_pid.d_cutoff + params_pid.kd) / params_pid.ki;
        float delta = b * b - 4.f * c;

        assert(delta >= 0.f);
        delta = sqrtf(delta);

        float tmp1 = -b + delta;
        assert(tmp1 != 0.f);

        float tmp2 = -b - delta;
        assert(tmp2 != 0.f);

        z1 = 2.f / tmp1;
        z2 = 2.f / tmp2;
    }

    /*!
     * H(s) = 1 / ((s/wn)^2 + 2*zeta*(s/wn) + 1)
     */
    struct ParamsLpf {
        float wn;
        float zeta;

        ParamsLpf() {};
        ParamsLpf(float wn, float zeta) {
            assert(wn > 0.f);
            assert(zeta > 0.f);

            this->wn = wn;
            this->zeta = zeta;
        }

        friend std::ostream & operator<<(std::ostream &out, const ParamsLpf &params_lpf);
        friend std::ostream & operator<<(std::ostream &out, ParamsLpf &params_lpf);
    };

    std::ostream & operator<<(std::ostream &out, const ParamsPID &pid){
        out << "kp = " << pid.kp << ", ";
        out << "ki = " << pid.ki << ", ";
        out << "kd = " << pid.kd << ", ";
        out << "d_cutoff = " << pid.d_cutoff;
        return out;
    }

    std::ostream & operator<<(std::ostream &out, ParamsPID &pid) {
        out << (const ParamsPID &)pid;
        return out;
    }

    std::ostream & operator<<(std::ostream &out, const ParamsLeadLag &leadlag){
        out << "z1 = " << leadlag.z1 << ", ";
        out << "z2 = " << leadlag.z2 << ", ";
        out << "p1 = " << leadlag.p1 << ", ";
        out << "p2 = " << leadlag.p2 << ", ";
        out << "k = " << leadlag.k;
        return out;
    }

    std::ostream & operator<<(std::ostream &out, ParamsLeadLag &leadlag) {
        out << (const ParamsLeadLag &)leadlag;
        return out;
    }

    std::ostream & operator<<(std::ostream &out, const ParamsLpf &lpf) {
        out << "wn = " << lpf.wn << ", ";
        out << "zeta = " << lpf.zeta;
        return out;
    }

    std::ostream & operator<<(std::ostream &out, ParamsLpf &lpf) {
        out << (const ParamsLpf &)lpf;
        return out;
    }

    struct ParamsLoopShaping {
        float modeled_cutoff {};
        float disturbance_cutoff {};
        float noise_cutoff {};
        float cross {};
        float PM {};

        ParamsLoopShaping() {};

        ParamsLoopShaping(float modeled_cutoff, float disturbance_cutoff, float noise_cutoff, float cross, float PM) {
            assert(disturbance_cutoff > 0.f);
            assert(modeled_cutoff > disturbance_cutoff);
            assert(noise_cutoff > modeled_cutoff);
            assert(cross > disturbance_cutoff);
            assert(cross < modeled_cutoff);
            assert(PM > 0.f);
            assert(PM < 90.f);

            this->modeled_cutoff = modeled_cutoff;
            this->disturbance_cutoff = disturbance_cutoff;
            this->noise_cutoff = noise_cutoff;
            this->cross = cross;
            this->PM = PM;
        }

        friend std::ostream & operator<<(std::ostream &out, const ParamsLoopShaping &loop_shaping);
        friend std::ostream & operator<<(std::ostream &out, ParamsLoopShaping &loop_shaping);
    };

    std::ostream & operator<<(std::ostream &out, const ParamsLoopShaping &loop_shaping) {
        out << "Parameters of loop shaping: " << std::endl;
        out << "Modeled cutoff = " << loop_shaping.modeled_cutoff << std::endl;
        out << "Disturbance cutoff: " << loop_shaping.disturbance_cutoff << std::endl;
        out << "Noise cutoff: " << loop_shaping.noise_cutoff << std::endl;
        out << "Cross frequency: " << loop_shaping.cross << std::endl;
        out << "Phase margin: " << loop_shaping.PM << std::endl;
        return out;
    }

    std::ostream & operator<<(std::ostream &out, ParamsLoopShaping &loop_shaping) {
        out << (const ParamsLoopShaping &)loop_shaping;
        return out;
    }

    /*!
     * Controller: C(s) * (r - H(s)*y)
     */
    class LoopShapingAutoTuner {
    public:
        static constexpr float FREQ2RADS = 2.f * M_PI_F;
        static constexpr float ANG2RAD = M_PI_F / 180.f;
        static constexpr float HALF_PI = 0.5f * M_PI_F;

        /*!
         * 输入系统模型: G(s) = k / ( (s + p1)*(s + p2) ) * exp(-Td*s)
         * @param k - dcgain
         * @param p1 - 极点
         * @param p2 - 极点
         * @param Td - 纯滞后
         */
        LoopShapingAutoTuner(float k, float p1, float p2, float Td=0.f) : _model(k, p1, p2, Td) {}

        LoopShapingAutoTuner(const ParamsModel &model) : _model(model) {}

        float operator()(const ParamsLoopShaping &loop_shaping, float lpf_damp=7.071068e-01f) {
            float modeled_cutoff = loop_shaping.modeled_cutoff * FREQ2RADS;
            float disturbance_cutoff = loop_shaping.disturbance_cutoff * FREQ2RADS;
            float noise_cutoff = loop_shaping.noise_cutoff * FREQ2RADS;
            float cross = loop_shaping.cross * FREQ2RADS;
            float PM = loop_shaping.PM * ANG2RAD;

            _lpf.wn = noise_cutoff;
            _lpf.zeta = lpf_damp;

            _leadlag.p2 = modeled_cutoff;
            _leadlag.z1 = disturbance_cutoff;

            float cross2 = cross * cross;
            _Gamp2 = _model.k * _model.k / (cross2 + _model.p1 * _model.p1) / (cross2 + _model.p2 * _model.p2);
            _Gpha = -atan2f(cross, _model.p1) - atan2f(cross, _model.p2) - _model.Td * cross;

            float cross_noise = cross / noise_cutoff;
            float Hre = 1.f - cross_noise * cross_noise;
            float Him = 2.f * lpf_damp * cross_noise;
            _Hamp2 = 1.f / (Hre * Hre + Him * Him);
            _Hpha = atan2f(-Him, Hre);

            float cross_dis = cross / disturbance_cutoff;
            float cross_modeled = cross / modeled_cutoff;
            float part_Camp2 = (1.f + cross_dis * cross_dis) / (1.f + cross_modeled * cross_modeled) / cross2;
            float part_Cpha = atanf(cross_dis) - atanf(cross_modeled) - HALF_PI;

            _leadlag.z2 = cross / tanf(PM - M_PI_F - _Gpha - _Hpha - part_Cpha);

            float cross_comp = cross / _leadlag.z2;
            _Cpha = part_Cpha + atanf(cross_comp);

            float comp_Camp2 = 1.f / (_Gamp2 * _Hamp2 * part_Camp2);
            _Camp2 = part_Camp2 *comp_Camp2;

            _leadlag.k = sqrtf(comp_Camp2 / (1.f + cross_comp * cross_comp));

            _quality = _leadlag.z2 / _leadlag.z1;

            return _quality;
        }

        /*!
         * 回路成形
         * @param modeled_cutoff - 建模的频率边界(Hz)
         * @param disturbance_cutoff - 扰动截止频率(Hz)
         * @param noise_cutoff - 噪声截止频率(Hz)
         * @param cross - 穿越频率(Hz)
         * @param PM - 相位裕度(°)
         * @param lpf_damp - 阻尼比
         * @return qulity - 质量
         */
        float operator()(float modeled_cutoff, float disturbance_cutoff, float noise_cutoff,
                         float cross, float PM, float lpf_damp=7.071068e-01f) {
            ParamsLoopShaping loop_shaping(modeled_cutoff, disturbance_cutoff, noise_cutoff, cross, PM);
            return (*this)(loop_shaping, lpf_damp);
        }

        float leadlag2performance(const ParamsLeadLag &leadlag, const ParamsLpf &lpf, const ParamsModel &model,
                                 ParamsLoopShaping &loop_shaping) {
            loop_shaping.disturbance_cutoff = (leadlag.z1 < leadlag.z2 ? leadlag.z1 : leadlag.z2) / FREQ2RADS;
            loop_shaping.modeled_cutoff = leadlag.p2 / FREQ2RADS;
            loop_shaping.noise_cutoff = lpf.wn / FREQ2RADS;

            float w1 = 0.01f;
            float w2 = 1000.f;

            float Lamp2_w1 = G_amp2(w1, _model) * H_amp2(w1, lpf) * C_amp2(w1, leadlag);
            while (Lamp2_w1 < 1.f) {
                w1 *= 0.1f;
                Lamp2_w1 = G_amp2(w1, _model) * H_amp2(w1, lpf) * C_amp2(w1, leadlag);
            }

            float Lamp2_w2 = G_amp2(w2, _model) * H_amp2(w2, lpf) * C_amp2(w2, leadlag);
            while (Lamp2_w2 > 1.f) {
                w2 *= 10.f;
                Lamp2_w2 = G_amp2(w2, _model) * H_amp2(w2, lpf) * C_amp2(w2, leadlag);
            }

            float w_mid;
            float Lamp2_w_mid;
            float iter_error = 1e6f;
            unsigned int iter_n = 0;
            while (iter_error > 1e-6f && iter_n < 10000) {
                w_mid = 0.5f * (w1 + w2);
                Lamp2_w_mid = G_amp2(w_mid, _model) * H_amp2(w_mid, lpf) * C_amp2(w_mid, leadlag);

                if (Lamp2_w_mid < 1.f) {
                    w2 = w_mid;
                }
                if (Lamp2_w_mid > 1.f) {
                    w1 = w_mid;
                }

                iter_error = fabs(1.f - Lamp2_w_mid);
                ++iter_n;
            }
            std::cout << "Cross frequency iterator:" << std::endl;
            std::cout << "iter_n = " << iter_n << ", iter_error = " << iter_error << std::endl;

            loop_shaping.cross = w_mid / FREQ2RADS;
            loop_shaping.PM = (Gang(w_mid, model) + Hang(w_mid, lpf) + Cang(w_mid, leadlag) + M_PI_F) / ANG2RAD;

            return leadlag.z1 < leadlag.z2 ? (leadlag.z2 / leadlag.z1) : (leadlag.z1 / leadlag.z2);
        }

        float pid2performance(const ParamsPID &pid, const ParamsLpf &lpf, const ParamsModel &model,
                                 ParamsLoopShaping &loop_shaping) {
            ParamsLeadLag leadlag(pid);
            return leadlag2performance(leadlag, lpf, model, loop_shaping);
        }

        float quality() const { return _quality; }
        const ParamsModel &model() const { return _model; }
        const ParamsLpf &lpf() const { return _lpf; }
        const ParamsLeadLag &leadlag() const { return _leadlag; }

        float G_amp2() const { return _Gamp2; }
        float G_amp() const { return sqrtf(_Gamp2); }
        float G_pha() const { return _Gpha / ANG2RAD; }
        float H_amp2() const { return _Hamp2; }
        float H_amp() const { return sqrtf(_Hamp2); }
        float H_pha() const { return _Hpha / ANG2RAD; }
        float C_amp2() const { return _Camp2; }
        float C_amp() const { return sqrtf(_Camp2); }
        float C_pha() const { return _Cpha / ANG2RAD; }

        static float G_amp2(float w, const ParamsModel &model) {
            float w2 = w * w;
            return model.k * model.k / (w2 + model.p1 * model.p1) / (w2 + model.p2 * model.p2);
        }

        static float H_amp2(float w, const ParamsLpf &lpf) {
            float w_unit = w / lpf.wn;
            float H_inv_re = 1.f - w_unit * w_unit;
            float H_inv_im = 2.f * lpf.zeta * w_unit;
            return 1.f / (H_inv_re * H_inv_re + H_inv_im * H_inv_im);
        }

        static float C_amp2(float w, const ParamsLeadLag &leadlag) {
            float w_z1 = w / leadlag.z1;
            float w_z2 = w / leadlag.z2;
            float w_p2 = w / leadlag.p2;
            return leadlag.k * leadlag.k * (1.f + w_z1 * w_z1) * (1.f + w_z2 * w_z2) / (w * w) / (1.f + w_p2 * w_p2);
        }

        static float C_amp2(float w, const ParamsPID &pid) {
            ParamsLeadLag leadlag(pid);
            return C_amp2(w, leadlag);
        }

        static float G_amp(float w, const ParamsModel &model) {
            return sqrtf(G_amp2(w, model));
        }

        static float H_amp(float w, const ParamsLpf &lpf) {
            return sqrtf(H_amp2(w, lpf));
        }

        static float C_amp(float w, const ParamsLeadLag &leadlag) {
            return sqrtf(C_amp2(w, leadlag));
        }

        static float C_amp(float w, const ParamsPID &pid) {
            return sqrtf(C_amp2(w, pid));
        }

        static float Gang(float w, const ParamsModel &model) {
            return -atan2f(w, model.p1) - atan2f(w, model.p2) - w * model.Td;
        }

        static float Hang(float w, const ParamsLpf &lpf) {
            float w_uint = w / lpf.wn;
            return atan2f(-2.f * lpf.zeta * w_uint, 1.f - w_uint * w_uint);
        }

        static float Cang(float w, const ParamsLeadLag &leadlag) {
            return atan2f(w, leadlag.z1) + atan2(w, leadlag.z2) - atan2(w, leadlag.p2) - HALF_PI;
        }

        /*!
         * 输入系统模型: G(s) = k / ( (s + p1)*(s + p2) ) * exp(-Td*s)
         * @param k - dcgain
         * @param p1 - 极点
         * @param p2 - 极点
         * @param Td - 纯滞后
         */
        void set_model(float k, float p1, float p2, float Td=0.f) {
            assert(Td >= 0.f);
            _model.k = k;
            _model.p1 = p1;
            _model.p2 = p2;
            _model.Td = Td;
        }

        friend std::ostream & operator<<(std::ostream &out, const LoopShapingAutoTuner &tuner);
        friend std::ostream & operator<<(std::ostream &out, LoopShapingAutoTuner &tuner);

    private:
        float _Gamp2 {1.f}, _Gpha {0.f};
        float _Hamp2 {1.f}, _Hpha {0.f};
        float _Camp2 {1.f}, _Cpha {0.f};
        float _quality {0.f};

        ParamsModel _model;
        ParamsLpf _lpf;
        ParamsLeadLag _leadlag;
    };

    std::ostream & operator<<(std::ostream &out, const LoopShapingAutoTuner &tuner) {
        out << "Quality = " << tuner._quality << std::endl;
        out << "Parameters of leadlag: " << tuner._leadlag << std::endl;
        out << "Parameters of pid: " << ParamsPID(tuner._leadlag) << std::endl;
        out << "Low Pass Filter: " << tuner._lpf << std::endl;

        return out;
    }

    std::ostream & operator<<(std::ostream &out, LoopShapingAutoTuner &tuner) {
        out << (const LoopShapingAutoTuner &)tuner;
        return out;
    }
}

#endif //CONTROL_LOOPSHAPING_AUTO_TUNER_HPP
