//
// Created by Cain on 2023/1/13.
//

#ifndef DSP_BIQUADRATIC_HPP
#define DSP_BIQUADRATIC_HPP

#include <lib/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

#define PX4_ISFINITE std::isfinite

namespace digital_signal_processing {
    /*!
     * Bi-Second Order System:
     * H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
     * y = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2) * u
     * v = 1 / (1 + a1*z^-1 + a2*z^-2) * u
     * v = u - a1*v1 - a2*v2
     * y = (b0 + b1*z^-1 + b2*z^-2) * v
     *   = b0 * v + b1 * v1 + b2 * v2
     */
    class BiQuadratic {
    public:
        explicit BiQuadratic(float a1=0.f, float a2=0.f, float b0=1.f, float b1=0.f, float b2=0.f)
                : _a1(a1), _a2(a2), _b0(b0), _b1(b1), _b2(b2) { }

        float operator()(float u) {
            float s = u - _a1 * _state[0] - _a2 * _state[1];
            float y = _b0 * s + _b1 * _state[0] + _b2 * _state[1];
            _state[1] = _state[0];
            _state[0] = s;
            return y;
        }

        void reset(float state_0=0.f, float state_1=0.f) {
            _state[0] = state_0;
            _state[1] = state_1;
        }

        bool reset_by_input(float u=0.f) {
            float state = u / (1.f + _a1 + _a2);
            if (PX4_ISFINITE(state)) {
                _state[0] = _state[1] = state;
                return true;
            }
            _state[0] = _state[1] = 0.f;
            return false;
        }

        bool reset_by_output(float y=0.f) {
            float state = y / (_b0 + _b1 + _b2);
            if (PX4_ISFINITE(state)) {
                _state[0] = _state[1] = state;
                return true;
            }
            _state[0] = _state[1] = 0.f;
            return false;
        }

        void set_parameters(float a1, float a2, float b0, float b1, float b2) {
            _a1 = a1;
            _a2 = a2;
            _b0 = b0;
            _b1 = b1;
            _b2 = b2;
        }

        void set_parameters_pole(float a1, float a2) {
            _a1 = a1;
            _a2 = a2;
        }

        void set_parameters_zero(float b0, float b1, float b2) {
            _b0 = b0;
            _b1 = b1;
            _b2 = b2;
        }

        float &a1() { return _a1; }
        float &a2() { return _a2; }
        float &b0() { return _b0; }
        float &b1() { return _b1; }
        float &b2() { return _b2; }

    private:
        // 系统参数
        float _a1 {0.f};
        float _a2 {0.f};
        float _b0 {1.f};
        float _b1 {0.f};
        float _b2 {0.f};

        // 系统状态
        float _state[2] = {0.f, 0.f};

    };
}

#endif //DSP_BIQUADRATIC_HPP
