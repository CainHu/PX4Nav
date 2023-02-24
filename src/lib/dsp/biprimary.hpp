//
// Created by Cain on 2023/1/13.
//

#ifndef DSP_BIPRIMARY_HPP
#define DSP_BIPRIMARY_HPP

#include <lib/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

#define PX4_ISFINITE std::isfinite

namespace digital_signal_processing {
    /*!
     * Bi-First Order System:
     * H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
     * y = (b0 + b1*z^-1) / (1 + a1*z^-1) * u
     * v = 1 / (1 + a1*z^-1) * u
     * v = u - a1*v1
     * y = (b0 + b1*z^-1) * v
     *   = b0 * v + b1 * v1
     */
    class BiPrimary {
    public:
        explicit BiPrimary(float a1=0.f, float b0=1.f, float b1=0.f)
                : _a1(a1), _b0(b0), _b1(b1) { }

        float operator()(float u) {
            float s = u - _a1 * _state;
            float y = _b0 * s + _b1 * _state;
            _state = s;
            return y;
        }

        void reset(float state=0.f) {
            _state = state;
        }

        bool reset_by_input(float u=0.f) {
            float state = u / (1.f + _a1);
            if (PX4_ISFINITE(state)) {
                _state = state;
                return true;
            }
            _state = 0.f;
            return false;
        }

        bool reset_by_output(float y=0.f) {
            float state = y / (_b0 + _b1);
            if (PX4_ISFINITE(state)) {
                _state = state;
                return true;
            }
            _state = 0.f;
            return false;
        }

        void set_parameters(float a1, float b0, float b1) {
            _a1 = a1;
            _b0 = b0;
            _b1 = b1;
        }

        void set_parameters_pole(float a1) {
            _a1 = a1;
        }

        void set_parameters_zero(float b0, float b1) {
            _b0 = b0;
            _b1 = b1;
        }

        float &a1() { return _a1; }
        float &b0() { return _b0; }
        float &b1() { return _b1; }

    private:
        // 系统参数
        float _a1 {0.f};
        float _b0 {1.f};
        float _b1 {0.f};

        // 系统状态
        float _state {0.f};

    };
}

#endif //DSP_BIPRIMARY_HPP
