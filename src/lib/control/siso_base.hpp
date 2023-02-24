//
// Created by Cain on 2022/12/7.
//

#ifndef CONTROL_SISO_BASE_HPP
#define CONTROL_SISO_BASE_HPP

#include <lib/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

namespace control {
    using namespace matrix;

    template<uint8_t STATE_DIM>
    class SISO {
    public:
        explicit SISO(float u_min=-1.f, float u_max=1.f) :_u_min(u_min), _u_max(u_max) {};

        virtual float operator()(const float &ref, const float &obs) = 0;
        virtual void reset_integral() = 0;

        bool reached_saturation() {
            return (fabsf(_u_clip - _u_raw) < 1e-6f);
        }

        void reset() {
            _x.setZero();
            _u_clip = 0.f;
            _u_raw = 0.f;
        }

        float &operator()(uint8_t index) {
            return _x(index);
        }

        const Vector<float, STATE_DIM> &get_state() {
            return _x;
        }

        float get_u_clip() {
            return _u_clip;
        };

        float get_u_raw() {
            return _u_raw;
        };

        void set_u_max(float u_max) {
            _u_max = u_max;
        }

        void set_u_min(float u_min) {
            _u_min = u_min;
        }

    protected:
        float _u_clip {0.f};
        float _u_raw {0.f};
        float _u_min {-1.f};
        float _u_max {1.f};
        Vector<float, STATE_DIM> _x {};

        void clip() {
            if (_u_raw > _u_max) {
                _u_clip = _u_max;
            } else if (_u_raw < _u_min) {
                _u_clip = _u_min;
            } else {
                _u_clip = _u_raw;
            }
        }
    };
}

#endif //CONTROL_SISO_BASE_H