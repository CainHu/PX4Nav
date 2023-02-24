//
// Created by Cain on 2022/12/7.
//

#ifndef CONTROL_MISO_BASE_HPP
#define CONTROL_MISO_BASE_HPP

#include <lib/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

namespace control {
    using namespace matrix;

    template<uint8_t STATE_DIM, uint8_t IN_DIM>
    class MISO {
    public:
        MISO(const Vector<float, IN_DIM> &u_min, const Vector<float, IN_DIM> &u_max) : _u_min(u_min), _u_max(u_max) {};
        MISO(const float u_min[IN_DIM], const float u_max[IN_DIM]) : _u_min(u_min), _u_max(u_max) {}
        template<size_t P, size_t Q>
        MISO(const Slice<float, IN_DIM, 1, P, Q> &u_min, const Slice<float, IN_DIM, 1, P, Q> & u_max) :_u_min(u_min), _u_max(u_max) {}
        template<size_t P, size_t Q>
        MISO(const Slice<float, 1, IN_DIM, P, Q> &u_min, const Slice<float, 1, IN_DIM, P, Q> & u_max) :_u_min(u_min), _u_max(u_max) {}
        explicit MISO(float u_min=-1.f, float u_max=1.f) { _u_min.setAll(u_min); _u_max.setAll(u_max); }

        virtual const Vector<float, IN_DIM> &operator()(const float &ref, const float &obs) = 0;
        virtual void reset_integral() = 0;

        bool reached_saturation(uint8_t index) {
            return (fabsf(_u_clip(index) - _u_raw(index)) < 1e-6f);
        }

        void reset() {
            _x.setZero();
            _u_clip.setZero();
            _u_raw.setZero();
        }

        float &operator()(uint8_t index) {
            return _x(index);
        }

        const Vector<float, STATE_DIM> &get_state() {
            return _x;
        }

        const Vector<float, IN_DIM> &get_u_clip() {
            return _u_clip;
        };

        const Vector<float, IN_DIM> &get_u_raw() {
            return _u_raw;
        };

        void set_u_max(const Vector<float, IN_DIM> &u_max) {
            _u_max = u_max;
        }

        void set_u_min(const Vector<float, IN_DIM> &u_min) {
            _u_min = u_min;
        }

        void set_u_max(const float u_max[IN_DIM]) {
            for (uint8_t i = 0; i < IN_DIM; ++i) {
                _u_max(i) = u_max[i];
            }
        }

        void set_u_min(const float u_min[IN_DIM]) {
            for (uint8_t i = 0; i < IN_DIM; ++i) {
                _u_min(i) = u_min[i];
            }
        }

        void set_u_max(const float u_max) {
            _u_max.setAll(u_max);
        }

        void set_u_min(const float u_min) {
            _u_min.setAll(u_min);
        }

    protected:
        Vector<float, IN_DIM> _u_clip {};
        Vector<float, IN_DIM> _u_raw {};
        Vector<float, IN_DIM> _u_max;
        Vector<float, IN_DIM> _u_min;
        Vector<float, STATE_DIM> _x {};

        void clip() {
            for (uint8_t i = 0; i < IN_DIM; ++i) {
                if (_u_raw(i) > _u_max(i)) {
                    _u_clip(i) = _u_max(i);
                } else if (_u_raw(i) < _u_min(i)) {
                    _u_clip(i) = _u_min(i);
                } else {
                    _u_clip(i) = _u_raw(i);
                }
            }
        }
    };
}

#endif //CONTROL_MISO_BASE_H