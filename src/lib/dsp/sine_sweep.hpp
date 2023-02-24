//
// Created by Cain on 2022/12/7.
//

#ifndef DSP_SINE_SWEEP_HPP
#define DSP_SINE_SWEEP_HPP

#include <lib/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

namespace digital_signal_processing {
    class SineSweep {
    public:
        SineSweep(const float f0, const float f1, const float dt, const float period, const float amp=1.f, const unsigned int periodic_numbers=1)
        : _amp(amp), _periodic_numbers(periodic_numbers) {
            assert(dt > 0.f);
            assert(f0 > 0.f);
            assert(f1 > f0);
            assert(f1 < 0.5f / dt);
            assert(period > 2.f * dt);

            _f0 = f0;
            _f1 = f1;
            _dt = dt;
            _period = period;

#ifdef EXP_SINE_SWEEP
            _lambda = logf(_f1 / _f0) / _period;
            _gain = 2.f * (float)M_PI * _f0 / _lambda;

            float k = ceilf(2.f / _lambda * (_f1 - _f0));
            float t = logf(_f0 / _f1 * (1.f + k * M_PI_F / _gain)) / _lambda;
            _period += t;
#else
            _a = 2.f * M_PI_F * (f1 - f0) / period;
            _v = 2.f * M_PI_F * f0;

            float df = (f1 - f0) / period;
            float delta = f0 * period + 0.5f * df * period * period;
            delta = ceilf(delta) - delta;
            _period += (sqrtf(f1 * f1 + 2.f * df * delta) - f1) / df;
#endif
        };

        float operator()() {
            float value = 0.f;
            if (_count < _periodic_numbers) {
#ifdef EXP_SINE_SWEEP
                value = _amp * sinf(_gain * (expf(_lambda * _time) - 1.f));
#else
                value = _amp * sinf(_v * _time + 0.5f * _a * _time * _time);
#endif

                _time += _dt;
                if (_time > _period + _dt) {
                    _time = 0.f;
                    ++_count;
                }

                return value;
            }
            _finished = true;
            return value;
        }

        bool finished() const {
            return _finished;
        }

        void restart() {
            _time = 0.f;
            _count = 0;
            _finished = false;
        }

    private:
        float _f0 {0.01f};
        float _f1 {30.f};
        float _dt {0.002f};
        float _period {300.f};
        float _time {0.f};
        float _amp {1.f};
        unsigned int _count {0};
        unsigned int _periodic_numbers {1};
        bool _finished {false};

#ifdef EXP_SINE_SWEEP
        float _lambda;
        float _gain;
#else
        float _a;
        float _v;
#endif
    };
}


#endif //DSP_SINE_SWEEP_HPP
