//
// Created by Cain on 2022/12/7.
//

#ifndef DSP_SINESTREAM_HPP
#define DSP_SINESTREAM_HPP

#include <lib/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

//#define ARRAY_SINE_STREAM

namespace digital_signal_processing {
    template<unsigned int N>
    class SineStream {
    public:
        SineStream(
#ifdef ARRAY_SINE_STREAM
                   const float f[N],
#else
                   float f0, float f1,
#endif
                   float dt, float amp=1.f, unsigned int settling_periods=1, unsigned int num_periods=0)
        : _amp(amp) {
            assert(dt > 0.f);
#ifdef ARRAY_SINE_STREAM
            assert(f);
#else
            assert(f0 > 0.f);
            assert(f1 > f0);
            assert(f1 < 0.5f / dt);
#endif

            _dt = dt;

            if (num_periods) {
                _num_periods = num_periods;
            } else {
                _num_periods = 3 + settling_periods;
            }

#ifdef ARRAY_SINE_STREAM
            for (unsigned int i = 0; i < N; ++i) {
                _f[i] = f[i];
            }
#else
            _f_min = f0;
            _f_max = f1;
#ifdef EXP_SINE_STREAM
            _df = _f_max / _f_min;
#else
            _df = (_f_max - _f_min) / (N - 1);
#endif
#endif
        }

        float operator()() {
            float value = 0.f;
            if (_f_index < N) {
#ifdef ARRAY_SINE_STREAM
                value = sinf(2.f * M_PI_F * _f[_f_index] * _time);

                _time += _dt;
                if (_time > (float) _num_periods / _f[_f_index] * floorf(_f[_f_index] / _f[0])) {
                    _time = 0.f;
                    ++_f_index;
                }
#else
#ifdef EXP_SINE_STREAM
                float f = f0 * powf(df, float(_f_index) / float(N - 1));
#else
                float f = _f_min + (float)_f_index * _df;
#endif
                value = sinf(2.f * M_PI_F * f * _time);

                _time += _dt;
                if (_time > (float) _num_periods / f * floorf(f / _f_min)) {
                    _time = 0.f;
                    ++_f_index;
                }
#endif
                return value;
            }
            _finished = true;
            return value;
        }

        float get_total_time() const {
            float sum_t = 0.f;
            float t = 0.f;
            unsigned int i = 0;
            while (i < N) {
                t += _dt;
#ifdef ARRAY_SINE_STREAM
                if (t > (float) _num_periods / _f[i] * floorf(_f[i] / _f[0])) {
                    sum_t += t;
                    t = 0.f;
                    ++i;
                }
#else
#ifdef EXP_SINE_STREAM
                float f = f0 * powf(df, float(_f_index) / float(N - 1));
#else
                float f = _f_min + (float)i * _df;
#endif
                if (t > (float) _num_periods / f * floorf(f / _f_min)) {
                    sum_t += t;
                    t = 0.f;
                    ++i;
                }
#endif
            }
            return sum_t;
        }

#ifdef ARRAY_SINE_STREAM
        const float *get_f() {
            return _f;
        }

        void set_f(const float f[N]) {
            if (f) {
                for (unsigned int i = 0; i < N; ++i) {
                    _f[i] = f[i];
                }
            }
        }
#endif

        bool finished() const {
            return _finished;
        }

        void restart() {
            _time = 0.f;
            _f_index = 0;
            _finished = false;
        }

    private:
        bool _finished {false};
        unsigned int _num_periods {0};
        unsigned int _f_index {0};
        float _dt {0.002f};
        float _time {0.f};
        float _amp {1.f};
#ifdef ARRAY_SINE_STREAM
        float _f[N] {};
#else
        float _f_min;
        float _f_max;
        float _df;
#endif
    };
}

#endif //DSP_SINESTREAM_HPP
