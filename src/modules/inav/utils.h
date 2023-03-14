//
// Created by Cain on 2023/3/11.
//

#ifndef INAV_UTILS_H
#define INAV_UTILS_H

#include <matrix/math.hpp>
#include <lib/dsp/butterworth.hpp>

namespace inav {
    using matrix::AxisAnglef;
    using matrix::Dcmf;
    using matrix::Eulerf;
    using matrix::Matrix3f;
    using matrix::Quatf;
    using matrix::Vector2f;
    using matrix::Vector3f;
    using matrix::wrap_pi;
    using digital_signal_processing::Butterworth;

    template<typename T>
    class Queue {
    public:
        explicit Queue(uint8_t n) : N(n), _head(n - 1) {
            _data = new T[N];
        }

        explicit Queue(uint8_t n, const T &value) : N(n), _head(n - 1) {
            _data = new T[N];
            for (const T &v : _data) {
                v = value;
            }
        }

        ~Queue() {
            delete [] _data;
        }

        T &operator[](const uint8_t index) {
            return _data[index];
        }

        bool is_empty() const { return _size == 0; };
        bool is_full() const { return _size == N; };
        const T &newest() const { return _data[_head]; };
        const T &oldest() const { return _data[_tail]; };
        uint8_t newest_index() const { return _head; };
        uint8_t oldest_index() const { return _tail; };
        uint8_t size() const { return _size; };
        uint8_t capacity() const { return N; };
        void clear() { _size = 0; _head = N - 1; _tail = 0; }

        void push(const T &value) {
            if (++_head == N) {
                _head = 0;
            }
            _data[_head] = value;

            if (++_size > N) {
                _size = N;
                if (++_tail == N) {
                    _tail = 0;
                }
            }
        }

        bool pop_first_older_than(const uint64_t &timestamp, T & value) {
            for (uint8_t i = 0; i < _size; ++i) {
                uint8_t index = (i > _head) ? (N - i - _head) : _head - i;

                // 离timestamp最接近且延迟小于100ms
                if (timestamp >= _data[index].time_us && timestamp < _data[index].time_us + (uint64_t)1e5) {
                    value = _data[index];

                    // 清空index及index后面的元素
                    _tail = index + 1;
                    if (_tail == N) {
                        _tail = 0;
                    }
                    _size = i;

                    // TODO: 需要在作处理
//                    _data[index].time_us = 0;

                    return true;
                }
            }

            // 队列为空 或者 没有timestamp之前的数据, 则返回失败
            return false;
        }

    protected:
        const uint8_t N;
        T *_data {nullptr};
        uint8_t _size {0};
        uint8_t _head;
        uint8_t _tail {0};
    };

    class LowPassFilter3d {
    public:
        LowPassFilter3d(float fs, float cutoff) : _filter{Butterworth<1>(fs, cutoff), Butterworth<1>(fs, cutoff), Butterworth<1>(fs, cutoff)} {};

        Vector3f operator()(const Vector3f &u) {
            return {_filter[0](u(0)), _filter[1](u(1)), _filter[2](u(2))};
        }

        void set_fs_and_cutoff(float fs, float cutoff) {
            _filter[0].set_fs_and_cutoff(fs, cutoff);
            _filter[1].set_fs_and_cutoff(fs, cutoff);
            _filter[2].set_fs_and_cutoff(fs, cutoff);
        }

        void set_fs(float fs) {
            _filter[0].set_fs(fs);
            _filter[1].set_fs(fs);
            _filter[2].set_fs(fs);
        }

        void set_cutoff(float cutoff) {
            _filter[0].set_cutoff(cutoff);
            _filter[1].set_cutoff(cutoff);
            _filter[2].set_cutoff(cutoff);
        }

        void reset_filter_state() {
            _filter[0].reset_filter_state();
            _filter[1].reset_filter_state();
            _filter[2].reset_filter_state();
        }

        void reset_filter_state_by_input(const Vector3f &input) {
            _filter[0].reset_filter_state_by_input(input(0));
            _filter[1].reset_filter_state_by_input(input(1));
            _filter[2].reset_filter_state_by_input(input(2));
        }

        void reset_filter_state_by_output(const Vector3f &output) {
            _filter[0].reset_filter_state_by_output(output(0));
            _filter[1].reset_filter_state_by_output(output(1));
            _filter[2].reset_filter_state_by_output(output(2));
        }
    private:
        Butterworth<1> _filter[3];
    };
}

#endif //INAV_UTILS_H
