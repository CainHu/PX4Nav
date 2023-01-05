//
// Created by Cain on 2023/1/3.
//

#ifndef ECL_UTILS_H
#define ECL_UTILS_H

#include <matrix/math.hpp>
#include <geo/geo.h>

namespace eskf {
    using matrix::AxisAnglef;
    using matrix::Dcmf;
    using matrix::Eulerf;
    using matrix::Matrix3f;
    using matrix::Quatf;
    using matrix::Vector2f;
    using matrix::Vector3f;
    using matrix::wrap_pi;

    static constexpr float sq(float var) { return var * var; }

    template<typename T, uint8_t N>
    class Queue {
    public:
        Queue() = default;
        explicit Queue(const T &value) {
            for (const T &v : _data) {
                v = value;
            }
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

                    _data[index].time_us = 0;

                    return true;
                }
            }

            // 队列为空 或者 没有timestamp之前的数据, 则返回失败
            return false;
        }

    protected:
        T _data[N] {};
        uint8_t _size {0};
        uint8_t _head {N - 1};
        uint8_t _tail {0};
    };

    void rotation_from_axis_angle(Dcmf &r, const Vector3f &axis_angle);
    void quaternion_from_axis_angle(Quatf &q, const Vector3f &axis_angle);

}


#endif //ECL_UTILS_H
