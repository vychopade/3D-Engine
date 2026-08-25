#pragma once

#include "physics/mat3.hpp"
#include "physics/vec3.hpp"

#include <cmath>

namespace physics {

struct Quat {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    static constexpr Quat identity() { return {}; }

    static Quat from_axis_angle(Vec3 axis, float angle) {
        axis = axis.normalized();
        const float half = angle * 0.5f;
        const float s = std::sin(half);
        return {std::cos(half), axis.x * s, axis.y * s, axis.z * s};
    }

    constexpr float length_sq() const { return w * w + x * x + y * y + z * z; }

    Quat normalized() const {
        const float len = std::sqrt(length_sq());
        if (len < 1e-8f) {
            return identity();
        }
        return {w / len, x / len, y / len, z / len};
    }

    Vec3 rotate(Vec3 v) const {
        const Vec3 qv{x, y, z};
        const Vec3 t = cross(qv, v) * 2.0f;
        return v + t * w + cross(qv, t);
    }

    Mat3 to_mat3() const {
        const float xx = x * x, yy = y * y, zz = z * z;
        const float xy = x * y, xz = x * z, yz = y * z;
        const float wx = w * x, wy = w * y, wz = w * z;
        return {{1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz), 2.0f * (xz - wy)},
                {2.0f * (xy - wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx)},
                {2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy)}};
    }
};

constexpr Quat operator+(Quat a, Quat b) { return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Quat operator*(Quat q, float s) { return {q.w * s, q.x * s, q.y * s, q.z * s}; }
constexpr Quat operator*(float s, Quat q) { return q * s; }

constexpr Quat operator*(Quat a, Quat b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

}  // namespace physics
