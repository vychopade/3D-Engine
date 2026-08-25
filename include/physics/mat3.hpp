#pragma once

#include "physics/vec3.hpp"

namespace physics {

struct Mat3 {
    Vec3 c0{1.0f, 0.0f, 0.0f};
    Vec3 c1{0.0f, 1.0f, 0.0f};
    Vec3 c2{0.0f, 0.0f, 1.0f};

    static constexpr Mat3 identity() { return {}; }
    static constexpr Mat3 zero() { return {{}, {}, {}}; }
    static constexpr Mat3 diagonal(float x, float y, float z) {
        return {{x, 0.0f, 0.0f}, {0.0f, y, 0.0f}, {0.0f, 0.0f, z}};
    }

    constexpr Vec3 operator*(Vec3 v) const { return c0 * v.x + c1 * v.y + c2 * v.z; }

    constexpr Mat3 operator*(const Mat3& o) const { return {(*this) * o.c0, (*this) * o.c1, (*this) * o.c2}; }

    constexpr Mat3 transposed() const {
        return {{c0.x, c1.x, c2.x}, {c0.y, c1.y, c2.y}, {c0.z, c1.z, c2.z}};
    }
};

}  // namespace physics
