#pragma once

#include "physics/body.hpp"
#include "physics/vec3.hpp"

namespace physics {

// Distance spring: F = -k (L - rest) - c * v_rel  along the axis (simple harmonic motion).
struct Spring {
    Body* a = nullptr;
    Body* b = nullptr;  // nullptr = fixed world anchor at local_b
    Vec3 local_a;
    Vec3 local_b;
    float rest_length = 1.0f;
    float stiffness = 80.0f;
    float damping = 4.0f;
    bool bungee = false;  // force only when stretched

    Vec3 point_a() const { return a != nullptr ? a->local_to_world(local_a) : local_a; }
    Vec3 point_b() const { return b != nullptr ? b->local_to_world(local_b) : local_b; }
};

}  // namespace physics
