#pragma once

#include "physics/body.hpp"
#include "physics/vec3.hpp"

namespace physics {

// Fixed-length rod. Used for pendulums: a bob hangs from a pivot (body or world point)
// and gravity provides the restoring torque.
struct DistanceJoint {
    Body* a = nullptr;
    Body* b = nullptr;  // nullptr = world-space pivot at local_b
    Vec3 local_a;
    Vec3 local_b;
    float length = 1.0f;
    bool pendulum = false;

    Vec3 point_a() const { return a != nullptr ? a->local_to_world(local_a) : local_a; }
    Vec3 point_b() const { return b != nullptr ? b->local_to_world(local_b) : local_b; }
};

}  // namespace physics
