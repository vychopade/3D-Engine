#pragma once

#include "physics/vec3.hpp"

namespace physics {

struct Body;

struct Contact {
    Body* a = nullptr;
    Body* b = nullptr;
    Vec3 normal;  // from A toward B
    Vec3 point;
    float penetration = 0.0f;
};

}  // namespace physics
