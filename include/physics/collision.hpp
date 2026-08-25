#pragma once

#include "physics/contact.hpp"

namespace physics {

struct Body;

// Returns true and fills `out` when A and B overlap.
bool collide(Body& a, Body& b, Contact& out);

}  // namespace physics
