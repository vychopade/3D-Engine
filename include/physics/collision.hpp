#pragma once

#include "physics/contact.hpp"
#include "physics/vec3.hpp"

namespace physics {

struct Body;

// Returns true and fills `out` with the first contact when A and B overlap.
bool collide(Body& a, Body& b, Contact& out);

// Writes up to max_contacts contacts. Returns how many were generated.
int collide(Body& a, Body& b, Contact* out, int max_contacts);

// Sweeps `moving` along `displacement`. t is in [0, 1] at first impact.
// `normal` points from the obstacle toward the moving body.
bool sweep(const Body& moving, const Body& obstacle, Vec3 displacement, float& t, Vec3& normal);

}  // namespace physics
