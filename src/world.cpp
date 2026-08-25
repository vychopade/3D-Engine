#include "physics/world.hpp"

#include "physics/collision.hpp"

#include <algorithm>
#include <cmath>

namespace physics {
namespace {

constexpr float kSlop = 0.01f;
constexpr float kBaumgarte = 0.2f;
constexpr float kFrictionEpsilon = 1e-6f;

float angular_effective_mass(const Body& body, Vec3 radius, Vec3 direction) {
    const Vec3 r_cross_n = cross(radius, direction);
    return dot(r_cross_n, body.inv_inertia_world() * r_cross_n);
}

void resolve_velocity(Contact& c) {
    Body& a = *c.a;
    Body& b = *c.b;

    const Vec3 ra = c.point - a.position;
    const Vec3 rb = c.point - b.position;

    const Vec3 va = a.velocity + cross(a.angular_velocity, ra);
    const Vec3 vb = b.velocity + cross(b.angular_velocity, rb);
    const Vec3 rv = vb - va;

    const float vel_n = dot(rv, c.normal);
    if (vel_n > 0.0f) {
        return;
    }

    const float inv_mass = a.inv_mass + b.inv_mass + angular_effective_mass(a, ra, c.normal) +
                           angular_effective_mass(b, rb, c.normal);
    if (inv_mass <= 0.0f) {
        return;
    }

    const float e = std::min(a.restitution, b.restitution);
    const float j = -(1.0f + e) * vel_n / inv_mass;
    const Vec3 impulse = c.normal * j;
    a.apply_impulse(-impulse, ra);
    b.apply_impulse(impulse, rb);

    const Vec3 rv2 = (b.velocity + cross(b.angular_velocity, rb)) -
                     (a.velocity + cross(a.angular_velocity, ra));
    Vec3 tangent = rv2 - c.normal * dot(rv2, c.normal);
    if (tangent.length_sq() < kFrictionEpsilon) {
        return;
    }
    tangent = tangent.normalized();

    const float inv_mass_t = a.inv_mass + b.inv_mass + angular_effective_mass(a, ra, tangent) +
                             angular_effective_mass(b, rb, tangent);
    float jt = -dot(rv2, tangent) / inv_mass_t;

    const float mu = std::sqrt(a.friction * b.friction);
    const float max_friction = j * mu;
    jt = std::clamp(jt, -max_friction, max_friction);

    const Vec3 friction = tangent * jt;
    a.apply_impulse(-friction, ra);
    b.apply_impulse(friction, rb);
}

void resolve_position(Contact& c) {
    Body& a = *c.a;
    Body& b = *c.b;

    const float correction_mag =
        std::max(c.penetration - kSlop, 0.0f) / (a.inv_mass + b.inv_mass + 1e-8f) * kBaumgarte;
    const Vec3 correction = c.normal * correction_mag;
    a.position -= correction * a.inv_mass;
    b.position += correction * b.inv_mass;
}

}  // namespace

Body* World::add_sphere(Vec3 position, float radius, float density, BodyType type) {
    auto body = std::make_unique<Body>();
    body->position = position;
    body->shape = Shape::sphere(radius);
    body->type = type;
    compute_mass(*body, density);
    Body* ptr = body.get();
    bodies_.push_back(std::move(body));
    return ptr;
}

Body* World::add_box(Vec3 position, Vec3 half_extents, float density, BodyType type) {
    auto body = std::make_unique<Body>();
    body->position = position;
    body->shape = Shape::box(half_extents);
    body->type = type;
    compute_mass(*body, density);
    Body* ptr = body.get();
    bodies_.push_back(std::move(body));
    return ptr;
}

void World::clear() {
    bodies_.clear();
    contacts_.clear();
}

void World::integrate_velocities(float dt) {
    for (auto& body : bodies_) {
        if (body->type != BodyType::Dynamic) {
            body->force = {};
            body->torque = {};
            continue;
        }

        body->velocity += (gravity + body->force * body->inv_mass) * dt;
        body->angular_velocity += body->inv_inertia_world() * body->torque * dt;

        const float lin = 1.0f / (1.0f + dt * body->linear_damping);
        const float ang = 1.0f / (1.0f + dt * body->angular_damping);
        body->velocity *= lin;
        body->angular_velocity *= ang;

        body->force = {};
        body->torque = {};
    }
}

void World::integrate_positions(float dt) {
    for (auto& body : bodies_) {
        if (body->type == BodyType::Static) {
            continue;
        }
        body->position += body->velocity * dt;
        const Vec3 w = body->angular_velocity;
        const Quat omega_q{0.0f, w.x, w.y, w.z};
        body->orientation = (body->orientation + (omega_q * body->orientation) * (0.5f * dt)).normalized();
    }
}

void World::find_contacts() {
    contacts_.clear();
    const std::size_t n = bodies_.size();
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            Body& a = *bodies_[i];
            Body& b = *bodies_[j];
            if (a.inv_mass == 0.0f && b.inv_mass == 0.0f) {
                continue;
            }
            Contact contact;
            if (collide(a, b, contact)) {
                contacts_.push_back(contact);
            }
        }
    }
}

void World::solve_velocities() {
    for (int i = 0; i < velocity_iterations; ++i) {
        for (Contact& c : contacts_) {
            resolve_velocity(c);
        }
    }
}

void World::solve_positions() {
    for (int i = 0; i < position_iterations; ++i) {
        find_contacts();
        for (Contact& c : contacts_) {
            resolve_position(c);
        }
    }
}

void World::step(float dt) {
    if (dt <= 0.0f) {
        return;
    }
    integrate_velocities(dt);
    find_contacts();
    solve_velocities();
    integrate_positions(dt);
    solve_positions();
}

}  // namespace physics
