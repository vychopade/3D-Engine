#include "physics/world.hpp"

#include "physics/collision.hpp"

#include <algorithm>
#include <cmath>

namespace physics {
namespace {

constexpr float kSlop = 0.01f;
constexpr float kBaumgarte = 0.25f;
constexpr float kFrictionEpsilon = 1e-6f;
constexpr float kRestitutionThreshold = 1.0f;
constexpr float kMaxBias = 4.0f;
constexpr float kMaxLinearSpeed = 40.0f;
constexpr float kMaxAngularSpeed = 50.0f;

float angular_effective_mass(const Body& body, Vec3 radius, Vec3 direction) {
    const Vec3 r_cross_n = cross(radius, direction);
    return dot(r_cross_n, body.inv_inertia_world() * r_cross_n);
}

void resolve_velocity(Contact& c, float dt) {
    Body& a = *c.a;
    Body& b = *c.b;

    const Vec3 ra = c.point - a.position;
    const Vec3 rb = c.point - b.position;

    const Vec3 va = a.velocity + cross(a.angular_velocity, ra);
    const Vec3 vb = b.velocity + cross(b.angular_velocity, rb);
    const Vec3 rv = vb - va;

    const float vel_n = dot(rv, c.normal);
    if (vel_n > 0.0f && c.penetration <= kSlop) {
        return;
    }

    const float inv_mass = a.inv_mass + b.inv_mass + angular_effective_mass(a, ra, c.normal) +
                           angular_effective_mass(b, rb, c.normal);
    if (inv_mass <= 0.0f) {
        return;
    }

    float e = std::min(a.restitution, b.restitution);
    if (vel_n > -kRestitutionThreshold) {
        e = 0.0f;
    }

    float bias = 0.0f;
    if (c.penetration > kSlop && dt > 1e-6f) {
        bias = std::min(kBaumgarte * (c.penetration - kSlop) / dt, kMaxBias);
    }

    float j = -((1.0f + e) * vel_n - bias) / inv_mass;
    j = std::max(j, 0.0f);
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
    if (inv_mass_t <= 0.0f) {
        return;
    }
    float jt = -dot(rv2, tangent) / inv_mass_t;

    const float mu = std::sqrt(std::max(a.friction, 0.0f) * std::max(b.friction, 0.0f));
    const float max_friction = j * mu;
    jt = std::clamp(jt, -max_friction, max_friction);

    const Vec3 friction = tangent * jt;
    a.apply_impulse(-friction, ra);
    b.apply_impulse(friction, rb);
}

void resolve_position(Contact& c) {
    Body& a = *c.a;
    Body& b = *c.b;

    const float inv_mass_sum = a.inv_mass + b.inv_mass;
    if (inv_mass_sum <= 1e-8f) {
        return;
    }

    const float percent = (a.inv_mass <= 0.0f || b.inv_mass <= 0.0f) ? 0.95f : 0.4f;
    const float correction_mag =
        std::max(c.penetration - kSlop, 0.0f) / inv_mass_sum * percent;
    const Vec3 correction = c.normal * correction_mag;
    a.position -= correction * a.inv_mass;
    b.position += correction * b.inv_mass;
}

void clamp_velocity(Body& body) {
    const float v2 = body.velocity.length_sq();
    if (v2 > kMaxLinearSpeed * kMaxLinearSpeed) {
        body.velocity = body.velocity * (kMaxLinearSpeed / std::sqrt(v2));
    }
    const float w2 = body.angular_velocity.length_sq();
    if (w2 > kMaxAngularSpeed * kMaxAngularSpeed) {
        body.angular_velocity = body.angular_velocity * (kMaxAngularSpeed / std::sqrt(w2));
    }
}

}  // namespace

Body* World::add_sphere(Vec3 position, float radius, float density, BodyType type) {
    auto body = std::make_unique<Body>();
    body->position = position;
    body->prev_position = position;
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
    body->prev_position = position;
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
        clamp_velocity(*body);

        body->force = {};
        body->torque = {};
    }
}

void World::integrate_positions(float dt) {
    for (auto& body : bodies_) {
        if (body->type == BodyType::Static) {
            continue;
        }

        body->prev_position = body->position;
        const Vec3 disp = body->velocity * dt;

        constexpr float kCcdThreshold = 0.35f;
        float t_hit = 1.0f;
        Vec3 n_hit{0.0f, 1.0f, 0.0f};
        bool hit = false;
        if (body->type == BodyType::Dynamic && disp.length_sq() > kCcdThreshold * kCcdThreshold) {
            for (auto& other : bodies_) {
                if (other.get() == body.get() || other->type != BodyType::Static) {
                    continue;
                }
                float t = 1.0f;
                Vec3 n;
                if (sweep(*body, *other, disp, t, n) && t < t_hit) {
                    t_hit = t;
                    n_hit = n;
                    hit = true;
                }
            }
        }

        if (hit) {
            const float adv = std::max(t_hit - 1e-3f, 0.0f);
            body->position += disp * adv;
            const float vn = dot(body->velocity, n_hit);
            if (vn < 0.0f) {
                body->velocity -= n_hit * vn;
            }
        } else {
            body->position += disp;
        }

        const Vec3 w = body->angular_velocity;
        const Quat omega_q{0.0f, w.x, w.y, w.z};
        body->orientation = (body->orientation + (omega_q * body->orientation) * (0.5f * dt)).normalized();
    }
}

void World::find_contacts() {
    contacts_.clear();
    Contact buffer[8];
    const std::size_t n = bodies_.size();
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            Body& a = *bodies_[i];
            Body& b = *bodies_[j];
            if (a.inv_mass == 0.0f && b.inv_mass == 0.0f) {
                continue;
            }
            const int count = collide(a, b, buffer, 8);
            for (int k = 0; k < count; ++k) {
                contacts_.push_back(buffer[k]);
            }
        }
    }
}

void World::solve_velocities(float dt) {
    for (int i = 0; i < velocity_iterations; ++i) {
        for (Contact& c : contacts_) {
            resolve_velocity(c, dt);
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

void World::step_once(float dt) {
    integrate_velocities(dt);
    find_contacts();
    solve_velocities(dt);
    integrate_positions(dt);
    solve_positions();
}

void World::step(float dt) {
    if (dt <= 0.0f) {
        return;
    }
    dt = std::min(dt, max_substep * static_cast<float>(max_substeps));
    while (dt > 1e-8f) {
        const float sub = std::min(dt, max_substep);
        step_once(sub);
        dt -= sub;
    }
}

}  // namespace physics
