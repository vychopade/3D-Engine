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

Spring* World::add_spring(Body* a, Body* b, Vec3 local_a, Vec3 local_b, float rest_length,
                          float stiffness, float damping, bool bungee) {
    if (a == nullptr) {
        return nullptr;
    }
    auto spring = std::make_unique<Spring>();
    spring->a = a;
    spring->b = b;
    spring->local_a = local_a;
    spring->local_b = local_b;
    spring->rest_length = std::max(rest_length, 0.0f);
    spring->stiffness = std::max(stiffness, 0.0f);
    spring->damping = std::max(damping, 0.0f);
    spring->bungee = bungee;
    Spring* ptr = spring.get();
    springs_.push_back(std::move(spring));
    return ptr;
}

DistanceJoint* World::add_rod(Body* a, Body* b, Vec3 local_a, Vec3 local_b, float length, bool pendulum) {
    if (a == nullptr) {
        return nullptr;
    }
    auto rod = std::make_unique<DistanceJoint>();
    rod->a = a;
    rod->b = b;
    rod->local_a = local_a;
    rod->local_b = local_b;
    rod->length = std::max(length, 0.05f);
    rod->pendulum = pendulum;
    DistanceJoint* ptr = rod.get();
    rods_.push_back(std::move(rod));
    return ptr;
}

void World::clear() {
    bodies_.clear();
    springs_.clear();
    rods_.clear();
    contacts_.clear();
}

bool World::destroy_body(Body* body) {
    if (body == nullptr) {
        return false;
    }
    const auto it = std::find_if(bodies_.begin(), bodies_.end(),
                                 [body](const std::unique_ptr<Body>& owned) { return owned.get() == body; });
    if (it == bodies_.end()) {
        return false;
    }
    bodies_.erase(it);
    springs_.erase(std::remove_if(springs_.begin(), springs_.end(),
                                  [body](const std::unique_ptr<Spring>& s) {
                                      return s->a == body || s->b == body;
                                  }),
                   springs_.end());
    rods_.erase(std::remove_if(rods_.begin(), rods_.end(),
                               [body](const std::unique_ptr<DistanceJoint>& j) {
                                   return j->a == body || j->b == body;
                               }),
                rods_.end());
    contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
                                   [body](const Contact& c) { return c.a == body || c.b == body; }),
                    contacts_.end());
    return true;
}

bool World::destroy_spring(Spring* spring) {
    if (spring == nullptr) {
        return false;
    }
    const auto it = std::find_if(springs_.begin(), springs_.end(),
                                 [spring](const std::unique_ptr<Spring>& owned) { return owned.get() == spring; });
    if (it == springs_.end()) {
        return false;
    }
    springs_.erase(it);
    return true;
}

bool World::destroy_rod(DistanceJoint* rod) {
    if (rod == nullptr) {
        return false;
    }
    const auto it = std::find_if(rods_.begin(), rods_.end(),
                                 [rod](const std::unique_ptr<DistanceJoint>& owned) { return owned.get() == rod; });
    if (it == rods_.end()) {
        return false;
    }
    rods_.erase(it);
    return true;
}

void World::solve_rods_velocity(float dt) {
    for (auto& owned : rods_) {
        DistanceJoint& j = *owned;
        if (j.a == nullptr) {
            continue;
        }
        const Vec3 pa = j.point_a();
        const Vec3 pb = j.point_b();
        Vec3 delta = pb - pa;
        const float length = delta.length();
        if (length < 1e-6f) {
            continue;
        }
        const Vec3 n = delta / length;
        const Vec3 ra = pa - j.a->position;
        Vec3 rb{};
        Vec3 va = j.a->velocity + cross(j.a->angular_velocity, ra);
        Vec3 vb{};
        float inv_mass = j.a->inv_mass + angular_effective_mass(*j.a, ra, n);
        if (j.b != nullptr) {
            rb = pb - j.b->position;
            vb = j.b->velocity + cross(j.b->angular_velocity, rb);
            inv_mass += j.b->inv_mass + angular_effective_mass(*j.b, rb, n);
        }
        if (inv_mass <= 1e-8f) {
            continue;
        }
        const float cdot = dot(vb - va, n);
        const float c = length - j.length;
        const float bias = std::clamp(0.08f * c / std::max(dt, 1e-5f), -1.5f, 1.5f);
        const float lambda = std::clamp(-(cdot + bias) / (inv_mass + 0.08f), -25.0f, 25.0f);
        const Vec3 impulse = n * lambda;
        j.a->apply_impulse(-impulse, ra);
        if (j.b != nullptr) {
            j.b->apply_impulse(impulse, rb);
        }
    }
}

void World::solve_rods_position() {
    for (auto& owned : rods_) {
        DistanceJoint& j = *owned;
        if (j.a == nullptr) {
            continue;
        }
        const Vec3 pa = j.point_a();
        const Vec3 pb = j.point_b();
        Vec3 delta = pb - pa;
        const float length = delta.length();
        if (length < 1e-6f) {
            continue;
        }
        const Vec3 n = delta / length;
        const float c = length - j.length;
        const float inv_mass = j.a->inv_mass + (j.b != nullptr ? j.b->inv_mass : 0.0f);
        if (inv_mass <= 1e-8f) {
            continue;
        }
        // n points from a toward b. Positive length error means pull a along +n.
        const Vec3 correction = n * (c * 0.85f);
        j.a->position += correction * (j.a->inv_mass / inv_mass);
        if (j.b != nullptr) {
            j.b->position -= correction * (j.b->inv_mass / inv_mass);
        }
    }
}

void World::apply_springs() {
    constexpr float kMaxForce = 2500.0f;
    for (auto& owned : springs_) {
        Spring& s = *owned;
        if (s.a == nullptr) {
            continue;
        }
        const Vec3 pa = s.point_a();
        const Vec3 pb = s.point_b();
        Vec3 delta = pb - pa;
        const float length = delta.length();
        if (length < 1e-6f) {
            continue;
        }
        const Vec3 n = delta / length;
        const float extension = length - s.rest_length;
        if (s.bungee && extension < 0.0f) {
            continue;
        }

        const Vec3 ra = pa - s.a->position;
        Vec3 va = s.a->velocity + cross(s.a->angular_velocity, ra);
        Vec3 vb{};
        if (s.b != nullptr) {
            const Vec3 rb = pb - s.b->position;
            vb = s.b->velocity + cross(s.b->angular_velocity, rb);
        }
        const float closing = dot(vb - va, n);
        float mag = s.stiffness * extension + s.damping * closing;
        mag = std::clamp(mag, -kMaxForce, kMaxForce);
        const Vec3 force = n * mag;
        s.a->apply_force_at(force, pa);
        if (s.b != nullptr) {
            s.b->apply_force_at(-force, pb);
        }
    }
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
    for (int i = 0; i < velocity_iterations; ++i) {
        solve_rods_velocity(dt);
    }
}

void World::solve_positions() {
    for (int i = 0; i < position_iterations; ++i) {
        find_contacts();
        for (Contact& c : contacts_) {
            resolve_position(c);
        }
        solve_rods_position();
    }
}

void World::step_once(float dt) {
    apply_springs();
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
