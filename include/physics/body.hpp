#pragma once

#include "physics/mat3.hpp"
#include "physics/quat.hpp"
#include "physics/vec3.hpp"

namespace physics {

enum class BodyType { Dynamic, Static, Kinematic };

enum class ShapeType { Sphere, Box };

struct Shape {
    ShapeType type = ShapeType::Sphere;
    float radius = 0.5f;
    Vec3 half_extents{0.5f, 0.5f, 0.5f};

    static Shape sphere(float radius) { return {ShapeType::Sphere, radius, {}}; }
    static Shape box(Vec3 half) { return {ShapeType::Box, 0.0f, half}; }
};

struct Body {
    Vec3 position;
    Vec3 velocity;
    Vec3 force;

    Quat orientation = Quat::identity();
    Vec3 angular_velocity;
    Vec3 torque;

    float mass = 1.0f;
    float inv_mass = 1.0f;
    Mat3 inertia_local = Mat3::identity();
    Mat3 inv_inertia_local = Mat3::identity();

    float restitution = 0.35f;
    float friction = 0.35f;
    float linear_damping = 0.02f;
    float angular_damping = 0.02f;

    BodyType type = BodyType::Dynamic;
    Shape shape;

    void* user_data = nullptr;

    Mat3 rotation() const { return orientation.to_mat3(); }

    Mat3 inv_inertia_world() const {
        if (type != BodyType::Dynamic) {
            return Mat3::zero();
        }
        const Mat3 r = rotation();
        return r * inv_inertia_local * r.transposed();
    }

    void set_type(BodyType t) {
        type = t;
        if (t != BodyType::Dynamic) {
            inv_mass = 0.0f;
            inv_inertia_local = Mat3::zero();
            if (t == BodyType::Static) {
                velocity = {};
                angular_velocity = {};
            }
        } else {
            inv_mass = mass > 0.0f ? 1.0f / mass : 0.0f;
        }
    }

    void set_mass(float m) {
        mass = m;
        inv_mass = (type == BodyType::Dynamic && m > 0.0f) ? 1.0f / m : 0.0f;
    }

    void apply_force(Vec3 f) { force += f; }
    void apply_force_at(Vec3 f, Vec3 world_point) {
        force += f;
        torque += cross(world_point - position, f);
    }
    void apply_impulse(Vec3 impulse, Vec3 radius) {
        velocity += impulse * inv_mass;
        angular_velocity += inv_inertia_world() * cross(radius, impulse);
    }

    Vec3 world_to_local(Vec3 world) const { return rotation().transposed() * (world - position); }
    Vec3 local_to_world(Vec3 local) const { return position + rotation() * local; }

    Vec3 x_axis() const { return rotation() * Vec3{1.0f, 0.0f, 0.0f}; }
    Vec3 y_axis() const { return rotation() * Vec3{0.0f, 1.0f, 0.0f}; }
    Vec3 z_axis() const { return rotation() * Vec3{0.0f, 0.0f, 1.0f}; }
};

inline void compute_mass(Body& body, float density) {
    if (body.shape.type == ShapeType::Sphere) {
        const float r = body.shape.radius;
        const float m = (4.0f / 3.0f) * 3.14159265f * r * r * r * density;
        const float i = 0.4f * m * r * r;
        body.inertia_local = Mat3::diagonal(i, i, i);
        body.set_mass(m);
    } else {
        const Vec3 h = body.shape.half_extents;
        const float m = 8.0f * h.x * h.y * h.z * density;
        const float ixx = (m / 3.0f) * (h.y * h.y + h.z * h.z);
        const float iyy = (m / 3.0f) * (h.x * h.x + h.z * h.z);
        const float izz = (m / 3.0f) * (h.x * h.x + h.y * h.y);
        body.inertia_local = Mat3::diagonal(ixx, iyy, izz);
        body.set_mass(m);
    }

    if (body.type != BodyType::Dynamic) {
        body.inv_mass = 0.0f;
        body.inv_inertia_local = Mat3::zero();
        return;
    }

    auto inv_diag = [](float v) { return v > 1e-8f ? 1.0f / v : 0.0f; };
    body.inv_inertia_local = Mat3::diagonal(inv_diag(body.inertia_local.c0.x),
                                            inv_diag(body.inertia_local.c1.y),
                                            inv_diag(body.inertia_local.c2.z));
}

}  // namespace physics
