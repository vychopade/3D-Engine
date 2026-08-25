#include "physics/collision.hpp"

#include "physics/body.hpp"

#include <algorithm>
#include <cmath>

namespace physics {
namespace {

constexpr float kEpsilon = 1e-6f;

struct BoxGeometry {
    Vec3 vertices[8];
    Vec3 axes[3];
};

BoxGeometry make_box(const Body& body) {
    const Vec3 hx = body.x_axis() * body.shape.half_extents.x;
    const Vec3 hy = body.y_axis() * body.shape.half_extents.y;
    const Vec3 hz = body.z_axis() * body.shape.half_extents.z;
    BoxGeometry g;
    int i = 0;
    for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
            for (int sz : {-1, 1}) {
                g.vertices[i++] = body.position + hx * static_cast<float>(sx) +
                                  hy * static_cast<float>(sy) + hz * static_cast<float>(sz);
            }
        }
    }
    g.axes[0] = body.x_axis();
    g.axes[1] = body.y_axis();
    g.axes[2] = body.z_axis();
    return g;
}

void project(const BoxGeometry& box, Vec3 axis, float& min_p, float& max_p) {
    min_p = max_p = dot(box.vertices[0], axis);
    for (int i = 1; i < 8; ++i) {
        const float p = dot(box.vertices[i], axis);
        min_p = std::min(min_p, p);
        max_p = std::max(max_p, p);
    }
}

Vec3 support(const BoxGeometry& box, Vec3 direction) {
    Vec3 best = box.vertices[0];
    float best_dot = dot(best, direction);
    for (int i = 1; i < 8; ++i) {
        const float d = dot(box.vertices[i], direction);
        if (d > best_dot) {
            best_dot = d;
            best = box.vertices[i];
        }
    }
    return best;
}

bool collide_spheres(Body& a, Body& b, Contact& out) {
    const Vec3 delta = b.position - a.position;
    const float dist_sq = delta.length_sq();
    const float radius = a.shape.radius + b.shape.radius;
    if (dist_sq >= radius * radius) {
        return false;
    }

    const float dist = std::sqrt(dist_sq);
    out.normal = dist > kEpsilon ? delta / dist : Vec3{1.0f, 0.0f, 0.0f};
    out.penetration = radius - dist;
    out.point = a.position + out.normal * a.shape.radius;
    return true;
}

bool collide_boxes(Body& a, Body& b, Contact& out) {
    const BoxGeometry ga = make_box(a);
    const BoxGeometry gb = make_box(b);

    Vec3 best_axis;
    float min_overlap = 1e9f;

    auto test_axis = [&](Vec3 axis) {
        if (axis.length_sq() < 1e-8f) {
            return true;
        }
        axis = axis.normalized();
        float min_a, max_a, min_b, max_b;
        project(ga, axis, min_a, max_a);
        project(gb, axis, min_b, max_b);
        const float overlap = std::min(max_a, max_b) - std::max(min_a, min_b);
        if (overlap <= 0.0f) {
            return false;
        }
        if (overlap < min_overlap) {
            min_overlap = overlap;
            best_axis = axis;
        }
        return true;
    };

    for (const Vec3& axis : ga.axes) {
        if (!test_axis(axis)) {
            return false;
        }
    }
    for (const Vec3& axis : gb.axes) {
        if (!test_axis(axis)) {
            return false;
        }
    }
    for (const Vec3& aa : ga.axes) {
        for (const Vec3& bb : gb.axes) {
            if (!test_axis(cross(aa, bb))) {
                return false;
            }
        }
    }

    if (dot(b.position - a.position, best_axis) < 0.0f) {
        best_axis = -best_axis;
    }

    out.normal = best_axis;
    out.penetration = min_overlap;
    out.point = (support(ga, best_axis) + support(gb, -best_axis)) * 0.5f;
    return true;
}

bool collide_sphere_box(Body& sphere, Body& box, Contact& out) {
    const Vec3 local = box.world_to_local(sphere.position);
    const Vec3 half = box.shape.half_extents;
    Vec3 closest{std::clamp(local.x, -half.x, half.x), std::clamp(local.y, -half.y, half.y),
                 std::clamp(local.z, -half.z, half.z)};

    Vec3 local_normal;
    float penetration = 0.0f;
    const bool inside = (closest.x == local.x && closest.y == local.y && closest.z == local.z);

    if (inside) {
        const float dx = half.x - std::abs(local.x);
        const float dy = half.y - std::abs(local.y);
        const float dz = half.z - std::abs(local.z);
        if (dx <= dy && dx <= dz) {
            local_normal = {local.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f};
            penetration = dx + sphere.shape.radius;
            closest.x = local.x >= 0.0f ? half.x : -half.x;
        } else if (dy <= dz) {
            local_normal = {0.0f, local.y >= 0.0f ? 1.0f : -1.0f, 0.0f};
            penetration = dy + sphere.shape.radius;
            closest.y = local.y >= 0.0f ? half.y : -half.y;
        } else {
            local_normal = {0.0f, 0.0f, local.z >= 0.0f ? 1.0f : -1.0f};
            penetration = dz + sphere.shape.radius;
            closest.z = local.z >= 0.0f ? half.z : -half.z;
        }
        local_normal = -local_normal;
    } else {
        const Vec3 delta = local - closest;
        const float dist_sq = delta.length_sq();
        const float r = sphere.shape.radius;
        if (dist_sq >= r * r) {
            return false;
        }
        const float dist = std::sqrt(dist_sq);
        local_normal = -(delta / dist);
        penetration = r - dist;
    }

    out.normal = box.rotation() * local_normal;
    out.penetration = penetration;
    out.point = box.local_to_world(closest);
    return true;
}

}  // namespace

bool collide(Body& a, Body& b, Contact& out) {
    out.a = &a;
    out.b = &b;

    const bool a_sphere = a.shape.type == ShapeType::Sphere;
    const bool b_sphere = b.shape.type == ShapeType::Sphere;

    if (a_sphere && b_sphere) {
        return collide_spheres(a, b, out);
    }
    if (!a_sphere && !b_sphere) {
        return collide_boxes(a, b, out);
    }
    if (a_sphere) {
        return collide_sphere_box(a, b, out);
    }

    const bool hit = collide_sphere_box(b, a, out);
    if (hit) {
        out.a = &a;
        out.b = &b;
        out.normal = -out.normal;
    }
    return hit;
}

}  // namespace physics
