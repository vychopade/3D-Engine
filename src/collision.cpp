#include "physics/collision.hpp"

#include "physics/body.hpp"

#include <algorithm>
#include <cmath>

namespace physics {
namespace {

constexpr float kEpsilon = 1e-6f;
constexpr float kSkin = 0.04f;

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

Vec3 axis_of(const Body& body, int index) {
    if (index == 0) {
        return body.x_axis();
    }
    if (index == 1) {
        return body.y_axis();
    }
    return body.z_axis();
}

float half_of(const Body& body, int index) {
    if (index == 0) {
        return body.shape.half_extents.x;
    }
    if (index == 1) {
        return body.shape.half_extents.y;
    }
    return body.shape.half_extents.z;
}

void box_face(const Body& body, int axis, float sign, Vec3 verts[4]) {
    const Vec3 n = axis_of(body, axis);
    const int ia = (axis + 1) % 3;
    const int ib = (axis + 2) % 3;
    const Vec3 t = axis_of(body, ia) * half_of(body, ia);
    const Vec3 b = axis_of(body, ib) * half_of(body, ib);
    const Vec3 c = body.position + n * (sign * half_of(body, axis));
    verts[0] = c + t + b;
    verts[1] = c + t - b;
    verts[2] = c - t - b;
    verts[3] = c - t + b;
}

int clip_plane(const Vec3* in, int n_in, Vec3 nrm, float plane_d, Vec3* out) {
    int n_out = 0;
    for (int i = 0; i < n_in; ++i) {
        const Vec3 a = in[i];
        const Vec3 b = in[(i + 1) % n_in];
        const float da = dot(nrm, a) - plane_d;
        const float db = dot(nrm, b) - plane_d;
        const bool a_in = da <= 0.0f;
        const bool b_in = db <= 0.0f;
        if (a_in && b_in) {
            out[n_out++] = b;
        } else if (a_in && !b_in) {
            const float t = da / (da - db);
            out[n_out++] = a + (b - a) * t;
        } else if (!a_in && b_in) {
            const float t = da / (da - db);
            out[n_out++] = a + (b - a) * t;
            out[n_out++] = b;
        }
        if (n_out >= 8) {
            break;
        }
    }
    return n_out;
}

int collide_spheres(Body& a, Body& b, Contact* out, int max_contacts) {
    if (max_contacts <= 0) {
        return 0;
    }
    const Vec3 delta = b.position - a.position;
    const float dist_sq = delta.length_sq();
    const float radius = a.shape.radius + b.shape.radius;
    if (dist_sq >= (radius + kSkin) * (radius + kSkin)) {
        return 0;
    }

    const float dist = std::sqrt(std::max(dist_sq, 0.0f));
    out[0].a = &a;
    out[0].b = &b;
    out[0].normal = dist > kEpsilon ? delta / dist : Vec3{1.0f, 0.0f, 0.0f};
    out[0].penetration = radius - dist;
    out[0].point = a.position + out[0].normal * a.shape.radius;
    return 1;
}

void pick_exit_axis(Vec3 guide, Vec3 local, Vec3 half, Vec3& local_normal, Vec3& closest,
                    float radius, float& penetration) {
    int axis = 0;
    float mag = std::abs(guide.x);
    float guide_comp = guide.x;
    if (std::abs(guide.y) > mag) {
        axis = 1;
        mag = std::abs(guide.y);
        guide_comp = guide.y;
    }
    if (std::abs(guide.z) > mag) {
        axis = 2;
        guide_comp = guide.z;
    }

    const float comps[3] = {local.x, local.y, local.z};
    const float halves[3] = {half.x, half.y, half.z};
    const float sign = guide_comp >= 0.0f ? 1.0f : -1.0f;
    local_normal = {};
    if (axis == 0) {
        local_normal.x = -sign;
        closest.x = sign * half.x;
    } else if (axis == 1) {
        local_normal.y = -sign;
        closest.y = sign * half.y;
    } else {
        local_normal.z = -sign;
        closest.z = sign * half.z;
    }
    penetration = halves[axis] - sign * comps[axis] + radius;
}

int collide_sphere_box(Body& sphere, Body& box, Contact* out, int max_contacts) {
    if (max_contacts <= 0) {
        return 0;
    }

    const Vec3 local = box.world_to_local(sphere.position);
    const Vec3 half = box.shape.half_extents;
    Vec3 closest{std::clamp(local.x, -half.x, half.x), std::clamp(local.y, -half.y, half.y),
                 std::clamp(local.z, -half.z, half.z)};

    Vec3 local_normal;
    float penetration = 0.0f;
    const bool inside = (closest.x == local.x && closest.y == local.y && closest.z == local.z);
    const float r = sphere.shape.radius;

    if (inside) {
        const Vec3 local_prev = box.world_to_local(sphere.prev_position);
        const Vec3 prev_clamped{std::clamp(local_prev.x, -half.x, half.x),
                                std::clamp(local_prev.y, -half.y, half.y),
                                std::clamp(local_prev.z, -half.z, half.z)};
        const bool prev_outside =
            prev_clamped.x != local_prev.x || prev_clamped.y != local_prev.y || prev_clamped.z != local_prev.z;

        Vec3 guide;
        if (prev_outside) {
            guide = local_prev - prev_clamped;
        } else {
            const Vec3 local_vel = box.rotation().transposed() * (sphere.velocity - box.velocity);
            if (local_vel.length_sq() > 1e-8f) {
                guide = -local_vel;
            } else {
                const float dx = half.x - std::abs(local.x);
                const float dy = half.y - std::abs(local.y);
                const float dz = half.z - std::abs(local.z);
                if (dx <= dy && dx <= dz) {
                    guide = {local.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f};
                } else if (dy <= dz) {
                    guide = {0.0f, local.y >= 0.0f ? 1.0f : -1.0f, 0.0f};
                } else {
                    guide = {0.0f, 0.0f, local.z >= 0.0f ? 1.0f : -1.0f};
                }
            }
        }
        pick_exit_axis(guide, local, half, local_normal, closest, r, penetration);
    } else {
        const Vec3 delta = local - closest;
        const float dist_sq = delta.length_sq();
        if (dist_sq >= (r + kSkin) * (r + kSkin)) {
            return 0;
        }
        const float dist = std::sqrt(std::max(dist_sq, 0.0f));
        local_normal = dist > kEpsilon ? -(delta / dist) : Vec3{0.0f, -1.0f, 0.0f};
        penetration = r - dist;
    }

    out[0].a = &sphere;
    out[0].b = &box;
    out[0].normal = box.rotation() * local_normal;
    const float nlen = out[0].normal.length();
    if (nlen > kEpsilon) {
        out[0].normal = out[0].normal / nlen;
    }
    out[0].penetration = penetration;
    out[0].point = box.local_to_world(closest);
    return 1;
}

int generate_face_contacts(Body& a, Body& b, bool ref_is_a, int ref_axis, Vec3 sat_normal,
                           Contact* out, int max_contacts) {
    Body& ref = ref_is_a ? a : b;
    Body& inc = ref_is_a ? b : a;
    const Vec3 n = ref_is_a ? sat_normal : -sat_normal;
    const float sign = dot(axis_of(ref, ref_axis), n) >= 0.0f ? 1.0f : -1.0f;

    int inc_axis = 0;
    float best = 1.0e9f;
    for (int i = 0; i < 3; ++i) {
        const float d = dot(axis_of(inc, i), n);
        if (d < best) {
            best = d;
            inc_axis = i;
        }
    }
    const float inc_sign = dot(axis_of(inc, inc_axis), n) > 0.0f ? -1.0f : 1.0f;

    Vec3 incident[4];
    box_face(inc, inc_axis, inc_sign, incident);

    const int ta = (ref_axis + 1) % 3;
    const int tb = (ref_axis + 2) % 3;
    const Vec3 t = axis_of(ref, ta);
    const Vec3 bit = axis_of(ref, tb);
    const Vec3 c = ref.position + axis_of(ref, ref_axis) * (sign * half_of(ref, ref_axis));
    const float ht = half_of(ref, ta);
    const float hb = half_of(ref, tb);

    Vec3 buf_a[8];
    Vec3 buf_b[8];
    int n_pts = 4;
    for (int i = 0; i < 4; ++i) {
        buf_a[i] = incident[i];
    }
    n_pts = clip_plane(buf_a, n_pts, t, dot(t, c) + ht, buf_b);
    n_pts = clip_plane(buf_b, n_pts, -t, dot(-t, c) + ht, buf_a);
    n_pts = clip_plane(buf_a, n_pts, bit, dot(bit, c) + hb, buf_b);
    n_pts = clip_plane(buf_b, n_pts, -bit, dot(-bit, c) + hb, buf_a);

    const float ref_extent = half_of(ref, ref_axis);
    int written = 0;
    for (int i = 0; i < n_pts && written < max_contacts; ++i) {
        const float plane_sep = dot(n, buf_a[i] - ref.position) - ref_extent;
        const float pen = -plane_sep;
        if (pen < -kSkin) {
            continue;
        }
        Contact& ctc = out[written++];
        ctc.a = &a;
        ctc.b = &b;
        ctc.normal = sat_normal;
        ctc.point = buf_a[i];
        ctc.penetration = pen;
    }
    return written;
}

int collide_boxes(Body& a, Body& b, Contact* out, int max_contacts) {
    if (max_contacts <= 0) {
        return 0;
    }

    const BoxGeometry ga = make_box(a);
    const BoxGeometry gb = make_box(b);

    Vec3 best_axis;
    float min_overlap = 1e9f;
    int source = 0;  // 0 = A face, 1 = B face, 2 = edge
    int ref_axis = 0;

    auto test_axis = [&](Vec3 axis, int src, int axis_index) {
        if (axis.length_sq() < 1e-8f) {
            return true;
        }
        axis = axis.normalized();
        float min_a, max_a, min_b, max_b;
        project(ga, axis, min_a, max_a);
        project(gb, axis, min_b, max_b);
        const float overlap = std::min(max_a, max_b) - std::max(min_a, min_b);
        if (overlap <= -kSkin) {
            return false;
        }
        if (overlap < min_overlap) {
            min_overlap = overlap;
            best_axis = axis;
            source = src;
            ref_axis = axis_index;
        }
        return true;
    };

    for (int i = 0; i < 3; ++i) {
        if (!test_axis(ga.axes[i], 0, i)) {
            return 0;
        }
    }
    for (int i = 0; i < 3; ++i) {
        if (!test_axis(gb.axes[i], 1, i)) {
            return 0;
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!test_axis(cross(ga.axes[i], gb.axes[j]), 2, 0)) {
                return 0;
            }
        }
    }

    if (dot(b.position - a.position, best_axis) < 0.0f) {
        best_axis = -best_axis;
    }

    if (source < 2) {
        const int n = generate_face_contacts(a, b, source == 0, ref_axis, best_axis, out, max_contacts);
        if (n > 0) {
            return n;
        }
    }

    out[0].a = &a;
    out[0].b = &b;
    out[0].normal = best_axis;
    out[0].penetration = min_overlap;
    out[0].point = (support(ga, best_axis) + support(gb, -best_axis)) * 0.5f;
    return 1;
}

float bounding_radius(const Body& body) {
    if (body.shape.type == ShapeType::Sphere) {
        return body.shape.radius;
    }
    return body.shape.half_extents.length();
}

bool ray_expanded_aabb(Vec3 origin, Vec3 disp, Vec3 half, float& t_enter, Vec3& local_n) {
    float tmin = 0.0f;
    float tmax = 1.0f;
    int enter_axis = -1;
    float enter_sign = 0.0f;
    const float o[3] = {origin.x, origin.y, origin.z};
    const float d[3] = {disp.x, disp.y, disp.z};
    const float h[3] = {half.x, half.y, half.z};

    for (int i = 0; i < 3; ++i) {
        if (std::abs(d[i]) < 1e-12f) {
            if (o[i] < -h[i] || o[i] > h[i]) {
                return false;
            }
            continue;
        }
        float t1 = (-h[i] - o[i]) / d[i];
        float t2 = (h[i] - o[i]) / d[i];
        float s1 = -1.0f;
        float s2 = 1.0f;
        if (t1 > t2) {
            std::swap(t1, t2);
            std::swap(s1, s2);
        }
        if (t1 > tmin) {
            tmin = t1;
            enter_axis = i;
            enter_sign = s1;
        }
        tmax = std::min(tmax, t2);
        if (tmin > tmax) {
            return false;
        }
    }

    if (tmax < 0.0f || tmin > 1.0f) {
        return false;
    }

    if (tmin <= 0.0f) {
        t_enter = 0.0f;
        if (disp.length_sq() > kEpsilon) {
            local_n = -disp.normalized();
        } else {
            const float ax = h[0] - std::abs(o[0]);
            const float ay = h[1] - std::abs(o[1]);
            const float az = h[2] - std::abs(o[2]);
            if (ax <= ay && ax <= az) {
                local_n = {o[0] >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f};
            } else if (ay <= az) {
                local_n = {0.0f, o[1] >= 0.0f ? 1.0f : -1.0f, 0.0f};
            } else {
                local_n = {0.0f, 0.0f, o[2] >= 0.0f ? 1.0f : -1.0f};
            }
        }
        return true;
    }

    t_enter = tmin;
    local_n = {};
    if (enter_axis == 0) {
        local_n.x = enter_sign;
    } else if (enter_axis == 1) {
        local_n.y = enter_sign;
    } else if (enter_axis == 2) {
        local_n.z = enter_sign;
    } else {
        local_n.y = 1.0f;
    }
    return true;
}

}  // namespace

int collide(Body& a, Body& b, Contact* out, int max_contacts) {
    if (out == nullptr || max_contacts <= 0) {
        return 0;
    }

    const bool a_sphere = a.shape.type == ShapeType::Sphere;
    const bool b_sphere = b.shape.type == ShapeType::Sphere;

    if (a_sphere && b_sphere) {
        return collide_spheres(a, b, out, max_contacts);
    }
    if (!a_sphere && !b_sphere) {
        return collide_boxes(a, b, out, max_contacts);
    }
    if (a_sphere) {
        return collide_sphere_box(a, b, out, max_contacts);
    }

    const int n = collide_sphere_box(b, a, out, max_contacts);
    for (int i = 0; i < n; ++i) {
        out[i].a = &a;
        out[i].b = &b;
        out[i].normal = -out[i].normal;
    }
    return n;
}

bool collide(Body& a, Body& b, Contact& out) { return collide(a, b, &out, 1) > 0; }

bool sweep(const Body& moving, const Body& obstacle, Vec3 displacement, float& t, Vec3& normal) {
    if (displacement.length_sq() < 1e-12f) {
        return false;
    }

    const float radius = bounding_radius(moving);

    if (obstacle.shape.type == ShapeType::Sphere) {
        const Vec3 oc = moving.position - obstacle.position;
        const float r = radius + obstacle.shape.radius;
        const float a = dot(displacement, displacement);
        const float b = 2.0f * dot(oc, displacement);
        const float c = dot(oc, oc) - r * r;
        const float disc = b * b - 4.0f * a * c;
        if (a < 1e-12f) {
            return false;
        }
        if (c <= 0.0f) {
            t = 0.0f;
            Vec3 n = oc;
            if (n.length_sq() < kEpsilon) {
                n = -displacement;
            }
            normal = n.normalized();
            return true;
        }
        if (disc < 0.0f) {
            return false;
        }
        const float sqrt_d = std::sqrt(disc);
        float t0 = (-b - sqrt_d) / (2.0f * a);
        if (t0 < 0.0f || t0 > 1.0f) {
            t0 = (-b + sqrt_d) / (2.0f * a);
        }
        if (t0 < 0.0f || t0 > 1.0f) {
            return false;
        }
        t = t0;
        const Vec3 hit = moving.position + displacement * t;
        Vec3 n = hit - obstacle.position;
        if (n.length_sq() < kEpsilon) {
            n = -displacement;
        }
        normal = n.normalized();
        return true;
    }

    const Vec3 local_o = obstacle.world_to_local(moving.position);
    const Vec3 local_d = obstacle.rotation().transposed() * displacement;
    const Vec3 expanded = obstacle.shape.half_extents + Vec3{radius, radius, radius};
    Vec3 local_n;
    if (!ray_expanded_aabb(local_o, local_d, expanded, t, local_n)) {
        return false;
    }
    normal = obstacle.rotation() * local_n;
    const float nlen = normal.length();
    normal = nlen > kEpsilon ? normal / nlen : Vec3{0.0f, 1.0f, 0.0f};
    return true;
}

}  // namespace physics
