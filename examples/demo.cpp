#include "physics/physics.hpp"

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <vector>

using physics::Body;
using physics::BodyType;
using physics::Quat;
using physics::ShapeType;
using physics::Vec3;
using physics::World;

namespace {

const Color kPalette[] = {
    {239, 83, 80, 255},  {66, 165, 245, 255}, {102, 187, 106, 255}, {255, 167, 38, 255},
    {171, 71, 188, 255}, {38, 198, 218, 255}, {255, 202, 40, 255},  {239, 154, 154, 255},
};

constexpr float kMouseSens = 0.0025f;
constexpr float kFlySpeed = 12.0f;
constexpr float kWalkSpeed = 7.0f;
constexpr float kJumpSpeed = 6.5f;
constexpr float kPlaceDistance = 5.0f;

Vec3 to_vec(Vector3 v) { return {v.x, v.y, v.z}; }
Vector3 to_rl(Vec3 v) { return {v.x, v.y, v.z}; }

struct RayHit {
    Body* body = nullptr;
    float t = 0.0f;
    Vec3 point;
};

bool ray_aabb(Vec3 origin, Vec3 dir, Vec3 half, float& t_hit) {
    float tmin = 0.0f;
    float tmax = 1.0e6f;
    const float orig[3] = {origin.x, origin.y, origin.z};
    const float d[3] = {dir.x, dir.y, dir.z};
    const float h[3] = {half.x, half.y, half.z};

    for (int i = 0; i < 3; ++i) {
        if (std::abs(d[i]) < 1e-8f) {
            if (orig[i] < -h[i] || orig[i] > h[i]) {
                return false;
            }
            continue;
        }
        float t1 = (-h[i] - orig[i]) / d[i];
        float t2 = (h[i] - orig[i]) / d[i];
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) {
            return false;
        }
    }

    t_hit = tmin >= 0.0f ? tmin : tmax;
    return t_hit >= 0.0f;
}

bool ray_sphere(Vec3 origin, Vec3 dir, Vec3 center, float radius, float& t_hit) {
    const Vec3 oc = origin - center;
    const float a = physics::dot(dir, dir);
    const float b = 2.0f * physics::dot(oc, dir);
    const float c = physics::dot(oc, oc) - radius * radius;
    const float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f || a < 1e-12f) {
        return false;
    }
    const float sqrt_d = std::sqrt(disc);
    const float t0 = (-b - sqrt_d) / (2.0f * a);
    const float t1 = (-b + sqrt_d) / (2.0f * a);
    t_hit = t0 >= 0.05f ? t0 : t1;
    return t_hit >= 0.05f;
}

std::optional<RayHit> raycast(World& world, Vec3 origin, Vec3 dir, const Body* ignore) {
    std::optional<RayHit> best;
    float best_t = std::numeric_limits<float>::max();

    for (auto& owned : world.bodies()) {
        Body* body = owned.get();
        if (body == ignore) {
            continue;
        }

        float t = 0.0f;
        bool hit = false;
        if (body->shape.type == ShapeType::Sphere) {
            hit = ray_sphere(origin, dir, body->position, body->shape.radius, t);
        } else {
            const Vec3 local_o = body->world_to_local(origin);
            const Vec3 local_d = body->rotation().transposed() * dir;
            hit = ray_aabb(local_o, local_d, body->shape.half_extents, t);
        }

        if (hit && t > 0.05f && t < best_t) {
            best_t = t;
            best = RayHit{body, t, origin + dir * t};
        }
    }
    return best;
}

void populate(World& world) {
    world.clear();

    world.add_box({0.0f, -0.5f, 0.0f}, {12.0f, 0.5f, 12.0f}, 1.0f, BodyType::Static);

    Body* ramp = world.add_box({4.0f, 1.6f, 0.0f}, {3.5f, 0.18f, 2.2f}, 1.0f, BodyType::Static);
    ramp->orientation = Quat::from_axis_angle({0.0f, 0.0f, 1.0f}, -0.32f);

    world.add_box({-8.5f, 2.0f, 0.0f}, {0.25f, 2.0f, 6.0f}, 1.0f, BodyType::Static);
    world.add_box({8.5f, 2.0f, 0.0f}, {0.25f, 2.0f, 6.0f}, 1.0f, BodyType::Static);
    world.add_box({0.0f, 2.0f, -8.5f}, {8.5f, 2.0f, 0.25f}, 1.0f, BodyType::Static);

    for (int i = 0; i < 8; ++i) {
        const float x = -3.0f + static_cast<float>(i % 4) * 1.4f;
        const float z = -1.5f + static_cast<float>(i / 4) * 1.6f;
        Body* ball = world.add_sphere({x, 7.0f + 0.4f * static_cast<float>(i), z},
                                      0.38f + 0.04f * static_cast<float>(i % 3), 1.0f);
        ball->restitution = 0.4f;
        ball->friction = 0.3f;
    }

    for (int i = 0; i < 6; ++i) {
        Body* crate = world.add_box({-2.0f + static_cast<float>(i % 3) * 1.3f, 4.5f + static_cast<float>(i / 3) * 1.2f,
                                     2.5f},
                                    {0.45f, 0.45f, 0.45f}, 1.1f);
        crate->restitution = 0.15f;
        crate->friction = 0.5f;
        crate->orientation = Quat::from_axis_angle({0.0f, 1.0f, 0.2f}, 0.2f * static_cast<float>(i));
    }

    Body* plank = world.add_box({-1.0f, 8.5f, 0.5f}, {1.6f, 0.16f, 0.45f}, 0.7f);
    plank->restitution = 0.2f;
    plank->orientation = Quat::from_axis_angle({0.0f, 0.0f, 1.0f}, 0.45f);
}

Color color_for(std::size_t index, const Body& body, bool hovered) {
    if (body.type == BodyType::Static) {
        return hovered ? Color{140, 150, 170, 255} : Color{90, 96, 110, 255};
    }
    Color c = kPalette[index % (sizeof(kPalette) / sizeof(kPalette[0]))];
    if (hovered) {
        c.r = static_cast<unsigned char>(std::min(255, c.r + 50));
        c.g = static_cast<unsigned char>(std::min(255, c.g + 50));
        c.b = static_cast<unsigned char>(std::min(255, c.b + 50));
    }
    return c;
}

void draw_body(const Body& body, Color color) {
    const Vector3 pos = to_rl(body.position);
    if (body.shape.type == ShapeType::Sphere) {
        DrawSphere(pos, body.shape.radius, color);
        DrawSphereWires(pos, body.shape.radius, 12, 12, ColorAlpha(BLACK, 0.35f));
        return;
    }

    const Quaternion q{body.orientation.x, body.orientation.y, body.orientation.z, body.orientation.w};
    rlPushMatrix();
    rlTranslatef(pos.x, pos.y, pos.z);
    rlMultMatrixf(MatrixToFloat(QuaternionToMatrix(q)));
    const Vec3 h = body.shape.half_extents;
    DrawCube({0.0f, 0.0f, 0.0f}, h.x * 2.0f, h.y * 2.0f, h.z * 2.0f, color);
    DrawCubeWires({0.0f, 0.0f, 0.0f}, h.x * 2.0f, h.y * 2.0f, h.z * 2.0f, ColorAlpha(BLACK, 0.45f));
    rlPopMatrix();
}

struct Controller {
    float yaw = 0.7f;
    float pitch = -0.25f;
    Vector3 eye{8.0f, 3.5f, 8.0f};
    Body* possessed = nullptr;
    Body* last_placed = nullptr;
    float saved_restitution = 0.0f;
    float saved_friction = 0.0f;
    float saved_ang_damp = 0.0f;

    Vec3 forward() const {
        const float cp = std::cos(pitch);
        return {std::sin(yaw) * cp, std::sin(pitch), std::cos(yaw) * cp};
    }

    Vec3 flat_forward() const {
        Vec3 f{std::sin(yaw), 0.0f, std::cos(yaw)};
        return f.normalized();
    }

    Vec3 right() const {
        const Vec3 f = forward();
        return physics::cross(f, {0.0f, 1.0f, 0.0f}).normalized();
    }

    Vec3 flat_right() const {
        return physics::cross(flat_forward(), {0.0f, 1.0f, 0.0f}).normalized();
    }

    void look() {
        const Vector2 delta = GetMouseDelta();
        yaw -= delta.x * kMouseSens;
        pitch -= delta.y * kMouseSens;
        pitch = std::clamp(pitch, -1.45f, 1.45f);
    }

    void to_camera(Camera3D& camera) const {
        const Vec3 f = forward();
        camera.position = eye;
        camera.target = {eye.x + f.x, eye.y + f.y, eye.z + f.z};
        camera.up = {0.0f, 1.0f, 0.0f};
    }

    void release() {
        if (possessed == nullptr) {
            return;
        }
        possessed->restitution = saved_restitution;
        possessed->friction = saved_friction;
        possessed->angular_damping = saved_ang_damp;
        possessed->angular_velocity = {};
        eye = to_rl(possessed->position + Vec3{0.0f, 0.6f, 0.0f});
        possessed = nullptr;
    }

    void possess(Body* body) {
        if (body == nullptr || body->type != BodyType::Dynamic) {
            return;
        }
        if (possessed == body) {
            return;
        }
        release();
        possessed = body;
        saved_restitution = body->restitution;
        saved_friction = body->friction;
        saved_ang_damp = body->angular_damping;
        body->restitution = 0.05f;
        body->friction = 0.9f;
        body->angular_damping = 8.0f;
        body->angular_velocity = {};
    }
};

Vec3 placement_point(const Controller& ctrl, const std::optional<RayHit>& look_hit) {
    const Vec3 origin = to_vec(ctrl.eye);
    const Vec3 dir = ctrl.forward();
    if (look_hit && look_hit->body != ctrl.possessed) {
        return look_hit->point - dir * 0.55f;
    }
    return origin + dir * kPlaceDistance;
}

Body* spawn_sphere(World& world, Vec3 at) {
    Body* ball = world.add_sphere(at, 0.45f, 1.0f);
    ball->restitution = 0.45f;
    ball->friction = 0.3f;
    return ball;
}

Body* spawn_box(World& world, Vec3 at) {
    Body* box = world.add_box(at, {0.4f, 0.4f, 0.4f}, 1.0f);
    box->restitution = 0.18f;
    box->friction = 0.5f;
    return box;
}

bool is_grounded(const World& world, const Body& body) {
    for (const auto& c : world.contacts()) {
        if (c.a == &body && c.normal.y < -0.45f) {
            return true;
        }
        if (c.b == &body && c.normal.y > 0.45f) {
            return true;
        }
    }
    return false;
}

void drive_possessed(Controller& ctrl, World& world, float /*dt*/) {
    Body* body = ctrl.possessed;
    if (body == nullptr) {
        return;
    }

    Vec3 wish{};
    if (IsKeyDown(KEY_W)) {
        wish += ctrl.flat_forward();
    }
    if (IsKeyDown(KEY_S)) {
        wish -= ctrl.flat_forward();
    }
    if (IsKeyDown(KEY_D)) {
        wish += ctrl.flat_right();
    }
    if (IsKeyDown(KEY_A)) {
        wish -= ctrl.flat_right();
    }
    if (wish.length_sq() > 1e-6f) {
        wish = wish.normalized() * kWalkSpeed;
    }

    body->velocity.x = wish.x;
    body->velocity.z = wish.z;
    body->angular_velocity = {};
    body->orientation = Quat::from_axis_angle({0.0f, 1.0f, 0.0f}, ctrl.yaw);

    if (IsKeyPressed(KEY_SPACE) && is_grounded(world, *body)) {
        body->velocity.y = kJumpSpeed;
    }

    ctrl.eye = to_rl(body->position);
}

void fly_spectator(Controller& ctrl, float dt) {
    Vec3 wish{};
    if (IsKeyDown(KEY_W)) {
        wish += ctrl.forward();
    }
    if (IsKeyDown(KEY_S)) {
        wish -= ctrl.forward();
    }
    if (IsKeyDown(KEY_D)) {
        wish += ctrl.right();
    }
    if (IsKeyDown(KEY_A)) {
        wish -= ctrl.right();
    }
    if (IsKeyDown(KEY_SPACE)) {
        wish += Vec3{0.0f, 1.0f, 0.0f};
    }
    if (IsKeyDown(KEY_LEFT_CONTROL)) {
        wish -= Vec3{0.0f, 1.0f, 0.0f};
    }

    const float speed = IsKeyDown(KEY_LEFT_SHIFT) ? kFlySpeed * 2.0f : kFlySpeed;
    if (wish.length_sq() > 1e-6f) {
        wish = wish.normalized();
        ctrl.eye.x += wish.x * speed * dt;
        ctrl.eye.y += wish.y * speed * dt;
        ctrl.eye.z += wish.z * speed * dt;
    }
}

}  // namespace

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI);
    InitWindow(1280, 720, "Physics Engine 3D");
    SetTargetFPS(60);
    SetExitKey(KEY_NULL);
    DisableCursor();

    Camera3D camera{};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 70.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    World world;
    populate(world);

    Controller ctrl;

    while (!WindowShouldClose()) {
        const float dt = std::min(GetFrameTime(), 1.0f / 30.0f);
        ctrl.look();

        if (IsKeyPressed(KEY_Q)) {
            break;
        }
        if (IsKeyPressed(KEY_ESCAPE)) {
            if (ctrl.possessed != nullptr) {
                ctrl.release();
            }
        }
        if (IsKeyPressed(KEY_R)) {
            ctrl.release();
            ctrl.last_placed = nullptr;
            populate(world);
        }

        const Vec3 origin = to_vec(ctrl.eye);
        const Vec3 dir = ctrl.forward();
        const auto look_hit = raycast(world, origin, dir, ctrl.possessed);

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsKeyPressed(KEY_ONE)) {
            ctrl.last_placed = spawn_sphere(world, placement_point(ctrl, look_hit));
        }
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) || IsKeyPressed(KEY_TWO)) {
            ctrl.last_placed = spawn_box(world, placement_point(ctrl, look_hit));
        }

        if (IsKeyPressed(KEY_F)) {
            Body* target = nullptr;
            if (look_hit && look_hit->body->type == BodyType::Dynamic) {
                target = look_hit->body;
            } else if (ctrl.last_placed != nullptr && ctrl.last_placed->type == BodyType::Dynamic) {
                target = ctrl.last_placed;
            }
            if (target != nullptr) {
                if (ctrl.possessed == target) {
                    ctrl.release();
                } else {
                    ctrl.possess(target);
                }
            }
        }

        if (ctrl.possessed != nullptr) {
            drive_possessed(ctrl, world, dt);
        } else {
            fly_spectator(ctrl, dt);
        }

        world.step(dt);

        if (ctrl.possessed != nullptr) {
            ctrl.eye = to_rl(ctrl.possessed->position);
        }
        ctrl.to_camera(camera);

        BeginDrawing();
        ClearBackground({16, 18, 24, 255});

        BeginMode3D(camera);
        DrawGrid(24, 1.0f);
        std::size_t index = 0;
        for (const auto& body : world.bodies()) {
            if (body.get() == ctrl.possessed) {
                ++index;
                continue;
            }
            const bool hovered = look_hit && look_hit->body == body.get();
            draw_body(*body, color_for(index++, *body, hovered));
        }
        EndMode3D();

        const int cx = GetScreenWidth() / 2;
        const int cy = GetScreenHeight() / 2;
        DrawLine(cx - 8, cy, cx + 8, cy, RAYWHITE);
        DrawLine(cx, cy - 8, cx, cy + 8, RAYWHITE);

        DrawRectangle(12, 12, 560, 118, ColorAlpha({10, 12, 16, 255}, 0.72f));
        DrawText(ctrl.possessed ? "Possessed  (first person)" : "Spectator  (fly)", 24, 22, 20, RAYWHITE);
        DrawText("WASD move   mouse look   SPACE fly up/jump   Ctrl down   Shift sprint", 24, 48, 16, LIGHTGRAY);
        DrawText("LMB / 1 sphere   RMB / 2 box   F become object   ESC leave body   Q quit", 24, 70, 16,
                 LIGHTGRAY);

        if (look_hit && look_hit->body->type == BodyType::Dynamic && ctrl.possessed == nullptr) {
            DrawText("F  become this object", cx - 110, cy + 24, 18, {255, 224, 130, 255});
        }

        char stats[160];
        std::snprintf(stats, sizeof(stats), "bodies %zu   contacts %zu", world.bodies().size(),
                      world.contacts().size());
        DrawText(stats, 24, 96, 16, GRAY);
        DrawFPS(GetScreenWidth() - 90, 16);
        EndDrawing();
    }

    EnableCursor();
    CloseWindow();
    return 0;
}
