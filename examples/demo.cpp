#include "physics/physics.hpp"

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <cmath>
#include <cstdio>
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

Color color_for(std::size_t index, const Body& body) {
    if (body.type == BodyType::Static) {
        return {90, 96, 110, 255};
    }
    return kPalette[index % (sizeof(kPalette) / sizeof(kPalette[0]))];
}

void draw_body(const Body& body, Color color) {
    const Vector3 pos{body.position.x, body.position.y, body.position.z};
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

}  // namespace

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI);
    InitWindow(1280, 720, "Physics Engine 3D");
    SetTargetFPS(60);

    Camera3D camera{};
    camera.position = {14.0f, 10.0f, 14.0f};
    camera.target = {0.0f, 2.0f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    World world;
    populate(world);

    while (!WindowShouldClose()) {
        UpdateCamera(&camera, CAMERA_ORBITAL);

        if (IsKeyPressed(KEY_R)) {
            populate(world);
        }
        if (IsKeyPressed(KEY_SPACE)) {
            Body* ball = world.add_sphere({GetRandomValue(-20, 20) * 0.1f, 9.0f, GetRandomValue(-20, 20) * 0.1f},
                                          0.42f, 1.0f);
            ball->restitution = 0.5f;
        }
        if (IsKeyPressed(KEY_B)) {
            Body* box = world.add_box({GetRandomValue(-20, 20) * 0.1f, 9.0f, GetRandomValue(-20, 20) * 0.1f},
                                      {0.4f, 0.4f, 0.4f}, 1.0f);
            box->restitution = 0.2f;
            box->orientation = Quat::from_axis_angle({0.2f, 1.0f, 0.1f}, 0.7f);
        }

        const float dt = std::min(GetFrameTime(), 1.0f / 30.0f);
        world.step(dt);

        BeginDrawing();
        ClearBackground({16, 18, 24, 255});

        BeginMode3D(camera);
        DrawGrid(24, 1.0f);
        std::size_t index = 0;
        for (const auto& body : world.bodies()) {
            draw_body(*body, color_for(index++, *body));
        }
        EndMode3D();

        DrawRectangle(12, 12, 420, 92, ColorAlpha({10, 12, 16, 255}, 0.7f));
        DrawText("3D rigid-body physics", 24, 22, 20, RAYWHITE);
        DrawText("R reset   SPACE spawn sphere   B spawn box", 24, 50, 16, LIGHTGRAY);

        char stats[128];
        std::snprintf(stats, sizeof(stats), "bodies %zu   contacts %zu", world.bodies().size(),
                      world.contacts().size());
        DrawText(stats, 24, 74, 16, GRAY);
        DrawFPS(GetScreenWidth() - 90, 16);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
