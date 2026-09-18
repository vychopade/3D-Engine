#include "physics/physics.hpp"

#include <cmath>
#include <iostream>

using physics::BodyType;
using physics::Contact;
using physics::Vec3;
using physics::World;
using physics::collide;

static int g_failed = 0;

#define CHECK(cond)                                                                          \
    do {                                                                                     \
        if (!(cond)) {                                                                       \
            std::cerr << "FAIL  " << __FILE__ << ":" << __LINE__ << "  " << #cond << "\n";   \
            ++g_failed;                                                                      \
        }                                                                                    \
    } while (0)

int main() {
    {
        World world;
        auto* a = world.add_sphere({0.0f, 0.0f, 0.0f}, 1.0f);
        auto* b = world.add_sphere({1.5f, 0.0f, 0.0f}, 1.0f);
        Contact c;
        CHECK(collide(*a, *b, c));
        CHECK(std::abs(c.penetration - 0.5f) < 1e-4f);
        CHECK(std::abs(c.normal.x - 1.0f) < 1e-4f);
    }

    {
        World world;
        auto* a = world.add_sphere({0.0f, 0.0f, 0.0f}, 1.0f);
        auto* b = world.add_sphere({5.0f, 0.0f, 0.0f}, 1.0f);
        Contact c;
        CHECK(!collide(*a, *b, c));
    }

    {
        World world;
        auto* box = world.add_box({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
        auto* ball = world.add_sphere({0.0f, 1.4f, 0.0f}, 0.5f);
        Contact c;
        CHECK(collide(*ball, *box, c));
        CHECK(c.penetration > 0.0f);
        CHECK(c.normal.y < 0.0f);
    }

    {
        World world;
        world.gravity = {0.0f, -9.81f, 0.0f};
        world.add_box({0.0f, -0.5f, 0.0f}, {10.0f, 0.5f, 10.0f}, 1.0f, BodyType::Static);
        auto* ball = world.add_sphere({0.0f, 5.0f, 0.0f}, 0.5f);
        ball->restitution = 0.2f;

        for (int i = 0; i < 240; ++i) {
            world.step(1.0f / 60.0f);
        }

        CHECK(std::isfinite(ball->position.y));
        CHECK(ball->position.y > 0.4f);
        CHECK(ball->position.y < 1.2f);
        CHECK(std::abs(ball->velocity.y) < 1.5f);
    }

    {
        World world;
        world.gravity = {};
        auto* a = world.add_sphere({-0.4f, 0.0f, 0.0f}, 0.5f);
        auto* b = world.add_sphere({0.4f, 0.0f, 0.0f}, 0.5f);
        a->velocity = {2.0f, 0.0f, 0.0f};
        b->velocity = {-2.0f, 0.0f, 0.0f};
        a->restitution = 1.0f;
        b->restitution = 1.0f;
        a->friction = 0.0f;
        b->friction = 0.0f;

        for (int i = 0; i < 30; ++i) {
            world.step(1.0f / 60.0f);
        }

        CHECK(a->velocity.x < 0.0f);
        CHECK(b->velocity.x > 0.0f);
        CHECK(a->position.x < b->position.x);
    }

    {
        World world;
        world.add_box({0.0f, -0.5f, 0.0f}, {10.0f, 0.5f, 10.0f}, 1.0f, BodyType::Static);
        auto* ball = world.add_sphere({0.0f, -0.7f, 0.0f}, 0.4f);
        ball->prev_position = {0.0f, 0.6f, 0.0f};
        ball->velocity = {0.0f, -8.0f, 0.0f};
        Contact c;
        CHECK(collide(*ball, *world.bodies().front().get(), c));
        CHECK(c.normal.y < 0.0f);
    }

    {
        World world;
        world.gravity = {0.0f, -9.81f, 0.0f};
        world.add_box({0.0f, -0.5f, 0.0f}, {10.0f, 0.5f, 10.0f}, 1.0f, BodyType::Static);
        auto* ball = world.add_sphere({0.0f, 4.0f, 0.0f}, 0.5f);
        ball->velocity = {0.0f, -55.0f, 0.0f};
        ball->restitution = 0.0f;
        for (int i = 0; i < 180; ++i) {
            world.step(1.0f / 60.0f);
        }
        CHECK(std::isfinite(ball->position.y));
        CHECK(ball->position.y > 0.35f);
        CHECK(ball->position.y < 2.5f);
    }

    {
        World world;
        world.gravity = {0.0f, -9.81f, 0.0f};
        world.add_box({0.0f, -0.5f, 0.0f}, {10.0f, 0.5f, 10.0f}, 1.0f, BodyType::Static);
        auto* box = world.add_box({0.0f, 5.0f, 0.0f}, {0.5f, 0.5f, 0.5f});
        box->restitution = 0.05f;
        for (int i = 0; i < 240; ++i) {
            world.step(1.0f / 60.0f);
        }
        CHECK(std::isfinite(box->position.y));
        CHECK(box->position.y > 0.4f);
        CHECK(box->position.y < 1.3f);
    }

    if (g_failed == 0) {
        std::cout << "All tests passed.\n";
        return 0;
    }
    std::cerr << g_failed << " check(s) failed.\n";
    return 1;
}
