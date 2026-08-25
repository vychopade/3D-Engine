#pragma once

#include "physics/body.hpp"
#include "physics/contact.hpp"

#include <memory>
#include <vector>

namespace physics {

class World {
public:
    Vec3 gravity{0.0f, -9.81f, 0.0f};
    int velocity_iterations = 8;
    int position_iterations = 3;

    Body* add_sphere(Vec3 position, float radius, float density = 1.0f,
                     BodyType type = BodyType::Dynamic);
    Body* add_box(Vec3 position, Vec3 half_extents, float density = 1.0f,
                  BodyType type = BodyType::Dynamic);

    void clear();
    void step(float dt);

    const std::vector<std::unique_ptr<Body>>& bodies() const { return bodies_; }
    const std::vector<Contact>& contacts() const { return contacts_; }

private:
    std::vector<std::unique_ptr<Body>> bodies_;
    std::vector<Contact> contacts_;

    void integrate_velocities(float dt);
    void integrate_positions(float dt);
    void find_contacts();
    void solve_velocities();
    void solve_positions();
};

}  // namespace physics
