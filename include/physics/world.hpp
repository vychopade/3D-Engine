#pragma once

#include "physics/body.hpp"
#include "physics/contact.hpp"

#include <memory>
#include <vector>

namespace physics {

class World {
public:
    Vec3 gravity{0.0f, -9.81f, 0.0f};
    int velocity_iterations = 12;
    int position_iterations = 8;
    float max_substep = 1.0f / 120.0f;
    int max_substeps = 8;

    Body* add_sphere(Vec3 position, float radius, float density = 1.0f,
                     BodyType type = BodyType::Dynamic);
    Body* add_box(Vec3 position, Vec3 half_extents, float density = 1.0f,
                  BodyType type = BodyType::Dynamic);

    void clear();
    void step(float dt);

    std::vector<std::unique_ptr<Body>>& bodies() { return bodies_; }
    const std::vector<std::unique_ptr<Body>>& bodies() const { return bodies_; }
    const std::vector<Contact>& contacts() const { return contacts_; }

private:
    std::vector<std::unique_ptr<Body>> bodies_;
    std::vector<Contact> contacts_;

    void step_once(float dt);
    void integrate_velocities(float dt);
    void integrate_positions(float dt);
    void find_contacts();
    void solve_velocities(float dt);
    void solve_positions();
};

}  // namespace physics
