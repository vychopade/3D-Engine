#pragma once

#include "physics/body.hpp"
#include "physics/contact.hpp"
#include "physics/joint.hpp"
#include "physics/spring.hpp"

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

    Spring* add_spring(Body* a, Body* b, Vec3 local_a, Vec3 local_b, float rest_length,
                       float stiffness = 80.0f, float damping = 4.0f, bool bungee = false);
    DistanceJoint* add_rod(Body* a, Body* b, Vec3 local_a, Vec3 local_b, float length,
                           bool pendulum = false);

    void clear();
    bool destroy_body(Body* body);
    bool destroy_spring(Spring* spring);
    bool destroy_rod(DistanceJoint* rod);
    void step(float dt);

    std::vector<std::unique_ptr<Body>>& bodies() { return bodies_; }
    const std::vector<std::unique_ptr<Body>>& bodies() const { return bodies_; }
    std::vector<std::unique_ptr<Spring>>& springs() { return springs_; }
    const std::vector<std::unique_ptr<Spring>>& springs() const { return springs_; }
    std::vector<std::unique_ptr<DistanceJoint>>& rods() { return rods_; }
    const std::vector<std::unique_ptr<DistanceJoint>>& rods() const { return rods_; }
    const std::vector<Contact>& contacts() const { return contacts_; }

private:
    std::vector<std::unique_ptr<Body>> bodies_;
    std::vector<std::unique_ptr<Spring>> springs_;
    std::vector<std::unique_ptr<DistanceJoint>> rods_;
    std::vector<Contact> contacts_;

    void apply_springs();
    void solve_rods_velocity(float dt);
    void solve_rods_position();
    void step_once(float dt);
    void integrate_velocities(float dt);
    void integrate_positions(float dt);
    void find_contacts();
    void solve_velocities(float dt);
    void solve_positions();
};

}  // namespace physics
