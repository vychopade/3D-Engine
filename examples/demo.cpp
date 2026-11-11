#include "physics/physics.hpp"
#include "ui.hpp"

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

using physics::Body;
using physics::BodyType;
using physics::Quat;
using physics::ShapeType;
using physics::Vec3;
using physics::World;

namespace {

constexpr float kMouseSens = 0.0025f;
constexpr float kFlySpeed = 14.0f;
constexpr float kWalkSpeed = 7.0f;
constexpr float kJumpSpeed = 6.5f;

std::filesystem::path worlds_directory() {
    std::error_code ec;
    const auto cwd = std::filesystem::current_path(ec);
    if (!ec) {
        const auto in_cwd = cwd / "worlds";
        if (std::filesystem::exists(in_cwd)) {
            return in_cwd;
        }
        const auto sibling = cwd.parent_path() / "worlds";
        if (std::filesystem::exists(sibling)) {
            return sibling;
        }
    }
    const auto exe_dir = std::filesystem::path(GetApplicationDirectory());
    const auto next_to_project = exe_dir.parent_path() / "worlds";
    if (std::filesystem::exists(next_to_project)) {
        return next_to_project;
    }
    auto created = (ec ? exe_dir : cwd) / "worlds";
    std::filesystem::create_directories(created, ec);
    return created;
}

std::string sanitize_world_name(const char* raw) {
    std::string out;
    for (const char* p = raw; p != nullptr && *p != '\0'; ++p) {
        const unsigned char c = static_cast<unsigned char>(*p);
        if (std::isalnum(c) || c == '_' || c == '-') {
            out.push_back(static_cast<char>(c));
        } else if (c == ' ' && !out.empty() && out.back() != '_') {
            out.push_back('_');
        }
    }
    while (!out.empty() && out.back() == '_') {
        out.pop_back();
    }
    if (out.empty()) {
        out = "MyWorld";
    }
    return out;
}

std::vector<std::string> list_saved_worlds() {
    std::vector<std::string> names;
    std::error_code ec;
    const auto dir = worlds_directory();
    if (!std::filesystem::exists(dir, ec)) {
        return names;
    }
    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        if (entry.path().extension() == ".world") {
            names.push_back(entry.path().stem().string());
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

const Color kPalette[] = {
    {239, 83, 80, 255},  {66, 165, 245, 255}, {102, 187, 106, 255}, {255, 167, 38, 255},
    {171, 71, 188, 255}, {38, 198, 218, 255}, {255, 202, 40, 255},  {120, 144, 156, 255},
};

Vec3 to_vec(Vector3 v) { return {v.x, v.y, v.z}; }
Vector3 to_rl(Vec3 v) { return {v.x, v.y, v.z}; }

float snap_value(float v, float step) {
    if (step <= 1e-6f) {
        return v;
    }
    return std::round(v / step) * step;
}

Vec3 snap_vec(Vec3 v, float step) { return {snap_value(v.x, step), snap_value(v.y, step), snap_value(v.z, step)}; }

enum class Tool { Select, Sphere, Box, Floor, Wall, Ramp };
enum class Kind { Sphere, Box, Floor, Wall, Ramp };

struct Entity {
    Body* body = nullptr;
    Kind kind = Kind::Box;
    Color color{90, 96, 110, 255};
    float density = 1.0f;
    char name[48]{"Object"};
};

struct RayHit {
    Body* body = nullptr;
    float t = 0.0f;
    Vec3 point;
    Vec3 normal{0.0f, 1.0f, 0.0f};
};

struct PoseSnap {
    Body* body = nullptr;
    Vec3 position;
    Vec3 velocity;
    Vec3 angular_velocity;
    Quat orientation;
};

bool ray_aabb(Vec3 origin, Vec3 dir, Vec3 half, float& t_hit, Vec3& local_n) {
    float tmin = 0.0f;
    float tmax = 1.0e6f;
    int enter_axis = -1;
    float enter_sign = 1.0f;
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

    t_hit = tmin >= 0.0f ? tmin : tmax;
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
    t_hit = t0 >= 0.02f ? t0 : t1;
    return t_hit >= 0.02f;
}

std::optional<RayHit> raycast_world(World& world, Vec3 origin, Vec3 dir, const Body* ignore) {
    std::optional<RayHit> best;
    float best_t = std::numeric_limits<float>::max();

    for (auto& owned : world.bodies()) {
        Body* body = owned.get();
        if (body == ignore) {
            continue;
        }
        float t = 0.0f;
        Vec3 normal{0.0f, 1.0f, 0.0f};
        bool hit = false;
        if (body->shape.type == ShapeType::Sphere) {
            hit = ray_sphere(origin, dir, body->position, body->shape.radius, t);
            if (hit) {
                Vec3 n = origin + dir * t - body->position;
                normal = n.length_sq() > 1e-8f ? n.normalized() : Vec3{0.0f, 1.0f, 0.0f};
            }
        } else {
            const Vec3 local_o = body->world_to_local(origin);
            const Vec3 local_d = body->rotation().transposed() * dir;
            Vec3 local_n;
            hit = ray_aabb(local_o, local_d, body->shape.half_extents, t, local_n);
            if (hit) {
                normal = body->rotation() * local_n;
                const float nlen = normal.length();
                normal = nlen > 1e-8f ? normal / nlen : Vec3{0.0f, 1.0f, 0.0f};
            }
        }
        if (hit && t > 0.05f && t < best_t) {
            best_t = t;
            best = RayHit{body, t, origin + dir * t, normal};
        }
    }
    return best;
}

std::optional<RayHit> raycast_ground(Vec3 origin, Vec3 dir) {
    if (std::abs(dir.y) < 1e-6f) {
        return std::nullopt;
    }
    const float t = -origin.y / dir.y;
    if (t < 0.2f || t > 250.0f) {
        return std::nullopt;
    }
    return RayHit{nullptr, t, origin + dir * t, {0.0f, 1.0f, 0.0f}};
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
    float pitch = -0.35f;
    Vector3 eye{12.0f, 8.0f, 12.0f};
    Body* possessed = nullptr;
    float saved_restitution = 0.0f;
    float saved_friction = 0.0f;
    float saved_ang_damp = 0.0f;
    bool looking = false;

    Vec3 forward() const {
        const float cp = std::cos(pitch);
        return {std::sin(yaw) * cp, std::sin(pitch), std::cos(yaw) * cp};
    }
    Vec3 flat_forward() const { return Vec3{std::sin(yaw), 0.0f, std::cos(yaw)}.normalized(); }
    Vec3 right() const { return physics::cross(forward(), {0.0f, 1.0f, 0.0f}).normalized(); }
    Vec3 flat_right() const { return physics::cross(flat_forward(), {0.0f, 1.0f, 0.0f}).normalized(); }

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
        eye = to_rl(possessed->position + Vec3{0.0f, 0.8f, 0.0f});
        possessed = nullptr;
    }

    void possess(Body* body) {
        if (body == nullptr || body->type != BodyType::Dynamic) {
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

struct Editor {
    World world;
    std::vector<Entity> entities;
    Controller ctrl;
    Tool tool = Tool::Select;
    Body* selected = nullptr;
    Body* dragging = nullptr;
    float drag_y = 0.0f;
    bool playing = false;
    bool grid_snap = true;
    float snap = 0.5f;
    std::vector<PoseSnap> snapshot;
    std::string status = "Place a floor, then add objects. Right-drag to look.";
    int counters[6]{};

    char world_name[64]{"Untitled"};
    bool save_dialog = false;
    bool load_dialog = false;
    bool name_focused = true;
    std::vector<std::string> saved_worlds;
    int load_scroll = 0;

    Color brush_color{66, 165, 245, 255};
    bool place_dynamic = true;
    float sphere_r = 0.5f;
    Vec3 box_half{0.45f, 0.45f, 0.45f};
    Vec3 floor_half{5.0f, 0.15f, 5.0f};
    Vec3 wall_half{0.12f, 1.6f, 3.0f};
    Vec3 ramp_half{2.4f, 0.12f, 1.6f};
    float ramp_angle = 0.35f;

    Entity* find(Body* body) {
        for (auto& e : entities) {
            if (e.body == body) {
                return &e;
            }
        }
        return nullptr;
    }

    void bind(Body* body, Kind kind, Color color, float density, const char* prefix) {
        Entity e;
        e.body = body;
        e.kind = kind;
        e.color = color;
        e.density = density;
        const int idx = ++counters[static_cast<int>(kind)];
        std::snprintf(e.name, sizeof(e.name), "%s_%d", prefix, idx);
        body->user_data = &entities.emplace_back(e);
        // user_data will dangle if vector reallocates — fix after push by rebinding all
        rebind_user_data();
    }

    void rebind_user_data() {
        for (auto& e : entities) {
            if (e.body != nullptr) {
                e.body->user_data = &e;
            }
        }
    }

    void remove_entity(Body* body) {
        if (selected == body) {
            selected = nullptr;
        }
        if (dragging == body) {
            dragging = nullptr;
        }
        if (ctrl.possessed == body) {
            ctrl.release();
        }
        entities.erase(std::remove_if(entities.begin(), entities.end(),
                                      [body](const Entity& e) { return e.body == body; }),
                       entities.end());
        world.destroy_body(body);
        rebind_user_data();
    }

    void clear_world() {
        ctrl.release();
        selected = nullptr;
        dragging = nullptr;
        entities.clear();
        world.clear();
        snapshot.clear();
        playing = false;
        for (int& c : counters) {
            c = 0;
        }
    }

    Quat wall_orientation() const { return Quat::from_axis_angle({0.0f, 1.0f, 0.0f}, ctrl.yaw + 1.5707963f); }

    Body* spawn(Kind kind, Vec3 at, Vec3 normal) {
        const Color color = brush_color;
        const bool dynamic = (kind == Kind::Sphere || kind == Kind::Box) && place_dynamic;
        const BodyType type = dynamic ? BodyType::Dynamic : BodyType::Static;
        const float density = 1.0f;
        Body* body = nullptr;
        const char* prefix = "Object";

        Vec3 n = normal.length_sq() > 1e-8f ? normal.normalized() : Vec3{0.0f, 1.0f, 0.0f};
        if (grid_snap) {
            at = snap_vec(at, snap);
        }

        switch (kind) {
            case Kind::Sphere:
                at += n * sphere_r;
                body = world.add_sphere(at, sphere_r, density, type);
                body->restitution = 0.4f;
                body->friction = 0.3f;
                prefix = "Sphere";
                break;
            case Kind::Box:
                at += n * box_half.y;
                body = world.add_box(at, box_half, density, type);
                body->restitution = 0.15f;
                body->friction = 0.45f;
                prefix = "Box";
                break;
            case Kind::Floor:
                at += Vec3{0.0f, floor_half.y, 0.0f};
                body = world.add_box(at, floor_half, density, BodyType::Static);
                body->restitution = 0.1f;
                body->friction = 0.7f;
                prefix = "Floor";
                break;
            case Kind::Wall: {
                if (std::abs(n.y) > 0.65f) {
                    at.y += wall_half.y;
                } else {
                    at += n * wall_half.x;
                    at.y += wall_half.y;
                }
                body = world.add_box(at, wall_half, density, BodyType::Static);
                body->orientation = wall_orientation();
                body->restitution = 0.1f;
                body->friction = 0.6f;
                prefix = "Wall";
                break;
            }
            case Kind::Ramp:
                at += Vec3{0.0f, ramp_half.y + 0.8f, 0.0f};
                body = world.add_box(at, ramp_half, density, BodyType::Static);
                body->orientation = Quat::from_axis_angle({0.0f, 0.0f, 1.0f}, -ramp_angle) *
                                    Quat::from_axis_angle({0.0f, 1.0f, 0.0f}, ctrl.yaw);
                body->restitution = 0.05f;
                body->friction = 0.8f;
                prefix = "Ramp";
                break;
        }

        if (body == nullptr) {
            return nullptr;
        }
        bind(body, kind, color, density, prefix);
        selected = body;
        return body;
    }

    std::filesystem::path path_for_name(const char* name) const {
        return worlds_directory() / (sanitize_world_name(name) + ".world");
    }

    void open_save_dialog() {
        save_dialog = true;
        load_dialog = false;
        name_focused = true;
        if (std::strlen(world_name) == 0) {
            std::snprintf(world_name, sizeof(world_name), "MyWorld");
        }
    }

    void open_load_dialog() {
        load_dialog = true;
        save_dialog = false;
        name_focused = false;
        saved_worlds = list_saved_worlds();
        load_scroll = 0;
        if (saved_worlds.empty()) {
            status = "No saved worlds yet. Use Save and give your scene a name.";
        }
    }

    bool save_current() {
        const std::string stem = sanitize_world_name(world_name);
        std::snprintf(world_name, sizeof(world_name), "%s", stem.c_str());
        const auto path = path_for_name(world_name);
        if (!save(path.string().c_str())) {
            return false;
        }
        status = "Saved world \"" + stem + "\"";
        save_dialog = false;
        return true;
    }

    bool load_named(const std::string& stem) {
        const auto path = path_for_name(stem.c_str());
        if (!load(path.string().c_str())) {
            return false;
        }
        std::snprintf(world_name, sizeof(world_name), "%s", stem.c_str());
        load_dialog = false;
        status = "Loaded world \"" + stem + "\"";
        return true;
    }

    void new_world() {
        clear_world();
        spawn(Kind::Floor, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
        selected = nullptr;
        std::snprintf(world_name, sizeof(world_name), "Untitled");
        status = "New world. Use Floor / Wall / Ramp / Box / Sphere to build.";
    }

    void begin_play() {
        snapshot.clear();
        for (auto& e : entities) {
            Body* b = e.body;
            snapshot.push_back({b, b->position, b->velocity, b->angular_velocity, b->orientation});
        }
        playing = true;
        tool = Tool::Select;
        dragging = nullptr;
        status = "Play mode. F possess a dynamic object. Stop to restore the edit pose.";
    }

    void stop_play() {
        ctrl.release();
        for (const auto& s : snapshot) {
            if (find(s.body) == nullptr) {
                continue;
            }
            s.body->position = s.position;
            s.body->prev_position = s.position;
            s.body->velocity = {};
            s.body->angular_velocity = {};
            s.body->orientation = s.orientation;
            s.body->force = {};
            s.body->torque = {};
        }
        snapshot.clear();
        playing = false;
        status = "Edit mode. Physics paused.";
    }

    bool save(const char* path) {
        std::filesystem::create_directories(std::filesystem::path(path).parent_path());
        std::ofstream out(path);
        if (!out) {
            status = "Save failed.";
            return false;
        }
        out << "PEWORLD 1\n";
        out << "gravity " << world.gravity.x << ' ' << world.gravity.y << ' ' << world.gravity.z << '\n';
        out << "count " << entities.size() << '\n';
        for (const auto& e : entities) {
            const Body& b = *e.body;
            out << static_cast<int>(e.kind) << ' ' << e.name << ' ' << static_cast<int>(e.color.r) << ' '
                << static_cast<int>(e.color.g) << ' ' << static_cast<int>(e.color.b) << ' ' << e.density << ' '
                << static_cast<int>(b.type) << ' ' << b.restitution << ' ' << b.friction << ' ' << b.position.x << ' '
                << b.position.y << ' ' << b.position.z << ' ' << b.orientation.w << ' ' << b.orientation.x << ' '
                << b.orientation.y << ' ' << b.orientation.z << ' ' << b.shape.radius << ' ' << b.shape.half_extents.x
                << ' ' << b.shape.half_extents.y << ' ' << b.shape.half_extents.z << '\n';
        }
        status = std::string("Saved ") + path;
        return true;
    }

    bool load(const char* path) {
        std::ifstream in(path);
        if (!in) {
            status = "No saved world yet. Build something, then Save.";
            return false;
        }
        std::string magic;
        int version = 0;
        in >> magic >> version;
        if (magic != "PEWORLD") {
            status = "Unrecognized world file.";
            return false;
        }
        std::string tag;
        in >> tag >> world.gravity.x >> world.gravity.y >> world.gravity.z;
        std::size_t count = 0;
        in >> tag >> count;
        clear_world();
        for (std::size_t i = 0; i < count; ++i) {
            int kind_i = 0, cr = 0, cg = 0, cb = 0, type_i = 0;
            char name[48];
            float density, rest, fric, px, py, pz, qw, qx, qy, qz, radius, hx, hy, hz;
            in >> kind_i >> name >> cr >> cg >> cb >> density >> type_i >> rest >> fric >> px >> py >> pz >> qw >> qx >>
                qy >> qz >> radius >> hx >> hy >> hz;
            if (!in) {
                break;
            }
            const Kind kind = static_cast<Kind>(kind_i);
            const BodyType type = static_cast<BodyType>(type_i);
            Body* body = nullptr;
            if (kind == Kind::Sphere) {
                body = world.add_sphere({px, py, pz}, radius, density, type);
            } else {
                body = world.add_box({px, py, pz}, {hx, hy, hz}, density, type);
            }
            body->orientation = {qw, qx, qy, qz};
            body->restitution = rest;
            body->friction = fric;
            bind(body, kind, Color{static_cast<unsigned char>(cr), static_cast<unsigned char>(cg),
                                   static_cast<unsigned char>(cb), 255},
                 density, "Loaded");
            std::snprintf(entities.back().name, sizeof(entities.back().name), "%s", name);
        }
        selected = nullptr;
        status = std::string("Loaded ") + path;
        return true;
    }
};

void fly(Controller& ctrl, float dt) {
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
    if (IsKeyDown(KEY_SPACE) && ctrl.possessed == nullptr) {
        wish += Vec3{0.0f, 1.0f, 0.0f};
    }
    if (IsKeyDown(KEY_LEFT_CONTROL)) {
        wish -= Vec3{0.0f, 1.0f, 0.0f};
    }
    const float speed = IsKeyDown(KEY_LEFT_SHIFT) ? kFlySpeed * 2.2f : kFlySpeed;
    if (wish.length_sq() > 1e-6f) {
        wish = wish.normalized();
        ctrl.eye.x += wish.x * speed * dt;
        ctrl.eye.y += wish.y * speed * dt;
        ctrl.eye.z += wish.z * speed * dt;
    }
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

Kind kind_from_tool(Tool tool) {
    switch (tool) {
        case Tool::Sphere:
            return Kind::Sphere;
        case Tool::Box:
            return Kind::Box;
        case Tool::Floor:
            return Kind::Floor;
        case Tool::Wall:
            return Kind::Wall;
        case Tool::Ramp:
            return Kind::Ramp;
        default:
            return Kind::Box;
    }
}

void apply_shape(Entity& e) {
    Body& b = *e.body;
    physics::compute_mass(b, e.density);
    if (b.type != BodyType::Dynamic) {
        b.set_type(BodyType::Static);
    }
}

}  // namespace

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI);
    InitWindow(1440, 860, "Physics World Builder");
    SetTargetFPS(60);
    SetExitKey(KEY_NULL);

    Camera3D camera{};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Editor editor;
    editor.new_world();
    ui::State ui;

    while (!WindowShouldClose()) {
        const float dt = std::min(GetFrameTime(), 1.0f / 30.0f);
        ui.begin();

        const int sw = GetScreenWidth();
        const int sh = GetScreenHeight();
        const Rectangle top{0, 0, static_cast<float>(sw), 48};
        const Rectangle left{0, 48, 188, static_cast<float>(sh - 76)};
        const Rectangle right{static_cast<float>(sw - 300), 48, 300, static_cast<float>(sh - 76)};
        const Rectangle bottom{0, static_cast<float>(sh - 28), static_cast<float>(sw), 28};
        const Rectangle viewport{188, 48, static_cast<float>(sw - 488), static_cast<float>(sh - 76)};
        const bool in_view = CheckCollisionPointRec(GetMousePosition(), viewport);
        const bool modal = editor.save_dialog || editor.load_dialog;

        if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_Q)) {
            break;
        }

        if (!modal && in_view && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            editor.ctrl.looking = true;
            DisableCursor();
        }
        if (editor.ctrl.looking && (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT) || modal)) {
            editor.ctrl.looking = false;
            EnableCursor();
        }
        if (!modal && (editor.ctrl.looking || editor.ctrl.possessed != nullptr)) {
            editor.ctrl.look();
        }

        if (!modal) {
            if (editor.ctrl.possessed != nullptr) {
                drive_possessed(editor.ctrl, editor.world, dt);
            } else {
                fly(editor.ctrl, dt);
            }
        }
        editor.ctrl.to_camera(camera);

        const Ray mouse_ray = GetMouseRay(GetMousePosition(), camera);
        const Vec3 origin = to_vec(mouse_ray.position);
        const Vec3 dir = to_vec(mouse_ray.direction);
        auto world_hit = raycast_world(editor.world, origin, dir, editor.ctrl.possessed);
        auto ground_hit = raycast_ground(origin, dir);
        std::optional<RayHit> place_hit = world_hit ? world_hit : ground_hit;

        if (!editor.playing && !modal) {
            if (IsKeyPressed(KEY_ONE)) {
                editor.tool = Tool::Select;
            }
            if (IsKeyPressed(KEY_TWO)) {
                editor.tool = Tool::Sphere;
            }
            if (IsKeyPressed(KEY_THREE)) {
                editor.tool = Tool::Box;
            }
            if (IsKeyPressed(KEY_FOUR)) {
                editor.tool = Tool::Floor;
            }
            if (IsKeyPressed(KEY_FIVE)) {
                editor.tool = Tool::Wall;
            }
            if (IsKeyPressed(KEY_SIX)) {
                editor.tool = Tool::Ramp;
            }
            if (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE)) {
                if (editor.selected != nullptr) {
                    editor.remove_entity(editor.selected);
                    editor.status = "Deleted object.";
                }
            }
            if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_D) && editor.selected != nullptr) {
                Entity* src = editor.find(editor.selected);
                if (src != nullptr) {
                    editor.brush_color = src->color;
                    editor.place_dynamic = src->body->type == BodyType::Dynamic;
                    if (src->kind == Kind::Sphere) {
                        editor.sphere_r = src->body->shape.radius;
                    } else {
                        editor.box_half = src->body->shape.half_extents;
                    }
                    Body* copy = editor.spawn(src->kind, src->body->position + Vec3{1.0f, 0.0f, 0.0f},
                                              {0.0f, 1.0f, 0.0f});
                    if (copy != nullptr) {
                        copy->orientation = src->body->orientation;
                    }
                    editor.status = "Duplicated.";
                }
            }
            if (in_view && !ui.hovered && GetMouseWheelMove() != 0.0f) {
                const float s = GetMouseWheelMove() > 0.0f ? 1.08f : 1.0f / 1.08f;
                editor.sphere_r = std::clamp(editor.sphere_r * s, 0.15f, 6.0f);
                editor.box_half = editor.box_half * s;
                editor.floor_half.x = std::clamp(editor.floor_half.x * s, 0.5f, 30.0f);
                editor.floor_half.z = std::clamp(editor.floor_half.z * s, 0.5f, 30.0f);
                editor.wall_half.y = std::clamp(editor.wall_half.y * s, 0.3f, 12.0f);
                editor.wall_half.z = std::clamp(editor.wall_half.z * s, 0.5f, 20.0f);
                editor.ramp_half = editor.ramp_half * s;
            }
        }

        if (!modal && IsKeyPressed(KEY_P)) {
            if (editor.playing) {
                editor.stop_play();
            } else {
                editor.begin_play();
            }
        }
        if (IsKeyPressed(KEY_ESCAPE)) {
            if (editor.save_dialog || editor.load_dialog) {
                editor.save_dialog = false;
                editor.load_dialog = false;
            } else if (editor.ctrl.possessed != nullptr) {
                editor.ctrl.release();
            } else if (editor.playing) {
                editor.stop_play();
            } else {
                editor.selected = nullptr;
            }
        }
        if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_S)) {
            const std::string stem = sanitize_world_name(editor.world_name);
            if (stem == "Untitled") {
                editor.open_save_dialog();
            } else {
                editor.save_current();
            }
        }
        if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_O)) {
            editor.open_load_dialog();
        }
        if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_N)) {
            editor.new_world();
        }

        if (editor.playing && editor.ctrl.possessed == nullptr && IsKeyPressed(KEY_F) && world_hit &&
            world_hit->body != nullptr) {
            editor.ctrl.possess(world_hit->body);
        }

        if (!modal && !editor.playing && in_view && !ui.hovered && !editor.ctrl.looking) {
            if (editor.tool == Tool::Select) {
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                    editor.selected = world_hit ? world_hit->body : nullptr;
                    if (editor.selected != nullptr) {
                        editor.dragging = editor.selected;
                        editor.drag_y = editor.selected->position.y;
                    }
                }
                if (editor.dragging != nullptr && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                    if (std::abs(dir.y) > 1e-5f) {
                        const float t = (editor.drag_y - origin.y) / dir.y;
                        if (t > 0.0f) {
                            Vec3 p = origin + dir * t;
                            if (editor.grid_snap) {
                                p = snap_vec(p, editor.snap);
                                p.y = editor.drag_y;
                            }
                            editor.dragging->position = p;
                            editor.dragging->prev_position = p;
                            editor.dragging->velocity = {};
                        }
                    }
                }
                if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                    editor.dragging = nullptr;
                }
            } else if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && place_hit) {
                editor.spawn(kind_from_tool(editor.tool), place_hit->point, place_hit->normal);
                editor.status = "Placed object.";
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            editor.dragging = nullptr;
        }

        if (editor.playing) {
            editor.world.step(dt);
            if (editor.ctrl.possessed != nullptr) {
                editor.ctrl.eye = to_rl(editor.ctrl.possessed->position);
                editor.ctrl.to_camera(camera);
            }
        }

        BeginDrawing();
        ClearBackground({14, 16, 22, 255});

        BeginMode3D(camera);
        DrawGrid(48, 1.0f);
        for (const auto& e : editor.entities) {
            if (e.body == editor.ctrl.possessed) {
                continue;
            }
            Color c = e.color;
            if (e.body == editor.selected) {
                c = Color{255, 224, 130, 255};
            } else if (world_hit && world_hit->body == e.body) {
                c.r = static_cast<unsigned char>(std::min(255, c.r + 40));
                c.g = static_cast<unsigned char>(std::min(255, c.g + 40));
                c.b = static_cast<unsigned char>(std::min(255, c.b + 40));
            }
            draw_body(*e.body, c);
        }

        if (!editor.playing && editor.tool != Tool::Select && in_view && place_hit && !ui.hovered) {
            Body ghost;
            ghost.position = place_hit->point;
            ghost.orientation = Quat::identity();
            const Vec3 n = place_hit->normal;
            Color gc = ColorAlpha(editor.brush_color, 120);
            switch (editor.tool) {
                case Tool::Sphere:
                    ghost.shape = physics::Shape::sphere(editor.sphere_r);
                    ghost.position += n * editor.sphere_r;
                    break;
                case Tool::Box:
                    ghost.shape = physics::Shape::box(editor.box_half);
                    ghost.position += n * editor.box_half.y;
                    break;
                case Tool::Floor:
                    ghost.shape = physics::Shape::box(editor.floor_half);
                    ghost.position.y += editor.floor_half.y;
                    break;
                case Tool::Wall:
                    ghost.shape = physics::Shape::box(editor.wall_half);
                    ghost.position += n * editor.wall_half.x;
                    ghost.position.y += editor.wall_half.y;
                    ghost.orientation = editor.wall_orientation();
                    break;
                case Tool::Ramp:
                    ghost.shape = physics::Shape::box(editor.ramp_half);
                    ghost.position.y += editor.ramp_half.y + 0.8f;
                    ghost.orientation = Quat::from_axis_angle({0.0f, 0.0f, 1.0f}, -editor.ramp_angle) *
                                        Quat::from_axis_angle({0.0f, 1.0f, 0.0f}, editor.ctrl.yaw);
                    break;
                default:
                    break;
            }
            if (editor.grid_snap) {
                ghost.position = snap_vec(ghost.position, editor.snap);
            }
            draw_body(ghost, gc);
        }
        EndMode3D();

        ui.absorb = modal;
        ui::panel(top, {18, 20, 28, 245});
        ui::panel(left, {16, 18, 26, 240});
        ui::panel(right, {16, 18, 26, 240});
        ui::panel(bottom, {12, 14, 20, 255});

        char title[96];
        std::snprintf(title, sizeof(title), "World Builder  —  %s", editor.world_name);
        ui::label(16, 14, title, RAYWHITE, 22);
        if (ui::button(ui, {268, 10, 80, 28}, "New")) {
            editor.new_world();
        }
        if (ui::button(ui, {356, 10, 80, 28}, "Save")) {
            editor.open_save_dialog();
        }
        if (ui::button(ui, {444, 10, 80, 28}, "Load")) {
            editor.open_load_dialog();
        }
        if (ui::button(ui, {540, 10, 100, 28}, editor.playing ? "Stop" : "Play", editor.playing)) {
            if (editor.playing) {
                editor.stop_play();
            } else {
                editor.begin_play();
            }
        }
        editor.grid_snap =
            ui::toggle(ui, {650, 10, 100, 28}, editor.grid_snap ? "Snap on" : "Snap off", editor.grid_snap);

        ui::label(16, 60, "TOOLS", GRAY, 14);
        float ty = 84;
        auto tool_btn = [&](Tool t, const char* name, const char* key) {
            char label[48];
            std::snprintf(label, sizeof(label), "%s  %s", key, name);
            if (ui::button(ui, {14, ty, 160, 34}, label, editor.tool == t)) {
                editor.tool = t;
            }
            ty += 40;
        };
        tool_btn(Tool::Select, "Select", "1");
        tool_btn(Tool::Sphere, "Sphere", "2");
        tool_btn(Tool::Box, "Box", "3");
        tool_btn(Tool::Floor, "Floor", "4");
        tool_btn(Tool::Wall, "Wall", "5");
        tool_btn(Tool::Ramp, "Ramp", "6");

        ty += 8;
        ui::label(16, static_cast<int>(ty), "BRUSH", GRAY, 14);
        ty += 24;
        if (editor.tool == Tool::Sphere || editor.tool == Tool::Box) {
            editor.place_dynamic =
                ui::toggle(ui, {14, ty, 160, 30}, editor.place_dynamic ? "Dynamic" : "Static", editor.place_dynamic);
            ty += 44;
        }
        if (editor.tool == Tool::Sphere) {
            editor.sphere_r = ui::slider(ui, {14, ty + 16, 160, 16}, editor.sphere_r, 0.15f, 4.0f, "Radius");
            ty += 50;
        } else if (editor.tool == Tool::Box) {
            editor.box_half.x = ui::slider(ui, {14, ty + 16, 160, 16}, editor.box_half.x, 0.15f, 6.0f, "Width");
            editor.box_half.y = ui::slider(ui, {14, ty + 56, 160, 16}, editor.box_half.y, 0.15f, 6.0f, "Height");
            editor.box_half.z = ui::slider(ui, {14, ty + 96, 160, 16}, editor.box_half.z, 0.15f, 6.0f, "Depth");
            ty += 130;
        } else if (editor.tool == Tool::Floor) {
            editor.floor_half.x = ui::slider(ui, {14, ty + 16, 160, 16}, editor.floor_half.x, 1.0f, 20.0f, "Width");
            editor.floor_half.z = ui::slider(ui, {14, ty + 56, 160, 16}, editor.floor_half.z, 1.0f, 20.0f, "Depth");
            ty += 90;
        } else if (editor.tool == Tool::Wall) {
            editor.wall_half.y = ui::slider(ui, {14, ty + 16, 160, 16}, editor.wall_half.y, 0.4f, 8.0f, "Height");
            editor.wall_half.z = ui::slider(ui, {14, ty + 56, 160, 16}, editor.wall_half.z, 0.5f, 12.0f, "Width");
            ty += 90;
        } else if (editor.tool == Tool::Ramp) {
            editor.ramp_angle = ui::slider(ui, {14, ty + 16, 160, 16}, editor.ramp_angle, 0.1f, 0.8f, "Angle");
            ty += 50;
        }

        ui::label(16, static_cast<int>(ty + 8), "COLOR", GRAY, 14);
        ty += 28;
        for (int i = 0; i < 8; ++i) {
            const float x = 14.0f + static_cast<float>(i % 4) * 40.0f;
            const float y = ty + static_cast<float>(i / 4) * 36.0f;
            const Rectangle sw{x, y, 34, 28};
            ui.mark(sw);
            DrawRectangleRec(sw, kPalette[i]);
            if (editor.brush_color.r == kPalette[i].r && editor.brush_color.g == kPalette[i].g &&
                editor.brush_color.b == kPalette[i].b) {
                DrawRectangleLinesEx(sw, 2, RAYWHITE);
            }
            if (!ui.absorb && ui.over(sw) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                editor.brush_color = kPalette[i];
                ui.used = true;
                if (editor.selected != nullptr) {
                    if (Entity* e = editor.find(editor.selected)) {
                        e->color = kPalette[i];
                    }
                }
            }
        }

        ui::label(static_cast<int>(right.x + 16), 60, "INSPECTOR", GRAY, 14);
        Entity* sel = editor.find(editor.selected);
        if (sel == nullptr) {
            ui::label(static_cast<int>(right.x + 16), 96, "Click an object to edit it.", DARKGRAY, 16);
            ui::label(static_cast<int>(right.x + 16), 124, "Or choose a tool and click", DARKGRAY, 16);
            ui::label(static_cast<int>(right.x + 16), 144, "in the viewport to place.", DARKGRAY, 16);
        } else {
            Body& b = *sel->body;
            ui::label(static_cast<int>(right.x + 16), 88, sel->name, RAYWHITE, 20);
            const char* kind_name = "Object";
            if (sel->kind == Kind::Sphere) {
                kind_name = "Sphere";
            } else if (sel->kind == Kind::Box) {
                kind_name = "Box";
            } else if (sel->kind == Kind::Floor) {
                kind_name = "Floor plane";
            } else if (sel->kind == Kind::Wall) {
                kind_name = "Wall plane";
            } else if (sel->kind == Kind::Ramp) {
                kind_name = "Ramp";
            }
            ui::label(static_cast<int>(right.x + 16), 112, kind_name, GRAY, 16);

            const float rx = right.x + 16;
            bool is_dyn = b.type == BodyType::Dynamic;
            const bool want_dyn = ui::toggle(ui, {rx, 140, 268, 30}, is_dyn ? "Dynamic body" : "Static body", is_dyn);
            if (want_dyn != is_dyn) {
                b.set_type(want_dyn ? BodyType::Dynamic : BodyType::Static);
                if (want_dyn) {
                    physics::compute_mass(b, sel->density);
                }
            }

            b.position.x = ui::slider(ui, {rx, 196, 268, 16}, b.position.x, -24.0f, 24.0f, "Position X");
            b.position.y = ui::slider(ui, {rx, 236, 268, 16}, b.position.y, -2.0f, 20.0f, "Position Y");
            b.position.z = ui::slider(ui, {rx, 276, 268, 16}, b.position.z, -24.0f, 24.0f, "Position Z");
            b.prev_position = b.position;

            if (b.shape.type == ShapeType::Sphere) {
                const float r = ui::slider(ui, {rx, 326, 268, 16}, b.shape.radius, 0.15f, 6.0f, "Radius");
                if (r != b.shape.radius) {
                    b.shape.radius = r;
                    apply_shape(*sel);
                }
            } else {
                const float hx = ui::slider(ui, {rx, 326, 268, 16}, b.shape.half_extents.x, 0.08f, 20.0f, "Half width");
                const float hy = ui::slider(ui, {rx, 366, 268, 16}, b.shape.half_extents.y, 0.08f, 12.0f, "Half height");
                const float hz = ui::slider(ui, {rx, 406, 268, 16}, b.shape.half_extents.z, 0.08f, 20.0f, "Half depth");
                if (hx != b.shape.half_extents.x || hy != b.shape.half_extents.y || hz != b.shape.half_extents.z) {
                    b.shape.half_extents = {hx, hy, hz};
                    apply_shape(*sel);
                }
            }

            b.restitution = ui::slider(ui, {rx, 456, 268, 16}, b.restitution, 0.0f, 1.0f, "Bounciness");
            b.friction = ui::slider(ui, {rx, 496, 268, 16}, b.friction, 0.0f, 1.2f, "Friction");

            if (ui::button(ui, {rx, 536, 128, 32}, "Duplicate")) {
                editor.brush_color = sel->color;
                Body* copy = editor.spawn(sel->kind, b.position + Vec3{1, 0, 0}, {0, 1, 0});
                if (copy != nullptr) {
                    copy->orientation = b.orientation;
                }
            }
            if (ui::button(ui, {rx + 140, 536, 128, 32}, "Delete")) {
                editor.remove_entity(sel->body);
                sel = nullptr;
            }
        }

        ui.absorb = false;
        if (editor.save_dialog) {
            DrawRectangle(0, 0, sw, sh, Color{0, 0, 0, 140});
            const Rectangle box{static_cast<float>(sw) * 0.5f - 220.0f, static_cast<float>(sh) * 0.5f - 110.0f, 440, 220};
            ui.mark(box);
            ui::panel(box, {24, 26, 36, 255});
            ui::label(static_cast<int>(box.x) + 24, static_cast<int>(box.y) + 20, "Save world", RAYWHITE, 22);
            ui::label(static_cast<int>(box.x) + 24, static_cast<int>(box.y) + 52, "Name this scene to keep it.", GRAY, 16);
            ui::text_field(ui, {box.x + 24, box.y + 88, 392, 36}, editor.world_name, static_cast<int>(sizeof(editor.world_name)),
                           editor.name_focused);
            if (ui::button(ui, {box.x + 24, box.y + 148, 180, 36}, "Save")) {
                editor.save_current();
            }
            if (ui::button(ui, {box.x + 216, box.y + 148, 180, 36}, "Cancel")) {
                editor.save_dialog = false;
            }
            if (IsKeyPressed(KEY_ENTER)) {
                editor.save_current();
            }
        }

        if (editor.load_dialog) {
            DrawRectangle(0, 0, sw, sh, Color{0, 0, 0, 140});
            const Rectangle box{static_cast<float>(sw) * 0.5f - 230.0f, static_cast<float>(sh) * 0.5f - 200.0f, 460, 400};
            ui.mark(box);
            ui::panel(box, {24, 26, 36, 255});
            ui::label(static_cast<int>(box.x) + 24, static_cast<int>(box.y) + 20, "Load world", RAYWHITE, 22);

            const int visible = 7;
            if (ui.over(box) && GetMouseWheelMove() != 0.0f) {
                editor.load_scroll -= static_cast<int>(GetMouseWheelMove());
            }
            const int max_scroll =
                std::max(0, static_cast<int>(editor.saved_worlds.size()) - visible);
            editor.load_scroll = std::clamp(editor.load_scroll, 0, max_scroll);

            float y = box.y + 60;
            if (editor.saved_worlds.empty()) {
                ui::label(static_cast<int>(box.x) + 24, static_cast<int>(y), "No saved worlds yet.", DARKGRAY, 16);
            }
            for (int i = 0; i < visible; ++i) {
                const int idx = editor.load_scroll + i;
                if (idx >= static_cast<int>(editor.saved_worlds.size())) {
                    break;
                }
                const auto& name = editor.saved_worlds[static_cast<std::size_t>(idx)];
                if (ui::button(ui, {box.x + 24, y, 300, 32}, name.c_str())) {
                    editor.load_named(name);
                }
                if (ui::button(ui, {box.x + 332, y, 104, 32}, "Delete")) {
                    std::error_code ec;
                    std::filesystem::remove(editor.path_for_name(name.c_str()), ec);
                    editor.saved_worlds = list_saved_worlds();
                    editor.status = "Deleted saved world \"" + name + "\"";
                }
                y += 38;
            }

            if (ui::button(ui, {box.x + 24, box.y + 348, 412, 36}, "Cancel")) {
                editor.load_dialog = false;
            }
        }

        if (in_view && !editor.ctrl.looking && editor.ctrl.possessed == nullptr && !modal) {
            const Vector2 m = GetMousePosition();
            DrawLine(static_cast<int>(m.x) - 7, static_cast<int>(m.y), static_cast<int>(m.x) + 7, static_cast<int>(m.y),
                     ColorAlpha(RAYWHITE, 0.7f));
            DrawLine(static_cast<int>(m.x), static_cast<int>(m.y) - 7, static_cast<int>(m.x), static_cast<int>(m.y) + 7,
                     ColorAlpha(RAYWHITE, 0.7f));
        }

        char stats[256];
        std::snprintf(stats, sizeof(stats),
                      "%s    |    objects %zu    |    Ctrl+S save   Ctrl+O load   WASD fly   RMB look",
                      editor.status.c_str(), editor.entities.size());
        DrawText(stats, 12, sh - 22, 16, LIGHTGRAY);
        DrawFPS(sw - 90, 14);
        ui.end();
        EndDrawing();
    }

    EnableCursor();
    CloseWindow();
    return 0;
}
