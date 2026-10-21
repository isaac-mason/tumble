#include "app/app.h"
#include "app/examples.h"
#include "app/random.h"
#include "imgui.h"
#include "sokol_app.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

static constexpr int COUNT = 100;
static constexpr float SPHERE_RADIUS = 1.0f;
static constexpr float LINEAR_DAMPING = 2.0f;
static constexpr float ANGULAR_DAMPING = 4.0f;
static constexpr float POINTER_RADIUS = 5.0f;

static struct {
    app::Layers layers;
    rigid_body::RigidBody *spheres[COUNT];
    rigid_body::RigidBody *pointer;
    Float3 pointer_position;
    Random random;
} state;

static Color hex(uint32_t rgb) { return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, 1.0f}; }

static void init(App &app) {
    state.random = {3};
    state.pointer_position = {0.0f, -100.0f, 0.0f};
    app.world = app::create_world(state.layers, {0.0f, 0.0f, 0.0f});
    // no ground: the grid would cut through the cluster
    app.drawing.grid = false;

    // orange, hotpink, white
    const Color colors[] = {hex(0xffa500), hex(0xff69b4), hex(0xffffff)};
    shapes::Shape *ball = app::keep(app, sphere::create({.radius = SPHERE_RADIUS}));
    for (int i = 0; i < COUNT; i++) {
        const float side = seeded::next(state.random) > 0.5f ? 1.0f : -1.0f;
        rigid_body::RigidBodySettings settings;
        settings.shape = ball;
        settings.object_layer = state.layers.moving;
        settings.motion_type = motion_type::MOTION_DYNAMIC;
        settings.position = {side * seeded::range(state.random, 50.0f, 100.0f), seeded::range(state.random, -50.0f, 50.0f),
                             seeded::range(state.random, -10.0f, 10.0f)};
        settings.linear_damping = LINEAR_DAMPING;
        settings.angular_damping = ANGULAR_DAMPING;
        settings.friction = 0.1f;
        state.spheres[i] = rigid_body::create(*app.world, settings);
        app::set_color(app, state.spheres[i], colors[i % 3]);
    }

    rigid_body::RigidBodySettings pointer;
    pointer.shape = app::keep(app, sphere::create({.radius = POINTER_RADIUS}));
    pointer.object_layer = state.layers.moving;
    pointer.motion_type = motion_type::MOTION_KINEMATIC;
    pointer.position = state.pointer_position;
    pointer.restitution = 0.8f;
    pointer.friction = 0.5f;
    state.pointer = rigid_body::create(*app.world, pointer);
    // crashcat draws the pointer only as a light: a faint ball here
    app::set_color(app, state.pointer, {1.0f, 1.0f, 1.0f, 0.08f});

    camera::look_at(app.camera, {0.0f, 0.0f, 20.0f}, {0.0f, 0.0f, 0.0f});
}

static void step(App &app, float delta_time) {
    // the pointer on the z = 0 plane; crashcat's pointer starts at the middle of the screen
    const bool moved = app.pointer.x != 0.0f || app.pointer.y != 0.0f;
    const Ray ray = moved ? app.pointer.ray : camera::screen_ray(app.camera, 0.5f * app.camera.width, 0.5f * app.camera.height);
    if (ray.direction.z < -1e-6f || ray.direction.z > 1e-6f) {
        const float t = -ray.origin.z / ray.direction.z;
        if (t > 0.0f) state.pointer_position = {ray.origin.x + ray.direction.x * t, ray.origin.y + ray.direction.y * t, 0.0f};
    }
    rigid_body::move_kinematic(*app.world, state.pointer, vec3f::load(state.pointer_position), quatf::identity(), delta_time);

    // space held stands in for crashcat's pointer held: the cluster gathers at the pointer
    const bool held = app.keys[SAPP_KEYCODE_SPACE];
    const Vec3f centre = held ? vec3f::load(state.pointer_position) : vec3f::zero();
    const float intensity = held ? 0.3f : 0.2f;
    for (int i = 0; i < COUNT; i++) {
        rigid_body::RigidBody *body = state.spheres[i];
        const Vec3f pull = vec3f::scale(vec3f::subtract(centre, vec3f::load(body->position)), intensity);
        Float3 &velocity = body->motion_properties.linear_velocity;
        vec3f::store(velocity, vec3f::add(vec3f::load(velocity), pull));
    }
}

static void ui(App &) { ImGui::TextUnformatted("hold space to pull the cluster to the pointer"); }

static const Example EXAMPLE_CLUSTER = {"Cluster", init, step, nullptr, ui};
static const examples::Registration REGISTERED(EXAMPLE_CLUSTER);
