#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/model.h"
#include "app/random.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

static constexpr float MAXWELL_SCALE = 0.1f;
/** maxwell.glb's body is its first primitive, these vertices; its whiskers follow, and stay out of the hull */
static constexpr uint32_t MAXWELL_BODY_VERTICES = 345;
static constexpr float MOVEMENT_RANGE = 2.5f;
static constexpr float MOVEMENT_SPEED = 2.0f;
static constexpr float ROTATION_SPEED = scalar::PI * 1.5f;
static constexpr float SPAWN_HEIGHT = 12.0f;
/** a ball every this many steps: 0.05 s */
static constexpr int SPAWN_STEPS = 3;
static constexpr int MAX_BALLS = 100;
/** balls further than this from the y axis start again from the top */
static constexpr float RESPAWN_RADIUS = 15.0f;

static struct {
    app::Layers layers;
    rigid_body::RigidBody *maxwell;
    shapes::Shape *ball;
    /** oldest first */
    collections::Vec<rigid_body::RigidBody *> balls;
    float time;
    int steps;
    Random random;
} state;

static Color hex(uint32_t rgb) { return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, 1.0f}; }

static Color hsl(float hue, float saturation, float lightness) {
    const auto channel = [&](float n) {
        const float k = fmodf(n + hue * 12.0f, 12.0f);
        const float a = saturation * fminf(lightness, 1.0f - lightness);
        return lightness - a * fmaxf(-1.0f, fminf(fminf(k - 3.0f, 9.0f - k), 1.0f));
    };
    return {channel(0.0f), channel(8.0f), channel(4.0f), 1.0f};
}

/** somewhere over the cat's path, high up, at rest */
static void reset_ball(App &app, rigid_body::RigidBody *ball) {
    const float x = (random::next(state.random) - 0.5f) * MOVEMENT_RANGE * 2.0f;
    rigid_body::set_position(*app.world, ball, vec3f::make(x, SPAWN_HEIGHT, 0.0f), true);
    rigid_body::set_linear_velocity(*app.world, ball, vec3f::zero());
    rigid_body::set_angular_velocity(*app.world, ball, vec3f::zero());
}

static void spawn_ball(App &app) {
    if (state.balls.length >= MAX_BALLS) {
        rigid_body::remove(*app.world, state.balls[0]);
        for (uint32_t i = 1; i < state.balls.length; i++) state.balls[i - 1] = state.balls[i];
        state.balls.length--;
    }
    rigid_body::RigidBodySettings settings;
    settings.shape = state.ball;
    settings.object_layer = state.layers.moving;
    settings.motion_type = motion_type::MOTION_DYNAMIC;
    settings.position = {0.0f, SPAWN_HEIGHT, 0.0f};
    settings.restitution = 0.6f;
    settings.friction = 0.4f;
    rigid_body::RigidBody *ball = rigid_body::create(*app.world, settings);
    reset_ball(app, ball);
    const float hue = random::next(state.random);
    const float saturation = 0.65f + random::next(state.random) * 0.25f;
    const float lightness = 0.55f + random::next(state.random) * 0.15f;
    app::set_color(app, ball, hsl(hue, saturation, lightness));
    state.balls.push(ball);
}

static void init(App &app) {
    state.balls.length = 0;
    state.time = 0.0f;
    state.steps = 0;
    state.random = {11};
    app.world = app::create_world(state.layers);

    rigid_body::RigidBodySettings ground;
    ground.shape = app::keep(app, box::create({.half_extents = {50.0f, 0.5f, 50.0f}}));
    ground.object_layer = state.layers.not_moving;
    ground.motion_type = motion_type::MOTION_STATIC;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.restitution = 0.2f;
    ground.friction = 0.5f;
    app::set_color(app, rigid_body::create(*app.world, ground), hex(0x666666));

    // maxwell the cat, as the hull of its body's vertices
    Model maxwell = model::load("maxwell.glb");
    collections::Vec<float> points = {};
    const uint32_t vertices = maxwell.positions.length / 3 < MAXWELL_BODY_VERTICES ? maxwell.positions.length / 3 : MAXWELL_BODY_VERTICES;
    for (uint32_t i = 0; i < vertices * 3; i++) points.push(maxwell.positions[i] * MAXWELL_SCALE);
    model::release(maxwell);
    convex_hull::ConvexHullData *hull = convex_hull::build({.positions = points.data, .num_positions = points.length / 3});
    points.release();
    if (hull != nullptr) {
        app::keep(app, hull);
        rigid_body::RigidBodySettings settings;
        settings.shape = app::keep(app, convex_hull::create({.data = hull}));
        settings.object_layer = state.layers.moving;
        settings.motion_type = motion_type::MOTION_KINEMATIC;
        settings.position = {0.0f, 3.0f, 0.0f};
        settings.restitution = 0.5f;
        settings.friction = 0.3f;
        state.maxwell = rigid_body::create(*app.world, settings);
        app::set_color(app, state.maxwell, hex(0x66ccff));
    } else {
        state.maxwell = nullptr;
    }

    state.ball = app::keep(app, sphere::create({.radius = 0.15f}));
    camera::look_at(app.camera, {0.0f, 5.0f, 10.0f}, {0.0f, 3.0f, 0.0f});
}

static void step(App &app, float delta_time) {
    // balls that rolled away start again from the top
    for (uint32_t i = 0; i < state.balls.length; i++) {
        const Float3 &position = state.balls[i]->position;
        if (position.x * position.x + position.z * position.z > RESPAWN_RADIUS * RESPAWN_RADIUS) reset_ball(app, state.balls[i]);
    }

    // side to side along x, spinning about y
    state.time += delta_time;
    if (state.maxwell != nullptr) {
        const Vec3f target = vec3f::make(scalar::sin(state.time * MOVEMENT_SPEED) * MOVEMENT_RANGE, 3.0f, 0.0f);
        const Quatf rotation = quatf::set_axis_angle(vec3f::axis_y(), state.time * ROTATION_SPEED);
        rigid_body::move_kinematic(*app.world, state.maxwell, target, rotation, delta_time);
    }

    if (++state.steps < SPAWN_STEPS) return;
    state.steps = 0;
    spawn_ball(app);
}

static void shutdown(App &) { state.balls.release(); }

static const Example EXAMPLE_KINEMATIC = {"Kinematic Body", init, step, nullptr, nullptr, shutdown, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_KINEMATIC);
