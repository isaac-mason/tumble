#include "app/app.h"
#include "app/examples.h"
#include "app/random.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

static constexpr int COUNT = 1000;
static constexpr float RAY_LENGTH = 100.0f;
static constexpr float PI = 3.14159265f;

/** a static body pulsing in and out along its direction from the centre, turned by hand each step */
struct Orbiter {
    rigid_body::RigidBody *body;
    Float3 radial_direction;
    float min_radius;
    float max_radius;
    float radial_speed;
    float radial_phase;
    Float3 angular_velocity;
};

static struct {
    app::Layers layers;
    Orbiter orbiters[COUNT];
    int steps;
    filter::Filter filter;
    cast_ray::CastRayCollector collector;
    bool hit;
    Float3 hit_point;
    Random random;
} state;

static Color hex(uint32_t rgb) { return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, 1.0f}; }

static void init(App &app) {
    state.steps = 0;
    state.hit = false;
    state.random = {11};
    app.world = app::create_world(state.layers);
    state.filter = filter::create(app.world->settings.layers);
    state.collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    // the bodies float around the origin: no ground
    app.drawing.grid = false;

    const float pyramid[] = {-0.8f, 0.0f, -0.8f, 0.8f, 0.0f, -0.8f, 0.8f, 0.0f, 0.8f, -0.8f, 0.0f, 0.8f, 0.0f, 1.2f, 0.0f};
    convex_hull::ConvexHullData *hull = app::keep(app, convex_hull::build({.positions = pyramid, .num_positions = 5}, nullptr));
    shapes::Shape *const kinds[] = {
        app::keep(app, box::create({.half_extents = {0.8f, 0.8f, 0.8f}})),
        app::keep(app, sphere::create({.radius = 0.8f})),
        app::keep(app, convex_hull::create({.data = hull})),
    };
    const uint32_t colors[] = {0xff6b9d, 0x4ecdc4, 0xffe66d, 0x95e1d3, 0xff6b6b, 0x6c5ce7, 0x00b894, 0xfdcb6e};

    const float golden_ratio = (1.0f + sqrtf(5.0f)) / 2.0f;
    for (int i = 0; i < COUNT; i++) {
        // a fibonacci sphere, in five shells
        const float theta = 2.0f * PI * (float)i / golden_ratio;
        const float phi = acosf(1.0f - 2.0f * ((float)i + 0.5f) / (float)COUNT);
        const float radius = 10.0f + (float)(i % 5) * 5.0f;
        const Float3 position = {radius * sinf(phi) * cosf(theta), radius * sinf(phi) * sinf(theta), radius * cosf(phi)};

        const Eulerf euler = {seeded::next(state.random) * 2.0f * PI, seeded::next(state.random) * 2.0f * PI,
                              seeded::next(state.random) * 2.0f * PI, EULER_XYZ};
        Orbiter &orbiter = state.orbiters[i];
        orbiter.angular_velocity = {(seeded::next(state.random) - 0.5f) * 0.5f, (seeded::next(state.random) - 0.5f) * 0.5f,
                                    (seeded::next(state.random) - 0.5f) * 0.5f};

        rigid_body::RigidBodySettings settings;
        settings.shape = kinds[i % 3];
        settings.object_layer = state.layers.moving;
        settings.motion_type = motion_type::MOTION_STATIC;
        settings.position = position;
        quatf::store(settings.quaternion, quatf::from_euler(euler));
        settings.restitution = 0.5f;
        settings.friction = 0.3f;
        orbiter.body = rigid_body::create(*app.world, settings);
        app::set_color(app, orbiter.body, hex(colors[i % 8]));

        orbiter.radial_direction = vec::normalize(position);
        orbiter.min_radius = vec::length(position) - 3.0f;
        orbiter.max_radius = vec::length(position) + 3.0f;
        orbiter.radial_speed = 0.2f + seeded::next(state.random) * 0.3f;
        orbiter.radial_phase = seeded::next(state.random) * 2.0f * PI;
    }

    camera::look_at(app.camera, {0.0f, 5.0f, 40.0f}, {0.0f, 0.0f, 0.0f});
}

static void step(App &app, float delta_time) {
    const float time = (float)++state.steps * delta_time;
    for (int i = 0; i < COUNT; i++) {
        Orbiter &orbiter = state.orbiters[i];
        const float t = time * orbiter.radial_speed + orbiter.radial_phase;
        const float radius = orbiter.min_radius + (sinf(t) + 1.0f) * 0.5f * (orbiter.max_radius - orbiter.min_radius);
        const Vec3f position = vec3f::scale(vec3f::load(orbiter.radial_direction), radius);

        Quatf quaternion = quatf::load(orbiter.body->quaternion);
        const Vec3f angular_velocity = vec3f::load(orbiter.angular_velocity);
        const float speed = vec3f::length(angular_velocity);
        if (speed > 1e-6f) {
            const Quatf turn = quatf::set_axis_angle(vec3f::scale(angular_velocity, 1.0f / speed), speed * delta_time);
            quaternion = quatf::normalize(quatf::multiply(turn, quaternion));
        }
        rigid_body::set_transform(*app.world, orbiter.body, position, quaternion, false);
    }
}

static void update(App &app, float) {
    // crashcat's pointer starts at the middle of the screen
    const bool moved = app.pointer.x != 0.0f || app.pointer.y != 0.0f;
    const Ray pointer = moved ? app.pointer.ray : camera::screen_ray(app.camera, 0.5f * app.camera.width, 0.5f * app.camera.height);
    const cast_ray::Rayf ray = {vec3f::load(pointer.origin), vec3f::load(pointer.direction), RAY_LENGTH};
    cast_ray::reset(state.collector);
    query::cast_ray(*app.world, state.collector, cast_ray::default_settings(), ray, state.filter);
    const cast_ray::CastRayHit *hit = cast_ray::hit(state.collector);
    state.hit = hit != nullptr;
    if (hit) state.hit_point = vec::scale_and_add(pointer.origin, pointer.direction, hit->fraction * RAY_LENGTH);
    if (state.hit) renderer::point(app::frame(app), state.hit_point, 0.3f, {1.0f, 0.0f, 0.0f, 1.0f});
}

static void ui(App &) {
    if (state.hit) ImGui::Text("hit at %.2f %.2f %.2f", state.hit_point.x, state.hit_point.y, state.hit_point.z);
    else ImGui::TextUnformatted("no hit");
}

static void shutdown(App &) {
    cast_ray::release(state.collector);
    filter::release(state.filter);
}

static const Example EXAMPLE_POINTER_RAY = {"Pointer Raycast", init, step, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_POINTER_RAY);
