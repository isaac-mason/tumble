#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

static constexpr float MAX_DISTANCE = 100.0f;
/** how long a hit stays drawn, s */
static constexpr float HIT_SHOWN_FOR = 0.2f;

static struct {
    app::Layers layers;
    CastRayCollector collector;
    Filter filter;
    float impulse_strength;
    Float3 hit_position;
    Float3 hit_direction;
    float hit_shown_for;
} state;

static RigidBody *add_body(App &app, Shape *shape, Float3 position, bool dynamic) {
    RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = dynamic ? state.layers.moving : state.layers.not_moving;
    settings.motion_type = dynamic ? MotionType::DYNAMIC : MotionType::STATIC;
    settings.position = position;
    settings.restitution = 0.3f;
    settings.friction = 0.5f;
    if (dynamic) settings.mass = 10.0f;
    return rigid_body::create(*app.world, settings);
}

static void init(App &app) {
    state.impulse_strength = 20.0f;
    state.hit_shown_for = 0.0f;
    app.world = app::create_world(state.layers);
    add_body(app, app::keep(app, box::create({.half_extents = {30.0f, 0.5f, 30.0f}})), {0.0f, -0.5f, 0.0f}, false);
    add_body(app, app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}, .convex_radius = 0.05f})), {-3.0f, 2.0f, 0.0f}, true);
    add_body(app, app::keep(app, sphere::create({.radius = 1.0f})), {3.0f, 2.0f, 0.0f}, true);
    state.collector = cast_ray::create_collector(CollectorKind::CLOSEST);
    state.filter = filter::create(app.world->settings.layers);
    camera::look_at(app.camera, {0.0f, 10.0f, 15.0f}, {0.0f, 2.0f, 0.0f});
}

static void update(App &app, float delta_time) {
    state.hit_shown_for -= delta_time;
    if (state.hit_shown_for > 0.0f) {
        renderer::point(app::frame(app), state.hit_position, 0.1f, {1.0f, 0.0f, 0.0f, 1.0f});
        renderer::arrow(app::frame(app), state.hit_position, vec::scale_and_add(state.hit_position, state.hit_direction, 2.0f), {0.0f, 1.0f, 0.0f, 1.0f});
    }

    if (!app.click.clicked || app.click.secondary) return;
    const Ray &ray = app.click.ray;
    cast_ray::reset(state.collector);
    query::cast_ray(*app.world, state.collector, cast_ray::default_settings(),
                    {vec3f::load(ray.origin), vec3f::load(ray.direction), MAX_DISTANCE}, state.filter);
    const CastRayHit *hit = cast_ray::hit(state.collector);
    if (hit == nullptr) return;
    RigidBody *body = rigid_body::get(*app.world, hit->body_id_b);
    if (body == nullptr || body->motion_type != MotionType::DYNAMIC) return;

    // pushed along the ray, where it hit
    const Float3 position = vec::scale_and_add(ray.origin, ray.direction, hit->fraction * MAX_DISTANCE);
    rigid_body::add_impulse_at_position(*app.world, body, vec3f::scale(vec3f::load(ray.direction), state.impulse_strength), vec3f::load(position));
    state.hit_position = position;
    state.hit_direction = ray.direction;
    state.hit_shown_for = HIT_SHOWN_FOR;
}

static void ui(App &) {
    ImGui::SliderFloat("impulse strength", &state.impulse_strength, 1.0f, 100.0f, "%.0f");
    ImGui::TextUnformatted("click a body to push it where you clicked");
}

static void shutdown(App &) {
    state.collector = {};
    state.filter = {};
}

static const Example EXAMPLE_ADD_IMPULSE_AT_POSITION = {"Add Impulse at Position", init, nullptr, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_ADD_IMPULSE_AT_POSITION);
