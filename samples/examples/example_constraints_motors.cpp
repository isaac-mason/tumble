#include "app/app.h"
#include "app/examples.h"
#include "imgui.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/constraints/slider_constraint.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/update.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

static constexpr int SLIDER_OFFSET_COUNT = 4;
static const float SLIDER_OFFSETS[SLIDER_OFFSET_COUNT] = {0.0f, 4.0f, 13.0f, 21.0f};
/** the slider's target moves every this many steps, a second at 60 */
static constexpr int SLIDER_STEPS = 60;

static struct {
    app::Layers layers;
    constraint_id::ConstraintId slider;
    rigid_body::RigidBody *slider_box;
    int steps;
    int counter;
} state;

static const Float4 IDENTITY = {0.0f, 0.0f, 0.0f, 1.0f};

/** text centred on a world point, over the scene */
static rigid_body::RigidBody *add_static_box(App &app, Float3 half_extents, Float3 position) {
    rigid_body::RigidBodySettings settings;
    settings.shape = app::keep(app, box::create({.half_extents = half_extents}));
    settings.object_layer = state.layers.not_moving;
    settings.motion_type = motion_type::MOTION_STATIC;
    settings.position = position;
    settings.restitution = 0.0f;
    settings.friction = 0.5f;
    return rigid_body::create(*app.world, settings);
}

/** a windmill turned by a velocity motor on its hinge, and a ball rolled into it */
static void make_windmill(App &app) {
    const Float3 center = {-20.0f, 10.0f, 0.0f};
    rigid_body::RigidBody *hub = add_static_box(app, {0.25f, 0.25f, 0.25f}, center);

    const float wing_half_extent = 4.0f;
    const float wing_thickness = 0.5f;
    const float wing_offset = 5.0f;
    shapes::Shape *across = app::keep(app, box::create({.half_extents = {wing_half_extent, wing_thickness, wing_thickness}, .convex_radius = 0.05f}));
    shapes::Shape *upright = app::keep(app, box::create({.half_extents = {wing_thickness, wing_half_extent, wing_thickness}, .convex_radius = 0.05f}));
    const compound::CompoundShapeChild wings[] = {
        {.position = {wing_offset, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = across},
        {.position = {-wing_offset, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = across},
        {.position = {0.0f, wing_offset, 0.0f}, .quaternion = IDENTITY, .shape = upright},
        {.position = {0.0f, -wing_offset, 0.0f}, .quaternion = IDENTITY, .shape = upright},
    };
    rigid_body::RigidBodySettings settings;
    settings.shape = app::keep(app, compound::create({.children = wings, .num_children = 4}));
    settings.object_layer = state.layers.moving;
    settings.motion_type = motion_type::MOTION_DYNAMIC;
    settings.position = center;
    settings.restitution = 0.0f;
    settings.friction = 0.5f;
    rigid_body::RigidBody *rotor = rigid_body::create(*app.world, settings);

    const constraint_id::ConstraintId hinge_id = hinge_constraint::create(*app.world, hub, rotor,
                                                                          {.point_a = center,
                                                                           .point_b = center,
                                                                           .hinge_axis_a = {0.0f, 0.0f, 1.0f},
                                                                           .hinge_axis_b = {0.0f, 0.0f, 1.0f},
                                                                           .normal_axis_a = {1.0f, 0.0f, 0.0f},
                                                                           .normal_axis_b = {1.0f, 0.0f, 0.0f}});
    hinge_constraint::HingeConstraint *hinge = hinge_constraint::get(*app.world, hinge_id);
    hinge_constraint::set_motor_state(*hinge, motor_settings::VELOCITY);
    hinge_constraint::set_target_angular_velocity(*hinge, -5.0f);

    rigid_body::RigidBodySettings ball;
    ball.shape = app::keep(app, sphere::create({.radius = 2.0f}));
    ball.object_layer = state.layers.moving;
    ball.motion_type = motion_type::MOTION_DYNAMIC;
    ball.position = {center.x - 20.0f, 2.0f, 0.0f};
    ball.restitution = 0.45f;
    ball.friction = 0.5f;
    rigid_body::RigidBody *ball_body = rigid_body::create(*app.world, ball);
    rigid_body::set_linear_velocity(*app.world, ball_body, vec3f::make(25.0f, 0.0f, 0.0f));

    // a wall behind the ball, and walls right and above keeping it in
    add_static_box(app, {0.2f, 15.0f, 5.0f}, {center.x - 25.0f, 15.0f, 0.0f});
    add_static_box(app, {0.2f, 150.0f, 50.0f}, {50.0f, 0.0f, 0.0f});
    add_static_box(app, {150.0f, 0.2f, 50.0f}, {0.0f, 75.0f, 0.0f});
}

/** a box on a platform, driven along it by a position motor to each marker in turn */
static void make_slider(App &app) {
    add_static_box(app, {12.5f, 0.25f, 2.0f}, {18.0f, 0.25f, 0.0f});
    rigid_body::RigidBody *target = add_static_box(app, {0.5f, 2.0f, 2.0f}, {30.0f, 2.0f, 0.0f});

    rigid_body::RigidBodySettings settings;
    settings.shape = app::keep(app, box::create({.half_extents = {0.75f, 0.75f, 0.75f}}));
    settings.object_layer = state.layers.moving;
    settings.motion_type = motion_type::MOTION_DYNAMIC;
    settings.position = {15.0f, 1.25f, 0.0f};
    settings.restitution = 0.0f;
    settings.friction = 0.5f;
    settings.allow_sleeping = false;
    state.slider_box = rigid_body::create(*app.world, settings);

    shapes::Shape *marker = app::keep(app, box::create({.half_extents = {1.0f, 4.0f, 0.75f}}));
    for (int i = 0; i < SLIDER_OFFSET_COUNT; i++) {
        rigid_body::RigidBodySettings marker_settings;
        marker_settings.shape = marker;
        marker_settings.object_layer = state.layers.not_moving;
        marker_settings.motion_type = motion_type::MOTION_STATIC;
        marker_settings.position = {28.0f - SLIDER_OFFSETS[i], 2.0f, -2.75f};
        marker_settings.restitution = 0.0f;
        marker_settings.friction = 0.5f;
        rigid_body::create(*app.world, marker_settings);
    }

    state.slider = slider_constraint::create(*app.world, state.slider_box, target,
                                             {.point_a = {15.0f, 1.25f, 0.0f},
                                              .point_b = {28.0f, 1.25f, 0.0f},
                                              .slider_axis_a = {1.0f, 0.0f, 0.0f},
                                              .slider_axis_b = {1.0f, 0.0f, 0.0f},
                                              .normal_axis_a = {0.0f, 1.0f, 0.0f},
                                              .normal_axis_b = {0.0f, 1.0f, 0.0f}});
    slider_constraint::SliderConstraint *slider = slider_constraint::get(*app.world, state.slider);
    slider_constraint::set_motor_state(*slider, motor_settings::POSITION);
    slider_constraint::set_target_position(*slider, 0.0f);
}

static void init(App &app) {
    state.steps = 0;
    // starts near the box's spawn point
    state.counter = 4;
    app.world = app::create_world(state.layers);
    add_static_box(app, {200.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    make_windmill(app);
    make_slider(app);
    camera::look_at(app.camera, {0.0f, 15.0f, 35.0f}, {0.0f, 5.0f, 0.0f});
}

static void step(App &app, float) {
    if (++state.steps < SLIDER_STEPS) return;
    state.steps = 0;
    // up and down the markers: 0 1 2 3 2 1 0 ...
    const int period = SLIDER_OFFSET_COUNT * 2 - 2;
    int index = state.counter++ % period - (SLIDER_OFFSET_COUNT - 1);
    if (index < 0) index = -index;
    slider_constraint::set_target_position(*slider_constraint::get(*app.world, state.slider), SLIDER_OFFSETS[index]);
    rigid_body::wake(*app.world, state.slider_box);
}

static void ui(App &app) {
    app::label(app, {-20.0f, 22.0f, 0.0f}, "Powered Hinge", {8.0f, 2.0f, 0.375f});
    app::label(app, {18.0f, 10.0f, 0.0f}, "Powered Slider", {8.0f, 2.0f, 0.375f});
}

/** crashcat's listener: bodies held by a constraint between them do not collide */
static void step_world(App &app, float delta_time) {
    world::World &world = *app.world;
    update::step_begin(world, delta_time);
    do {
        update::find_pairs(world);
        for (uint32_t i = 0; i < world.events.pairs.length; i++) {
            const events::PairEvent &pair = world.events.pairs[i];
            if (rigid_body::bodies_share_constraint(world.bodies.pool[pair.body_index_a], world.bodies.pool[pair.body_index_b])) {
                events::reject_pair(world.answers, i);
            }
        }
        update::narrowphase(world);
    } while (world.events.woken > 0);
    update::solve(world);
    update::step_end(world);
}

static const Example EXAMPLE_CONSTRAINTS_MOTORS = {"Constraint Motors", init, step, nullptr, ui, nullptr, step_world};
static const examples::Registration REGISTERED(EXAMPLE_CONSTRAINTS_MOTORS);
