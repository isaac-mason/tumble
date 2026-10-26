#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "imgui.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/update.h"

using namespace tumble;

static constexpr uint32_t GROUP_GROUND = 1u << 0;
static constexpr uint32_t GROUP_BODY = 1u << 1;

static constexpr float PART_DEPTH = 0.3f;
static constexpr float BODY_WIDTH = 10.0f;
static constexpr float BODY_HEIGHT = 2.0f;
static constexpr float LEG_LENGTH = 6.0f;
/** the chassis' thickness in z, between the two leg sets */
static constexpr float CHASSIS_DEPTH = 6.0f;

static struct {
    app::Layers layers;
    RigidBody *chassis;
    ConstraintId crank_hinges[2];
    int num_crank_hinges;
    float motor_speed;
} state;

static Float4 quat_z(float angle) { return {0.0f, 0.0f, scalar::sin(0.5f * angle), scalar::cos(0.5f * angle)}; }

static void update_motor_targets(App &app) {
    for (int i = 0; i < state.num_crank_hinges; i++) {
        hinge_constraint::set_target_angular_velocity(*hinge_constraint::get(*app.world, state.crank_hinges[i]), -state.motor_speed);
    }
}

static RigidBody *add_part(App &app, Float3 full_extents, Float3 position, float angle) {
    RigidBodySettings settings;
    settings.shape =
        app::keep(app, box::create({.half_extents = {0.5f * full_extents.x, 0.5f * full_extents.y, 0.5f * full_extents.z}, .convex_radius = 0.02f}));
    settings.object_layer = state.layers.moving;
    settings.motion_type = MotionType::DYNAMIC;
    settings.position = position;
    settings.quaternion = quat_z(angle);
    settings.linear_damping = 0.4f;
    settings.friction = 0.5f;
    settings.collision_groups = GROUP_BODY;
    settings.collision_mask = GROUP_GROUND;
    settings.allow_sleeping = false;
    return rigid_body::create(*app.world, settings);
}

/** a hinge about z between points local to each body */
static ConstraintId hinge(App &app, RigidBody *a, RigidBody *b, Float3 point_a, Float3 point_b,
                                         MotorSettings motor = {}) {
    return hinge_constraint::create(*app.world, a, b,
                                    {.point_a = point_a,
                                     .point_b = point_b,
                                     .hinge_axis_a = {0.0f, 0.0f, 1.0f},
                                     .hinge_axis_b = {0.0f, 0.0f, 1.0f},
                                     .normal_axis_a = {1.0f, 0.0f, 0.0f},
                                     .normal_axis_b = {1.0f, 0.0f, 0.0f},
                                     .space = ConstraintSpace::LOCAL,
                                     .motor_settings = motor});
}

/** a planar Theo Jansen linkage on one side of the chassis (side +1 at +z, -1 at -z), its crank driven by a hinge motor */
static void make_leg_set(App &app, float side, float crank_phase) {
    const float leg_z = side * (0.5f * CHASSIS_DEPTH + 0.5f * PART_DEPTH);
    const float body_pivot_z = side * 0.5f * CHASSIS_DEPTH;

    const Float3 upper_front_leg_full = {1.0f, 3.0f, PART_DEPTH};
    const Float3 crank_full = {0.5f, 1.0f, PART_DEPTH};
    const Float3 front_leg_full = {1.0f, LEG_LENGTH, PART_DEPTH};
    const Float3 horizontal_bar_full = {1.0f, BODY_WIDTH, PART_DEPTH};
    const Float3 back_leg_full = {1.0f, LEG_LENGTH, PART_DEPTH};

    RigidBody *upper_front_leg = add_part(app, upper_front_leg_full, {-2.0f, 0.5f, leg_z}, scalar::PI / 3.0f);
    hinge(app, upper_front_leg, state.chassis, {0.0f, -0.5f * upper_front_leg_full.y, 0.0f}, {0.0f, BODY_WIDTH * 0.2f, body_pivot_z});

    RigidBody *crank = add_part(app, crank_full, {BODY_WIDTH * -0.5f, -1.5f / 2.0f, leg_z}, crank_phase);
    state.crank_hinges[state.num_crank_hinges++] = hinge(app, crank, state.chassis, {0.0f, 0.5f * crank_full.y, 0.0f}, {0.0f, BODY_WIDTH * 0.5f, body_pivot_z},
                                                         {.min_torque_limit = -1e6f, .max_torque_limit = 1e6f});
    hinge_constraint::set_motor_state(*hinge_constraint::get(*app.world, state.crank_hinges[state.num_crank_hinges - 1]), MotorState::VELOCITY);
    update_motor_targets(app);

    RigidBody *front_leg = add_part(app, front_leg_full, {BODY_WIDTH * -0.5f, -1.0f, leg_z}, scalar::PI / -6.0f);
    hinge(app, front_leg, crank, {0.0f, 0.0f, 0.0f}, {0.0f, -0.5f * crank_full.y, 0.0f});

    RigidBody *horizontal_bar = add_part(app, horizontal_bar_full, {0.0f, 0.0f, leg_z}, scalar::PI / -2.5f);
    hinge(app, horizontal_bar, front_leg, {0.0f, -0.5f * horizontal_bar_full.y, 0.0f}, {0.0f, 0.0f, 0.0f});

    RigidBody *back_leg = add_part(app, back_leg_full, {BODY_WIDTH * 0.5f, 0.0f, leg_z}, scalar::PI / 4.0f);
    hinge(app, back_leg, state.chassis, {0.0f, 0.0f, 0.0f}, {0.0f, -BODY_WIDTH * 0.5f, body_pivot_z});

    // closing the linkage's loops
    hinge(app, upper_front_leg, front_leg, {0.0f, 1.5f, 0.0f}, {0.0f, 2.5f, 0.0f});
    hinge(app, back_leg, horizontal_bar, {0.0f, 2.5f, 0.0f}, {0.0f, 5.0f, 0.0f});
}

static void init(App &app) {
    state.num_crank_hinges = 0;
    state.motor_speed = 7.0f;
    app.world = app::create_world(state.layers, {0.0f, -40.0f, 0.0f});

    RigidBodySettings ground;
    ground.shape = app::keep(app, box::create({.half_extents = {5000.0f, 0.5f, 5000.0f}}));
    ground.object_layer = state.layers.not_moving;
    ground.motion_type = MotionType::STATIC;
    ground.position = {0.0f, -5.5f, 0.0f};
    ground.restitution = 0.0f;
    ground.friction = 0.5f;
    ground.collision_groups = GROUP_GROUND;
    ground.collision_mask = GROUP_BODY | GROUP_GROUND;
    rigid_body::create(*app.world, ground);

    // turned a quarter about z: its height along world y, its length along world x
    RigidBodySettings chassis;
    chassis.shape =
        app::keep(app, box::create({.half_extents = {0.5f * BODY_HEIGHT, 0.5f * BODY_WIDTH, 0.5f * CHASSIS_DEPTH}, .convex_radius = 0.05f, .density = 1.0f}));
    chassis.object_layer = state.layers.moving;
    chassis.motion_type = MotionType::DYNAMIC;
    chassis.quaternion = quat_z(0.5f * scalar::PI);
    chassis.linear_damping = 0.4f;
    chassis.friction = 0.5f;
    chassis.collision_groups = GROUP_BODY;
    chassis.collision_mask = GROUP_GROUND;
    chassis.allow_sleeping = false;
    state.chassis = rigid_body::create(*app.world, chassis);

    make_leg_set(app, 1.0f, 0.0f);
    make_leg_set(app, -1.0f, scalar::PI);

    camera::look_at(app.camera, {-40.0f, 10.0f, 20.0f}, {0.0f, 0.0f, 0.0f});
    // the ground is lower than the grid
    app.drawing.grid = false;
}

static void follow_robot(App &app, float) {
    // the camera stays where it is and turns to follow the robot
    camera::look_at(app.camera, app.camera.eye, state.chassis->center_of_mass_position);
}

static void ui(App &app) {
    if (ImGui::SliderFloat("motor speed", &state.motor_speed, 1.0f, 15.0f, "%.1f")) {
        state.motor_speed = roundf(state.motor_speed * 2.0f) * 0.5f;
        update_motor_targets(app);
    }
}

/** crashcat's listener: bodies held by a constraint between them do not collide */
static void step_world(App &app, float delta_time) {
    World &world = *app.world;
    update::step_begin(world, delta_time);
    do {
        update::find_pairs(world);
        for (uint32_t i = 0; i < world.events.pairs.length; i++) {
            const PairEvent &pair = world.events.pairs[i];
            if (rigid_body::bodies_share_constraint(world.bodies.pool[pair.body_index_a], world.bodies.pool[pair.body_index_b])) {
                events::reject_pair(world.answers, i);
            }
        }
        update::narrowphase(world);
    } while (world.events.woken > 0);
    update::solve(world);
    update::step_end(world);
}

static const Example EXAMPLE_HINGE_MOTOR = {"Hinge Motor", init, nullptr, follow_robot, ui, nullptr, step_world};
static const examples::Registration REGISTERED(EXAMPLE_HINGE_MOTOR);
