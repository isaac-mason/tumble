#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "physics/constraints/constraint_test_scene.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "test.h"

using hinge_constraint::HingeConstraint;
using rigid_body::RigidBody;

static const Float3 GRAVITY = {0.0f, -9.81f, 0.0f};
static const Float3 ZERO = {0.0f, 0.0f, 0.0f};
static const float DT = 1.0f / 60.0f;
/** the scene's sphere: 2/5 m r^2 with m = 1000 * 4/3 pi 0.125 and r = 0.5 */
static const float SPHERE_INERTIA = 0.4f * (1000.0f * 4.0f / 3.0f * scalar::PI * 0.125f) * 0.25f;

/** a ball on a static base, both centered on the pivot, hinged about world z */
static HingeConstraint *make_spinner(ConstraintScene &scene, RigidBody *&ball, const hinge_constraint::HingeConstraintSettings &extra) {
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, ZERO);
    ball = add_body(scene, motion_type::MOTION_DYNAMIC, ZERO);
    hinge_constraint::HingeConstraintSettings settings = extra;
    settings.hinge_axis_a = {0.0f, 0.0f, 1.0f};
    settings.hinge_axis_b = {0.0f, 0.0f, 1.0f};
    settings.normal_axis_a = {1.0f, 0.0f, 0.0f};
    settings.normal_axis_b = {1.0f, 0.0f, 0.0f};
    const constraint_id::ConstraintId id = hinge_constraint::create(scene.constraints, scene.bodies, base, ball, settings);
    scene.ids.push(id);
    return hinge_constraint::get(scene.constraints, id);
}

void test_hinge_constraint_create(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI / 2.0f));
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {2.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::make(1.0f, 1.0f, 0.0f), 0.6f));

    // world space: axes normalized and taken to body space, positions relative to the centers of mass
    const constraint_id::ConstraintId id = hinge_constraint::create(scene.constraints, scene.bodies, a, b,
                                                                    {.point_a = {1.0f, 0.0f, 0.0f},
                                                                     .point_b = {1.0f, 0.0f, 0.0f},
                                                                     .hinge_axis_a = {0.0f, 0.0f, 3.0f},
                                                                     .hinge_axis_b = {0.0f, 0.0f, 3.0f},
                                                                     .normal_axis_a = {2.0f, 0.0f, 0.0f},
                                                                     .normal_axis_b = {2.0f, 0.0f, 0.0f},
                                                                     .limits_min = -scalar::PI,
                                                                     .limits_max = 1.0f});
    HingeConstraint *constraint = hinge_constraint::get(scene.constraints, id);
    CHECK(ctx, constraint != nullptr && constraint_id::type(id) == constraint_id::HINGE);
    CHECK(ctx, a->constraint_ids.length == 1 && b->constraint_ids.length == 1);
    // a is turned a quarter about z: world +x is its body -y, world z stays z
    CHECK(ctx, vec3f::distance(constraint->local_space_normal_axis1, vec3f::make(0.0f, -1.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(constraint->local_space_hinge_axis1, vec3f::make(0.0f, 0.0f, 1.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(world_point(a, constraint->local_space_position1), vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(world_point(b, constraint->local_space_position2), vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);
    // a lower limit of -pi with an upper limit below pi keeps the hinge limited
    CHECK(ctx, constraint->has_limits);
    CHECK(ctx, constraint->limits_min == -scalar::PI && constraint->limits_max == 1.0f);

    // the rest pose is angle 0, and turning b about the world hinge axis turns the angle by as much
    hinge_constraint::setup_velocity(*constraint, scene.bodies, DT);
    CHECK_NEAR(ctx, hinge_constraint::get_current_angle(*constraint), 0.0f, 1e-5f);
    rotate_body(b, axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.3f));
    hinge_constraint::setup_velocity(*constraint, scene.bodies, DT);
    CHECK_NEAR(ctx, hinge_constraint::get_current_angle(*constraint), 0.3f, 1e-5f);
    CHECK(ctx, vec3f::distance(constraint->world_space_hinge_axis1, vec3f::make(0.0f, 0.0f, 1.0f)) < 1e-5f);

    // the full circle is no limits
    const constraint_id::ConstraintId free_id = hinge_constraint::create(scene.constraints, scene.bodies, a, b, {});
    const HingeConstraint *free_hinge = hinge_constraint::get(scene.constraints, free_id);
    CHECK(ctx, !free_hinge->has_limits && free_hinge->limits_min == -scalar::PI && free_hinge->limits_max == scalar::PI);
    CHECK(ctx, free_hinge->motor_state == motor_settings::OFF);

    // local space: as given, and the angle of unrotated bodies with matching axes is 0
    RigidBody *c = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *d = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 1.0f, 0.0f});
    const constraint_id::ConstraintId local_id =
        hinge_constraint::create(scene.constraints, scene.bodies, c, d,
                                 {.point_a = {0.0f, 0.5f, 0.0f},
                                  .point_b = {0.0f, -0.5f, 0.0f},
                                  .hinge_axis_a = {1.0f, 0.0f, 0.0f},
                                  .hinge_axis_b = {1.0f, 0.0f, 0.0f},
                                  .normal_axis_a = {0.0f, 1.0f, 0.0f},
                                  .normal_axis_b = {0.0f, 1.0f, 0.0f},
                                  .space = constraints::SPACE_LOCAL,
                                  .limits_min = -1.0f,
                                  .limits_max = 1.0f});
    HingeConstraint *local = hinge_constraint::get(scene.constraints, local_id);
    CHECK(ctx, vec3f::exact_equals(local->local_space_position1, vec3f::make(0.0f, 0.5f, 0.0f)));
    CHECK(ctx, vec3f::exact_equals(local->local_space_hinge_axis2, vec3f::make(1.0f, 0.0f, 0.0f)));
    CHECK(ctx, quatf::exact_equals(local->inv_initial_orientation, quatf::make(0.0f, 0.0f, 0.0f, 1.0f)));
    hinge_constraint::setup_velocity(*local, scene.bodies, DT);
    CHECK(ctx, scalar::abs(hinge_constraint::get_current_angle(*local)) < 1e-6f);
    d->quaternion = stored_axis_angle(vec3f::make(1.0f, 0.0f, 0.0f), -0.4f);
    hinge_constraint::setup_velocity(*local, scene.bodies, DT);
    CHECK_NEAR(ctx, hinge_constraint::get_current_angle(*local), -0.4f, 1e-5f);

    // another type's id is not a hinge
    CHECK(ctx, hinge_constraint::get(scene.constraints, constraint_id::ser(0, constraint_id::SLIDER, 0)) == nullptr);
    release_constraint_scene(scene);
}

void test_hinge_constraint_setters(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *ball = nullptr;
    HingeConstraint *constraint = make_spinner(scene, ball, {});
    CHECK(ctx, !constraint->has_limits);

    // without limits the target angle is free
    hinge_constraint::set_target_angle(*constraint, 2.5f);
    CHECK(ctx, constraint->target_angle == 2.5f);

    // limits are stored as given; set_target_angle clamps to them, set_limits leaves the target alone
    hinge_constraint::set_limits(*constraint, -0.2f, scalar::PI);
    CHECK(ctx, constraint->has_limits);
    CHECK(ctx, constraint->limits_min == -0.2f && constraint->limits_max == scalar::PI);
    hinge_constraint::set_target_angle(*constraint, -1.0f);
    CHECK(ctx, constraint->target_angle == -0.2f);
    hinge_constraint::set_limits(*constraint, -0.1f, 0.1f);
    CHECK(ctx, constraint->target_angle == -0.2f);
    hinge_constraint::set_target_angle(*constraint, -1.0f);
    CHECK(ctx, constraint->target_angle == -0.1f);
    hinge_constraint::set_limits(*constraint, -scalar::PI, scalar::PI);
    CHECK(ctx, !constraint->has_limits);

    hinge_constraint::set_motor_state(*constraint, motor_settings::VELOCITY);
    hinge_constraint::set_target_angular_velocity(*constraint, 1.5f);
    CHECK(ctx, constraint->motor_state == motor_settings::VELOCITY && constraint->target_angular_velocity == 1.5f);
    release_constraint_scene(scene);
}

void test_hinge_constraint_holds_axis(TestCtx &ctx) {
    // a pendulum on a hinge about z, kicked out of its plane and spun about the other axes
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *pivot = add_body(scene, motion_type::MOTION_STATIC, ZERO);
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    const constraint_id::ConstraintId id = hinge_constraint::create(
        scene.constraints, scene.bodies, pivot, ball,
        {.hinge_axis_a = {0.0f, 0.0f, 1.0f}, .hinge_axis_b = {0.0f, 0.0f, 1.0f}, .normal_axis_a = {1.0f, 0.0f, 0.0f},
         .normal_axis_b = {1.0f, 0.0f, 0.0f}});
    scene.ids.push(id);
    const HingeConstraint *constraint = hinge_constraint::get(scene.constraints, id);
    ball->motion_properties.linear_velocity = {0.0f, 0.0f, 2.0f};
    ball->motion_properties.angular_velocity = {2.0f, 3.0f, 0.0f};

    float worst_axis = 0.0f;
    float worst_anchor = 0.0f;
    for (int i = 0; i < 180; i++) {
        step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
        const Vec3f axis2 = vec3f::transform_quat(constraint->local_space_hinge_axis2, quatf::load(ball->quaternion));
        worst_axis = scalar::max(worst_axis, vec3f::distance(axis2, vec3f::make(0.0f, 0.0f, 1.0f)));
        worst_anchor = scalar::max(worst_anchor, vec3f::length(world_point(ball, constraint->local_space_position2)));
    }
    // the axes stay aligned and the anchor on the pivot, after the first step's correction
    CHECK(ctx, worst_axis < 2e-2f);
    CHECK(ctx, worst_anchor < 2e-2f);
    // it swings in the plane: no motion along z, and it turns about z only
    CHECK(ctx, scalar::abs(ball->center_of_mass_position.z) < 1e-2f);
    CHECK(ctx, scalar::abs(ball->motion_properties.linear_velocity.z) < 1e-2f);
    CHECK(ctx, scalar::abs(ball->motion_properties.angular_velocity.x) < 1e-2f);
    CHECK(ctx, scalar::abs(ball->motion_properties.angular_velocity.y) < 1e-2f);
    CHECK_NEAR(ctx, vec3f::length(vec3f::load(ball->center_of_mass_position)), 1.0f, 1e-2f);
    // it fell: the pendulum is moving
    CHECK(ctx, ball->center_of_mass_position.y < -0.1f);
    release_constraint_scene(scene);
}

void test_hinge_constraint_limits(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *ball = nullptr;
    HingeConstraint *constraint = make_spinner(scene, ball, {.limits_min = -0.5f, .limits_max = 0.5f});
    ball->motion_properties.angular_velocity = {0.0f, 0.0f, 3.0f};

    float widest = 0.0f;
    for (int i = 0; i < 90; i++) {
        step_constraint_scene(scene, DT, vec3f::zero());
        widest = scalar::max(widest, hinge_constraint::get_current_angle(*constraint));
    }
    // it stops at the upper limit, overshooting by at most one step's turn since the limit engages once crossed
    CHECK(ctx, widest < 0.5f + 3.0f * DT);
    CHECK_NEAR(ctx, hinge_constraint::get_current_angle(*constraint), 0.5f, 1e-2f);
    CHECK(ctx, scalar::abs(ball->motion_properties.angular_velocity.z) < 1e-2f);
    // at the max limit the impulse only pushes back
    CHECK(ctx, constraint->rotation_limits_constraint_part.total_lambda <= 0.0f);

    // and the other way to the lower limit
    ball->motion_properties.angular_velocity = {0.0f, 0.0f, -3.0f};
    float lowest = 0.0f;
    for (int i = 0; i < 90; i++) {
        step_constraint_scene(scene, DT, vec3f::zero());
        lowest = scalar::min(lowest, hinge_constraint::get_current_angle(*constraint));
    }
    CHECK(ctx, lowest > -0.5f - 3.0f * DT);
    CHECK_NEAR(ctx, hinge_constraint::get_current_angle(*constraint), -0.5f, 1e-2f);
    CHECK(ctx, constraint->rotation_limits_constraint_part.total_lambda >= 0.0f);
    release_constraint_scene(scene);
}

void test_hinge_constraint_velocity_motor(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *ball = nullptr;
    HingeConstraint *constraint = make_spinner(scene, ball, {});
    hinge_constraint::set_motor_state(*constraint, motor_settings::VELOCITY);
    hinge_constraint::set_target_angular_velocity(*constraint, 2.0f);

    // an unlimited motor reaches its target speed in a step and holds it
    step_constraint_scene(scene, DT, vec3f::zero());
    CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.z, 2.0f, 1e-3f);
    for (int i = 0; i < 59; i++) step_constraint_scene(scene, DT, vec3f::zero());
    CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.z, 2.0f, 1e-4f);
    CHECK(ctx, scalar::abs(ball->motion_properties.angular_velocity.x) < 1e-4f);
    // it turned 2 radians in the second
    const Vec3f normal2 = vec3f::transform_quat(vec3f::axis_x(), quatf::load(ball->quaternion));
    CHECK_NEAR(ctx, scalar::atan2(vec3f::y(normal2), vec3f::x(normal2)), 2.0f, 1e-2f);

    // a torque limit of I rad/s^2 accelerates it from rest at 1 rad/s^2
    ball->motion_properties.angular_velocity = ZERO;
    hinge_constraint::reset_warm_start(*constraint);
    constraint->motor_settings.min_torque_limit = -SPHERE_INERTIA;
    constraint->motor_settings.max_torque_limit = SPHERE_INERTIA;
    for (int i = 0; i < 60; i++) step_constraint_scene(scene, DT, vec3f::zero());
    CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.z, 1.0f, 1e-3f);
    release_constraint_scene(scene);
}

void test_hinge_constraint_position_motor(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *ball = nullptr;
    HingeConstraint *constraint = make_spinner(scene, ball, {.limits_min = -1.0f, .limits_max = 1.0f});
    hinge_constraint::set_motor_state(*constraint, motor_settings::POSITION);
    // clamped to the limits
    hinge_constraint::set_target_angle(*constraint, 2.0f);
    CHECK(ctx, constraint->target_angle == 1.0f);
    hinge_constraint::set_target_angle(*constraint, 0.8f);

    // the default 2 hz critically damped spring settles on the target
    for (int i = 0; i < 180; i++) step_constraint_scene(scene, DT, vec3f::zero());
    CHECK_NEAR(ctx, hinge_constraint::get_current_angle(*constraint), 0.8f, 2e-3f);
    CHECK(ctx, scalar::abs(ball->motion_properties.angular_velocity.z) < 1e-2f);

    // a motor with no spring does nothing
    constraint->motor_settings.spring.frequency_or_stiffness = 0.0f;
    hinge_constraint::set_target_angle(*constraint, 0.0f);
    for (int i = 0; i < 30; i++) step_constraint_scene(scene, DT, vec3f::zero());
    CHECK(ctx, !angle_constraint_part::is_active(constraint->motor_constraint_part));
    release_constraint_scene(scene);
}

void test_hinge_constraint_friction(TestCtx &ctx) {
    // a friction torque of I slows a free spin by 1 rad/s each second
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *ball = nullptr;
    make_spinner(scene, ball, {.max_friction_torque = SPHERE_INERTIA});
    ball->motion_properties.angular_velocity = {0.0f, 0.0f, 3.0f};
    for (int i = 0; i < 60; i++) step_constraint_scene(scene, DT, vec3f::zero());
    CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.z, 2.0f, 1e-3f);
    // and stops it without turning it back
    for (int i = 0; i < 180; i++) step_constraint_scene(scene, DT, vec3f::zero());
    CHECK(ctx, scalar::abs(ball->motion_properties.angular_velocity.z) < 1e-4f);
    release_constraint_scene(scene);
}

/** setup, warm start and ten velocity iterations, as one solver step runs them */
static void solve_hinge_velocities(ConstraintScene &scene, HingeConstraint &constraint) {
    hinge_constraint::setup_velocity(constraint, scene.bodies, DT);
    hinge_constraint::warm_start_velocity(constraint, scene.bodies, 1.0f);
    for (uint32_t i = 0; i < 10; i++) hinge_constraint::solve_velocity(constraint, scene.bodies, DT);
}

void test_hinge_constraint_local_space_rest_with_rotated_bodies(TestCtx &ctx) {
    // the hinge frame in world space is normal +x, hinge +z; each body sees it through its own rotation
    const Float4 rotations[2][2] = {
        {{0.0f, 0.0f, 0.0f, 1.0f}, stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), -scalar::PI / 2.0f)},
        {stored_axis_angle(vec3f::make(1.0f, 1.0f, 0.0f), 0.7f), stored_axis_angle(vec3f::make(0.0f, 1.0f, 2.0f), -1.3f)},
    };
    const Vec3f world_normal = vec3f::axis_x();
    const Vec3f world_hinge = vec3f::axis_z();
    for (uint32_t c = 0; c < 2; c++) {
        ConstraintScene scene = make_constraint_scene(ctx);
        RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f}, rotations[c][0]);
        RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 2.0f, 0.0f}, rotations[c][1]);
        const Quatf inv_a = quatf::conjugate(quatf::load(a->quaternion));
        const Quatf inv_b = quatf::conjugate(quatf::load(b->quaternion));

        hinge_constraint::HingeConstraintSettings settings;
        settings.space = constraints::SPACE_LOCAL;
        settings.point_a = {0.0f, 1.0f, 0.0f};
        settings.point_b = {0.0f, -1.0f, 0.0f};
        vec3f::store(settings.hinge_axis_a, vec3f::transform_quat(world_hinge, inv_a));
        vec3f::store(settings.hinge_axis_b, vec3f::transform_quat(world_hinge, inv_b));
        vec3f::store(settings.normal_axis_a, vec3f::transform_quat(world_normal, inv_a));
        vec3f::store(settings.normal_axis_b, vec3f::transform_quat(world_normal, inv_b));
        settings.max_friction_torque = 1.0f;
        const constraint_id::ConstraintId id = hinge_constraint::create(scene.constraints, scene.bodies, a, b, settings);
        HingeConstraint *constraint = hinge_constraint::get(scene.constraints, id);

        hinge_constraint::setup_velocity(*constraint, scene.bodies, DT);
        CHECK_NEAR(ctx, hinge_constraint::get_current_angle(*constraint), 0.0f, 1e-5f);

        // turning body b about the world hinge axis turns the angle by as much
        rotate_body(b, axis_angle(world_hinge, 0.3f));
        hinge_constraint::setup_velocity(*constraint, scene.bodies, DT);
        CHECK_NEAR(ctx, hinge_constraint::get_current_angle(*constraint), 0.3f, 1e-5f);
        release_constraint_scene(scene);
    }
}

void test_hinge_constraint_axes_a_hair_apart(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, ZERO);
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, ZERO);
    // body 2's normal is turned by a hair about the hinge
    const float angle = 9e-7f;
    hinge_constraint::HingeConstraintSettings settings;
    settings.space = constraints::SPACE_LOCAL;
    settings.hinge_axis_a = {0.0f, 0.0f, 1.0f};
    settings.hinge_axis_b = {0.0f, 0.0f, 1.0f};
    settings.normal_axis_a = {1.0f, 0.0f, 0.0f};
    settings.normal_axis_b = {scalar::cos(angle), scalar::sin(angle), 0.0f};
    const constraint_id::ConstraintId id = hinge_constraint::create(scene.constraints, scene.bodies, base, ball, settings);
    const HingeConstraint *hair = hinge_constraint::get(scene.constraints, id);
    // r0^-1 = c2 c1^-1 turns by the hair about the hinge axis
    CHECK(ctx, quatf::z(hair->inv_initial_orientation) != 0.0f);
    CHECK_NEAR(ctx, quatf::z(hair->inv_initial_orientation), angle / 2.0f, 1e-8f);
    release_constraint_scene(scene);
}

void test_hinge_constraint_position_and_velocity_motor(TestCtx &ctx) {
    // with a frequency spring the motor drives the angular velocity at the target angle
    {
        ConstraintScene scene = make_constraint_scene(ctx);
        RigidBody *ball = nullptr;
        HingeConstraint *constraint = make_spinner(scene, ball, {});
        hinge_constraint::set_motor_state(*constraint, motor_settings::POSITION_AND_VELOCITY);
        hinge_constraint::set_target_angular_velocity(*constraint, 2.0f);
        hinge_constraint::set_target_angle(*constraint, 0.0f);
        solve_hinge_velocities(scene, *constraint);
        CHECK(ctx, ball->motion_properties.angular_velocity.z > 0.0f);
        release_constraint_scene(scene);
    }

    // a damping only spring is a soft velocity drive: w = w_target (dt c / I) / (dt c / I + 1); dt c / I = 1 halves it
    {
        ConstraintScene scene = make_constraint_scene(ctx);
        RigidBody *ball = nullptr;
        hinge_constraint::HingeConstraintSettings settings;
        settings.motor_settings.spring = {spring_settings::STIFFNESS_AND_DAMPING, 0.0f, SPHERE_INERTIA / DT};
        HingeConstraint *constraint = make_spinner(scene, ball, settings);
        hinge_constraint::set_motor_state(*constraint, motor_settings::POSITION_AND_VELOCITY);
        hinge_constraint::set_target_angular_velocity(*constraint, 2.0f);
        solve_hinge_velocities(scene, *constraint);
        CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.z, 1.0f, 1e-4f);
        release_constraint_scene(scene);
    }
}
