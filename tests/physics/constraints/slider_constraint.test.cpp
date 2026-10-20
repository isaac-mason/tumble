#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec2f.h"
#include "tumble/math/vec3f.h"
#include "physics/constraints/constraint_test_scene.h"
#include "tumble/physics/constraints/slider_constraint.h"
#include "test.h"

using rigid_body::RigidBody;
using slider_constraint::SliderConstraint;

static const Float3 GRAVITY = {0.0f, -9.81f, 0.0f};
static const Float3 ZERO = {0.0f, 0.0f, 0.0f};
static const float DT = 1.0f / 60.0f;
/** the scene's sphere: 1000 * 4/3 pi 0.5^3 */
static const float SPHERE_MASS = 1000.0f * 4.0f / 3.0f * scalar::PI * 0.125f;

/** a ball at (1, 0, 0) on a static base at the origin, sliding along world x from where it is */
static SliderConstraint *make_rail(ConstraintScene &scene, RigidBody *&ball, const slider_constraint::SliderConstraintSettings &extra) {
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, ZERO);
    ball = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    slider_constraint::SliderConstraintSettings settings = extra;
    settings.point_a = {1.0f, 0.0f, 0.0f};
    settings.point_b = {1.0f, 0.0f, 0.0f};
    const constraint_id::ConstraintId id = slider_constraint::create(*scene.world, base, ball, settings);
    scene.ids.push(id);
    return slider_constraint::get(*scene.world, id);
}

/** the angle of the ball's rotation away from its rest orientation (identity) */
static float rotation_angle(const RigidBody *body) { return 2.0f * scalar::acos(scalar::min(1.0f, scalar::abs(body->quaternion.w))); }

void test_slider_constraint_create(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI / 2.0f));
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {2.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), 0.4f));

    // world space: the slider axis and normal taken to body a's space, the second normal their cross product
    const constraint_id::ConstraintId id = slider_constraint::create(*scene.world, a, b,
                                                                     {.point_a = {1.0f, 0.0f, 0.0f},
                                                                      .point_b = {1.0f, 0.0f, 0.0f},
                                                                      .slider_axis_a = {2.0f, 0.0f, 0.0f},
                                                                      .slider_axis_b = {2.0f, 0.0f, 0.0f},
                                                                      .normal_axis_a = {0.0f, 3.0f, 0.0f},
                                                                      .normal_axis_b = {0.0f, 3.0f, 0.0f}});
    SliderConstraint *constraint = slider_constraint::get(*scene.world, id);
    CHECK(ctx, constraint != nullptr && constraint_id::type(id) == constraint_id::SLIDER);
    CHECK(ctx, a->constraint_ids.length == 1 && b->constraint_ids.length == 1);
    // a quarter turn about z: world +x is body -y, world +y is body +x
    CHECK(ctx, vec3f::distance(constraint->local_space_slider_axis_a, vec3f::make(0.0f, -1.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(constraint->local_space_normal_a, vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(constraint->local_space_normal_b, vec3f::make(0.0f, 0.0f, 1.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(world_point(a, constraint->local_space_position_a), vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(world_point(b, constraint->local_space_position_b), vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);
    // no limits by default; at rest the slide is 0
    CHECK(ctx, !constraint->has_limits);
    CHECK(ctx, scalar::abs(slider_constraint::get_current_position(*scene.world, *constraint)) < 1e-5f);
    b->center_of_mass_position.x += 0.25f;
    CHECK_NEAR(ctx, slider_constraint::get_current_position(*scene.world, *constraint), 0.25f, 1e-5f);
    // the rest orientation is the current one: q2 * inv_initial_orientation * q1^-1 is the identity
    const Quatf inv_a = quatf::conjugate(quatf::load(a->quaternion));
    const Quatf diff = quatf::multiply(quatf::multiply(quatf::load(b->quaternion), constraint->inv_initial_orientation), inv_a);
    CHECK(ctx, scalar::abs(quatf::w(diff)) > 1.0f - 1e-6f);

    // auto detect: the anchor sits on the lighter body, here the only movable one
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 5.0f, 0.0f});
    const constraint_id::ConstraintId auto_id =
        slider_constraint::create(*scene.world, base, b, {.auto_detect_point = true});
    const SliderConstraint *auto_slider = slider_constraint::get(*scene.world, auto_id);
    CHECK(ctx, vec3f::length(auto_slider->local_space_position_b) < 1e-6f);
    CHECK(ctx, vec3f::distance(world_point(base, auto_slider->local_space_position_a), vec3f::load(b->center_of_mass_position)) < 1e-5f);

    // local space: as given
    const constraint_id::ConstraintId local_id =
        slider_constraint::create(*scene.world, a, b,
                                  {.point_a = {0.5f, 0.0f, 0.0f},
                                   .slider_axis_a = {0.0f, 0.0f, 1.0f},
                                   .normal_axis_a = {1.0f, 0.0f, 0.0f},
                                   .space = constraints::SPACE_LOCAL});
    const SliderConstraint *local = slider_constraint::get(*scene.world, local_id);
    CHECK(ctx, vec3f::exact_equals(local->local_space_position_a, vec3f::make(0.5f, 0.0f, 0.0f)));
    CHECK(ctx, vec3f::exact_equals(local->local_space_slider_axis_a, vec3f::make(0.0f, 0.0f, 1.0f)));
    CHECK(ctx, vec3f::exact_equals(local->local_space_normal_b, vec3f::make(0.0f, 1.0f, 0.0f)));
    CHECK(ctx, slider_constraint::get(*scene.world, constraint_id::ser(0, constraint_id::HINGE, 0)) == nullptr);
    release_constraint_scene(scene);
}

void test_slider_constraint_setters(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    SliderConstraint *constraint = make_rail(scene, ball, {});
    CHECK(ctx, !constraint->has_limits);

    // stored as given, min <= 0 <= max
    slider_constraint::set_limits(*constraint, -1.0f, 2.0f);
    CHECK(ctx, constraint->limits_min == -1.0f && constraint->limits_max == 2.0f && constraint->has_limits);
    slider_constraint::set_target_position(*constraint, 3.0f);
    CHECK(ctx, constraint->target_position == 2.0f);
    slider_constraint::set_target_position(*constraint, -3.0f);
    CHECK(ctx, constraint->target_position == -1.0f);
    slider_constraint::set_limits(*constraint, -__FLT_MAX__, __FLT_MAX__);
    CHECK(ctx, !constraint->has_limits);
    slider_constraint::set_target_position(*constraint, 3.0f);
    CHECK(ctx, constraint->target_position == 3.0f);

    slider_constraint::set_max_friction_force(*constraint, 12.0f);
    CHECK(ctx, slider_constraint::get_max_friction_force(*constraint) == 12.0f);
    slider_constraint::set_motor_state(*constraint, motor_settings::VELOCITY);
    slider_constraint::set_target_velocity(*constraint, -2.0f);
    CHECK(ctx, constraint->motor_state == motor_settings::VELOCITY && constraint->target_velocity == -2.0f);
    release_constraint_scene(scene);
}

void test_slider_constraint_holds_axis(TestCtx &ctx) {
    // under gravity across the rail, kicked along every axis and spun
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    const SliderConstraint *constraint = make_rail(scene, ball, {});
    ball->motion_properties.linear_velocity = {1.0f, 2.0f, 3.0f};
    ball->motion_properties.angular_velocity = {1.0f, 2.0f, 3.0f};

    float worst_off_axis = 0.0f;
    float worst_rotation = 0.0f;
    for (int i = 0; i < 60; i++) {
        step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
        const Float3 &p = ball->center_of_mass_position;
        worst_off_axis = scalar::max(worst_off_axis, scalar::sqrt(p.y * p.y + p.z * p.z));
        worst_rotation = scalar::max(worst_rotation, rotation_angle(ball));
    }
    // the first step removes every velocity but the slide; nothing drifts after
    CHECK(ctx, worst_off_axis < 1e-2f);
    CHECK(ctx, worst_rotation < 1e-2f);
    CHECK(ctx, vec3f::length(vec3f::load(ball->motion_properties.angular_velocity)) < 1e-3f);
    CHECK(ctx, scalar::abs(ball->motion_properties.linear_velocity.y) < 1e-3f);
    CHECK(ctx, scalar::abs(ball->motion_properties.linear_velocity.z) < 1e-3f);
    // the slide is free: one meter in the second at 1 m/s
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, 1.0f, 1e-3f);
    CHECK_NEAR(ctx, slider_constraint::get_current_position(*scene.world, *constraint), 1.0f, 2e-2f);
    // the static base carries the weight: the perpendicular impulse is mg dt
    Vec2f lambda;
    slider_constraint::get_total_lambda_position(lambda, *constraint);
    CHECK_NEAR(ctx, scalar::sqrt(lambda.x * lambda.x + lambda.y * lambda.y), SPHERE_MASS * 9.81f * DT, 1e-1f);
    release_constraint_scene(scene);
}

void test_slider_constraint_limits(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    const SliderConstraint *constraint = make_rail(scene, ball, {.limits_min = -0.5f, .limits_max = 0.5f});
    ball->motion_properties.linear_velocity = {3.0f, 0.0f, 0.0f};

    float farthest = 0.0f;
    for (int i = 0; i < 60; i++) {
        step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
        farthest = scalar::max(farthest, slider_constraint::get_current_position(*scene.world, *constraint));
    }
    // it stops at the upper limit, overshooting by at most one step's travel
    CHECK(ctx, farthest < 0.5f + 3.0f * DT);
    CHECK_NEAR(ctx, slider_constraint::get_current_position(*scene.world, *constraint), 0.5f, 1e-2f);
    CHECK(ctx, scalar::abs(ball->motion_properties.linear_velocity.x) < 1e-2f);
    CHECK(ctx, slider_constraint::get_total_lambda_position_limits(*constraint) <= 0.0f);

    ball->motion_properties.linear_velocity = {-3.0f, 0.0f, 0.0f};
    float nearest = 0.0f;
    for (int i = 0; i < 60; i++) {
        step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
        nearest = scalar::min(nearest, slider_constraint::get_current_position(*scene.world, *constraint));
    }
    CHECK(ctx, nearest > -0.5f - 3.0f * DT);
    CHECK_NEAR(ctx, slider_constraint::get_current_position(*scene.world, *constraint), -0.5f, 1e-2f);
    CHECK(ctx, slider_constraint::get_total_lambda_position_limits(*constraint) >= 0.0f);
    release_constraint_scene(scene);
}

void test_slider_constraint_position_motor(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    SliderConstraint *constraint = make_rail(scene, ball, {});
    slider_constraint::set_motor_state(*constraint, motor_settings::POSITION);
    slider_constraint::set_target_position(*constraint, 0.3f);

    // the default 2 hz critically damped spring settles on the target, under gravity across the rail
    for (int i = 0; i < 180; i++) step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
    CHECK_NEAR(ctx, slider_constraint::get_current_position(*scene.world, *constraint), 0.3f, 1e-3f);
    CHECK(ctx, scalar::abs(ball->motion_properties.linear_velocity.x) < 1e-2f);
    CHECK_NEAR(ctx, ball->center_of_mass_position.x, 1.3f, 1e-3f);

    // and follows a new target the other way
    slider_constraint::set_target_position(*constraint, -0.4f);
    for (int i = 0; i < 180; i++) step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
    CHECK_NEAR(ctx, slider_constraint::get_current_position(*scene.world, *constraint), -0.4f, 1e-3f);
    release_constraint_scene(scene);
}

void test_slider_constraint_velocity_motor_and_friction(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    SliderConstraint *constraint = make_rail(scene, ball, {});
    slider_constraint::set_motor_state(*constraint, motor_settings::VELOCITY);
    slider_constraint::set_target_velocity(*constraint, 1.5f);
    for (int i = 0; i < 30; i++) step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, 1.5f, 1e-4f);

    // a force limit of m accelerates at 1 m/s^2
    constraint->motor_settings.min_force_limit = -SPHERE_MASS;
    constraint->motor_settings.max_force_limit = SPHERE_MASS;
    slider_constraint::set_target_velocity(*constraint, -1.5f);
    for (int i = 0; i < 60; i++) step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, 0.5f, 1e-3f);
    CHECK_NEAR(ctx, slider_constraint::get_total_lambda_motor(*constraint), -SPHERE_MASS * DT, 1e-2f);

    // friction of m N with the motor off decelerates at 1 m/s^2 and stops it
    slider_constraint::set_motor_state(*constraint, motor_settings::OFF);
    slider_constraint::set_max_friction_force(*constraint, SPHERE_MASS);
    ball->motion_properties.linear_velocity = {2.0f, 0.0f, 0.0f};
    for (int i = 0; i < 60; i++) step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, 1.0f, 1e-3f);
    for (int i = 0; i < 120; i++) step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
    CHECK(ctx, scalar::abs(ball->motion_properties.linear_velocity.x) < 1e-4f);
    release_constraint_scene(scene);
}

/** the anchor auto-detected between a body at the origin and one at (4, 0, 0) */
static void check_auto_detected(TestCtx &ctx, motion_type::MotionType motion_type_a, motion_type::MotionType motion_type_b, bool make_a_static,
                                const Float3 &expected_a, const Float3 &expected_b) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type_a, ZERO);
    RigidBody *b = add_body(scene, motion_type_b, {4.0f, 0.0f, 0.0f});
    if (make_a_static) rigid_body::set_motion_type(*scene.world, a, motion_type::MOTION_STATIC, false);
    const constraint_id::ConstraintId id =
        slider_constraint::create(*scene.world, a, b, {.auto_detect_point = true});
    const SliderConstraint *constraint = slider_constraint::get(*scene.world, id);
    CHECK(ctx, vec3f::distance(constraint->local_space_position_a, vec3f::load(expected_a)) < 1e-6f);
    CHECK(ctx, vec3f::distance(constraint->local_space_position_b, vec3f::load(expected_b)) < 1e-6f);
    release_constraint_scene(scene);
}

void test_slider_constraint_auto_detect_static_body(TestCtx &ctx) {
    // a static body puts the anchor on the other body's center of mass
    check_auto_detected(ctx, motion_type::MOTION_STATIC, motion_type::MOTION_STATIC, false, {4.0f, 0.0f, 0.0f}, ZERO);
    check_auto_detected(ctx, motion_type::MOTION_STATIC, motion_type::MOTION_KINEMATIC, false, {4.0f, 0.0f, 0.0f}, ZERO);
    check_auto_detected(ctx, motion_type::MOTION_KINEMATIC, motion_type::MOTION_STATIC, false, ZERO, {-4.0f, 0.0f, 0.0f});
    // whatever inverse mass a body made static kept
    check_auto_detected(ctx, motion_type::MOTION_DYNAMIC, motion_type::MOTION_DYNAMIC, true, {4.0f, 0.0f, 0.0f}, ZERO);
    // two equal dynamic bodies anchor half way
    check_auto_detected(ctx, motion_type::MOTION_DYNAMIC, motion_type::MOTION_DYNAMIC, false, {2.0f, 0.0f, 0.0f}, {-2.0f, 0.0f, 0.0f});
}

void test_slider_constraint_axes_a_hair_apart(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, ZERO);
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, ZERO);
    // body 2's normal is turned by a hair about the slider axis
    const float angle = 9e-7f;
    const constraint_id::ConstraintId id = slider_constraint::create(
        *scene.world, base, ball, {.normal_axis_b = {0.0f, scalar::cos(angle), scalar::sin(angle)}});
    const SliderConstraint *constraint = slider_constraint::get(*scene.world, id);
    // r0^-1 = c2 c1^-1 turns by the hair about the slider axis
    CHECK(ctx, quatf::x(constraint->inv_initial_orientation) != 0.0f);
    CHECK_NEAR(ctx, quatf::x(constraint->inv_initial_orientation), angle / 2.0f, 1e-8f);
    release_constraint_scene(scene);
}

static void solve_slider_velocities(ConstraintScene &scene, SliderConstraint &constraint) {
    slider_constraint::setup_velocity(constraint, scene.world->bodies, DT);
    slider_constraint::warm_start_velocity(constraint, scene.world->bodies, 1.0f);
    for (uint32_t i = 0; i < 10; i++) slider_constraint::solve_velocity(constraint, scene.world->bodies, DT);
}

void test_slider_constraint_position_and_velocity_motor(TestCtx &ctx) {
    // with a frequency spring the motor drives the velocity at the target position
    {
        ConstraintScene scene = make_constraint_scene();
        RigidBody *ball = nullptr;
        SliderConstraint *constraint = make_rail(scene, ball, {});
        slider_constraint::set_motor_state(*constraint, motor_settings::POSITION_AND_VELOCITY);
        slider_constraint::set_target_velocity(*constraint, 2.0f);
        slider_constraint::set_target_position(*constraint, 0.0f);
        solve_slider_velocities(scene, *constraint);
        CHECK(ctx, ball->motion_properties.linear_velocity.x > 0.0f);
        release_constraint_scene(scene);
    }

    // a damping only spring is a soft velocity drive: v = v_target (dt c / m) / (dt c / m + 1); dt c / m = 1 halves it
    {
        ConstraintScene scene = make_constraint_scene();
        RigidBody *ball = nullptr;
        slider_constraint::SliderConstraintSettings settings;
        settings.motor_settings.spring = {spring_settings::STIFFNESS_AND_DAMPING, 0.0f, SPHERE_MASS / DT};
        SliderConstraint *constraint = make_rail(scene, ball, settings);
        slider_constraint::set_motor_state(*constraint, motor_settings::POSITION_AND_VELOCITY);
        slider_constraint::set_target_velocity(*constraint, 2.0f);
        solve_slider_velocities(scene, *constraint);
        CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, 1.0f, 1e-4f);
        release_constraint_scene(scene);
    }

    // a damping only spring cannot drive a position motor, which needs stiffness
    {
        ConstraintScene scene = make_constraint_scene();
        RigidBody *ball = nullptr;
        slider_constraint::SliderConstraintSettings settings;
        settings.motor_settings.spring = {spring_settings::STIFFNESS_AND_DAMPING, 0.0f, 100.0f};
        SliderConstraint *constraint = make_rail(scene, ball, settings);
        slider_constraint::set_motor_state(*constraint, motor_settings::POSITION);
        slider_constraint::set_target_position(*constraint, 1.0f);
        solve_slider_velocities(scene, *constraint);
        CHECK(ctx, ball->motion_properties.linear_velocity.x == 0.0f);
        release_constraint_scene(scene);
    }
}
