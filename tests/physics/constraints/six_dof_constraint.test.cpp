#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "physics/constraints/constraint_test_scene.h"
#include "tumble/physics/constraints/fixed_constraint.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/constraints/six_dof_constraint.h"
#include "test.h"
#include "test_math.h"

using rigid_body::RigidBody;
using six_dof_constraint::SixDOFConstraint;
using six_dof_constraint::SixDOFConstraintSettings;

static const Float3 GRAVITY = {0.0f, -9.81f, 0.0f};
static const Float3 ZERO = {0.0f, 0.0f, 0.0f};
static const float DT = 1.0f / 60.0f;

/** settings with the translation axes fixed and the rotation axes free */
static SixDOFConstraintSettings translation_fixed_settings(const Float3 &pivot) {
    SixDOFConstraintSettings settings;
    settings.position1 = pivot;
    settings.position2 = pivot;
    for (uint32_t i = 0; i < 3; i++) {
        settings.limit_min[i] = __FLT_MAX__;
        settings.limit_max[i] = -__FLT_MAX__;
    }
    return settings;
}

/** a ball on a static base, both centered on the origin, with the given settings */
static SixDOFConstraint *make_joint(ConstraintScene &scene, RigidBody *&ball, const SixDOFConstraintSettings &settings) {
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, ZERO);
    ball = add_body(scene, motion_type::MOTION_DYNAMIC, ZERO);
    const constraint_id::ConstraintId id = six_dof_constraint::create(*scene.world, base, ball, settings);
    scene.ids.push(id);
    return six_dof_constraint::get(*scene.world, id);
}

/** the angle a rotation turns by */
static float rotation_angle(Quatf q) { return 2.0f * scalar::acos(scalar::min(1.0f, scalar::abs(quatf::w(q)))); }

/** the angle of q2 * q1^-1, the relative rotation of two bodies whose rest pose is equal orientations */
static float relative_angle(const RigidBody *a, const RigidBody *b) {
    return rotation_angle(quatf::multiply(quatf::load(b->quaternion), quatf::conjugate(quatf::load(a->quaternion))));
}

void test_six_dof_constraint_create(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI / 2.0f));
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {2.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::make(1.0f, 1.0f, 0.0f), 0.6f));

    // everything free by default; motor springs at 2 hz, critically damped, on every axis
    const constraint_id::ConstraintId id = six_dof_constraint::create(
        *scene.world, a, b,
        {.position1 = {1.0f, 0.0f, 0.0f}, .position2 = {1.0f, 0.0f, 0.0f}, .axis_x1 = {0.0f, 2.0f, 0.0f},
         .axis_y1 = {0.0f, 0.0f, 3.0f}, .axis_x2 = {0.0f, 2.0f, 0.0f}, .axis_y2 = {0.0f, 0.0f, 3.0f}});
    const SixDOFConstraint *constraint = six_dof_constraint::get(*scene.world, id);
    CHECK(ctx, constraint != nullptr && constraint_id::type(id) == constraint_id::SIX_DOF);
    CHECK(ctx, a->constraint_ids.length == 1 && b->constraint_ids.length == 1);
    CHECK(ctx, constraint->free_axis == 0b111111 && constraint->fixed_axis == 0);
    for (uint32_t i = 0; i < six_dof_constraint::NUM; i++) {
        CHECK(ctx, constraint->motor_settings[i].spring.mode == spring_settings::FREQUENCY_AND_DAMPING);
        CHECK(ctx, constraint->motor_settings[i].spring.frequency_or_stiffness == 2.0f);
        CHECK(ctx, constraint->motor_settings[i].spring.damping == 1.0f);
    }
    CHECK(ctx, !constraint->translation_motor_active && !constraint->rotation_motor_active);
    CHECK(ctx, vec3f::distance(world_point(a, constraint->local_space_position1), vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(world_point(b, constraint->local_space_position2), vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);
    // constraint x, y and z through either body are the given world axes
    for (uint32_t body = 0; body < 2; body++) {
        const Quatf to_world = body == 0 ? quatf::multiply(quatf::load(a->quaternion), constraint->constraint_to_body1)
                                         : quatf::multiply(quatf::load(b->quaternion), constraint->constraint_to_body2);
        const Vec3f x = vec3f::transform_quat(vec3f::axis_x(), to_world);
        const Vec3f z = vec3f::transform_quat(vec3f::axis_z(), to_world);
        CHECK(ctx, vec3f::distance(x, vec3f::make(0.0f, 1.0f, 0.0f)) < 1e-5f);
        CHECK(ctx, vec3f::distance(z, vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);
    }

    // inverted translation limits fix the axis at 0; a cone's swing limits become symmetric and non negative;
    // rotation limits clamp to [-pi, pi]
    SixDOFConstraintSettings limited;
    limited.limit_min[0] = 0.3f;
    limited.limit_max[0] = -0.2f;
    limited.limit_min[1] = -0.5f;
    limited.limit_max[1] = 0.5f;
    limited.limit_min[3] = -5.0f;
    limited.limit_max[3] = 5.0f;
    limited.limit_max[4] = -0.5f;
    limited.limit_min[5] = -0.1f;
    limited.limit_max[5] = 0.4f;
    const SixDOFConstraint *clamped =
        six_dof_constraint::get(*scene.world, six_dof_constraint::create(*scene.world, a, b, limited));
    CHECK(ctx, clamped->limit_min[0] == 0.0f && clamped->limit_max[0] == 0.0f);
    CHECK(ctx, clamped->limit_min[3] == -scalar::PI && clamped->limit_max[3] == scalar::PI);
    CHECK(ctx, clamped->limit_min[4] == 0.0f && clamped->limit_max[4] == 0.0f);
    CHECK(ctx, clamped->limit_min[5] == -0.4f && clamped->limit_max[5] == 0.4f);
    // x and swing y fixed; translation z and twist free; y and swing z limited
    CHECK(ctx, clamped->fixed_axis == ((1u << 0) | (1u << 4)));
    CHECK(ctx, clamped->free_axis == ((1u << 2) | (1u << 3)));
    CHECK(ctx, six_dof_constraint::get(*scene.world, constraint_id::ser(0, constraint_id::FIXED, 0)) == nullptr);
    release_constraint_scene(scene);
}

void test_six_dof_constraint_setters(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    SixDOFConstraint *constraint = make_joint(scene, ball, {});

    six_dof_constraint::set_translation_limits(*constraint, vec3f::make(-1.0f, 0.5f, -__FLT_MAX__), vec3f::make(1.0f, -0.5f, __FLT_MAX__));
    CHECK(ctx, constraint->limit_min[1] == 0.0f && constraint->limit_max[1] == 0.0f);
    CHECK(ctx, constraint->fixed_axis == (1u << 1) && constraint->free_axis == 0b111100);
    six_dof_constraint::set_rotation_limits(*constraint, vec3f::make(-0.2f, -0.3f, -0.3f), vec3f::make(0.2f, 0.3f, 0.3f));
    CHECK(ctx, constraint->free_axis == (1u << 2));
    CHECK_NEAR(ctx, constraint->swing_twist_constraint_part.sin_twist_half_max_angle, scalar::sin(0.1f), 1e-6f);

    six_dof_constraint::make_free_axis(*constraint, six_dof_constraint::TRANSLATION_X);
    CHECK(ctx, (constraint->free_axis & 1u) != 0);
    six_dof_constraint::set_limited_axis(*constraint, six_dof_constraint::TRANSLATION_X, -0.1f, 0.2f);
    CHECK(ctx, (constraint->free_axis & 1u) == 0 && (constraint->fixed_axis & 1u) == 0);

    // friction and motors switch the motor paths on
    six_dof_constraint::set_max_friction(*constraint, six_dof_constraint::TRANSLATION_X, 5.0f);
    CHECK(ctx, constraint->translation_motor_active && !constraint->rotation_motor_active);
    six_dof_constraint::set_motor_state(*constraint, six_dof_constraint::ROTATION_Y, motor_settings::POSITION);
    CHECK(ctx, constraint->rotation_motor_active && constraint->rotation_position_motor_active == 0b010);
    six_dof_constraint::set_motor_state(*constraint, six_dof_constraint::ROTATION_Y, motor_settings::OFF);
    CHECK(ctx, !constraint->rotation_motor_active && constraint->rotation_position_motor_active == 0);
    // friction on a fixed axis is ignored
    six_dof_constraint::set_max_friction(*constraint, six_dof_constraint::TRANSLATION_Y, 5.0f);
    six_dof_constraint::set_max_friction(*constraint, six_dof_constraint::TRANSLATION_X, 0.0f);
    CHECK(ctx, !constraint->translation_motor_active);

    six_dof_constraint::set_target_velocity_cs(*constraint, vec3f::make(1.0f, 2.0f, 3.0f));
    six_dof_constraint::set_target_angular_velocity_cs(*constraint, vec3f::make(4.0f, 5.0f, 6.0f));
    six_dof_constraint::set_target_position_cs(*constraint, vec3f::make(7.0f, 8.0f, 9.0f));
    CHECK(ctx, vec3f::exact_equals(constraint->target_velocity, vec3f::make(1.0f, 2.0f, 3.0f)));
    CHECK(ctx, vec3f::exact_equals(constraint->target_angular_velocity, vec3f::make(4.0f, 5.0f, 6.0f)));
    CHECK(ctx, vec3f::exact_equals(constraint->target_position, vec3f::make(7.0f, 8.0f, 9.0f)));
    // the target orientation is clamped to the rotation limits
    six_dof_constraint::set_target_orientation_cs(*constraint, axis_angle(vec3f::make(1.0f, 0.0f, 0.0f), 1.0f));
    CHECK_NEAR(ctx, 2.0f * scalar::atan2(quatf::x(constraint->target_orientation), quatf::w(constraint->target_orientation)), 0.2f, 1e-4f);
    release_constraint_scene(scene);
}

void test_six_dof_constraint_translation_locked_is_a_point_constraint(TestCtx &ctx) {
    // two pendulums kicked the same way, one on a point constraint and one on a six dof with only translation
    // locked: they swing alike
    ConstraintScene scene = make_constraint_scene();
    const Float3 offset = {5.0f, 0.0f, 0.0f};
    const Float3 pivot = {0.0f, 10.0f, 0.0f};
    const Float3 pivot2 = {5.0f, 10.0f, 0.0f};
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, pivot);
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 9.0f, 0.0f});
    RigidBody *base2 = add_body(scene, motion_type::MOTION_STATIC, pivot2);
    RigidBody *ball2 = add_body(scene, motion_type::MOTION_DYNAMIC, {6.0f, 9.0f, 0.0f});
    scene.ids.push(point_constraint::create(*scene.world, base, ball, {.point_a = pivot, .point_b = pivot}));
    const constraint_id::ConstraintId id =
        six_dof_constraint::create(*scene.world, base2, ball2, translation_fixed_settings(pivot2));
    scene.ids.push(id);
    const SixDOFConstraint *constraint = six_dof_constraint::get(*scene.world, id);
    CHECK(ctx, constraint->fixed_axis == 0b000111 && constraint->free_axis == 0b111000);
    RigidBody *const balls[2] = {ball, ball2};
    for (RigidBody *body : balls) {
        body->motion_properties.linear_velocity = {0.0f, 1.0f, 3.0f};
        body->motion_properties.angular_velocity = {1.0f, -2.0f, 0.5f};
    }

    float worst_position = 0.0f;
    float worst_rotation = 0.0f;
    for (int i = 0; i < 120; i++) {
        step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
        const Vec3f shifted = vec3f::subtract(vec3f::load(ball2->center_of_mass_position), vec3f::load(offset));
        worst_position = scalar::max(worst_position, vec3f::distance(shifted, vec3f::load(ball->center_of_mass_position)));
        worst_rotation = scalar::max(worst_rotation, relative_angle(ball, ball2));
    }
    CHECK(ctx, worst_position < 1e-5f);
    CHECK(ctx, worst_rotation < 1e-3f);
    CHECK(ctx, vec3f::distance(vec3f::load(ball2->motion_properties.angular_velocity), vec3f::load(ball->motion_properties.angular_velocity)) < 1e-4f);
    // and it is a pendulum: one meter from its pivot, free to spin
    CHECK_NEAR(ctx, vec3f::distance(vec3f::load(ball2->center_of_mass_position), vec3f::load(pivot2)), scalar::sqrt(2.0f), 1e-2f);
    CHECK(ctx, vec3f::length(vec3f::load(ball2->motion_properties.angular_velocity)) > 1.0f);
    release_constraint_scene(scene);
}

void test_six_dof_constraint_all_locked_is_a_fixed_constraint(TestCtx &ctx) {
    // two bodies welded to a static base by a fixed constraint and by a six dof with every axis locked
    ConstraintScene scene = make_constraint_scene();
    const Float3 offset = {5.0f, 0.0f, 0.0f};
    const Float4 tilt = stored_axis_angle(vec3f::make(0.0f, 1.0f, 1.0f), 0.3f);
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 10.0f, 0.0f});
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 9.5f, 0.0f}, tilt);
    RigidBody *base2 = add_body(scene, motion_type::MOTION_STATIC, {5.0f, 10.0f, 0.0f});
    RigidBody *ball2 = add_body(scene, motion_type::MOTION_DYNAMIC, {6.0f, 9.5f, 0.0f}, tilt);
    const constraint_id::ConstraintId fixed_id = fixed_constraint::create(
        *scene.world, base, ball, {.point1 = {0.5f, 10.0f, 0.0f}, .point2 = {0.5f, 10.0f, 0.0f}});
    scene.ids.push(fixed_id);
    SixDOFConstraintSettings settings = translation_fixed_settings({5.5f, 10.0f, 0.0f});
    for (uint32_t i = 3; i < 6; i++) {
        settings.limit_min[i] = 0.0f;
        settings.limit_max[i] = 0.0f;
    }
    const constraint_id::ConstraintId id = six_dof_constraint::create(*scene.world, base2, ball2, settings);
    scene.ids.push(id);
    const SixDOFConstraint *constraint = six_dof_constraint::get(*scene.world, id);
    CHECK(ctx, constraint->fixed_axis == 0b111111 && constraint->free_axis == 0);
    RigidBody *const balls[2] = {ball, ball2};
    for (RigidBody *body : balls) {
        body->motion_properties.linear_velocity = {2.0f, 1.0f, -1.0f};
        body->motion_properties.angular_velocity = {3.0f, 0.0f, -2.0f};
    }

    float worst_position = 0.0f;
    float worst_rotation = 0.0f;
    float worst_drift = 0.0f;
    for (int i = 0; i < 120; i++) {
        step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
        const Vec3f shifted = vec3f::subtract(vec3f::load(ball2->center_of_mass_position), vec3f::load(offset));
        worst_position = scalar::max(worst_position, vec3f::distance(shifted, vec3f::load(ball->center_of_mass_position)));
        worst_rotation = scalar::max(worst_rotation, relative_angle(ball, ball2));
        if (i >= 10) worst_drift = scalar::max(worst_drift, vec3f::distance(vec3f::load(ball2->center_of_mass_position), vec3f::make(6.0f, 9.5f, 0.0f)));
    }
    // the same pose as the fixed constraint's body throughout
    CHECK(ctx, worst_position < 2e-3f);
    CHECK(ctx, worst_rotation < 2e-3f);
    // held: back at the weld after the kick, hanging there under gravity
    CHECK(ctx, worst_drift < 2e-2f);
    CHECK(ctx, vec3f::distance(vec3f::load(ball2->center_of_mass_position), vec3f::make(6.0f, 9.5f, 0.0f)) < 5e-3f);
    CHECK(ctx, rotation_angle(quatf::multiply(quatf::load(ball2->quaternion), quatf::conjugate(quatf::load(tilt)))) < 5e-3f);
    CHECK(ctx, vec3f::length(vec3f::load(ball2->motion_properties.angular_velocity)) < 1e-2f);
    release_constraint_scene(scene);
}

void test_six_dof_constraint_limited_translation(TestCtx &ctx) {
    // a slider along x with limits, every other axis locked
    ConstraintScene scene = make_constraint_scene();
    SixDOFConstraintSettings settings = translation_fixed_settings(ZERO);
    settings.limit_min[0] = -0.2f;
    settings.limit_max[0] = 0.3f;
    for (uint32_t i = 3; i < 6; i++) {
        settings.limit_min[i] = 0.0f;
        settings.limit_max[i] = 0.0f;
    }
    RigidBody *ball = nullptr;
    const SixDOFConstraint *constraint = make_joint(scene, ball, settings);
    ball->motion_properties.linear_velocity = {3.0f, 1.0f, 0.0f};

    float farthest = 0.0f;
    for (int i = 0; i < 60; i++) {
        step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
        farthest = scalar::max(farthest, ball->center_of_mass_position.x);
    }
    CHECK(ctx, farthest < 0.3f + 3.0f * DT);
    CHECK_NEAR(ctx, ball->center_of_mass_position.x, 0.3f, 1e-2f);
    CHECK_NEAR(ctx, constraint->displacement[0], 0.3f, 2e-2f);
    CHECK(ctx, scalar::abs(ball->center_of_mass_position.y) < 1e-2f && scalar::abs(ball->center_of_mass_position.z) < 1e-2f);
    CHECK(ctx, 2.0f * scalar::acos(scalar::min(1.0f, scalar::abs(ball->quaternion.w))) < 1e-3f);

    ball->motion_properties.linear_velocity = {-3.0f, 0.0f, 0.0f};
    float nearest = 0.0f;
    for (int i = 0; i < 60; i++) {
        step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
        nearest = scalar::min(nearest, ball->center_of_mass_position.x);
    }
    CHECK(ctx, nearest > -0.2f - 3.0f * DT);
    CHECK_NEAR(ctx, ball->center_of_mass_position.x, -0.2f, 1e-2f);
    release_constraint_scene(scene);
}

void test_six_dof_constraint_swing_limits(TestCtx &ctx) {
    // translation locked, swing held to a 0.4 radian cone, twist free
    ConstraintScene scene = make_constraint_scene();
    SixDOFConstraintSettings settings = translation_fixed_settings(ZERO);
    settings.limit_max[4] = 0.4f;
    settings.limit_max[5] = 0.4f;
    RigidBody *ball = nullptr;
    const SixDOFConstraint *constraint = make_joint(scene, ball, settings);
    CHECK(ctx, constraint->free_axis == (1u << 3));
    ball->motion_properties.angular_velocity = {2.0f, 0.0f, 3.0f};

    float widest = 0.0f;
    for (int i = 0; i < 90; i++) {
        step_constraint_scene(scene, DT, vec3f::zero());
        Quatf swing, twist;
        swing_twist_constraint_part::get_swing_twist(swing, twist, quatf::load(ball->quaternion));
        widest = scalar::max(widest, rotation_angle(swing));
    }
    CHECK(ctx, widest < 0.4f + 3.0f * DT);
    CHECK_NEAR(ctx, widest, 0.4f, 3.0f * DT);
    // the twist keeps spinning
    CHECK(ctx, vec3f::length(vec3f::load(ball->motion_properties.angular_velocity)) > 0.5f);
    CHECK(ctx, vec3f::length(vec3f::load(ball->center_of_mass_position)) < 1e-3f);
    release_constraint_scene(scene);
}

void test_six_dof_constraint_motors(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();

    // a velocity motor on the free x axis, every other axis locked
    SixDOFConstraintSettings settings = translation_fixed_settings(ZERO);
    settings.limit_min[0] = -__FLT_MAX__;
    settings.limit_max[0] = __FLT_MAX__;
    for (uint32_t i = 3; i < 6; i++) {
        settings.limit_min[i] = 0.0f;
        settings.limit_max[i] = 0.0f;
    }
    RigidBody *ball = nullptr;
    SixDOFConstraint *constraint = make_joint(scene, ball, settings);
    six_dof_constraint::set_motor_state(*constraint, six_dof_constraint::TRANSLATION_X, motor_settings::VELOCITY);
    six_dof_constraint::set_target_velocity_cs(*constraint, vec3f::make(1.5f, 0.0f, 0.0f));
    for (int i = 0; i < 30; i++) step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, 1.5f, 1e-3f);
    CHECK(ctx, scalar::abs(ball->center_of_mass_position.y) < 1e-2f);
    CHECK_NEAR(ctx, ball->center_of_mass_position.x, 0.75f, 3e-2f);

    // a rotation position motor about z, translation locked and rotation free, settles on the target
    RigidBody *spinner = nullptr;
    SixDOFConstraint *rotor = make_joint(scene, spinner, translation_fixed_settings(ZERO));
    six_dof_constraint::set_motor_state(*rotor, six_dof_constraint::ROTATION_Z, motor_settings::POSITION);
    six_dof_constraint::set_target_orientation_cs(*rotor, axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.5f));
    for (int i = 0; i < 180; i++) step_constraint_scene(scene, DT, vec3f::zero());
    Vec3f axis;
    const float angle = axis_angle_of(axis, quatf::load(spinner->quaternion));
    CHECK_NEAR(ctx, angle, 0.5f, 2e-3f);
    CHECK(ctx, vec3f::distance(axis, vec3f::make(0.0f, 0.0f, 1.0f)) < 1e-3f);
    CHECK(ctx, vec3f::length(vec3f::load(spinner->motion_properties.angular_velocity)) < 1e-2f);
    release_constraint_scene(scene);
}

static void solve_six_dof_velocities(ConstraintScene &scene, SixDOFConstraint &constraint) {
    six_dof_constraint::setup_velocity(constraint, scene.world->bodies, DT);
    six_dof_constraint::warm_start_velocity(constraint, scene.world->bodies, 1.0f);
    for (uint32_t i = 0; i < 10; i++) six_dof_constraint::solve_velocity(constraint, scene.world->bodies, DT);
}

static SixDOFConstraintSettings all_fixed_settings() {
    SixDOFConstraintSettings settings;
    for (uint32_t i = 0; i < six_dof_constraint::NUM; i++) {
        settings.limit_min[i] = 0.0f;
        settings.limit_max[i] = 0.0f;
    }
    return settings;
}

void test_six_dof_constraint_solve_velocity_reports_impulses(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    SixDOFConstraint *fixed = make_joint(scene, ball, all_fixed_settings());
    ball->motion_properties.linear_velocity = {1.0f, 0.0f, 0.0f};
    six_dof_constraint::setup_velocity(*fixed, scene.world->bodies, DT);
    CHECK(ctx, six_dof_constraint::solve_velocity(*fixed, scene.world->bodies, DT));
    // the velocity is now consistent, nothing more to apply
    CHECK(ctx, !six_dof_constraint::solve_velocity(*fixed, scene.world->bodies, DT));

    RigidBody *free_ball = nullptr;
    SixDOFConstraint *free_joint = make_joint(scene, free_ball, {});
    free_ball->motion_properties.linear_velocity = {1.0f, 0.0f, 0.0f};
    six_dof_constraint::setup_velocity(*free_joint, scene.world->bodies, DT);
    CHECK(ctx, !six_dof_constraint::solve_velocity(*free_joint, scene.world->bodies, DT));
    release_constraint_scene(scene);
}

void test_six_dof_constraint_axis_set_change_clears_warm_start(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    SixDOFConstraint *constraint = make_joint(scene, ball, all_fixed_settings());
    ball->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    solve_six_dof_velocities(scene, *constraint);
    CHECK(ctx, vec3f::y(constraint->point_constraint_part.total_lambda) != 0.0f);

    // still fixed, at another offset: the sets are unchanged
    six_dof_constraint::set_limited_axis(*constraint, six_dof_constraint::TRANSLATION_Z, 0.5f, 0.5f);
    CHECK(ctx, vec3f::y(constraint->point_constraint_part.total_lambda) != 0.0f);

    // translation x becomes limited: the point part no longer applies
    six_dof_constraint::set_limited_axis(*constraint, six_dof_constraint::TRANSLATION_X, -1.0f, 1.0f);
    CHECK(ctx, vec3f::exact_equals(constraint->point_constraint_part.total_lambda, vec3f::load(ZERO)));
    CHECK(ctx, vec3f::exact_equals(constraint->rotation_constraint_part.total_lambda, vec3f::load(ZERO)));
    release_constraint_scene(scene);
}

void test_six_dof_constraint_runtime_fixed_and_free_axes(TestCtx &ctx) {
    // a translation axis made fixed at runtime is fixed at 0, and the position solve pulls it there
    {
        ConstraintScene scene = make_constraint_scene();
        RigidBody *ball = nullptr;
        SixDOFConstraint *constraint = make_joint(scene, ball, {});
        ball->center_of_mass_position.x = 2.0f;
        six_dof_constraint::make_fixed_axis(*constraint, six_dof_constraint::TRANSLATION_X);
        CHECK(ctx, constraint->limit_min[0] == 0.0f && constraint->limit_max[0] == 0.0f);
        for (uint32_t i = 0; i < 4; i++) six_dof_constraint::solve_position(*constraint, scene.world->bodies, DT, 1.0f);
        CHECK_NEAR(ctx, ball->center_of_mass_position.x, 0.0f, 1e-5f);
        release_constraint_scene(scene);
    }

    // a rotation axis made free or fixed at runtime reaches the swing twist part
    {
        ConstraintScene scene = make_constraint_scene();
        RigidBody *ball = nullptr;
        SixDOFConstraintSettings settings;
        settings.limit_min[six_dof_constraint::ROTATION_X] = -0.5f;
        settings.limit_max[six_dof_constraint::ROTATION_X] = 0.5f;
        SixDOFConstraint *constraint = make_joint(scene, ball, settings);
        const swing_twist_constraint_part::SwingTwistConstraintPart &part = constraint->swing_twist_constraint_part;
        CHECK_NEAR(ctx, part.sin_twist_half_max_angle, scalar::sin(0.25f), 1e-6f);

        // free: the twist limit is at its +-1 sentinel
        six_dof_constraint::make_free_axis(*constraint, six_dof_constraint::ROTATION_X);
        CHECK(ctx, constraint->limit_min[3] == -scalar::PI && constraint->limit_max[3] == scalar::PI);
        CHECK(ctx, part.sin_twist_half_min_angle == -1.0f && part.sin_twist_half_max_angle == 1.0f);

        // fixed: locked at 0
        six_dof_constraint::make_fixed_axis(*constraint, six_dof_constraint::ROTATION_X);
        CHECK(ctx, constraint->limit_min[3] == 0.0f && constraint->limit_max[3] == 0.0f);
        CHECK(ctx, part.sin_twist_half_min_angle == 0.0f && part.sin_twist_half_max_angle == 0.0f);
        release_constraint_scene(scene);
    }
}

void test_six_dof_constraint_position_solve_uses_fresh_rotations(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, ZERO);
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    SixDOFConstraintSettings settings = all_fixed_settings();
    const constraint_id::ConstraintId id = six_dof_constraint::create(*scene.world, base, ball, settings);
    SixDOFConstraint *constraint = six_dof_constraint::get(*scene.world, id);

    // the ball's center is a unit from the joint; turn it 0.3 about z around the joint: no point error, a rotation error
    ball->quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.3f);
    vec3f::store(ball->center_of_mass_position, vec3f::transform_quat(vec3f::axis_x(), quatf::load(ball->quaternion)));
    CHECK(ctx, vec3f::length(world_point(ball, constraint->local_space_position2)) < 1e-6f);

    six_dof_constraint::solve_position(*constraint, scene.world->bodies, DT, 1.0f);

    // turning the ball back swings the joint point 0.2955 away. the point solve, with its moment arm taken from the
    // turned back ball, splits the correction by 1/m : r^2/I = 1 : 10, so it turns the ball by about
    // 10/11 * 0.2955 = 0.269 and leaves the second order residual of that turn, 0.269^2 / 2 = 0.036
    const float error = vec3f::length(world_point(ball, constraint->local_space_position2));
    CHECK(ctx, error > 0.03f && error < 0.04f);
    release_constraint_scene(scene);
}

void test_six_dof_constraint_fixed_rotation_offsets_euler_zyx(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    SixDOFConstraintSettings settings = all_fixed_settings();
    settings.swing_type = swing_twist_constraint_part::PYRAMID;
    settings.limit_min[3] = settings.limit_max[3] = 0.4f;
    settings.limit_min[4] = settings.limit_max[4] = 0.3f;
    SixDOFConstraint *constraint = make_joint(scene, ball, settings);

    // the rest orientation composes the offsets z * y * x
    const Quatf rest = quatf::multiply(axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), 0.3f), axis_angle(vec3f::make(1.0f, 0.0f, 0.0f), 0.4f));
    quatf::store(ball->quaternion, rest);
    six_dof_constraint::solve_position(*constraint, scene.world->bodies, DT, 1.0f);
    CHECK(ctx, rotation_angle(quatf::multiply(quatf::load(ball->quaternion), quatf::conjugate(rest))) < 1e-4f);
    release_constraint_scene(scene);
}

void test_six_dof_constraint_motor_limits_and_targets(TestCtx &ctx) {
    // rotation motors are limited by the torque limits, translation motors by the force limits
    {
        ConstraintScene scene = make_constraint_scene();
        RigidBody *ball = nullptr;
        SixDOFConstraintSettings settings;
        settings.motor_settings[six_dof_constraint::ROTATION_X].min_torque_limit = -3.0f;
        settings.motor_settings[six_dof_constraint::ROTATION_X].max_torque_limit = 3.0f;
        settings.motor_settings[six_dof_constraint::TRANSLATION_X].min_force_limit = -5.0f;
        settings.motor_settings[six_dof_constraint::TRANSLATION_X].max_force_limit = 5.0f;
        SixDOFConstraint *constraint = make_joint(scene, ball, settings);
        six_dof_constraint::set_motor_state(*constraint, six_dof_constraint::ROTATION_X, motor_settings::VELOCITY);
        six_dof_constraint::set_motor_state(*constraint, six_dof_constraint::TRANSLATION_X, motor_settings::VELOCITY);
        six_dof_constraint::set_target_angular_velocity_cs(*constraint, vec3f::make(100.0f, 0.0f, 0.0f));
        six_dof_constraint::set_target_velocity_cs(*constraint, vec3f::make(100.0f, 0.0f, 0.0f));
        solve_six_dof_velocities(scene, *constraint);
        const float expected_angular = 3.0f * DT * ball->motion_properties.inv_inertia_diagonal.x;
        const float expected_linear = 5.0f * DT * ball->motion_properties.inv_mass;
        CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.x, expected_angular, expected_angular * 1e-4f);
        CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, expected_linear, expected_linear * 1e-4f);
        release_constraint_scene(scene);
    }

    // a target orientation inside the limits is kept as given, here with w < 0
    {
        ConstraintScene scene = make_constraint_scene();
        RigidBody *ball = nullptr;
        SixDOFConstraint *constraint = make_joint(scene, ball, {});
        const Quatf orientation = quatf::scale(axis_angle(vec3f::make(1.0f, 2.0f, 3.0f), 0.2f), -1.0f);
        six_dof_constraint::set_target_orientation_cs(*constraint, orientation);
        CHECK(ctx, same_bits(constraint->target_orientation, orientation));
        release_constraint_scene(scene);
    }

    // a position motor on x keeps the twist about x of a near half turn about y, a quarter turn though its
    // unnormalized length is tiny (2e-12)
    {
        ConstraintScene scene = make_constraint_scene();
        RigidBody *ball = nullptr;
        SixDOFConstraint *constraint = make_joint(scene, ball, {});
        six_dof_constraint::set_motor_state(*constraint, six_dof_constraint::ROTATION_X, motor_settings::POSITION);
        constraint->target_orientation = quatf::normalize(quatf::make(1e-6f, 1.0f, 0.0f, 1e-6f));
        solve_six_dof_velocities(scene, *constraint);
        CHECK(ctx, ball->motion_properties.angular_velocity.x > 0.0f);
        release_constraint_scene(scene);
    }
}

void test_six_dof_constraint_position_and_velocity_motors(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    SixDOFConstraintSettings settings;
    // a damping only spring on translation x: dt c / m = 1 halves the target
    const float mass = 1000.0f * 4.0f / 3.0f * scalar::PI * 0.125f;
    settings.motor_settings[six_dof_constraint::TRANSLATION_X].spring = {spring_settings::STIFFNESS_AND_DAMPING, 0.0f, mass / DT};
    SixDOFConstraint *constraint = make_joint(scene, ball, settings);
    six_dof_constraint::set_motor_state(*constraint, six_dof_constraint::TRANSLATION_X, motor_settings::POSITION_AND_VELOCITY);
    six_dof_constraint::set_motor_state(*constraint, six_dof_constraint::ROTATION_Z, motor_settings::POSITION_AND_VELOCITY);
    six_dof_constraint::set_target_velocity_cs(*constraint, vec3f::make(2.0f, 0.0f, 0.0f));
    six_dof_constraint::set_target_angular_velocity_cs(*constraint, vec3f::make(0.0f, 0.0f, 2.0f));
    solve_six_dof_velocities(scene, *constraint);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, 1.0f, 1e-4f);
    CHECK(ctx, ball->motion_properties.angular_velocity.z > 0.0f);
    release_constraint_scene(scene);
}
