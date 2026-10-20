#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "physics/constraints/constraint_test_scene.h"
#include "tumble/physics/constraints/swing_twist_constraint.h"
#include "test.h"
#include "test_math.h"

using rigid_body::RigidBody;
using swing_twist_constraint::SwingTwistConstraint;

static const Float3 GRAVITY = {0.0f, -9.81f, 0.0f};
static const Float3 ZERO = {0.0f, 0.0f, 0.0f};
static const float DT = 1.0f / 60.0f;
/** the scene's sphere: 2/5 m r^2 with m = 1000 * 4/3 pi 0.125 and r = 0.5 */
static const float SPHERE_INERTIA = 0.4f * (1000.0f * 4.0f / 3.0f * scalar::PI * 0.125f) * 0.25f;

struct SwingTwistAngles {
    float swing;
    float twist;
};

/** the swing angle (away from the twist axis) and signed twist angle of the constraint's current rotation */
static SwingTwistAngles angles(const SwingTwistConstraint &constraint, const bodies::Bodies &bodies) {
    Quatf swing, twist;
    swing_twist_constraint_part::get_swing_twist(swing, twist, swing_twist_constraint::get_rotation_in_constraint_space(constraint, bodies));
    return {2.0f * scalar::acos(scalar::min(1.0f, scalar::abs(quatf::w(swing)))), 2.0f * scalar::atan2(quatf::x(twist), quatf::w(twist))};
}

/** a ball on a static base, both centered on the pivot, twist axis world x, plane axis world z */
static SwingTwistConstraint *make_joint(ConstraintScene &scene, RigidBody *&ball,
                                        const swing_twist_constraint::SwingTwistConstraintSettings &settings) {
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, ZERO);
    ball = add_body(scene, motion_type::MOTION_DYNAMIC, ZERO);
    const constraint_id::ConstraintId id = swing_twist_constraint::create(scene.constraints, scene.bodies, base, ball, settings);
    scene.ids.push(id);
    return swing_twist_constraint::get(scene.constraints, id);
}

void test_swing_twist_constraint_create(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI / 2.0f));
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, -2.0f, 0.0f}, stored_axis_angle(vec3f::make(1.0f, 1.0f, 0.0f), 0.6f));

    // world space: twist down, plane along x; the constraint frames taken to each body's space
    const constraint_id::ConstraintId id =
        swing_twist_constraint::create(scene.constraints, scene.bodies, a, b,
                                       {.position1 = {0.0f, -1.0f, 0.0f},
                                        .position2 = {0.0f, -1.0f, 0.0f},
                                        .twist_axis1 = {0.0f, -2.0f, 0.0f},
                                        .plane_axis1 = {3.0f, 0.0f, 0.0f},
                                        .twist_axis2 = {0.0f, -2.0f, 0.0f},
                                        .plane_axis2 = {3.0f, 0.0f, 0.0f},
                                        .normal_half_cone_angle = 0.5f,
                                        .plane_half_cone_angle = 0.25f,
                                        .twist_min_angle = -0.1f,
                                        .twist_max_angle = 0.2f});
    SwingTwistConstraint *constraint = swing_twist_constraint::get(scene.constraints, id);
    CHECK(ctx, constraint != nullptr && constraint_id::type(id) == constraint_id::SWING_TWIST);
    CHECK(ctx, a->constraint_ids.length == 1 && b->constraint_ids.length == 1);
    CHECK(ctx, vec3f::distance(world_point(a, constraint->local_space_position1), vec3f::make(0.0f, -1.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(world_point(b, constraint->local_space_position2), vec3f::make(0.0f, -1.0f, 0.0f)) < 1e-5f);

    // constraint x is the twist axis and z the plane axis, in world space through either body
    const Vec3f twist1 = vec3f::transform_quat(vec3f::axis_x(), quatf::multiply(quatf::load(a->quaternion), constraint->constraint_to_body1));
    CHECK(ctx, vec3f::distance(twist1, vec3f::make(0.0f, -1.0f, 0.0f)) < 1e-5f);
    const Vec3f plane2 = vec3f::transform_quat(vec3f::axis_z(), quatf::multiply(quatf::load(b->quaternion), constraint->constraint_to_body2));
    CHECK(ctx, vec3f::distance(plane2, vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);

    // the rest pose is the identity in constraint space
    const SwingTwistAngles rest = angles(*constraint, scene.bodies);
    CHECK(ctx, rest.swing < 1e-3f && scalar::abs(rest.twist) < 1e-3f);

    // the limits reach the part: twist half angles, the plane angle about y and the normal angle about z
    const swing_twist_constraint_part::SwingTwistConstraintPart &part = constraint->swing_twist_constraint_part;
    CHECK_NEAR(ctx, part.sin_twist_half_max_angle, scalar::sin(0.1f), 1e-6f);
    CHECK_NEAR(ctx, part.sin_twist_half_min_angle, scalar::sin(-0.05f), 1e-6f);
    CHECK_NEAR(ctx, part.swing_y_half_max_angle, 0.125f, 1e-6f);
    CHECK_NEAR(ctx, part.swing_z_half_max_angle, 0.25f, 1e-6f);
    swing_twist_constraint::set_normal_half_cone_angle(*constraint, 0.3f);
    CHECK_NEAR(ctx, part.swing_z_half_max_angle, 0.15f, 1e-6f);
    CHECK_NEAR(ctx, part.swing_z_half_min_angle, -0.15f, 1e-6f);
    swing_twist_constraint::set_plane_half_cone_angle(*constraint, 0.6f);
    CHECK_NEAR(ctx, part.swing_y_half_max_angle, 0.3f, 1e-6f);
    swing_twist_constraint::set_twist_min_angle(*constraint, -0.4f);
    swing_twist_constraint::set_twist_max_angle(*constraint, 0.4f);
    CHECK_NEAR(ctx, part.sin_twist_half_min_angle, scalar::sin(-0.2f), 1e-6f);
    CHECK_NEAR(ctx, part.sin_twist_half_max_angle, scalar::sin(0.2f), 1e-6f);

    // local space positions are as given
    const constraint_id::ConstraintId local_id = swing_twist_constraint::create(
        scene.constraints, scene.bodies, a, b, {.position1 = {0.5f, 0.0f, 0.0f}, .space = constraints::SPACE_LOCAL});
    const SwingTwistConstraint *local = swing_twist_constraint::get(scene.constraints, local_id);
    CHECK(ctx, vec3f::exact_equals(local->local_space_position1, vec3f::make(0.5f, 0.0f, 0.0f)));
    CHECK(ctx, same_rotation(local->constraint_to_body1, quatf::make(0.0f, 0.0f, 0.0f, 1.0f)));
    CHECK(ctx, swing_twist_constraint::get(scene.constraints, constraint_id::ser(0, constraint_id::CONE, 0)) == nullptr);
    release_constraint_scene(scene);
}

void test_swing_twist_constraint_target_orientation(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    SwingTwistConstraint *constraint = make_joint(scene, ball,
                                                  {.normal_half_cone_angle = 0.4f,
                                                   .plane_half_cone_angle = 0.4f,
                                                   .twist_min_angle = -0.3f,
                                                   .twist_max_angle = 0.3f});

    // inside the limits the target is kept as given
    const Float4 inside = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.2f);
    swing_twist_constraint::set_target_orientation_cs(*constraint, quatf::load(inside));
    CHECK(ctx, same_bits(constraint->target_orientation, quatf::load(inside)));

    // outside them it is clamped to the cone and the twist range
    swing_twist_constraint::set_target_orientation_cs(*constraint, axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 1.0f));
    Quatf swing, twist;
    swing_twist_constraint_part::get_swing_twist(swing, twist, constraint->target_orientation);
    CHECK_NEAR(ctx, 2.0f * scalar::acos(scalar::min(1.0f, scalar::abs(quatf::w(swing)))), 0.4f, 1e-4f);
    swing_twist_constraint::set_target_orientation_cs(*constraint, axis_angle(vec3f::make(1.0f, 0.0f, 0.0f), -1.0f));
    swing_twist_constraint_part::get_swing_twist(swing, twist, constraint->target_orientation);
    CHECK_NEAR(ctx, 2.0f * scalar::atan2(quatf::x(twist), quatf::w(twist)), -0.3f, 1e-4f);

    // body space: R2 = R1 * orientation, here with both constraint frames the identity
    swing_twist_constraint::set_target_orientation_bs(*constraint, quatf::load(inside));
    CHECK(ctx, same_rotation(constraint->target_orientation, quatf::load(inside)));

    // changing a motor's state clears its warm start
    constraint->motor_constraint_parts[0].total_lambda = 1.0f;
    constraint->motor_constraint_parts[1].total_lambda = 1.0f;
    swing_twist_constraint::set_twist_motor_state(*constraint, motor_settings::VELOCITY);
    CHECK(ctx, constraint->motor_constraint_parts[0].total_lambda == 0.0f && constraint->motor_constraint_parts[1].total_lambda == 1.0f);
    swing_twist_constraint::set_swing_motor_state(*constraint, motor_settings::VELOCITY);
    CHECK(ctx, constraint->motor_constraint_parts[1].total_lambda == 0.0f);
    release_constraint_scene(scene);
}

void test_swing_twist_constraint_clamps_swing(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    const SwingTwistConstraint *constraint = make_joint(scene, ball,
                                                        {.normal_half_cone_angle = 0.4f,
                                                         .plane_half_cone_angle = 0.4f,
                                                         .twist_min_angle = -0.3f,
                                                         .twist_max_angle = 0.3f});
    // a swing about z, the normal axis
    ball->motion_properties.angular_velocity = {0.0f, 0.0f, 3.0f};

    float widest = 0.0f;
    float hardest = 0.0f;
    for (int i = 0; i < 90; i++) {
        step_constraint_scene(scene, DT, vec3f::zero());
        widest = scalar::max(widest, angles(*constraint, scene.bodies).swing);
        hardest = scalar::max(hardest, scalar::abs(swing_twist_constraint::get_total_lambda_swing_y(*constraint)) +
                                           scalar::abs(swing_twist_constraint::get_total_lambda_swing_z(*constraint)));
    }
    // unconstrained it would turn 4.5 radians; it stops at the cone, overshooting by at most a step's turn
    CHECK(ctx, widest < 0.4f + 3.0f * DT);
    const SwingTwistAngles end = angles(*constraint, scene.bodies);
    CHECK_NEAR(ctx, end.swing, 0.4f, 1e-2f);
    CHECK(ctx, scalar::abs(end.twist) < 1e-3f);
    CHECK(ctx, vec3f::length(vec3f::load(ball->motion_properties.angular_velocity)) < 1e-2f);
    // the limit took out the spin's angular momentum, I * 3, in about one step
    CHECK(ctx, hardest > 2.0f * SPHERE_INERTIA && hardest < 4.0f * SPHERE_INERTIA);
    release_constraint_scene(scene);
}

void test_swing_twist_constraint_clamps_twist(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    const SwingTwistConstraint *constraint = make_joint(scene, ball,
                                                        {.normal_half_cone_angle = 0.4f,
                                                         .plane_half_cone_angle = 0.4f,
                                                         .twist_min_angle = -0.3f,
                                                         .twist_max_angle = 0.2f});
    ball->motion_properties.angular_velocity = {3.0f, 0.0f, 0.0f};

    float most = 0.0f;
    for (int i = 0; i < 90; i++) {
        step_constraint_scene(scene, DT, vec3f::zero());
        most = scalar::max(most, angles(*constraint, scene.bodies).twist);
    }
    CHECK(ctx, most < 0.2f + 3.0f * DT);
    SwingTwistAngles end = angles(*constraint, scene.bodies);
    CHECK_NEAR(ctx, end.twist, 0.2f, 1e-2f);
    CHECK(ctx, end.swing < 1e-3f);
    CHECK(ctx, vec3f::length(vec3f::load(ball->motion_properties.angular_velocity)) < 1e-2f);

    // and the other way to the asymmetric lower limit
    ball->motion_properties.angular_velocity = {-3.0f, 0.0f, 0.0f};
    float least = 0.0f;
    float hardest = 0.0f;
    for (int i = 0; i < 90; i++) {
        step_constraint_scene(scene, DT, vec3f::zero());
        least = scalar::min(least, angles(*constraint, scene.bodies).twist);
        hardest = scalar::max(hardest, scalar::abs(swing_twist_constraint::get_total_lambda_twist(*constraint)));
    }
    CHECK(ctx, least > -0.3f - 3.0f * DT);
    end = angles(*constraint, scene.bodies);
    CHECK_NEAR(ctx, end.twist, -0.3f, 1e-2f);
    CHECK(ctx, hardest > 2.0f * SPHERE_INERTIA && hardest < 4.0f * SPHERE_INERTIA);
    release_constraint_scene(scene);
}

void test_swing_twist_constraint_pendulum(TestCtx &ctx) {
    // a ball on a rod held to a 20 degree cone about straight down with a little twist, kicked over the top
    ConstraintScene scene = make_constraint_scene();
    RigidBody *pivot = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 10.0f, 0.0f});
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 9.0f, 0.0f});
    const Float3 pivot_point = {0.0f, 10.0f, 0.0f};
    const float half_cone = 20.0f * scalar::DEGREES_TO_RADIANS;
    const constraint_id::ConstraintId id = swing_twist_constraint::create(scene.constraints, scene.bodies, pivot, ball,
                                                                          {.position1 = pivot_point,
                                                                           .position2 = pivot_point,
                                                                           .twist_axis1 = {0.0f, -1.0f, 0.0f},
                                                                           .plane_axis1 = {1.0f, 0.0f, 0.0f},
                                                                           .twist_axis2 = {0.0f, -1.0f, 0.0f},
                                                                           .plane_axis2 = {1.0f, 0.0f, 0.0f},
                                                                           .normal_half_cone_angle = half_cone,
                                                                           .plane_half_cone_angle = half_cone,
                                                                           .twist_min_angle = -0.1f,
                                                                           .twist_max_angle = 0.1f});
    scene.ids.push(id);
    const SwingTwistConstraint *constraint = swing_twist_constraint::get(scene.constraints, id);
    ball->motion_properties.linear_velocity = {5.0f, 0.0f, 2.0f};
    ball->motion_properties.angular_velocity = {0.0f, 4.0f, 0.0f};

    float widest = 0.0f;
    float worst_twist = 0.0f;
    float worst_anchor = 0.0f;
    for (int i = 0; i < 180; i++) {
        step_constraint_scene(scene, DT, vec3f::load(GRAVITY));
        const SwingTwistAngles now = angles(*constraint, scene.bodies);
        widest = scalar::max(widest, now.swing);
        worst_twist = scalar::max(worst_twist, scalar::abs(now.twist));
        worst_anchor = scalar::max(worst_anchor, vec3f::distance(world_point(ball, constraint->local_space_position2), vec3f::load(pivot_point)));
    }
    // it reaches the cone and stays in it, overshooting by at most a step's swing (about 5.4 rad/s)
    CHECK(ctx, widest > half_cone - 2e-2f);
    CHECK(ctx, widest < half_cone + 5.4f * DT);
    CHECK(ctx, worst_twist < 0.1f + 4.0f * DT);
    CHECK(ctx, worst_anchor < 5e-2f);
    CHECK(ctx, vec3f::distance(world_point(ball, constraint->local_space_position2), vec3f::load(pivot_point)) < 1e-2f);
    CHECK_NEAR(ctx, vec3f::distance(vec3f::load(ball->center_of_mass_position), vec3f::load(pivot_point)), 1.0f, 1e-2f);
    release_constraint_scene(scene);
}

void test_swing_twist_constraint_twist_motor_and_friction(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *ball = nullptr;
    SwingTwistConstraint *constraint = make_joint(scene, ball,
                                                  {.normal_half_cone_angle = 0.4f,
                                                   .plane_half_cone_angle = 0.4f,
                                                   .twist_min_angle = -3.0f,
                                                   .twist_max_angle = 3.0f});
    swing_twist_constraint::set_twist_motor_state(*constraint, motor_settings::VELOCITY);
    swing_twist_constraint::set_target_angular_velocity_cs(*constraint, vec3f::make(1.0f, 0.0f, 0.0f));

    // a twist velocity motor spins the ball about the twist axis at the target
    for (int i = 0; i < 30; i++) step_constraint_scene(scene, DT, vec3f::zero());
    CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.x, 1.0f, 1e-3f);
    CHECK(ctx, scalar::abs(ball->motion_properties.angular_velocity.y) < 1e-4f);
    CHECK_NEAR(ctx, angles(*constraint, scene.bodies).twist, 0.5f, 2e-2f);
    const Vec3f motor_lambda = swing_twist_constraint::get_total_lambda_motor(*constraint);
    CHECK(ctx, scalar::abs(vec3f::x(motor_lambda)) < 1e-3f && vec3f::y(motor_lambda) == 0.0f);

    // with the motor off, a friction torque of I slows it by 1 rad/s each second
    swing_twist_constraint::set_twist_motor_state(*constraint, motor_settings::OFF);
    swing_twist_constraint::set_max_friction_torque(*constraint, SPHERE_INERTIA);
    ball->motion_properties.angular_velocity = {-2.0f, 0.0f, 0.0f};
    for (int i = 0; i < 60; i++) step_constraint_scene(scene, DT, vec3f::zero());
    CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.x, -1.0f, 1e-3f);
    release_constraint_scene(scene);
}

void test_swing_twist_constraint_position_and_velocity_motors(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    swing_twist_constraint::SwingTwistConstraintSettings settings;
    settings.normal_half_cone_angle = scalar::PI / 2.0f;
    settings.plane_half_cone_angle = scalar::PI / 2.0f;
    settings.twist_min_angle = -scalar::PI;
    settings.twist_max_angle = scalar::PI;
    // a damping only twist spring: dt c / I = 1 halves the target
    const float inertia = 1.0f / ball->motion_properties.inv_inertia_diagonal.x;
    settings.twist_motor_settings.spring = {spring_settings::STIFFNESS_AND_DAMPING, 0.0f, inertia / DT};
    const constraint_id::ConstraintId id = swing_twist_constraint::create(scene.constraints, scene.bodies, base, ball, settings);
    SwingTwistConstraint *constraint = swing_twist_constraint::get(scene.constraints, id);
    swing_twist_constraint::set_twist_motor_state(*constraint, motor_settings::POSITION_AND_VELOCITY);
    swing_twist_constraint::set_swing_motor_state(*constraint, motor_settings::POSITION_AND_VELOCITY);
    swing_twist_constraint::set_target_angular_velocity_cs(*constraint, vec3f::make(2.0f, 0.0f, 2.0f));

    swing_twist_constraint::setup_velocity(*constraint, scene.bodies, DT);
    swing_twist_constraint::warm_start_velocity(*constraint, scene.bodies, 1.0f);
    for (uint32_t i = 0; i < 10; i++) swing_twist_constraint::solve_velocity(*constraint, scene.bodies, DT);
    CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.x, 1.0f, 1e-4f);
    CHECK(ctx, ball->motion_properties.angular_velocity.z > 0.0f);
    release_constraint_scene(scene);
}
