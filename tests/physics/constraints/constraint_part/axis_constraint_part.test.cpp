#include "math/mat4f.h"
#include "math/quatf.h"
#include "math/vec3f.h"
#include "physics/body/dof.h"
#include "physics/body/motion_properties.h"
#include "physics/body/rigid_body.h"
#include "physics/constraints/constraint_part/axis_constraint_part.h"
#include "physics/constraints/constraint_part/part_body.h"
#include "test.h"
#include "test_math.h"

using axis_constraint_part::AxisConstraintPart;
using rigid_body::RigidBody;

// a body with inverse mass `inv_mass` and isotropic inverse inertia `inv_inertia` at `com`
static Float4x4 inverse_inertia(RigidBody &body) {
    Float4x4 out;
    if (body.motion_type != motion_type::MOTION_DYNAMIC) return mat4f::store(out, mat4f::zero());
    return mat4f::store(out, motion_properties::get_world_inverse_inertia(body.motion_properties, quatf::load(body.quaternion),
                                                                          motion_properties::STEP_STAMP_NONE));
}

static void setup(AxisConstraintPart &part, RigidBody &a, RigidBody &b, const Float3 &r1, const Float3 &r2, const Float3 &axis,
                  float bias) {
    const Float4x4 inv_i_a = inverse_inertia(a);
    const Float4x4 inv_i_b = inverse_inertia(b);
    axis_constraint_part::calculate_constraint_properties(part, a, b, a.motion_properties.inv_mass, b.motion_properties.inv_mass,
                                                          mat4f::load(inv_i_a), mat4f::load(inv_i_b), vec3f::load(r1), vec3f::load(r2), vec3f::load(axis), bias);
}

static const Float3 X_AXIS = {1.0f, 0.0f, 0.0f};

void test_axis_constraint_part_create_reset(TestCtx &ctx) {
    AxisConstraintPart part = axis_constraint_part::create();
    CHECK(ctx, !axis_constraint_part::is_active(part));
    CHECK(ctx, part.total_lambda == 0.0f && part.spring.softness == 0.0f && part.spring.bias == 0.0f);

    part.effective_mass = 2.0f;
    part.total_lambda = 3.0f;
    part.r2_x_axis = {1.0f, 2.0f, 3.0f};
    CHECK(ctx, axis_constraint_part::is_active(part));
    axis_constraint_part::reset(part);
    CHECK(ctx, !axis_constraint_part::is_active(part) && part.total_lambda == 0.0f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(part.r2_x_axis), vec3f::zero()));

    axis_constraint_part::set_total_lambda(part, 1.25f);
    CHECK(ctx, axis_constraint_part::get_total_lambda_value(part) == 1.25f);
    part.effective_mass = 1.0f;
    axis_constraint_part::deactivate(part);
    CHECK(ctx, !axis_constraint_part::is_active(part) && part.total_lambda == 0.0f);
}

void test_axis_constraint_part_effective_mass(TestCtx &ctx) {
    // K = m1^-1 + (r1 x n) . I1^-1 (r1 x n) + m2^-1 + (r2 x n) . I2^-1 (r2 x n)
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {2.0f, 2.0f, 2.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {4.0f, 4.0f, 4.0f}, {2.0f, 0.0f, 0.0f});
    AxisConstraintPart part = axis_constraint_part::create();
    setup(part, a, b, {1.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.5f}, X_AXIS, 0.25f);

    // (1, 1, 0) x x = (0, 0, -1); (-1, 0, 0.5) x x = (0, 0.5, 0)
    CHECK(ctx, vec3f::exact_equals(vec3f::load(part.r1_plus_u_x_axis), vec3f::make(0.0f, 0.0f, -1.0f)));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(part.r2_x_axis), vec3f::make(0.0f, 0.5f, 0.0f)));
    CHECK(ctx, vec3f::distance(vec3f::load(part.inv_i1_r1_plus_u_x_axis), vec3f::make(0.0f, 0.0f, -2.0f)) < 1e-6f);
    CHECK(ctx, vec3f::distance(vec3f::load(part.inv_i2_r2_x_axis), vec3f::make(0.0f, 2.0f, 0.0f)) < 1e-6f);
    const float k = 1.0f + 2.0f + 0.5f + 4.0f * 0.25f;
    CHECK_NEAR(ctx, part.effective_mass, 1.0f / k, 1e-6f);
    CHECK(ctx, part.spring.bias == 0.25f && part.spring.softness == 0.0f);

    // a static body adds nothing
    RigidBody wall = make_body(motion_type::MOTION_STATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f});
    setup(part, a, wall, {1.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.5f}, X_AXIS, 0.0f);
    CHECK_NEAR(ctx, part.effective_mass, 1.0f / 3.0f, 1e-6f);

    // two static bodies: nothing to solve, the part switches off and drops its impulse
    RigidBody floor = make_body(motion_type::MOTION_STATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
    part.total_lambda = 5.0f;
    setup(part, floor, wall, {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, X_AXIS, 0.0f);
    CHECK(ctx, !axis_constraint_part::is_active(part) && part.total_lambda == 0.0f);
}

void test_axis_constraint_part_contact_removes_approach_velocity(TestCtx &ctx) {
    // a (mass 1) moving +2 meets b (mass 2) moving -1 along x; zero total momentum
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {1.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 0.0f});
    a.motion_properties.linear_velocity = {2.0f, 0.0f, 0.0f};
    b.motion_properties.linear_velocity = {-1.0f, 0.0f, 0.0f};
    AxisConstraintPart part = axis_constraint_part::create();
    setup(part, a, b, {0.5f, 0.0f, 0.0f}, {-0.5f, 0.0f, 0.0f}, X_AXIS, 0.0f);
    CHECK_NEAR(ctx, part.effective_mass, 2.0f / 3.0f, 1e-6f);

    // one solve, a contact pushes only: both stop dead, momentum is still zero. the lambda is K^-1 (v1 - v2) . n = 2/3 * 3
    CHECK(ctx, axis_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(X_AXIS), 0.0f, __builtin_inff()));
    CHECK_NEAR(ctx, part.total_lambda, 2.0f, 1e-6f);
    CHECK_NEAR(ctx, a.motion_properties.linear_velocity.x, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, b.motion_properties.linear_velocity.x, 0.0f, 1e-6f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(a.motion_properties.angular_velocity), vec3f::zero()));

    // solved: a second iteration has nothing to add
    CHECK(ctx, !axis_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(X_AXIS), 0.0f, __builtin_inff()));
    CHECK_NEAR(ctx, part.total_lambda, 2.0f, 1e-6f);

    // next step the bodies approach again; the warm start reapplies the stored lambda and stops them up front
    a.motion_properties.linear_velocity = {2.0f, 0.0f, 0.0f};
    b.motion_properties.linear_velocity = {-1.0f, 0.0f, 0.0f};
    axis_constraint_part::warm_start(part, a, b, a.motion_properties.inv_mass, b.motion_properties.inv_mass, vec3f::load(X_AXIS), 1.0f);
    CHECK_NEAR(ctx, a.motion_properties.linear_velocity.x, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, b.motion_properties.linear_velocity.x, 0.0f, 1e-6f);
    CHECK(ctx, !axis_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(X_AXIS), 0.0f, __builtin_inff()));

    // a warm start ratio scales the stored impulse; half of it removes half the approach
    a.motion_properties.linear_velocity = {2.0f, 0.0f, 0.0f};
    b.motion_properties.linear_velocity = {-1.0f, 0.0f, 0.0f};
    axis_constraint_part::warm_start(part, a, b, 1.0f, 0.5f, vec3f::load(X_AXIS), 0.5f);
    CHECK_NEAR(ctx, part.total_lambda, 1.0f, 1e-6f);
    CHECK_NEAR(ctx, a.motion_properties.linear_velocity.x, 1.0f, 1e-6f);
    CHECK_NEAR(ctx, b.motion_properties.linear_velocity.x, -0.5f, 1e-6f);
}

void test_axis_constraint_part_contact_never_pulls(TestCtx &ctx) {
    // separating bodies: the unclamped lambda is negative, the contact clamps it to 0 and leaves them be
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 0.0f});
    a.motion_properties.linear_velocity = {-1.0f, 0.0f, 0.0f};
    b.motion_properties.linear_velocity = {1.0f, 0.0f, 0.0f};
    AxisConstraintPart part = axis_constraint_part::create();
    setup(part, a, b, {0.5f, 0.0f, 0.0f}, {-0.5f, 0.0f, 0.0f}, X_AXIS, 0.0f);
    CHECK(ctx, !axis_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(X_AXIS), 0.0f, __builtin_inff()));
    CHECK(ctx, a.motion_properties.linear_velocity.x == -1.0f && b.motion_properties.linear_velocity.x == 1.0f);

    // a friction bound clamps the other way too
    a.motion_properties.linear_velocity = {3.0f, 0.0f, 0.0f};
    b.motion_properties.linear_velocity = {0.0f, 0.0f, 0.0f};
    CHECK(ctx, axis_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(X_AXIS), -0.5f, 0.5f));
    CHECK(ctx, part.total_lambda == 0.5f);
    CHECK_NEAR(ctx, a.motion_properties.linear_velocity.x, 2.5f, 1e-6f);
    CHECK_NEAR(ctx, b.motion_properties.linear_velocity.x, 0.5f, 1e-6f);
}

void test_axis_constraint_part_off_center_contact(TestCtx &ctx) {
    // a box hits a static wall with a point above its center: after one solve the contact point no longer
    // approaches the wall, and part of the stop goes into spin
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {3.0f, 3.0f, 3.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody wall = make_body(motion_type::MOTION_STATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    a.motion_properties.linear_velocity = {1.0f, 0.0f, 0.0f};
    const Float3 r1 = {0.5f, 0.5f, 0.0f};
    AxisConstraintPart part = axis_constraint_part::create();
    setup(part, a, wall, r1, {-0.5f, 0.5f, 0.0f}, X_AXIS, 0.0f);
    CHECK(ctx, axis_constraint_part::solve_velocity_constraint(part, a, wall, vec3f::load(X_AXIS), 0.0f, __builtin_inff()));

    Vec3f point_velocity = vec3f::cross(vec3f::load(a.motion_properties.angular_velocity), vec3f::load(r1));
    point_velocity = vec3f::add(point_velocity, vec3f::load(a.motion_properties.linear_velocity));
    CHECK_NEAR(ctx, vec3f::x(point_velocity), 0.0f, 1e-6f);
    CHECK(ctx, a.motion_properties.linear_velocity.x > 0.0f);
    CHECK(ctx, a.motion_properties.angular_velocity.z > 0.0f);
}

void test_axis_constraint_part_kinematic_pushes_dynamic(TestCtx &ctx) {
    // a kinematic body moving -1 into a resting dynamic one: the dynamic body takes the kinematic's velocity
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody pusher = make_body(motion_type::MOTION_KINEMATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    pusher.motion_properties.linear_velocity = {-1.0f, 0.0f, 0.0f};
    AxisConstraintPart part = axis_constraint_part::create();
    setup(part, a, pusher, {0.5f, 0.0f, 0.0f}, {-0.5f, 0.0f, 0.0f}, X_AXIS, 0.0f);
    CHECK(ctx, axis_constraint_part::solve_velocity_constraint(part, a, pusher, vec3f::load(X_AXIS), 0.0f, __builtin_inff()));
    CHECK_NEAR(ctx, a.motion_properties.linear_velocity.x, -1.0f, 1e-6f);
    CHECK(ctx, pusher.motion_properties.linear_velocity.x == -1.0f);
    CHECK_NEAR(ctx, part.total_lambda, 2.0f, 1e-6f);
}

void test_axis_constraint_part_spinning_kinematic_off_center(TestCtx &ctx) {
    // a kinematic box spinning about z hits a resting dynamic one with its top right corner, r1 = (1, 1, 0).
    // the corner moves at w x r1 = (0, 0, 2) x (1, 1, 0) = (-2, 2, 0), closing at 2 along the axis; the hit is
    // straight below the dynamic body's centre of mass (r2 x axis = 0), so it leaves with the corner's normal speed
    RigidBody spinner = make_body(motion_type::MOTION_KINEMATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody resting = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {1.0f, 1.0f, 1.0f}, {1.0f, 2.0f, 0.0f});
    spinner.motion_properties.angular_velocity = {0.0f, 0.0f, 2.0f};
    const Float3 axis = {0.0f, 1.0f, 0.0f};
    AxisConstraintPart part = axis_constraint_part::create();
    setup(part, spinner, resting, {1.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, axis, 0.0f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(part.r1_plus_u_x_axis), vec3f::make(0.0f, 0.0f, 1.0f)));
    CHECK(ctx, axis_constraint_part::solve_velocity_constraint(part, spinner, resting, vec3f::load(axis), 0.0f, __builtin_inff()));
    CHECK_NEAR(ctx, part.total_lambda, 2.0f / 0.5f, 1e-6f);
    CHECK(ctx, vec3f::distance(vec3f::load(resting.motion_properties.linear_velocity), vec3f::make(0.0f, 2.0f, 0.0f)) < 1e-6f);
    CHECK(ctx, vec3f::length(vec3f::load(resting.motion_properties.angular_velocity)) < 1e-6f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(spinner.motion_properties.angular_velocity), vec3f::make(0.0f, 0.0f, 2.0f)));
}

void test_axis_constraint_part_respects_locked_translation(TestCtx &ctx) {
    // an impulse along a diagonal leaves a locked axis at rest
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody wall = make_body(motion_type::MOTION_STATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    a.motion_properties.allowed_degrees_of_freedom = dof::DOF_ALL & ~dof::TRANSLATION_Y;
    a.motion_properties.linear_velocity = {1.0f, 0.0f, 0.0f};
    const Float3 axis = {0.6f, 0.8f, 0.0f};
    AxisConstraintPart part = axis_constraint_part::create();
    setup(part, a, wall, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, axis, 0.0f);
    CHECK(ctx, axis_constraint_part::solve_velocity_constraint(part, a, wall, vec3f::load(axis), 0.0f, __builtin_inff()));
    CHECK(ctx, a.motion_properties.linear_velocity.y == 0.0f);
    CHECK_NEAR(ctx, a.motion_properties.linear_velocity.x, 1.0f - 0.6f * 0.6f, 1e-6f);
}

void test_axis_constraint_part_position_solve(TestCtx &ctx) {
    // two unit-mass bodies 0.1 into each other along x: one full baumgarte step pushes each back half the depth
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f}, {0.9f, 0.0f, 0.0f});
    AxisConstraintPart part = axis_constraint_part::create();
    setup(part, a, b, {0.45f, 0.0f, 0.0f}, {-0.45f, 0.0f, 0.0f}, X_AXIS, 0.0f);

    CHECK(ctx, !axis_constraint_part::solve_position_constraint(part, a, b, vec3f::load(X_AXIS), 0.0f, 1.0f));
    CHECK(ctx, axis_constraint_part::solve_position_constraint(part, a, b, vec3f::load(X_AXIS), -0.1f, 1.0f));
    CHECK_NEAR(ctx, a.center_of_mass_position.x, -0.05f, 1e-6f);
    CHECK_NEAR(ctx, b.center_of_mass_position.x, 0.95f, 1e-6f);
    CHECK(ctx, quatf::exact_equals(quatf::load(a.quaternion), quatf::identity()));
    // velocities are untouched: the correction does not add momentum
    CHECK(ctx, vec3f::exact_equals(vec3f::load(a.motion_properties.linear_velocity), vec3f::zero()));

    // a soft constraint leaves position drift to its velocity bias
    const Float4x4 inv_i = inverse_inertia(a);
    axis_constraint_part::calculate_constraint_properties_with_frequency_and_damping(
        part, 1.0f / 60.0f, a, b, 1.0f, 1.0f, mat4f::load(inv_i), mat4f::load(inv_i), vec3f::make(0.45f, 0.0f, 0.0f), vec3f::make(-0.45f, 0.0f, 0.0f), vec3f::load(X_AXIS), 0.0f, -0.1f, 2.0f,
        1.0f);
    CHECK(ctx, axis_constraint_part::is_active(part) && spring_part::is_spring_active(part.spring));
    CHECK(ctx, part.effective_mass < 0.5f);
    CHECK(ctx, !axis_constraint_part::solve_position_constraint(part, a, b, vec3f::load(X_AXIS), -0.1f, 1.0f));

    // the settings variant agrees with the direct one
    AxisConstraintPart from_settings = axis_constraint_part::create();
    spring_settings::SpringSettings settings = spring_settings::create();
    settings.mode = spring_settings::STIFFNESS_AND_DAMPING;
    settings.frequency_or_stiffness = 80.0f;
    settings.damping = 3.0f;
    axis_constraint_part::calculate_constraint_properties_with_settings(from_settings, 1.0f / 60.0f, a, b, 1.0f, 1.0f, mat4f::load(inv_i),
                                                                        mat4f::load(inv_i), vec3f::make(0.45f, 0.0f, 0.0f), vec3f::make(-0.45f, 0.0f, 0.0f),
                                                                        vec3f::load(X_AXIS), 0.0f, -0.1f, settings);
    axis_constraint_part::calculate_constraint_properties_with_stiffness_and_damping(
        part, 1.0f / 60.0f, a, b, 1.0f, 1.0f, mat4f::load(inv_i), mat4f::load(inv_i), vec3f::make(0.45f, 0.0f, 0.0f), vec3f::make(-0.45f, 0.0f, 0.0f), vec3f::load(X_AXIS), 0.0f, -0.1f, 80.0f,
        3.0f);
    CHECK(ctx, part.effective_mass == from_settings.effective_mass && part.spring.bias == from_settings.spring.bias);
}

// crashcat axis-constraint-part.test.ts: jolt clamps as min(max(lambda, min), max), so inverted bounds (the ccd friction
// limit of a contact whose normal lambda came out negative) give the max bound
void test_axis_constraint_part_inverted_bounds_clamp_to_the_max(TestCtx &ctx) {
    RigidBody ground = make_body(motion_type::MOTION_STATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody dynamic = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {1.0f, 1.0f, 1.0f}, {0.0f, 2.0f, 0.0f});
    // b falls onto a at 3, so the unclamped lambda is positive (3 x b's mass)
    dynamic.motion_properties.linear_velocity = {0.0f, -3.0f, 0.0f};

    const Float3 axis = {0.0f, 1.0f, 0.0f};
    AxisConstraintPart part = axis_constraint_part::create();
    setup(part, ground, dynamic, {0.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, axis, 0.0f);

    axis_constraint_part::solve_velocity_constraint(part, ground, dynamic, vec3f::load(axis), 1.0f, -1.0f);

    CHECK(ctx, part.total_lambda == -1.0f);
    CHECK_NEAR(ctx, dynamic.motion_properties.linear_velocity.y, -3.0f - 0.5f, 1e-6f);
}
