#include "math/mat4f.h"
#include "math/quatf.h"
#include "math/vec3f.h"
#include "physics/body/motion_properties.h"
#include "physics/body/rigid_body.h"
#include "physics/constraints/constraint_part/contact_constraint_part.h"
#include "physics/constraints/constraint_part/part_body.h"
#include "test.h"
#include "test_math.h"

using contact_constraint_part::ContactConstraintPart;
using motion_type::MOTION_DYNAMIC;
using motion_type::MOTION_KINEMATIC;
using motion_type::MOTION_STATIC;
using rigid_body::RigidBody;

/** isotropic inverse inertia */
static Mat4f inverse_inertia(float inv_inertia) {
    return mat4f::from_scaling(vec3f::make(inv_inertia, inv_inertia, inv_inertia));
}

void test_contact_constraint_part_effective_mass(TestCtx &ctx) {
    // K = m1^-1 + (r1 x n) . I1^-1 (r1 x n) + m2^-1 + (r2 x n) . I2^-1 (r2 x n)
    const Vec3f r1 = vec3f::make(1.0f, 1.0f, 0.0f);
    const Vec3f r2 = vec3f::make(-1.0f, 0.0f, 0.5f);
    ContactConstraintPart part = {};
    contact_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_DYNAMIC>(
        part, 1.0f, inverse_inertia(2.0f), r1, 0.5f, inverse_inertia(4.0f), r2, vec3f::axis_x(), 0.25f);

    // (1, 1, 0) x x = (0, 0, -1); (-1, 0, 0.5) x x = (0, 0.5, 0)
    CHECK(ctx, vec3f::exact_equals(vec3f::load(part.r1_plus_u_x_axis), vec3f::make(0.0f, 0.0f, -1.0f)));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(part.r2_x_axis), vec3f::make(0.0f, 0.5f, 0.0f)));
    CHECK(ctx, vec3f::distance(vec3f::load(part.inv_i1_r1_plus_u_x_axis), vec3f::make(0.0f, 0.0f, -2.0f)) < 1e-6f);
    CHECK(ctx, vec3f::distance(vec3f::load(part.inv_i2_r2_x_axis), vec3f::make(0.0f, 2.0f, 0.0f)) < 1e-6f);
    CHECK_NEAR(ctx, part.effective_mass, 1.0f / (1.0f + 2.0f + 0.5f + 4.0f * 0.25f), 1e-6f);
    CHECK(ctx, part.bias == 0.25f);

    // a static side adds nothing; a kinematic side keeps its jacobian term but adds no mass
    contact_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_STATIC>(part, 1.0f, inverse_inertia(2.0f), r1, 0.0f,
                                                                                            mat4f::zero(), r2, vec3f::axis_x(), 0.0f);
    CHECK_NEAR(ctx, part.effective_mass, 1.0f / 3.0f, 1e-6f);
    part.r2_x_axis = {9.0f, 9.0f, 9.0f};
    contact_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_KINEMATIC>(part, 1.0f, inverse_inertia(2.0f), r1, 0.0f,
                                                                                               mat4f::zero(), r2, vec3f::axis_x(), 0.0f);
    CHECK_NEAR(ctx, part.effective_mass, 1.0f / 3.0f, 1e-6f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(part.r2_x_axis), vec3f::make(0.0f, 0.5f, 0.0f)));

    // a scaled inverse inertia of 0 takes rotation out: only the masses remain
    contact_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_DYNAMIC>(part, 0.25f, mat4f::zero(), r1, 0.75f,
                                                                                             mat4f::zero(), r2, vec3f::axis_x(), 0.0f);
    CHECK_NEAR(ctx, part.effective_mass, 1.0f, 1e-6f);

    // no mass left: the part switches off and drops its impulse
    part.total_lambda = 5.0f;
    contact_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_STATIC>(part, 0.0f, mat4f::zero(), r1, 0.0f, mat4f::zero(),
                                                                                            r2, vec3f::axis_x(), 0.0f);
    CHECK(ctx, !contact_constraint_part::is_active(part) && part.total_lambda == 0.0f);
}

void test_contact_constraint_part_removes_approach_velocity(TestCtx &ctx) {
    // a (mass 1) moving +2 meets b (mass 2) moving -1 along x; zero total momentum
    Vec3f linear_a = vec3f::make(2.0f, 0.0f, 0.0f);
    Vec3f linear_b = vec3f::make(-1.0f, 0.0f, 0.0f);
    Vec3f angular_a = vec3f::zero();
    Vec3f angular_b = vec3f::zero();
    const Vec3f axis = vec3f::axis_x();
    ContactConstraintPart part = {};
    contact_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_DYNAMIC>(
        part, 1.0f, inverse_inertia(1.0f), vec3f::make(0.5f, 0.0f, 0.0f), 0.5f, inverse_inertia(1.0f), vec3f::make(-0.5f, 0.0f, 0.0f), axis, 0.0f);
    CHECK_NEAR(ctx, part.effective_mass, 2.0f / 3.0f, 1e-6f);

    // the unclamped lambda: K^-1 (v1 - v2) . n = 2/3 * 3
    const float total_lambda = contact_constraint_part::solve_velocity_constraint_get_total_lambda<MOTION_DYNAMIC, MOTION_DYNAMIC>(
        part, linear_a, angular_a, linear_b, angular_b, axis);
    CHECK_NEAR(ctx, total_lambda, 2.0f, 1e-6f);

    // applied, both stop dead and momentum is still zero
    CHECK(ctx, (contact_constraint_part::solve_velocity_constraint_apply_lambda<MOTION_DYNAMIC, MOTION_DYNAMIC>(
                   part, linear_a, angular_a, linear_b, angular_b, 1.0f, 0.5f, axis, total_lambda)));
    CHECK(ctx, part.total_lambda == total_lambda);
    CHECK_NEAR(ctx, vec3f::x(linear_a), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::x(linear_b), 0.0f, 1e-6f);
    CHECK(ctx, vec3f::exact_equals(angular_a, vec3f::make(0.0f, 0.0f, 0.0f)));

    // solved: a second iteration has nothing to add
    CHECK(ctx, (!contact_constraint_part::solve_velocity_constraint<MOTION_DYNAMIC, MOTION_DYNAMIC>(part, linear_a, angular_a, linear_b, angular_b,
                                                                                                 1.0f, 0.5f, axis, 0.0f, __FLT_MAX__)));

    // next step the bodies approach again; the warm start reapplies the stored lambda and stops them up front
    linear_a = vec3f::make(2.0f, 0.0f, 0.0f);
    linear_b = vec3f::make(-1.0f, 0.0f, 0.0f);
    CHECK(ctx, (contact_constraint_part::warm_start<MOTION_DYNAMIC, MOTION_DYNAMIC>(part, linear_a, angular_a, linear_b, angular_b, 1.0f, 0.5f,
                                                                                  axis, 1.0f)));
    CHECK_NEAR(ctx, vec3f::x(linear_a), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::x(linear_b), 0.0f, 1e-6f);

    // a warm start ratio scales the stored impulse; half of it removes half the approach
    linear_a = vec3f::make(2.0f, 0.0f, 0.0f);
    linear_b = vec3f::make(-1.0f, 0.0f, 0.0f);
    contact_constraint_part::warm_start<MOTION_DYNAMIC, MOTION_DYNAMIC>(part, linear_a, angular_a, linear_b, angular_b, 1.0f, 0.5f, axis, 0.5f);
    CHECK_NEAR(ctx, part.total_lambda, 1.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::x(linear_a), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::x(linear_b), -0.5f, 1e-6f);
}

void test_contact_constraint_part_bias_and_clamp(TestCtx &ctx) {
    // a dynamic body resting on static ground: the bias is the target separating velocity
    Vec3f linear_a = vec3f::zero();
    Vec3f angular_a = vec3f::zero();
    Vec3f linear_b = vec3f::zero();
    Vec3f angular_b = vec3f::zero();
    const Vec3f up = vec3f::make(0.0f, 1.0f, 0.0f);
    ContactConstraintPart part = {};
    contact_constraint_part::calculate_constraint_properties<MOTION_STATIC, MOTION_DYNAMIC>(
        part, 0.0f, mat4f::zero(), vec3f::make(0.0f, 1.0f, 0.0f), 0.5f, inverse_inertia(1.0f), vec3f::make(0.0f, -1.0f, 0.0f), up, -3.0f);
    CHECK_NEAR(ctx, part.effective_mass, 2.0f, 1e-6f);

    // lambda = K^-1 (jv - b) = 2 * 3; b leaves at 3
    CHECK(ctx, (contact_constraint_part::solve_velocity_constraint<MOTION_STATIC, MOTION_DYNAMIC>(part, linear_a, angular_a, linear_b, angular_b,
                                                                                                0.0f, 0.5f, up, 0.0f, __FLT_MAX__)));
    CHECK_NEAR(ctx, part.total_lambda, 6.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(linear_b), 3.0f, 1e-6f);
    CHECK(ctx, vec3f::exact_equals(linear_a, vec3f::make(0.0f, 0.0f, 0.0f)));

    // inverted bounds (a ccd friction limit from a negative normal lambda) clamp to the max
    part.total_lambda = 0.0f;
    linear_b = vec3f::make(0.0f, -3.0f, 0.0f);
    part.bias = 0.0f;
    contact_constraint_part::solve_velocity_constraint<MOTION_STATIC, MOTION_DYNAMIC>(part, linear_a, angular_a, linear_b, angular_b, 0.0f, 0.5f,
                                                                                     up, 1.0f, -1.0f);
    CHECK(ctx, part.total_lambda == -1.0f);
    CHECK_NEAR(ctx, vec3f::y(linear_b), -3.0f - 0.5f, 1e-6f);
}

void test_contact_constraint_part_position_solve(TestCtx &ctx) {
    // two unit-mass bodies 0.1 into each other along x: one full baumgarte step pushes each back half the depth
    RigidBody a = make_body(MOTION_DYNAMIC, 1.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(MOTION_DYNAMIC, 1.0f, {0.0f, 0.0f, 0.0f}, {0.9f, 0.0f, 0.0f});
    ContactConstraintPart part = {};
    contact_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_DYNAMIC>(
        part, 1.0f, inverse_inertia(1.0f), vec3f::make(0.45f, 0.0f, 0.0f), 1.0f, inverse_inertia(1.0f), vec3f::make(-0.45f, 0.0f, 0.0f),
        vec3f::axis_x(), 0.0f);

    CHECK(ctx, (!contact_constraint_part::solve_position_constraint<MOTION_DYNAMIC, MOTION_DYNAMIC>(part, a, 1.0f, b, 1.0f, vec3f::axis_x(),
                                                                                                 0.0f, 1.0f)));
    CHECK(ctx, (contact_constraint_part::solve_position_constraint<MOTION_DYNAMIC, MOTION_DYNAMIC>(part, a, 1.0f, b, 1.0f, vec3f::axis_x(),
                                                                                                -0.1f, 1.0f)));
    CHECK_NEAR(ctx, a.center_of_mass_position.x, -0.05f, 1e-6f);
    CHECK_NEAR(ctx, b.center_of_mass_position.x, 0.95f, 1e-6f);
    CHECK(ctx, quatf::exact_equals(quatf::load(a.quaternion), quatf::identity()));
    // velocities are untouched: the correction does not add momentum
    CHECK(ctx, vec3f::exact_equals(vec3f::load(a.motion_properties.linear_velocity), vec3f::zero()));

    // against static ground only the dynamic side moves, by the whole depth
    RigidBody ground = make_body(MOTION_STATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f});
    RigidBody box = make_body(MOTION_DYNAMIC, 1.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
    const Vec3f up = vec3f::make(0.0f, 1.0f, 0.0f);
    contact_constraint_part::calculate_constraint_properties<MOTION_STATIC, MOTION_DYNAMIC>(
        part, 0.0f, mat4f::zero(), vec3f::make(0.0f, 0.95f, 0.0f), 1.0f, inverse_inertia(1.0f), vec3f::make(0.0f, -0.05f, 0.0f), up, 0.0f);
    CHECK(ctx, (contact_constraint_part::solve_position_constraint<MOTION_STATIC, MOTION_DYNAMIC>(part, ground, 0.0f, box, 1.0f, up, -0.1f, 1.0f)));
    CHECK_NEAR(ctx, box.center_of_mass_position.y, 0.1f, 1e-6f);
    CHECK(ctx, ground.center_of_mass_position.y == -1.0f);
}
