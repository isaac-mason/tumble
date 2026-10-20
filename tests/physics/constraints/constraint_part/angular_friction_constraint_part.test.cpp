#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/constraints/constraint_part/angular_friction_constraint_part.h"
#include "test.h"
#include "test_math.h"

using angular_friction_constraint_part::AngularFrictionConstraintPart;
using motion_type::MOTION_DYNAMIC;
using motion_type::MOTION_STATIC;

static Mat4f inverse_inertia(const Float3 &inv_inertia_diagonal, Quatf rotation) {
    motion_properties::MotionProperties motion_properties = motion_properties::create();
    motion_properties.inv_mass = 1.0f;
    motion_properties.inv_inertia_diagonal = inv_inertia_diagonal;
    return motion_properties::get_world_inverse_inertia(motion_properties, rotation, motion_properties::STEP_STAMP_NONE);
}

static const Float3 Z_AXIS = {0.0f, 0.0f, 1.0f};

void test_angular_friction_constraint_part_deactivate(TestCtx &ctx) {
    AngularFrictionConstraintPart part = {};
    CHECK(ctx, !angular_friction_constraint_part::is_active(part));
    part.effective_mass = 1.0f;
    part.total_lambda = 2.0f;
    CHECK(ctx, angular_friction_constraint_part::is_active(part));
    angular_friction_constraint_part::deactivate(part);
    CHECK(ctx, !angular_friction_constraint_part::is_active(part) && part.total_lambda == 0.0f);
}

void test_angular_friction_constraint_part_effective_mass(TestCtx &ctx) {
    // 1 / (n . (I1^-1 + I2^-1) n) about z: the z entries of the inverse inertias
    AngularFrictionConstraintPart part = {};
    angular_friction_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_DYNAMIC>(
        part, inverse_inertia({1.0f, 2.0f, 3.0f}, quatf::identity()), inverse_inertia({5.0f, 5.0f, 1.0f}, quatf::identity()), vec3f::load(Z_AXIS),
        0.5f);
    CHECK(ctx, vec3f::distance(vec3f::load(part.inv_i1_axis), vec3f::make(0.0f, 0.0f, 3.0f)) < 1e-6f);
    CHECK(ctx, vec3f::distance(vec3f::load(part.inv_i2_axis), vec3f::make(0.0f, 0.0f, 1.0f)) < 1e-6f);
    CHECK_NEAR(ctx, part.effective_mass, 0.25f, 1e-6f);
    CHECK(ctx, part.bias == 0.5f);

    // a body turned a quarter about x has its y inertia along world z; static ground adds nothing
    const Quatf turned = quatf::set_axis_angle(vec3f::axis_x(), 0.5f * scalar::PI);
    angular_friction_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_STATIC>(
        part, inverse_inertia({1.0f, 2.0f, 3.0f}, turned), mat4f::zero(), vec3f::load(Z_AXIS), 0.0f);
    CHECK_NEAR(ctx, part.effective_mass, 0.5f, 1e-5f);

    // no inertia about the axis: inactive, the impulse dropped
    part.total_lambda = 3.0f;
    angular_friction_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_STATIC>(
        part, inverse_inertia({1.0f, 1.0f, 0.0f}, quatf::identity()), mat4f::zero(), vec3f::load(Z_AXIS), 0.0f);
    CHECK(ctx, !angular_friction_constraint_part::is_active(part) && part.total_lambda == 0.0f);
}

void test_angular_friction_constraint_part_stops_relative_spin(TestCtx &ctx) {
    // a (I = 1) spins at 3 rad/s about the normal on b (I = 2) at rest. one unclamped iteration leaves them spinning
    // together with the angular momentum kept
    Vec3f angular_a = vec3f::make(0.0f, 0.0f, 3.0f);
    Vec3f angular_b = vec3f::zero();
    const Vec3f axis = vec3f::load(Z_AXIS);
    AngularFrictionConstraintPart part = {};
    angular_friction_constraint_part::calculate_constraint_properties<MOTION_DYNAMIC, MOTION_DYNAMIC>(
        part, inverse_inertia({1.0f, 1.0f, 1.0f}, quatf::identity()), inverse_inertia({0.5f, 0.5f, 0.5f}, quatf::identity()), axis, 0.0f);

    CHECK(ctx, (angular_friction_constraint_part::solve_velocity_constraint<MOTION_DYNAMIC, MOTION_DYNAMIC>(part, angular_a, angular_b, axis,
                                                                                                         -__FLT_MAX__, __FLT_MAX__)));
    CHECK_NEAR(ctx, part.total_lambda, 2.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(angular_a), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(angular_b), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, 1.0f * vec3f::z(angular_a) + 2.0f * vec3f::z(angular_b), 3.0f, 1e-6f);

    // spinning together, the next iteration adds nothing
    CHECK(ctx, (!angular_friction_constraint_part::solve_velocity_constraint<MOTION_DYNAMIC, MOTION_DYNAMIC>(part, angular_a, angular_b, axis,
                                                                                                          -__FLT_MAX__, __FLT_MAX__)));

    // the cap holds the impulse to the friction limit
    angular_a = vec3f::make(0.0f, 0.0f, 3.0f);
    angular_b = vec3f::zero();
    part.total_lambda = 0.0f;
    angular_friction_constraint_part::solve_velocity_constraint<MOTION_DYNAMIC, MOTION_DYNAMIC>(part, angular_a, angular_b, axis, -0.5f, 0.5f);
    CHECK(ctx, part.total_lambda == 0.5f);
    CHECK_NEAR(ctx, vec3f::z(angular_a), 2.5f, 1e-6f);

    // the warm start scales the stored impulse for the new step and applies it
    angular_a = vec3f::zero();
    angular_b = vec3f::zero();
    CHECK(ctx, (angular_friction_constraint_part::warm_start<MOTION_DYNAMIC, MOTION_DYNAMIC>(part, angular_a, angular_b, 0.5f)));
    CHECK_NEAR(ctx, part.total_lambda, 0.25f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(angular_a), -0.25f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(angular_b), 0.125f, 1e-6f);
}
