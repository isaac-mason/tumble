#include "math/quatf.h"
#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/body/motion_properties.h"
#include "physics/body/rigid_body.h"
#include "physics/constraints/constraint_part/angle_constraint_part.h"
#include "physics/constraints/constraint_part/spring_part.h"
#include "physics/constraints/constraint_part/part_body.h"
#include "test.h"
#include "test_math.h"

using angle_constraint_part::AngleConstraintPart;
using rigid_body::RigidBody;

static float angle_about_z(const Float4 &q) { return 2.0f * scalar::atan2(q.z, q.w); }

static const Float3 AXIS_Z = {0.0f, 0.0f, 1.0f};

void test_angle_constraint_part_create_and_deactivate(TestCtx &ctx) {
    AngleConstraintPart part = angle_constraint_part::create();
    CHECK(ctx, !angle_constraint_part::is_active(part));
    CHECK(ctx, part.total_lambda == 0.0f && part.spring.softness == 0.0f && part.spring.bias == 0.0f);

    part.effective_mass = 2.0f;
    part.total_lambda = 3.0f;
    CHECK(ctx, angle_constraint_part::is_active(part));
    angle_constraint_part::deactivate(part);
    CHECK(ctx, !angle_constraint_part::is_active(part) && part.total_lambda == 0.0f);

    angle_constraint_part::set_total_lambda(part, 4.5f);
    CHECK(ctx, angle_constraint_part::get_total_lambda(part) == 4.5f);
}

void test_angle_constraint_part_effective_mass(TestCtx &ctx) {
    // K = a . I1^-1 . a + a . I2^-1 . a; about z with inverse diagonals (1, 2, 3) and (4, 5, 6) that is 3 + 6
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {4.0f, 5.0f, 6.0f});
    AngleConstraintPart part = angle_constraint_part::create();
    angle_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z), 0.25f);
    CHECK_NEAR(ctx, part.effective_mass, 1.0f / 9.0f, 1e-7f);
    CHECK(ctx, vec3f::distance(part.inv_i1_axis, vec3f::make(0.0f, 0.0f, 3.0f)) < 1e-6f);
    CHECK(ctx, vec3f::distance(part.inv_i2_axis, vec3f::make(0.0f, 0.0f, 6.0f)) < 1e-6f);
    CHECK(ctx, part.spring.bias == 0.25f && !spring_part::is_spring_active(part.spring));

    // a static body contributes nothing
    b.motion_type = motion_type::MOTION_STATIC;
    angle_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z));
    CHECK_NEAR(ctx, part.effective_mass, 1.0f / 3.0f, 1e-7f);
    CHECK(ctx, vec3f::exact_equals(part.inv_i2_axis, vec3f::zero()));

    // turned a quarter about x, body a's z inertia is its local y: K = 2
    a.quaternion = stored_axis_angle(vec3f::axis_x(), scalar::PI * 0.5f);
    angle_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z));
    CHECK_NEAR(ctx, part.effective_mass, 0.5f, 1e-6f);

    // nothing dynamic: no effective mass, the part deactivates and drops its impulse
    a.motion_type = motion_type::MOTION_KINEMATIC;
    part.total_lambda = 1.0f;
    angle_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z));
    CHECK(ctx, !angle_constraint_part::is_active(part) && part.total_lambda == 0.0f);
}

void test_angle_constraint_part_soft_variants(TestCtx &ctx) {
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    const float dt = 1.0f / 60.0f;

    // zero frequency is a hard constraint: 1 / K and no spring
    AngleConstraintPart hard = angle_constraint_part::create();
    angle_constraint_part::calculate_constraint_properties_with_frequency_and_damping(hard, dt, a, b, vec3f::load(AXIS_Z), 0.0f, 0.1f, 0.0f,
                                                                                       1.0f);
    CHECK_NEAR(ctx, hard.effective_mass, 0.5f, 1e-7f);
    CHECK(ctx, !spring_part::is_spring_active(hard.spring));

    // a spring softens the constraint: K^-1 = 1 / (K + softness) with softness > 0
    AngleConstraintPart soft = angle_constraint_part::create();
    angle_constraint_part::calculate_constraint_properties_with_frequency_and_damping(soft, dt, a, b, vec3f::load(AXIS_Z), 0.0f, 0.1f, 2.0f,
                                                                                       1.0f);
    CHECK(ctx, spring_part::is_spring_active(soft.spring));
    CHECK(ctx, soft.spring.softness > 0.0f);
    CHECK_NEAR(ctx, soft.effective_mass, 1.0f / (2.0f + soft.spring.softness), 1e-6f);

    // settings pick the variant by mode, giving what the direct call gives
    AngleConstraintPart by_settings = angle_constraint_part::create();
    angle_constraint_part::calculate_constraint_properties_with_settings(
        by_settings, dt, a, b, vec3f::load(AXIS_Z), 0.0f, 0.1f, {spring_settings::FREQUENCY_AND_DAMPING, 2.0f, 1.0f});
    CHECK(ctx, by_settings.effective_mass == soft.effective_mass && by_settings.spring.softness == soft.spring.softness &&
                   by_settings.spring.bias == soft.spring.bias);

    AngleConstraintPart stiff = angle_constraint_part::create();
    angle_constraint_part::calculate_constraint_properties_with_stiffness_and_damping(stiff, dt, a, b, vec3f::load(AXIS_Z), 0.0f, 0.1f, 50.0f,
                                                                                       2.0f);
    AngleConstraintPart stiff_settings = angle_constraint_part::create();
    angle_constraint_part::calculate_constraint_properties_with_settings(
        stiff_settings, dt, a, b, vec3f::load(AXIS_Z), 0.0f, 0.1f, {spring_settings::STIFFNESS_AND_DAMPING, 50.0f, 2.0f});
    CHECK(ctx, spring_part::is_spring_active(stiff.spring));
    CHECK(ctx, stiff_settings.effective_mass == stiff.effective_mass && stiff_settings.spring.softness == stiff.spring.softness);

    // with nothing dynamic every variant deactivates
    a.motion_type = motion_type::MOTION_STATIC;
    b.motion_type = motion_type::MOTION_STATIC;
    angle_constraint_part::calculate_constraint_properties_with_stiffness_and_damping(stiff, dt, a, b, vec3f::load(AXIS_Z), 0.0f, 0.1f, 50.0f,
                                                                                       2.0f);
    CHECK(ctx, !angle_constraint_part::is_active(stiff));
}

void test_angle_constraint_part_velocity_removes_relative_spin(TestCtx &ctx) {
    // inverse inertias 3 and 6 about z; a hard unbounded solve leaves no relative spin about the axis, keeps the
    // angular momentum about it (w_a / 3 + w_b / 6) and does not touch the other components
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {4.0f, 5.0f, 6.0f});
    a.motion_properties.angular_velocity = {0.5f, -1.0f, 2.0f};
    b.motion_properties.angular_velocity = {0.25f, 0.75f, -1.0f};
    const float momentum = 2.0f / 3.0f + -1.0f / 6.0f;

    AngleConstraintPart part = angle_constraint_part::create();
    angle_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z));
    const bool applied = angle_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(AXIS_Z), -__FLT_MAX__, __FLT_MAX__);
    CHECK(ctx, applied);

    const Float3 &wa = a.motion_properties.angular_velocity;
    const Float3 &wb = b.motion_properties.angular_velocity;
    CHECK_NEAR(ctx, wa.z - wb.z, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, wa.z / 3.0f + wb.z / 6.0f, momentum, 1e-6f);
    CHECK(ctx, wa.x == 0.5f && wa.y == -1.0f && wb.x == 0.25f && wb.y == 0.75f);
    // lambda = (w_a - w_b) / K = 3 / 9
    CHECK_NEAR(ctx, part.total_lambda, 1.0f / 3.0f, 1e-6f);
}

void test_angle_constraint_part_velocity_clamps_impulse(TestCtx &ctx) {
    // a static body a: jv = -w_b . axis. with max_lambda 0 only negative impulses get through
    RigidBody a = make_body(motion_type::MOTION_STATIC, 1.0f, {1.0f, 1.0f, 1.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 2.0f});
    AngleConstraintPart part = angle_constraint_part::create();
    angle_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z));

    // b spinning negatively about z wants a positive impulse: clamped away, nothing changes
    b.motion_properties.angular_velocity = {0.0f, 0.0f, -1.0f};
    CHECK(ctx, !angle_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(AXIS_Z), -__FLT_MAX__, 0.0f));
    CHECK(ctx, b.motion_properties.angular_velocity.z == -1.0f && part.total_lambda == 0.0f);

    // spinning positively it takes a negative impulse and stops about z
    b.motion_properties.angular_velocity = {0.0f, 0.0f, 1.0f};
    CHECK(ctx, angle_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(AXIS_Z), -__FLT_MAX__, 0.0f));
    CHECK_NEAR(ctx, b.motion_properties.angular_velocity.z, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, part.total_lambda, -0.5f, 1e-6f);

    // a bounded impulse stops partway: |lambda| <= 0.1 takes 0.1 * 2 off the spin
    angle_constraint_part::set_total_lambda(part, 0.0f);
    b.motion_properties.angular_velocity = {0.0f, 0.0f, 1.0f};
    angle_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(AXIS_Z), -0.1f, 0.1f);
    CHECK_NEAR(ctx, b.motion_properties.angular_velocity.z, 0.8f, 1e-6f);
    CHECK(ctx, part.total_lambda == -0.1f);

    // inactive solves nothing
    angle_constraint_part::deactivate(part);
    CHECK(ctx, !angle_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(AXIS_Z), -__FLT_MAX__, __FLT_MAX__));
}

void test_angle_constraint_part_warm_start(TestCtx &ctx) {
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {4.0f, 5.0f, 6.0f});
    AngleConstraintPart part = angle_constraint_part::create();
    angle_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z));
    part.total_lambda = 2.0f;

    // the impulse is scaled first, then applied: a loses 1 * 3, b gains 1 * 6 about z
    angle_constraint_part::warm_start(part, a, b, 0.5f);
    CHECK(ctx, part.total_lambda == 1.0f);
    CHECK_NEAR(ctx, a.motion_properties.angular_velocity.z, -3.0f, 1e-6f);
    CHECK_NEAR(ctx, b.motion_properties.angular_velocity.z, 6.0f, 1e-6f);

    // a zero ratio clears the impulse and applies nothing
    angle_constraint_part::warm_start(part, a, b, 0.0f);
    CHECK(ctx, part.total_lambda == 0.0f);
    CHECK_NEAR(ctx, a.motion_properties.angular_velocity.z, -3.0f, 1e-6f);
}

void test_angle_constraint_part_position_corrects_angle(TestCtx &ctx) {
    // an angle error of c with baumgarte 1 rotates b against a by -c in one step, split by inverse inertia (3 : 6)
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {4.0f, 5.0f, 6.0f});
    AngleConstraintPart part = angle_constraint_part::create();
    angle_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z));

    const float c = 0.1f;
    CHECK(ctx, angle_constraint_part::solve_position_constraint(part, a, b, c, 1.0f));
    CHECK_NEAR(ctx, angle_about_z(a.quaternion), c / 3.0f, 1e-6f);
    CHECK_NEAR(ctx, angle_about_z(b.quaternion), -2.0f * c / 3.0f, 1e-6f);
    CHECK_NEAR(ctx, angle_about_z(b.quaternion) - angle_about_z(a.quaternion), -c, 1e-6f);
    CHECK(ctx, a.quaternion.x == 0.0f && a.quaternion.y == 0.0f);

    // no error, or a soft constraint: no position step
    const Float4 before = b.quaternion;
    CHECK(ctx, !angle_constraint_part::solve_position_constraint(part, a, b, 0.0f, 1.0f));
    angle_constraint_part::calculate_constraint_properties_with_frequency_and_damping(part, 1.0f / 60.0f, a, b, vec3f::load(AXIS_Z), 0.0f, c,
                                                                                       2.0f, 1.0f);
    CHECK(ctx, !angle_constraint_part::solve_position_constraint(part, a, b, c, 1.0f));
    CHECK(ctx, quatf::exact_equals(quatf::load(b.quaternion), quatf::load(before)));
}
