#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/constraint_part/dual_axis_constraint_part.h"
#include "physics/constraints/constraint_part/part_body.h"
#include "test.h"
#include "test_math.h"

using dual_axis_constraint_part::DualAxisConstraintPart;
using rigid_body::RigidBody;

static const Float3 N1 = {0.0f, 1.0f, 0.0f};
static const Float3 N2 = {0.0f, 0.0f, 1.0f};
static const Float3 ZERO = {0.0f, 0.0f, 0.0f};
static const uint32_t STAMP = motion_properties::STEP_STAMP_NONE;

/** the velocity error along n: n . (v1 - v2) + (r1 + u) x n . w1 - r2 x n . w2 */
static float constraint_velocity(const RigidBody &a, const RigidBody &b, const Float3 &r1_plus_u, const Float3 &r2, const Float3 &n) {
    const Vec3f r1n = vec3f::cross(vec3f::load(r1_plus_u), vec3f::load(n));
    const Vec3f r2n = vec3f::cross(vec3f::load(r2), vec3f::load(n));
    return vec3f::dot(vec3f::load(n), vec3f::load(a.motion_properties.linear_velocity)) - vec3f::dot(vec3f::load(n), vec3f::load(b.motion_properties.linear_velocity)) +
           vec3f::dot(r1n, vec3f::load(a.motion_properties.angular_velocity)) - vec3f::dot(r2n, vec3f::load(b.motion_properties.angular_velocity));
}

void test_dual_axis_constraint_part_create_and_deactivate(TestCtx &ctx) {
    DualAxisConstraintPart part = dual_axis_constraint_part::create();
    CHECK(ctx, !dual_axis_constraint_part::is_active(part));
    part.effective_mass.m[2] = 1.0f;
    part.total_lambda = {1.0f, 2.0f};
    CHECK(ctx, dual_axis_constraint_part::is_active(part));
    dual_axis_constraint_part::deactivate(part);
    CHECK(ctx, !dual_axis_constraint_part::is_active(part) && part.total_lambda.x == 0.0f && part.total_lambda.y == 0.0f);
}

void test_dual_axis_constraint_part_effective_mass(TestCtx &ctx) {
    // no moment arms: K = (m1^-1 + m2^-1) E, here (1 + 0.5)
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {1.0f, 1.0f, 1.0f});
    DualAxisConstraintPart part = dual_axis_constraint_part::create();
    dual_axis_constraint_part::calculate_constraint_properties(part, a, vec3f::load(ZERO), b, vec3f::load(ZERO), vec3f::load(N1), vec3f::load(N2), STAMP);
    CHECK_NEAR(ctx, part.effective_mass.m[0], 1.0f / 1.5f, 1e-6f);
    CHECK_NEAR(ctx, part.effective_mass.m[3], 1.0f / 1.5f, 1e-6f);
    CHECK(ctx, part.effective_mass.m[1] == 0.0f && part.effective_mass.m[2] == 0.0f);

    // a moment arm along x on body a with inverse inertia diag (1, 2, 3): (r1 + u) x n1 = (0, 0, 1), x n2 = (0, -1, 0),
    // so K = diag(1 + 3, 1 + 2) + diag(0.5, 0.5)
    a.motion_properties.inv_inertia_diagonal = {1.0f, 2.0f, 3.0f};
    dual_axis_constraint_part::calculate_constraint_properties(part, a, vec3f::make(1.0f, 0.0f, 0.0f), b, vec3f::load(ZERO), vec3f::load(N1), vec3f::load(N2), STAMP);
    CHECK_NEAR(ctx, part.effective_mass.m[0], 1.0f / 4.5f, 1e-6f);
    CHECK_NEAR(ctx, part.effective_mass.m[3], 1.0f / 3.5f, 1e-6f);
    CHECK(ctx, vec3f::distance(part.r1_plus_u_x_n1, vec3f::make(0.0f, 0.0f, 1.0f)) < 1e-6f);
    CHECK(ctx, vec3f::distance(part.inv_i1_r1_plus_u_x_n2, vec3f::make(0.0f, -2.0f, 0.0f)) < 1e-6f);

    // nothing dynamic: singular, deactivated
    a.motion_type = motion_type::MOTION_STATIC;
    b.motion_type = motion_type::MOTION_KINEMATIC;
    part.total_lambda = {1.0f, 1.0f};
    dual_axis_constraint_part::calculate_constraint_properties(part, a, vec3f::load(ZERO), b, vec3f::load(ZERO), vec3f::load(N1), vec3f::load(N2), STAMP);
    CHECK(ctx, !dual_axis_constraint_part::is_active(part) && part.total_lambda.x == 0.0f);
    CHECK(ctx, !dual_axis_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(N1), vec3f::load(N2)));
    CHECK(ctx, !dual_axis_constraint_part::solve_position_constraint(part, a, b, vec3f::make(0.0f, 1.0f, 0.0f), vec3f::load(N1), vec3f::load(N2), 1.0f));
}

void test_dual_axis_constraint_part_velocity_solve(TestCtx &ctx) {
    // arms, turned bodies and a full inertia: one 2d solve zeroes the velocity error on both axes, leaves the slide
    // axis alone and conserves linear momentum
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {2.0f, 1.0f, 0.5f});
    quatf::store(a.quaternion, quatf::set_axis_angle(vec3f::make(0.0f, 0.6f, 0.8f), 0.7f));
    quatf::store(b.quaternion, quatf::set_axis_angle(vec3f::make(0.8f, 0.0f, 0.6f), -0.4f));
    a.motion_properties.linear_velocity = {3.0f, 2.0f, -1.0f};
    a.motion_properties.angular_velocity = {0.5f, 0.25f, -0.75f};
    b.motion_properties.linear_velocity = {-1.0f, 0.5f, 0.25f};
    b.motion_properties.angular_velocity = {-0.5f, 1.0f, 0.0f};
    const Float3 r1_plus_u = {0.5f, 0.25f, -0.5f};
    const Float3 r2 = {-0.25f, 0.5f, 0.75f};

    Vec3f momentum = vec3f::scale(vec3f::load(a.motion_properties.linear_velocity), 1.0f);
    momentum = vec3f::scale_and_add(momentum, vec3f::load(b.motion_properties.linear_velocity), 2.0f);

    DualAxisConstraintPart part = dual_axis_constraint_part::create();
    dual_axis_constraint_part::calculate_constraint_properties(part, a, vec3f::load(r1_plus_u), b, vec3f::load(r2), vec3f::load(N1), vec3f::load(N2), STAMP);
    CHECK(ctx, dual_axis_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(N1), vec3f::load(N2)));

    CHECK_NEAR(ctx, constraint_velocity(a, b, r1_plus_u, r2, N1), 0.0f, 1e-5f);
    CHECK_NEAR(ctx, constraint_velocity(a, b, r1_plus_u, r2, N2), 0.0f, 1e-5f);
    CHECK(ctx, a.motion_properties.linear_velocity.x == 3.0f && b.motion_properties.linear_velocity.x == -1.0f);

    Vec3f after = vec3f::scale(vec3f::load(a.motion_properties.linear_velocity), 1.0f);
    after = vec3f::scale_and_add(after, vec3f::load(b.motion_properties.linear_velocity), 2.0f);
    CHECK(ctx, vec3f::distance(after, momentum) < 1e-5f);
}

void test_dual_axis_constraint_part_velocity_against_static(TestCtx &ctx) {
    // a static b and no arms: a's velocity across the slide axis is removed, along it kept
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 2.0f, {1.0f, 1.0f, 1.0f});
    RigidBody b = make_body(motion_type::MOTION_STATIC, 0.0f, {0.0f, 0.0f, 0.0f});
    a.motion_properties.linear_velocity = {4.0f, -3.0f, 2.0f};
    DualAxisConstraintPart part = dual_axis_constraint_part::create();
    dual_axis_constraint_part::calculate_constraint_properties(part, a, vec3f::load(ZERO), b, vec3f::load(ZERO), vec3f::load(N1), vec3f::load(N2), STAMP);
    CHECK(ctx, dual_axis_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(N1), vec3f::load(N2)));
    CHECK(ctx, vec3f::distance(vec3f::load(a.motion_properties.linear_velocity), vec3f::make(4.0f, 0.0f, 0.0f)) < 1e-6f);
    // lambda = K^-1 jv = (-3, 2) / 2
    CHECK_NEAR(ctx, part.total_lambda.x, -1.5f, 1e-6f);
    CHECK_NEAR(ctx, part.total_lambda.y, 1.0f, 1e-6f);

    // already solved: zero impulse, nothing applied
    CHECK(ctx, !dual_axis_constraint_part::solve_velocity_constraint(part, a, b, vec3f::load(N1), vec3f::load(N2)));
}

void test_dual_axis_constraint_part_warm_start(TestCtx &ctx) {
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {1.0f, 1.0f, 1.0f});
    DualAxisConstraintPart part = dual_axis_constraint_part::create();
    dual_axis_constraint_part::calculate_constraint_properties(part, a, vec3f::load(ZERO), b, vec3f::load(ZERO), vec3f::load(N1), vec3f::load(N2), STAMP);
    part.total_lambda = {2.0f, 4.0f};

    // scaled to (1, 2), then P = n1 + 2 n2 pushes a back by P / m1 and b on by P / m2
    dual_axis_constraint_part::warm_start(part, a, b, vec3f::load(N1), vec3f::load(N2), 0.5f);
    CHECK(ctx, part.total_lambda.x == 1.0f && part.total_lambda.y == 2.0f);
    CHECK(ctx, vec3f::distance(vec3f::load(a.motion_properties.linear_velocity), vec3f::make(0.0f, -1.0f, -2.0f)) < 1e-6f);
    CHECK(ctx, vec3f::distance(vec3f::load(b.motion_properties.linear_velocity), vec3f::make(0.0f, 0.5f, 1.0f)) < 1e-6f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(a.motion_properties.angular_velocity), vec3f::load(ZERO)));
}

void test_dual_axis_constraint_part_position_closes_separation(TestCtx &ctx) {
    // separation u = p2 - p1 across the slide axis is closed in one baumgarte 1 step, split by inverse mass
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    a.center_of_mass_position = {0.0f, 0.0f, 0.0f};
    b.center_of_mass_position = {5.0f, 0.2f, -0.1f};
    DualAxisConstraintPart part = dual_axis_constraint_part::create();
    dual_axis_constraint_part::calculate_constraint_properties(part, a, vec3f::load(ZERO), b, vec3f::load(ZERO), vec3f::load(N1), vec3f::load(N2), STAMP);

    const Vec3f u = vec3f::subtract(vec3f::load(b.center_of_mass_position), vec3f::load(a.center_of_mass_position));
    CHECK(ctx, dual_axis_constraint_part::solve_position_constraint(part, a, b, u, vec3f::load(N1), vec3f::load(N2), 1.0f));
    CHECK(ctx, vec3f::distance(vec3f::load(a.center_of_mass_position), vec3f::make(0.0f, 0.1f, -0.05f)) < 1e-6f);
    CHECK(ctx, vec3f::distance(vec3f::load(b.center_of_mass_position), vec3f::make(5.0f, 0.1f, -0.05f)) < 1e-6f);
    CHECK(ctx, quatf::exact_equals(quatf::load(a.quaternion), quatf::identity()));

    // separated only along the slide axis: nothing to correct
    CHECK(ctx, !dual_axis_constraint_part::solve_position_constraint(part, a, b, vec3f::make(5.0f, 0.0f, 0.0f), vec3f::load(N1), vec3f::load(N2), 1.0f));
}

void test_dual_axis_constraint_part_spinning_kinematic_drags_slider(TestCtx &ctx) {
    // a kinematic a spinning about z, its slider point at r1 + u = (1, 0, 0); dynamic b's centre of mass on the
    // point (r2 = 0). the point moves at w x r1 = (0, 0, 2) x (1, 0, 0) = (0, 2, 0); K^-1 = m2^-1 E, so
    // lambda = (2 m2, 0) and b leaves with the point's velocity across the slider
    RigidBody spinner = make_body(motion_type::MOTION_KINEMATIC, 0.0f, {0.0f, 0.0f, 0.0f});
    RigidBody slider = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {1.0f, 1.0f, 1.0f});
    spinner.motion_properties.angular_velocity = {0.0f, 0.0f, 2.0f};
    DualAxisConstraintPart part = dual_axis_constraint_part::create();
    dual_axis_constraint_part::calculate_constraint_properties(part, spinner, vec3f::make(1.0f, 0.0f, 0.0f), slider, vec3f::load(ZERO), vec3f::load(N1), vec3f::load(N2), STAMP);
    CHECK(ctx, vec3f::exact_equals(part.r1_plus_u_x_n1, vec3f::make(0.0f, 0.0f, 1.0f)));
    CHECK(ctx, vec3f::exact_equals(part.r1_plus_u_x_n2, vec3f::make(0.0f, -1.0f, 0.0f)));
    CHECK(ctx, dual_axis_constraint_part::solve_velocity_constraint(part, spinner, slider, vec3f::load(N1), vec3f::load(N2)));
    CHECK_NEAR(ctx, part.total_lambda.x, 2.0f / 0.5f, 1e-6f);
    CHECK_NEAR(ctx, part.total_lambda.y, 0.0f, 1e-6f);
    CHECK(ctx, vec3f::distance(vec3f::load(slider.motion_properties.linear_velocity), vec3f::make(0.0f, 2.0f, 0.0f)) < 1e-6f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(spinner.motion_properties.angular_velocity), vec3f::make(0.0f, 0.0f, 2.0f)));
}
