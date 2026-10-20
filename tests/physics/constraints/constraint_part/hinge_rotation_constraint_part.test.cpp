#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/constraint_part/hinge_rotation_constraint_part.h"
#include "physics/constraints/constraint_part/part_body.h"
#include "test.h"
#include "test_math.h"

using hinge_rotation_constraint_part::HingeRotationConstraintPart;
using rigid_body::RigidBody;

static const Float3 AXIS_Z = {0.0f, 0.0f, 1.0f};
static const uint32_t STAMP = motion_properties::STEP_STAMP_NONE;

static float axis_misalignment(const RigidBody &a, const RigidBody &b, const Float3 &local_axis) {
    const Vec3f a1 = vec3f::transform_quat(vec3f::load(local_axis), quatf::load(a.quaternion));
    const Vec3f a2 = vec3f::transform_quat(vec3f::load(local_axis), quatf::load(b.quaternion));
    return vec3f::angle(a1, a2);
}

void test_hinge_rotation_constraint_part_create_and_deactivate(TestCtx &ctx) {
    HingeRotationConstraintPart part = hinge_rotation_constraint_part::create();
    CHECK(ctx, !hinge_rotation_constraint_part::is_active(part));
    part.effective_mass[3] = 1.0f;
    part.total_lambda[0] = 2.0f;
    CHECK(ctx, hinge_rotation_constraint_part::is_active(part));
    hinge_rotation_constraint_part::deactivate(part);
    CHECK(ctx, !hinge_rotation_constraint_part::is_active(part) && part.total_lambda[0] == 0.0f);
}

void test_hinge_rotation_constraint_part_aligned_axes(TestCtx &ctx) {
    // aligned along z: b2 and c2 span the xy plane, so b2 x a1 and c2 x a1 are orthonormal and K = (1 + 1) E
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    HingeRotationConstraintPart part = hinge_rotation_constraint_part::create();
    hinge_rotation_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z), vec3f::load(AXIS_Z), STAMP);

    CHECK(ctx, hinge_rotation_constraint_part::is_active(part));
    CHECK_NEAR(ctx, vec3f::dot(part.b2, vec3f::load(AXIS_Z)), 0.0f, 1e-7f);
    CHECK_NEAR(ctx, vec3f::dot(part.c2, vec3f::load(AXIS_Z)), 0.0f, 1e-7f);
    CHECK_NEAR(ctx, vec3f::dot(part.b2, part.c2), 0.0f, 1e-7f);
    CHECK_NEAR(ctx, part.effective_mass[0], 0.5f, 1e-6f);
    CHECK_NEAR(ctx, part.effective_mass[3], 0.5f, 1e-6f);
    CHECK_NEAR(ctx, part.effective_mass[1], 0.0f, 1e-7f);
    CHECK_NEAR(ctx, part.effective_mass[2], 0.0f, 1e-7f);

    // nothing to correct when aligned
    CHECK(ctx, !hinge_rotation_constraint_part::solve_position_constraint(part, a, b, 1.0f));
}

void test_hinge_rotation_constraint_part_velocity_keeps_hinge_spin(TestCtx &ctx) {
    // relative spin off the hinge axis is removed, about it untouched, angular momentum kept (equal unit inertias)
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    a.motion_properties.angular_velocity = {1.0f, 2.0f, 3.0f};
    HingeRotationConstraintPart part = hinge_rotation_constraint_part::create();
    hinge_rotation_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z), vec3f::load(AXIS_Z), STAMP);
    CHECK(ctx, hinge_rotation_constraint_part::solve_velocity_constraint(part, a, b));

    const Float3 &wa = a.motion_properties.angular_velocity;
    const Float3 &wb = b.motion_properties.angular_velocity;
    CHECK(ctx, vec3f::distance(vec3f::load(wa), vec3f::make(0.5f, 1.0f, 3.0f)) < 1e-6f);
    CHECK(ctx, vec3f::distance(vec3f::load(wb), vec3f::make(0.5f, 1.0f, 0.0f)) < 1e-6f);

    // warm starting the stored impulse on fresh bodies reproduces the same velocity change
    RigidBody a2 = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    RigidBody b2 = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    a2.motion_properties.angular_velocity = {1.0f, 2.0f, 3.0f};
    hinge_rotation_constraint_part::warm_start(part, a2, b2, 1.0f);
    CHECK(ctx, vec3f::distance(vec3f::load(a2.motion_properties.angular_velocity), vec3f::load(wa)) < 1e-6f);
    CHECK(ctx, vec3f::distance(vec3f::load(b2.motion_properties.angular_velocity), vec3f::load(wb)) < 1e-6f);
}

void test_hinge_rotation_constraint_part_velocity_against_static(TestCtx &ctx) {
    // a static a: b keeps only its spin about the hinge
    RigidBody a = make_body(motion_type::MOTION_STATIC, 1.0f, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    quatf::store(b.quaternion, quatf::set_axis_angle(vec3f::make(0.6f, 0.0f, 0.8f), 0.9f));
    b.motion_properties.angular_velocity = {-1.0f, 0.5f, 2.0f};
    const Float3 hinge = {0.0f, 0.6f, 0.8f};
    HingeRotationConstraintPart part = hinge_rotation_constraint_part::create();
    hinge_rotation_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(hinge), vec3f::load(hinge), STAMP);
    hinge_rotation_constraint_part::solve_velocity_constraint(part, a, b);

    const Float3 &w = b.motion_properties.angular_velocity;
    const Vec3f along = vec3f::scale(vec3f::load(hinge), vec3f::dot(vec3f::load(w), vec3f::load(hinge)));
    CHECK(ctx, vec3f::distance(vec3f::load(w), along) < 1e-5f);
}

void test_hinge_rotation_constraint_part_position_realigns_axes(TestCtx &ctx) {
    // both bodies hinge about their local z; b starts turned 0.3 rad about x so the axes disagree. position steps,
    // each with properties recalculated from the current rotations, bring the axes back together
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {2.0f, 1.0f, 1.5f});
    b.quaternion = stored_axis_angle(vec3f::make(1.0f, 0.0f, 0.0f), 0.3f);
    CHECK_NEAR(ctx, axis_misalignment(a, b, AXIS_Z), 0.3f, 1e-5f);

    HingeRotationConstraintPart part = hinge_rotation_constraint_part::create();
    float misalignment[8];
    for (int i = 0; i < 8; i++) {
        const Vec3f a1 = vec3f::transform_quat(vec3f::load(AXIS_Z), quatf::load(a.quaternion));
        const Vec3f a2 = vec3f::transform_quat(vec3f::load(AXIS_Z), quatf::load(b.quaternion));
        hinge_rotation_constraint_part::calculate_constraint_properties(part, a, b, a1, a2, STAMP);
        hinge_rotation_constraint_part::solve_position_constraint(part, a, b, 1.0f);
        misalignment[i] = axis_misalignment(a, b, AXIS_Z);
    }

    // the first step takes almost all of it (the error is linearised), the rest converges
    CHECK(ctx, misalignment[0] < 0.3f * 0.05f);
    CHECK(ctx, misalignment[7] < 1e-3f);
    // both bodies moved: the correction is split by inverse inertia, not dumped on one side
    CHECK(ctx, !quatf::exact_equals(quatf::load(a.quaternion), quatf::identity()));
    CHECK_NEAR(ctx, quatf::length(quatf::load(a.quaternion)), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, quatf::length(quatf::load(b.quaternion)), 1.0f, 1e-6f);
}

void test_hinge_rotation_constraint_part_opposite_and_degenerate(TestCtx &ctx) {
    // axes more than 90 degrees apart: a2 is pulled to within 90 degrees of a1, the part stays usable
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    HingeRotationConstraintPart part = hinge_rotation_constraint_part::create();
    hinge_rotation_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z), vec3f::make(0.0f, 0.0f, -1.0f), STAMP);
    CHECK(ctx, hinge_rotation_constraint_part::is_active(part));
    CHECK_NEAR(ctx, vec3f::length(part.b2), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::length(part.c2), 1.0f, 1e-6f);
    for (int i = 0; i < 4; i++) CHECK(ctx, scalar::is_finite(part.effective_mass[i]));

    // nothing dynamic: K is zero, the part deactivates
    a.motion_type = motion_type::MOTION_STATIC;
    b.motion_type = motion_type::MOTION_KINEMATIC;
    hinge_rotation_constraint_part::calculate_constraint_properties(part, a, b, vec3f::load(AXIS_Z), vec3f::load(AXIS_Z), STAMP);
    CHECK(ctx, !hinge_rotation_constraint_part::is_active(part));
    CHECK(ctx, !hinge_rotation_constraint_part::solve_velocity_constraint(part, a, b));
}
