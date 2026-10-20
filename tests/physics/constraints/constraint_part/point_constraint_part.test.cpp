#include "math/mat4f.h"
#include "math/quatf.h"
#include "math/vec3f.h"
#include "physics/body/dof.h"
#include "physics/body/motion_properties.h"
#include "physics/body/rigid_body.h"
#include "physics/constraints/constraint_part/point_constraint_part.h"
#include "physics/constraints/constraint_part/part_body.h"
#include "test.h"
#include "test_math.h"

using point_constraint_part::PointConstraintPart;
using rigid_body::RigidBody;

static void setup(PointConstraintPart &part, RigidBody &a, const Float3 &r1_local, RigidBody &b, const Float3 &r2_local) {
    Float4x4 rotation_a;
    Float4x4 rotation_b;
    mat4f::store(rotation_a, mat4f::from_quat(quatf::load(a.quaternion)));
    mat4f::store(rotation_b, mat4f::from_quat(quatf::load(b.quaternion)));
    point_constraint_part::calculate_constraint_properties(part, a, mat4f::load(rotation_a), vec3f::load(r1_local), b, mat4f::load(rotation_b), vec3f::load(r2_local),
                                                           motion_properties::STEP_STAMP_NONE);
}

// (x2 + r2) - (x1 + r1) with the world space arms the last setup computed
static Vec3f separation(const PointConstraintPart &part, const RigidBody &a, const RigidBody &b) {
    const Vec3f on_b = vec3f::add(vec3f::load(b.center_of_mass_position), part.r2);
    return vec3f::subtract(vec3f::subtract(on_b, vec3f::load(a.center_of_mass_position)), part.r1);
}

static Vec3f point_velocity(const RigidBody &body, Vec3f r) {
    const Vec3f angular = vec3f::cross(vec3f::load(body.motion_properties.angular_velocity), r);
    return vec3f::add(angular, vec3f::load(body.motion_properties.linear_velocity));
}

void test_point_constraint_part_create_deactivate(TestCtx &ctx) {
    PointConstraintPart part = point_constraint_part::create();
    CHECK(ctx, vec3f::exact_equals(part.total_lambda, vec3f::zero()));
    CHECK(ctx, elements(part.effective_mass).m[0] == 1.0f && elements(part.effective_mass).m[15] == 1.0f);
    part.total_lambda = vec3f::make(1.0f, 2.0f, 3.0f);
    point_constraint_part::deactivate(part);
    CHECK(ctx, !point_constraint_part::is_active(part));
    CHECK(ctx, vec3f::exact_equals(part.total_lambda, vec3f::zero()));
}

void test_point_constraint_part_effective_mass(TestCtx &ctx) {
    // arms at the centers of mass: K^-1 = (m1^-1 + m2^-1) I
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 3.0f, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
    PointConstraintPart part = point_constraint_part::create();
    setup(part, a, {0.0f, 0.0f, 0.0f}, b, {0.0f, 0.0f, 0.0f});
    CHECK(ctx, point_constraint_part::is_active(part));
    CHECK_NEAR(ctx, elements(part.effective_mass).m[0], 0.25f, 1e-6f);
    CHECK_NEAR(ctx, elements(part.effective_mass).m[5], 0.25f, 1e-6f);
    CHECK_NEAR(ctx, elements(part.effective_mass).m[10], 0.25f, 1e-6f);
    CHECK(ctx, elements(part.effective_mass).m[1] == 0.0f && elements(part.effective_mass).m[4] == 0.0f);

    // an arm along x adds |r|^2 I^-1 on y and z only: [r]x I^-1 [r]x^T with isotropic I^-1 = 1
    setup(part, a, {1.0f, 0.0f, 0.0f}, b, {0.0f, 0.0f, 0.0f});
    CHECK(ctx, vec3f::exact_equals(part.r1, vec3f::make(1.0f, 0.0f, 0.0f)));
    CHECK_NEAR(ctx, elements(part.effective_mass).m[0], 0.25f, 1e-6f);
    CHECK_NEAR(ctx, elements(part.effective_mass).m[5], 1.0f / 5.0f, 1e-6f);
    CHECK_NEAR(ctx, elements(part.effective_mass).m[10], 1.0f / 5.0f, 1e-6f);

    // the arm turns with the body: a quarter turn about z puts local x on world y
    a.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.5f * scalar::PI);
    setup(part, a, {1.0f, 0.0f, 0.0f}, b, {0.0f, 0.0f, 0.0f});
    CHECK(ctx, vec3f::distance(part.r1, vec3f::make(0.0f, 1.0f, 0.0f)) < 1e-6f);

    // two static bodies: singular, inactive
    RigidBody s1 = make_body(motion_type::MOTION_STATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody s2 = make_body(motion_type::MOTION_STATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    part.total_lambda = vec3f::make(1.0f, 0.0f, 0.0f);
    setup(part, s1, {1.0f, 0.0f, 0.0f}, s2, {0.0f, 0.0f, 0.0f});
    CHECK(ctx, !point_constraint_part::is_active(part));
    CHECK(ctx, vec3f::exact_equals(part.total_lambda, vec3f::zero()));
    CHECK(ctx, !point_constraint_part::solve_velocity_constraint(part, s1, s2));
    CHECK(ctx, !point_constraint_part::solve_position_constraint(part, s1, s2, 1.0f));
}

void test_point_constraint_part_velocity_solve_joins_the_pin(TestCtx &ctx) {
    // two bodies pinned end to end, moving and spinning apart. one velocity solve makes the pin point move
    // the same on both, with the linear momentum kept
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {3.0f, 3.0f, 3.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {1.5f, 1.5f, 1.5f}, {2.0f, 0.0f, 0.0f});
    a.motion_properties.linear_velocity = {0.0f, -1.0f, 0.5f};
    a.motion_properties.angular_velocity = {0.0f, 0.0f, 1.0f};
    b.motion_properties.linear_velocity = {1.0f, 2.0f, 0.0f};
    b.motion_properties.angular_velocity = {0.5f, 0.0f, -2.0f};
    PointConstraintPart part = point_constraint_part::create();
    setup(part, a, {1.0f, 0.0f, 0.0f}, b, {-1.0f, 0.0f, 0.0f});

    Vec3f momentum_before = vec3f::scale(vec3f::load(a.motion_properties.linear_velocity), 1.0f);
    momentum_before = vec3f::scale_and_add(momentum_before, vec3f::load(b.motion_properties.linear_velocity), 2.0f);

    CHECK(ctx, point_constraint_part::solve_velocity_constraint(part, a, b));
    const Vec3f va = point_velocity(a, part.r1);
    const Vec3f vb = point_velocity(b, part.r2);
    CHECK(ctx, vec3f::distance(va, vb) < 1e-5f);

    Vec3f momentum_after = vec3f::scale(vec3f::load(a.motion_properties.linear_velocity), 1.0f);
    momentum_after = vec3f::scale_and_add(momentum_after, vec3f::load(b.motion_properties.linear_velocity), 2.0f);
    CHECK(ctx, vec3f::distance(momentum_before, momentum_after) < 1e-5f);

    // the impulse accumulated is the impulse applied: b's momentum changed by exactly total_lambda
    Vec3f b_momentum_change = vec3f::subtract(vec3f::load(b.motion_properties.linear_velocity), vec3f::make(1.0f, 2.0f, 0.0f));
    b_momentum_change = vec3f::scale(b_momentum_change, 2.0f);
    CHECK(ctx, vec3f::length(part.total_lambda) > 0.1f);
    CHECK(ctx, vec3f::distance(b_momentum_change, part.total_lambda) < 1e-5f);

    // solved: the next iteration's impulse is (all but) zero
    const Vec3f total = part.total_lambda;
    point_constraint_part::solve_velocity_constraint(part, a, b);
    CHECK(ctx, vec3f::distance(total, part.total_lambda) < 1e-5f);

    // the warm start reapplies the stored impulse: from the old velocities it lands on the solved ones
    const Float3 solved_a = a.motion_properties.linear_velocity;
    const Float3 solved_wb = b.motion_properties.angular_velocity;
    a.motion_properties.linear_velocity = {0.0f, -1.0f, 0.5f};
    a.motion_properties.angular_velocity = {0.0f, 0.0f, 1.0f};
    b.motion_properties.linear_velocity = {1.0f, 2.0f, 0.0f};
    b.motion_properties.angular_velocity = {0.5f, 0.0f, -2.0f};
    point_constraint_part::warm_start(part, a, b, 1.0f);
    CHECK(ctx, vec3f::distance(vec3f::load(a.motion_properties.linear_velocity), vec3f::load(solved_a)) < 1e-5f);
    CHECK(ctx, vec3f::distance(vec3f::load(b.motion_properties.angular_velocity), vec3f::load(solved_wb)) < 1e-5f);
    CHECK(ctx, vec3f::distance(point_velocity(a, part.r1), point_velocity(b, part.r2)) < 1e-4f);

    // a ratio scales the stored impulse
    point_constraint_part::warm_start(part, a, b, 0.5f);
    const Vec3f half_total = vec3f::scale(total, 0.5f);
    CHECK(ctx, vec3f::distance(part.total_lambda, half_total) < 1e-5f);
}

void test_point_constraint_part_respects_locked_translation(TestCtx &ctx) {
    // pinned at both centres of mass, the free body moving away diagonally. per axis lambda = (v1 - v2) / (1/m1 + 1/m2)
    // = 3 / 1.5 = 2 on x and y; the body locked on y takes lambda / m2 on x only
    RigidBody free_body = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
    RigidBody locked = make_body(motion_type::MOTION_DYNAMIC, 0.5f, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
    locked.motion_properties.allowed_degrees_of_freedom = dof::DOF_ALL & ~dof::TRANSLATION_Y;
    free_body.motion_properties.linear_velocity = {3.0f, 3.0f, 0.0f};
    PointConstraintPart part = point_constraint_part::create();
    setup(part, free_body, {0.0f, 0.0f, 0.0f}, locked, {0.0f, 0.0f, 0.0f});
    CHECK(ctx, point_constraint_part::solve_velocity_constraint(part, free_body, locked));
    CHECK_NEAR(ctx, locked.motion_properties.linear_velocity.x, 1.0f, 1e-6f);
    CHECK(ctx, locked.motion_properties.linear_velocity.y == 0.0f);
    CHECK(ctx, locked.motion_properties.linear_velocity.z == 0.0f);
    CHECK(ctx, vec3f::distance(vec3f::load(free_body.motion_properties.linear_velocity), vec3f::make(1.0f, 1.0f, 0.0f)) < 1e-6f);
}

void test_point_constraint_part_position_solve_closes_the_gap(TestCtx &ctx) {
    // no rotation (zero inverse inertia): one full baumgarte step closes the gap exactly, split by mass
    {
        RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
        RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 3.0f, {0.0f, 0.0f, 0.0f}, {2.4f, 0.4f, 0.0f});
        PointConstraintPart part = point_constraint_part::create();
        setup(part, a, {1.0f, 0.0f, 0.0f}, b, {-1.0f, 0.0f, 0.0f});
        CHECK(ctx, point_constraint_part::solve_position_constraint(part, a, b, 1.0f));
        CHECK(ctx, vec3f::length(separation(part, a, b)) < 1e-6f);
        // b, three times lighter, moves three times as far
        CHECK(ctx, vec3f::distance(vec3f::load(a.center_of_mass_position), vec3f::make(0.1f, 0.1f, 0.0f)) < 1e-6f);
        CHECK(ctx, vec3f::distance(vec3f::load(b.center_of_mass_position), vec3f::make(2.1f, 0.1f, 0.0f)) < 1e-6f);
        // joined: nothing more to do
        CHECK(ctx, !point_constraint_part::solve_position_constraint(part, a, b, 1.0f));
    }

    // a hinge-like pin pulled apart: bodies that can turn come back together over a few linearised
    // position iterations, each re-reading the arms from the new rotations
    {
        RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {3.0f, 3.0f, 3.0f}, {0.0f, 0.0f, 0.0f});
        RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {3.0f, 3.0f, 3.0f}, {2.3f, 0.5f, -0.2f});
        b.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.4f);
        PointConstraintPart part = point_constraint_part::create();
        setup(part, a, {1.0f, 0.0f, 0.0f}, b, {-1.0f, 0.0f, 0.0f});
        const float before = vec3f::length(separation(part, a, b));
        CHECK(ctx, before > 0.4f);

        CHECK(ctx, point_constraint_part::solve_position_constraint(part, a, b, 1.0f));
        setup(part, a, {1.0f, 0.0f, 0.0f}, b, {-1.0f, 0.0f, 0.0f});
        const float after_one = vec3f::length(separation(part, a, b));
        CHECK(ctx, after_one < 0.2f * before);

        for (int i = 0; i < 10; i++) {
            point_constraint_part::solve_position_constraint(part, a, b, 1.0f);
            setup(part, a, {1.0f, 0.0f, 0.0f}, b, {-1.0f, 0.0f, 0.0f});
        }
        CHECK(ctx, vec3f::length(separation(part, a, b)) < 1e-4f);
        // both turned to meet, and stayed unit rotations
        CHECK(ctx, scalar::abs(a.quaternion.w) < 1.0f);
        CHECK_NEAR(ctx, quatf::length(quatf::load(b.quaternion)), 1.0f, 1e-5f);
        // equal masses, no momentum added: the centre of mass between them has not moved
        const Vec3f centre = vec3f::add(vec3f::load(a.center_of_mass_position), vec3f::load(b.center_of_mass_position));
        CHECK(ctx, vec3f::distance(centre, vec3f::make(2.3f, 0.5f, -0.2f)) < 1e-5f);
    }

    // against a static body only the dynamic one moves
    {
        RigidBody anchor = make_body(motion_type::MOTION_STATIC, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
        RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {0.0f, 0.0f, 0.0f}, {0.0f, -1.5f, 0.0f});
        PointConstraintPart part = point_constraint_part::create();
        setup(part, anchor, {0.0f, 0.0f, 0.0f}, b, {0.0f, 1.0f, 0.0f});
        CHECK(ctx, point_constraint_part::solve_position_constraint(part, anchor, b, 0.5f));
        CHECK(ctx, vec3f::exact_equals(vec3f::load(anchor.center_of_mass_position), vec3f::zero()));
        CHECK(ctx, vec3f::distance(vec3f::load(b.center_of_mass_position), vec3f::make(0.0f, -1.25f, 0.0f)) < 1e-6f);
    }
}
