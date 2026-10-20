#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/constraints/cone_constraint.h"
#include "physics/constraints/constraint_test_scene.h"
#include "test.h"

using cone_constraint::ConeConstraint;
using rigid_body::RigidBody;

static const Float3 GRAVITY = {0.0f, -9.81f, 0.0f};

/** the angle between the two twist axes in world space */
static float swing_angle(const ConeConstraint &constraint, const RigidBody *a, const RigidBody *b) {
    const Vec3f twist1 = vec3f::transform_quat(constraint.local_space_twist_axis1, quatf::load(a->quaternion));
    const Vec3f twist2 = vec3f::transform_quat(constraint.local_space_twist_axis2, quatf::load(b->quaternion));
    return scalar::acos(scalar::max(-1.0f, scalar::min(1.0f, vec3f::dot(twist1, twist2))));
}

void test_cone_constraint_create(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    const Float4 turn = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI / 2.0f);
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f}, turn);
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {2.0f, 0.0f, 0.0f});

    // world space: twist axes normalized and taken to body space
    const constraint_id::ConstraintId id = cone_constraint::create(*scene.world, a, b,
                                                                   {.point_a = {1.0f, 0.0f, 0.0f},
                                                                    .twist_axis_a = {3.0f, 0.0f, 0.0f},
                                                                    .point_b = {1.0f, 0.0f, 0.0f},
                                                                    .twist_axis_b = {0.0f, 0.0f, 2.0f},
                                                                    .half_cone_angle = scalar::PI / 3.0f});
    const ConeConstraint *constraint = cone_constraint::get(*scene.world, id);
    CHECK(ctx, constraint != nullptr && constraint_id::type(id) == constraint_id::CONE);
    CHECK_NEAR(ctx, constraint->cos_half_cone_angle, 0.5f, 1e-6f);
    // a is turned a quarter about z, so world +x is its body -y
    CHECK(ctx, vec3f::distance(constraint->local_space_twist_axis1, vec3f::make(0.0f, -1.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(constraint->local_space_twist_axis2, vec3f::make(0.0f, 0.0f, 1.0f)) < 1e-6f);
    CHECK(ctx, vec3f::distance(world_point(a, constraint->local_space_position1), vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(constraint->local_space_position2, vec3f::make(-1.0f, 0.0f, 0.0f)) < 1e-6f);
    // the fallback rotation axis: a world unit vector perpendicular to the first twist axis
    CHECK(ctx, vec3f::distance(constraint->world_space_rotation_axis, vec3f::make(0.0f, 0.0f, -1.0f)) < 1e-6f);

    // local space: the fallback axis is taken to world space by body a's rotation
    const constraint_id::ConstraintId local_id = cone_constraint::create(
        *scene.world, a, b, {.twist_axis_a = {0.0f, 0.0f, 1.0f}, .space = constraints::SPACE_LOCAL});
    const ConeConstraint *local = cone_constraint::get(*scene.world, local_id);
    CHECK(ctx, vec3f::exact_equals(local->local_space_twist_axis1, vec3f::make(0.0f, 0.0f, 1.0f)));
    // the perpendicular of +z is (0, 1, 0) in body a, which a's quarter turn takes to world -x
    CHECK(ctx, vec3f::distance(local->world_space_rotation_axis, vec3f::make(-1.0f, 0.0f, 0.0f)) < 1e-5f);
    // the default cone allows no swing
    CHECK(ctx, local->cos_half_cone_angle == 1.0f);
    release_constraint_scene(scene);
}

void test_cone_constraint_half_cone_angle(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    ConeConstraint *constraint =
        cone_constraint::get(*scene.world, cone_constraint::create(*scene.world, a, b, {}));
    cone_constraint::set_half_cone_angle(*constraint, scalar::PI / 4.0f);
    CHECK_NEAR(ctx, cone_constraint::get_cos_half_cone_angle(*constraint), scalar::sqrt(0.5f), 1e-6f);
    CHECK_NEAR(ctx, cone_constraint::get_half_cone_angle(*constraint), scalar::PI / 4.0f, 1e-6f);
    // the ends of [0, pi]
    cone_constraint::set_half_cone_angle(*constraint, 0.0f);
    CHECK(ctx, cone_constraint::get_cos_half_cone_angle(*constraint) == 1.0f);
    cone_constraint::set_half_cone_angle(*constraint, scalar::PI);
    CHECK_NEAR(ctx, cone_constraint::get_cos_half_cone_angle(*constraint), -1.0f, 1e-6f);
    release_constraint_scene(scene);
}

void test_cone_constraint_rotation_axis_from_a_tiny_cross(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    ConeConstraint *constraint = cone_constraint::get(
        *scene.world, cone_constraint::create(*scene.world, a, b,
                                                   {.point_a = {0.5f, 0.0f, 0.0f},
                                                    .twist_axis_a = {1.0f, 0.0f, 0.0f},
                                                    .point_b = {0.5f, 0.0f, 0.0f},
                                                    .twist_axis_b = {1.0f, 0.0f, 0.0f},
                                                    .half_cone_angle = scalar::PI / 2.0f}));
    // the fallback starts as the perpendicular of +x, which is -z
    CHECK(ctx, vec3f::exact_equals(constraint->world_space_rotation_axis, vec3f::make(0.0f, 0.0f, -1.0f)));

    // b swung almost half a turn about y: its twist axis is (-1, 0, 5e-7), and twist2 x twist1 is +y of length 5e-7
    b->quaternion = {0.0f, 1.0f, 0.0f, -2.5e-7f};
    cone_constraint::setup_velocity(*constraint, scene.world->bodies, 1.0f / 60.0f);
    CHECK(ctx, constraint->cos_theta < constraint->cos_half_cone_angle);
    CHECK(ctx, vec3f::distance(constraint->world_space_rotation_axis, vec3f::make(0.0f, 1.0f, 0.0f)) < 1e-6f);
    release_constraint_scene(scene);
}

void test_cone_constraint_clamps_swing(TestCtx &ctx) {
    // a ball held at its center to a static body, spun about z (a swing of its +x twist axis) inside a 30 degree cone
    ConstraintScene scene = make_constraint_scene();
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    const float half_cone = scalar::PI / 6.0f;
    const constraint_id::ConstraintId id = cone_constraint::create(
        *scene.world, base, ball,
        {.twist_axis_a = {1.0f, 0.0f, 0.0f}, .twist_axis_b = {1.0f, 0.0f, 0.0f}, .half_cone_angle = half_cone});
    scene.ids.push(id);
    const ConeConstraint *constraint = cone_constraint::get(*scene.world, id);
    ball->motion_properties.angular_velocity = {0.0f, 0.0f, 3.0f};

    float widest = 0.0f;
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 120; i++) {
        step_constraint_scene(scene, dt, vec3f::zero());
        widest = scalar::max(widest, swing_angle(*constraint, base, ball));
    }
    // unconstrained it would have turned 6 radians; it stops at the cone's edge, overshooting by at most one
    // step's turn since the limit engages once crossed
    CHECK(ctx, widest < half_cone + 3.0f * dt);
    CHECK_NEAR(ctx, swing_angle(*constraint, base, ball), half_cone, 2e-2f);
    CHECK(ctx, scalar::abs(ball->motion_properties.angular_velocity.z) < 1e-2f);
    // the limit pushes back only: lambda one-signed
    CHECK(ctx, cone_constraint::get_total_lambda_rotation(*constraint) >= 0.0f);
    release_constraint_scene(scene);
}

void test_cone_constraint_leaves_twist_free(TestCtx &ctx) {
    // spinning about the twist axis itself never leaves the cone, so nothing slows it
    ConstraintScene scene = make_constraint_scene();
    RigidBody *base = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    const constraint_id::ConstraintId id = cone_constraint::create(*scene.world, base, ball,
                                                                   {.half_cone_angle = 0.1f});
    scene.ids.push(id);
    ball->motion_properties.angular_velocity = {5.0f, 0.0f, 0.0f};
    for (int i = 0; i < 60; i++) step_constraint_scene(scene, 1.0f / 60.0f, vec3f::zero());
    const ConeConstraint *constraint = cone_constraint::get(*scene.world, id);
    CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.x, 5.0f, 1e-4f);
    CHECK(ctx, swing_angle(*constraint, base, ball) < 1e-3f);
    CHECK(ctx, !angle_constraint_part::is_active(constraint->angle_constraint_part));
    release_constraint_scene(scene);
}

void test_cone_constraint_pendulum(TestCtx &ctx) {
    // a pendulum kicked hard enough to go over the top, held to a 20 degree cone about straight down
    ConstraintScene scene = make_constraint_scene();
    RigidBody *pivot = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 10.0f, 0.0f});
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 9.0f, 0.0f});
    const float half_cone = 20.0f * scalar::DEGREES_TO_RADIANS;
    const Float3 down = {0.0f, -1.0f, 0.0f};
    const Float3 pivot_point = {0.0f, 10.0f, 0.0f};
    const constraint_id::ConstraintId id = cone_constraint::create(
        *scene.world, pivot, ball,
        {.point_a = pivot_point, .twist_axis_a = down, .point_b = pivot_point, .twist_axis_b = down, .half_cone_angle = half_cone});
    scene.ids.push(id);
    const ConeConstraint *constraint = cone_constraint::get(*scene.world, id);
    ball->motion_properties.linear_velocity = {5.0f, 0.0f, 2.0f};

    const float dt = 1.0f / 60.0f;
    float widest = 0.0f;
    float worst_anchor = 0.0f;
    for (int i = 0; i < 180; i++) {
        step_constraint_scene(scene, dt, vec3f::load(GRAVITY));
        widest = scalar::max(widest, swing_angle(*constraint, pivot, ball));
        worst_anchor = scalar::max(worst_anchor, vec3f::distance(world_point(ball, constraint->local_space_position2), vec3f::load(pivot_point)));
    }
    // it reaches the edge, overshooting by at most one step's swing (about 5.4 rad/s at the bottom)
    CHECK(ctx, widest > half_cone - 2e-2f);
    CHECK(ctx, widest < half_cone + 5.4f * dt);
    // the impacts on the cone's edge open the joint a little; the position iterations close it again
    CHECK(ctx, worst_anchor < 5e-2f);
    CHECK(ctx, vec3f::distance(world_point(ball, constraint->local_space_position2), vec3f::load(pivot_point)) < 1e-2f);
    CHECK(ctx, swing_angle(*constraint, pivot, ball) < half_cone + 2e-2f);
    // the ball hangs on its rod: one meter from the pivot
    CHECK_NEAR(ctx, vec3f::distance(vec3f::load(ball->center_of_mass_position), vec3f::load(pivot_point)), 1.0f, 1e-2f);
    CHECK(ctx, vec3f::exact_equals(cone_constraint::get_total_lambda_position(*constraint), constraint->point_constraint_part.total_lambda));
    release_constraint_scene(scene);
}
