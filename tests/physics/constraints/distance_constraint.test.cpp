#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "physics/constraints/constraint_test_scene.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "test.h"

using distance_constraint::DistanceConstraint;
using rigid_body::RigidBody;

static const Float3 GRAVITY = {0.0f, -9.81f, 0.0f};

static float current_distance(const DistanceConstraint &constraint, const RigidBody *a, const RigidBody *b) {
    return vec3f::distance(world_point(a, constraint.local_space_position1), world_point(b, constraint.local_space_position2));
}

void test_distance_constraint_create_limits(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {4.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), 1.0f));
    const Float3 point_a = {0.5f, 0.0f, 0.0f};
    const Float3 point_b = {3.5f, 0.0f, 0.0f};

    // no limits given: both are the initial distance
    const DistanceConstraint *fixed_length = distance_constraint::get(
        scene.constraints, distance_constraint::create(scene.constraints, scene.bodies, a, b, {.point_a = point_a, .point_b = point_b}));
    CHECK(ctx, fixed_length->min_distance == 3.0f && fixed_length->max_distance == 3.0f);
    CHECK(ctx, vec3f::distance(world_point(b, fixed_length->local_space_position2), vec3f::load(point_b)) < 1e-5f);
    CHECK(ctx, vec3f::exact_equals(fixed_length->world_space_normal, vec3f::make(0.0f, 1.0f, 0.0f)));

    // one limit given: the other is the initial distance, or the given one when that is tighter
    const DistanceConstraint *min_only =
        distance_constraint::get(scene.constraints, distance_constraint::create(scene.constraints, scene.bodies, a, b,
                                                                                {.point_a = point_a, .point_b = point_b, .min_distance = 1.0f}));
    CHECK(ctx, min_only->min_distance == 1.0f && min_only->max_distance == 3.0f);
    const DistanceConstraint *min_above =
        distance_constraint::get(scene.constraints, distance_constraint::create(scene.constraints, scene.bodies, a, b,
                                                                                {.point_a = point_a, .point_b = point_b, .min_distance = 5.0f}));
    CHECK(ctx, min_above->min_distance == 5.0f && min_above->max_distance == 5.0f);
    const DistanceConstraint *max_only =
        distance_constraint::get(scene.constraints, distance_constraint::create(scene.constraints, scene.bodies, a, b,
                                                                                {.point_a = point_a, .point_b = point_b, .max_distance = 2.0f}));
    CHECK(ctx, max_only->min_distance == 2.0f && max_only->max_distance == 2.0f);

    // local space: world positions from the bodies
    const DistanceConstraint *local = distance_constraint::get(
        scene.constraints,
        distance_constraint::create(scene.constraints, scene.bodies, a, b,
                                    {.point_a = {0.0f, 1.0f, 0.0f}, .point_b = {0.0f, 1.0f, 0.0f}, .space = constraints::SPACE_LOCAL}));
    CHECK(ctx, vec3f::distance(local->world_space_position2, vec3f::make(4.0f, 1.0f, 0.0f)) < 1e-5f);
    CHECK_NEAR(ctx, local->min_distance, 4.0f, 1e-5f);
    release_constraint_scene(scene);
}

void test_distance_constraint_set_distance(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {2.0f, 0.0f, 0.0f});
    DistanceConstraint *constraint = distance_constraint::get(
        scene.constraints, distance_constraint::create(scene.constraints, scene.bodies, a, b, {.point_b = {2.0f, 0.0f, 0.0f}}));
    distance_constraint::set_distance(*constraint, 1.0f, 4.0f);
    CHECK(ctx, constraint->min_distance == 1.0f && constraint->max_distance == 4.0f);
    // both are set as given: a negative is a value like any other, not "keep the current one"
    distance_constraint::set_distance(*constraint, -1.0f, 3.0f);
    CHECK(ctx, constraint->min_distance == -1.0f && constraint->max_distance == 3.0f);
    release_constraint_scene(scene);
}

void test_distance_constraint_reused_slot_starts_fresh(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {5.0f, 0.0f, 0.0f});

    // a soft spring, and a setup that points the normal along +x from a to b
    const constraint_id::ConstraintId first_id = distance_constraint::create(
        scene.constraints, scene.bodies, a, b,
        {.point_b = {5.0f, 0.0f, 0.0f},
         .spring_settings = {.mode = spring_settings::STIFFNESS_AND_DAMPING, .frequency_or_stiffness = 500.0f, .damping = 3.0f}});
    DistanceConstraint *first = distance_constraint::get(scene.constraints, first_id);
    distance_constraint::setup_velocity(*first, scene.bodies, 1.0f / 60.0f);
    CHECK(ctx, vec3f::exact_equals(first->world_space_normal, vec3f::make(1.0f, 0.0f, 0.0f)));
    const uint32_t first_index = first->base.index;
    distance_constraint::remove(scene.constraints, scene.bodies, *first);

    // coincident points have no direction of their own, so the normal is the fallback; no spring given is a hard limit
    const DistanceConstraint *second = distance_constraint::get(
        scene.constraints, distance_constraint::create(scene.constraints, scene.bodies, a, b,
                                                       {.point_a = {2.0f, 0.0f, 0.0f}, .point_b = {2.0f, 0.0f, 0.0f}}));
    CHECK(ctx, second->base.index == first_index);
    CHECK(ctx, vec3f::exact_equals(second->world_space_normal, vec3f::make(0.0f, 1.0f, 0.0f)));
    CHECK(ctx, second->limits_spring_settings.mode == spring_settings::FREQUENCY_AND_DAMPING);
    CHECK(ctx, second->limits_spring_settings.frequency_or_stiffness == 0.0f);
    CHECK(ctx, second->limits_spring_settings.damping == 0.0f);
    release_constraint_scene(scene);
}

void test_distance_constraint_pendulum(TestCtx &ctx) {
    // a shapeless static anchor and a ball five below it, 30 steps of gravity
    ConstraintScene scene = make_constraint_scene();
    RigidBody *anchor = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 10.0f, 0.0f});
    RigidBody *bob = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 5.0f, 0.0f});
    const constraint_id::ConstraintId id = distance_constraint::create(
        scene.constraints, scene.bodies, anchor, bob, {.point_a = {0.0f, 10.0f, 0.0f}, .point_b = {0.0f, 5.0f, 0.0f}});
    scene.ids.push(id);
    for (int i = 0; i < 30; i++) step_constraint_scene(scene, 1.0f / 60.0f, vec3f::load(GRAVITY));
    CHECK_NEAR(ctx, vec3f::distance(vec3f::load(anchor->position), vec3f::load(bob->position)), 5.0f, 0.05f);
    // hanging straight down, the rope carries the weight: m g dt per step
    const float weight_impulse = 9.81f / 60.0f / bob->motion_properties.inv_mass;
    CHECK_NEAR(ctx, scalar::abs(distance_constraint::get_total_lambda(*distance_constraint::get(scene.constraints, id))), weight_impulse,
               weight_impulse * 1e-2f);
    release_constraint_scene(scene);
}

void test_distance_constraint_holds_length(TestCtx &ctx) {
    // a ball on a 3 m rod from a static pivot, started sideways and swinging
    ConstraintScene scene = make_constraint_scene();
    RigidBody *pivot = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 10.0f, 0.0f});
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {3.0f, 10.0f, 0.0f});
    ball->motion_properties.linear_velocity = {0.0f, 0.0f, 4.0f};
    const constraint_id::ConstraintId id = distance_constraint::create(
        scene.constraints, scene.bodies, pivot, ball, {.point_a = {0.0f, 10.0f, 0.0f}, .point_b = {3.0f, 10.0f, 0.0f}});
    scene.ids.push(id);
    const DistanceConstraint *constraint = distance_constraint::get(scene.constraints, id);

    float worst = 0.0f;
    float lowest = 10.0f;
    for (int i = 0; i < 180; i++) {
        step_constraint_scene(scene, 1.0f / 60.0f, vec3f::load(GRAVITY));
        worst = scalar::max(worst, scalar::abs(current_distance(*constraint, pivot, ball) - 3.0f));
        lowest = scalar::min(lowest, ball->center_of_mass_position.y);
    }
    CHECK(ctx, worst < 1e-2f);
    CHECK(ctx, lowest < 7.5f);
    release_constraint_scene(scene);
}

void test_distance_constraint_respects_min_max(TestCtx &ctx) {
    // a slack rope between 1 and 3: thrown apart it stops at 3, thrown together it stops at 1, in between it is free
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {2.0f, 0.0f, 0.0f});
    const constraint_id::ConstraintId id = distance_constraint::create(
        scene.constraints, scene.bodies, a, b, {.point_b = {2.0f, 0.0f, 0.0f}, .min_distance = 1.0f, .max_distance = 3.0f});
    scene.ids.push(id);
    const DistanceConstraint *constraint = distance_constraint::get(scene.constraints, id);
    const float dt = 1.0f / 60.0f;

    // free in between: the first step leaves the velocities alone
    a->motion_properties.linear_velocity = {-0.5f, 0.0f, 0.0f};
    b->motion_properties.linear_velocity = {0.5f, 0.0f, 0.0f};
    step_constraint_scene(scene, dt, vec3f::zero());
    CHECK(ctx, !axis_constraint_part::is_active(constraint->axis_constraint));
    CHECK(ctx, b->motion_properties.linear_velocity.x == 0.5f);

    b->motion_properties.linear_velocity = {4.0f, 0.0f, 0.0f};
    float longest = 0.0f;
    for (int i = 0; i < 90; i++) {
        step_constraint_scene(scene, dt, vec3f::zero());
        longest = scalar::max(longest, current_distance(*constraint, a, b));
    }
    // the limit engages once crossed, so it can overshoot by one step's travel before pulling back
    CHECK(ctx, longest < 3.0f + 4.0f * dt);
    CHECK_NEAR(ctx, current_distance(*constraint, a, b), 3.0f, 2e-2f);
    // at the max the rope only pulls
    CHECK(ctx, distance_constraint::get_total_lambda(*constraint) <= 0.0f);

    a->motion_properties.linear_velocity = {3.0f, 0.0f, 0.0f};
    b->motion_properties.linear_velocity = {-3.0f, 0.0f, 0.0f};
    float shortest = 10.0f;
    for (int i = 0; i < 90; i++) {
        step_constraint_scene(scene, dt, vec3f::zero());
        shortest = scalar::min(shortest, current_distance(*constraint, a, b));
    }
    CHECK(ctx, shortest > 1.0f - 6.0f * dt);
    CHECK_NEAR(ctx, current_distance(*constraint, a, b), 1.0f, 2e-2f);
    release_constraint_scene(scene);
}

void test_distance_constraint_soft_limit(TestCtx &ctx) {
    // a spring limit: no position correction, and the bob oscillates past the rest length before settling
    ConstraintScene scene = make_constraint_scene();
    RigidBody *anchor = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 10.0f, 0.0f});
    RigidBody *bob = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 8.0f, 0.0f});
    const constraint_id::ConstraintId id = distance_constraint::create(
        scene.constraints, scene.bodies, anchor, bob,
        {.point_a = {0.0f, 10.0f, 0.0f}, .point_b = {0.0f, 8.0f, 0.0f}, .spring_settings = {.frequency_or_stiffness = 2.0f, .damping = 0.5f}});
    scene.ids.push(id);
    DistanceConstraint *constraint = distance_constraint::get(scene.constraints, id);
    CHECK(ctx, !distance_constraint::solve_position(*constraint, scene.bodies, 1.0f / 60.0f, 0.2f));

    float longest = 0.0f;
    for (int i = 0; i < 240; i++) {
        step_constraint_scene(scene, 1.0f / 60.0f, vec3f::load(GRAVITY));
        longest = scalar::max(longest, current_distance(*constraint, anchor, bob));
    }
    CHECK(ctx, longest > 2.02f);
    // settled where the spring holds the weight: x = g / omega^2 below the rest length
    const float omega = 2.0f * scalar::PI * 2.0f;
    CHECK_NEAR(ctx, current_distance(*constraint, anchor, bob), 2.0f + 9.81f / (omega * omega), 1e-2f);
    release_constraint_scene(scene);
}
