#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "physics/constraints/constraint_test_scene.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "test.h"

using point_constraint::PointConstraint;
using rigid_body::RigidBody;

static const Float3 GRAVITY = {0.0f, -9.81f, 0.0f};

void test_point_constraint_create(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 2.0f, 3.0f}, stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI / 2.0f));
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {4.0f, 2.0f, 3.0f});

    // world space: the points become body space offsets from each center of mass
    const constraint_id::ConstraintId id = point_constraint::create(
        *scene.world, a, b,
        {.point_a = {2.0f, 2.0f, 3.0f}, .point_b = {3.0f, 2.0f, 3.0f}, .constraint_priority = 3, .num_velocity_steps_override = 7});
    CHECK(ctx, id != constraint_id::INVALID_CONSTRAINT_ID);
    CHECK(ctx, constraint_id::type(id) == constraint_id::POINT && constraint_id::index(id) == 0);
    PointConstraint *constraint = point_constraint::get(*scene.world, id);
    CHECK(ctx, constraint != nullptr);
    CHECK(ctx, !constraint->base.pooled && constraint->base.enabled && constraint->base.id == id);
    CHECK(ctx, constraint->base.body_index_a == a->index && constraint->base.body_index_b == b->index);
    // a is turned 90 degrees about z, so world +x is its body -y
    CHECK(ctx, vec3f::distance(constraint->local_space_position1, vec3f::make(0.0f, -1.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(constraint->local_space_position2, vec3f::make(-1.0f, 0.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(world_point(a, constraint->local_space_position1), vec3f::make(2.0f, 2.0f, 3.0f)) < 1e-5f);

    // both bodies track it
    CHECK(ctx, a->constraint_ids.length == 1 && a->constraint_ids[0] == id);
    CHECK(ctx, b->constraint_ids.length == 1 && b->constraint_ids[0] == id);

    constraints::ConstraintIterationOverrides overrides;
    point_constraint::get_iteration_overrides(overrides, *constraint);
    CHECK(ctx, overrides.velocity == 7 && overrides.position == 0);
    constraints::ConstraintSortFields fields;
    point_constraint::get_sort_fields(fields, *constraint);
    CHECK(ctx, fields.priority == 3 && fields.index == 0);

    // local space: the points are taken as given
    const constraint_id::ConstraintId local_id = point_constraint::create(
        *scene.world, a, b,
        {.point_a = {0.5f, 0.0f, 0.0f}, .point_b = {0.0f, 0.25f, 0.0f}, .space = constraints::SPACE_LOCAL});
    PointConstraint *local = point_constraint::get(*scene.world, local_id);
    CHECK(ctx, vec3f::exact_equals(local->local_space_position1, vec3f::make(0.5f, 0.0f, 0.0f)));
    CHECK(ctx, vec3f::exact_equals(local->local_space_position2, vec3f::make(0.0f, 0.25f, 0.0f)));
    // each record's first constraint has sequence 1
    CHECK(ctx, constraint_id::index(id) == 0 && constraint_id::sequence(id) == 1);
    CHECK(ctx, constraint_id::index(local_id) == 1 && constraint_id::sequence(local_id) == 1);

    // an id of another type does not resolve here
    CHECK(ctx, point_constraint::get(*scene.world, constraint_id::ser(0, constraint_id::FIXED, 0)) == nullptr);
    release_constraint_scene(scene);
}

void test_point_constraint_remove(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    const constraint_id::ConstraintId first = point_constraint::create(*scene.world, a, b, {});
    const constraint_id::ConstraintId second = point_constraint::create(*scene.world, a, b, {});

    PointConstraint *constraint = point_constraint::get(*scene.world, first);
    constraint->point_constraint_part.total_lambda = vec3f::make(1.0f, 2.0f, 3.0f);
    CHECK(ctx, constraints::remove(*scene.world, first));
    CHECK(ctx, point_constraint::get(*scene.world, first) == nullptr);
    CHECK(ctx, !constraints::remove(*scene.world, first));
    CHECK(ctx, constraint->base.pooled);
    CHECK(ctx, a->constraint_ids.length == 1 && a->constraint_ids[0] == second);
    CHECK(ctx, b->constraint_ids.length == 1 && b->constraint_ids[0] == second);

    // the freed slot is reused with its own sequence plus one, starting with nothing of the removed one; the old id stays dead
    const constraint_id::ConstraintId third = point_constraint::create(*scene.world, a, b, {});
    CHECK(ctx, point_constraint::get(*scene.world, third) == constraint);
    CHECK(ctx, vec3f::exact_equals(constraint->point_constraint_part.total_lambda, vec3f::zero()));
    CHECK(ctx, constraint_id::index(third) == constraint_id::index(first));
    CHECK(ctx, constraint_id::sequence(third) == 2);
    CHECK(ctx, point_constraint::get(*scene.world, first) == nullptr);
    CHECK(ctx, point_constraint::get(*scene.world, third) != nullptr);
    CHECK(ctx, scene.world->constraints.pools[constraint_id::POINT].count == 2);
    release_constraint_scene(scene);
}

void test_point_constraint_total_lambda(TestCtx &ctx) {
    // a static anchor holding a body against gravity: the impulse per step is m g dt, upward on b
    ConstraintScene scene = make_constraint_scene();
    RigidBody *anchor = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    RigidBody *body = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    const constraint_id::ConstraintId id = point_constraint::create(*scene.world, anchor, body, {});
    scene.ids.push(id);
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 10; i++) step_constraint_scene(scene, dt, vec3f::load(GRAVITY));
    const PointConstraint *constraint = point_constraint::get(*scene.world, id);
    const float mass = 1.0f / body->motion_properties.inv_mass;
    CHECK_NEAR(ctx, scalar::abs(vec3f::y(point_constraint::get_total_lambda(*constraint))), mass * 9.81f * dt, mass * 9.81f * dt * 1e-3f);
    CHECK(ctx, vec3f::length(vec3f::load(body->center_of_mass_position)) < 1e-4f);
    CHECK(ctx, vec3f::length(vec3f::load(body->motion_properties.linear_velocity)) < 1e-4f);
    release_constraint_scene(scene);
}

void test_point_constraint_holds_anchor(TestCtx &ctx) {
    // a pendulum: a static pivot at (0, 10, 0) and a ball at (2, 10, 0) attached at the pivot
    ConstraintScene scene = make_constraint_scene();
    RigidBody *pivot = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 10.0f, 0.0f});
    RigidBody *ball = add_body(scene, motion_type::MOTION_DYNAMIC, {2.0f, 10.0f, 0.0f});
    const Float3 anchor = {0.0f, 10.0f, 0.0f};
    const constraint_id::ConstraintId id =
        point_constraint::create(*scene.world, pivot, ball, {.point_a = anchor, .point_b = anchor});
    scene.ids.push(id);
    const PointConstraint *constraint = point_constraint::get(*scene.world, id);

    const float dt = 1.0f / 60.0f;
    float worst = 0.0f;
    float lowest = 10.0f;
    for (int i = 0; i < 120; i++) {
        step_constraint_scene(scene, dt, vec3f::load(GRAVITY));
        const Vec3f on_ball = world_point(ball, constraint->local_space_position2);
        worst = scalar::max(worst, vec3f::distance(on_ball, vec3f::load(anchor)));
        lowest = scalar::min(lowest, ball->center_of_mass_position.y);
    }
    // the ball swings down through the bottom of its arc, its attachment on the pivot throughout
    CHECK(ctx, lowest < 8.1f);
    CHECK(ctx, worst < 1e-2f);
    CHECK_NEAR(ctx, vec3f::distance(vec3f::load(ball->center_of_mass_position), vec3f::load(anchor)), 2.0f, 1e-2f);
    release_constraint_scene(scene);
}

void test_point_constraint_joins_two_dynamic_bodies(TestCtx &ctx) {
    const float dt = 1.0f / 60.0f;
    // two dynamic bodies thrown apart stay joined at the point, and momentum is kept
    ConstraintScene pair = make_constraint_scene();
    RigidBody *a = add_body(pair, motion_type::MOTION_DYNAMIC, {-0.5f, 0.0f, 0.0f});
    RigidBody *b = add_body(pair, motion_type::MOTION_DYNAMIC, {0.5f, 0.0f, 0.0f});
    a->motion_properties.linear_velocity = {-3.0f, 1.0f, 0.0f};
    b->motion_properties.linear_velocity = {3.0f, 1.0f, 0.0f};
    const constraint_id::ConstraintId pair_id =
        point_constraint::create(*pair.world, a, b, {.point_a = {0.0f, 0.0f, 0.0f}, .point_b = {0.0f, 0.0f, 0.0f}});
    pair.ids.push(pair_id);
    const PointConstraint *joint = point_constraint::get(*pair.world, pair_id);
    for (int i = 0; i < 60; i++) step_constraint_scene(pair, dt, vec3f::zero());
    CHECK(ctx, vec3f::distance(world_point(a, joint->local_space_position1), world_point(b, joint->local_space_position2)) < 1e-2f);
    const Vec3f momentum = vec3f::add(vec3f::load(a->motion_properties.linear_velocity), vec3f::load(b->motion_properties.linear_velocity));
    CHECK(ctx, vec3f::distance(momentum, vec3f::make(0.0f, 2.0f, 0.0f)) < 1e-3f);
    release_constraint_scene(pair);
}
