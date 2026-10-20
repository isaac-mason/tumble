#include "instance.h"
#include "math/scalar.h"
#include "physics/constraints/cone_constraint.h"
#include "physics/constraints/constraint_test_scene.h"
#include "physics/constraints/constraints.h"
#include "physics/constraints/distance_constraint.h"
#include "physics/constraints/fixed_constraint.h"
#include "physics/constraints/point_constraint.h"
#include "test.h"

using constraint_id::ConstraintId;
using rigid_body::RigidBody;

static bool has_id(const RigidBody *body, ConstraintId id) {
    for (uint32_t i = 0; i < body->constraint_ids.length; i++) {
        if (body->constraint_ids[i] == id) return true;
    }
    return false;
}

void test_constraints_pools(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        const constraints::ConstraintPool &pool = scene.constraints.pools[type];
        CHECK(ctx, pool.type == type && pool.stride == 0 && pool.count == 0);
    }

    // each type's stride holds its record, which starts with the base
    constraints::ConstraintPool &points = constraints::ensure_pool(scene.constraints, constraint_id::POINT);
    CHECK(ctx, points.stride >= sizeof(point_constraint::PointConstraint) && points.stride % 8 == 0);
    CHECK(ctx, constraints::ensure_pool(scene.constraints, constraint_id::DISTANCE).stride >=
                   sizeof(distance_constraint::DistanceConstraint));
    CHECK(ctx, constraints::ensure_pool(scene.constraints, constraint_id::FIXED).stride >= sizeof(fixed_constraint::FixedConstraint));
    CHECK(ctx, constraints::ensure_pool(scene.constraints, constraint_id::CONE).stride >= sizeof(cone_constraint::ConeConstraint));
    CHECK(ctx, &constraints::ensure_pool(scene.constraints, constraint_id::POINT) == &points);

    // records of different types in one world; growing a pool keeps earlier records intact
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    ConstraintId ids[20];
    for (uint32_t i = 0; i < 20; i++) {
        ids[i] = point_constraint::create(scene.constraints, scene.bodies, a, b, {.constraint_priority = (int32_t)i});
    }
    const ConstraintId fixed = fixed_constraint::create(scene.constraints, scene.bodies, a, b, {});
    CHECK(ctx, points.count == 20 && scene.constraints.pools[constraint_id::FIXED].count == 1);
    bool intact = true;
    for (uint32_t i = 0; i < 20; i++) {
        const constraints::ConstraintBase *base = constraints::get(scene.constraints, ids[i]);
        intact = intact && base != nullptr && base->index == i && base->constraint_priority == (int32_t)i && base->id == ids[i];
    }
    CHECK(ctx, intact);
    CHECK(ctx, constraint_id::index(fixed) == 0 && constraints::get(scene.constraints, fixed)->id == fixed);
    CHECK(ctx, a->constraint_ids.length == 21 && b->constraint_ids.length == 21);

    // an unknown index, a pool never used, a wrong sequence: nothing
    CHECK(ctx, constraints::get(scene.constraints, constraint_id::ser(20, constraint_id::POINT, 20)) == nullptr);
    CHECK(ctx, constraints::get(scene.constraints, constraint_id::ser(0, constraint_id::HINGE, 0)) == nullptr);
    CHECK(ctx, constraints::get(scene.constraints, constraint_id::ser(3, constraint_id::POINT, 4)) == nullptr);
    release_constraint_scene(scene);
}

void test_constraints_pool_caps_at_the_id_index_range(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    const ConstraintId live = point_constraint::create(scene.constraints, scene.bodies, a, b, {});
    constraints::ConstraintPool &points = scene.constraints.pools[constraint_id::POINT];
    // a pool already holding every index the id has room for: the next record is refused, not wrapped onto index 0
    const uint32_t count = points.count;
    points.count = constraint_id::MAX_CONSTRAINT_INDEX + 1;
    util::error_clear();
    CHECK(ctx, constraints::allocate_index(points) == ~0u);
    CHECK(ctx, util::error() == util::ERROR_QUOTA);
    util::error_clear();
    CHECK(ctx, point_constraint::create(scene.constraints, scene.bodies, a, b, {}) == constraint_id::INVALID_CONSTRAINT_ID);
    CHECK(ctx, util::error() == util::ERROR_QUOTA);
    util::error_clear();
    points.count = count;
    CHECK(ctx, constraints::get(scene.constraints, live) != nullptr && a->constraint_ids.length == 1);
    release_constraint_scene(scene);
}

void test_constraints_remove_by_id(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    RigidBody *c = add_body(scene, motion_type::MOTION_DYNAMIC, {2.0f, 0.0f, 0.0f});
    const ConstraintId ab = distance_constraint::create(scene.constraints, scene.bodies, a, b, {});
    const ConstraintId bc = cone_constraint::create(scene.constraints, scene.bodies, b, c, {});
    // a body constrained to itself is tracked once
    const ConstraintId aa = point_constraint::create(scene.constraints, scene.bodies, a, a, {});
    CHECK(ctx, a->constraint_ids.length == 2 && b->constraint_ids.length == 2 && c->constraint_ids.length == 1);

    constraints::remove_constraint_by_id(scene.constraints, scene.bodies, ab);
    CHECK(ctx, constraints::get(scene.constraints, ab) == nullptr);
    CHECK(ctx, !has_id(a, ab) && !has_id(b, ab) && has_id(b, bc) && has_id(a, aa));
    CHECK(ctx, scene.constraints.pools[constraint_id::DISTANCE].free_indices.length == 1);
    // removing again is a no-op
    constraints::remove_constraint_by_id(scene.constraints, scene.bodies, ab);
    CHECK(ctx, scene.constraints.pools[constraint_id::DISTANCE].free_indices.length == 1);

    constraints::remove_constraint_by_id(scene.constraints, scene.bodies, aa);
    CHECK(ctx, a->constraint_ids.length == 0);

    // the slot comes back with a fresh sequence
    const ConstraintId again = distance_constraint::create(scene.constraints, scene.bodies, a, c, {});
    CHECK(ctx, constraint_id::index(again) == constraint_id::index(ab) && again != ab);
    CHECK(ctx, constraints::get(scene.constraints, ab) == nullptr && constraints::get(scene.constraints, again) != nullptr);
    release_constraint_scene(scene);
}

void test_constraints_sequence_per_slot(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    const ConstraintId kept = point_constraint::create(scene.constraints, scene.bodies, a, b, {});
    ConstraintId churned = point_constraint::create(scene.constraints, scene.bodies, a, b, {});
    const uint32_t churned_index = constraint_id::index(churned);
    CHECK(ctx, constraint_id::sequence(kept) == 1 && constraint_id::sequence(churned) == 1);

    // a slot taken again bumps its own sequence, wrapping at 8 bits; the other slot's id is untouched
    bool bumped = true;
    for (uint32_t round = 0; round < constraint_id::MAX_SEQUENCE + 1; round++) {
        const ConstraintId stale = churned;
        constraints::remove_constraint_by_id(scene.constraints, scene.bodies, stale);
        churned = point_constraint::create(scene.constraints, scene.bodies, a, b, {});
        bumped = bumped && constraint_id::index(churned) == churned_index &&
                 constraint_id::sequence(churned) == ((constraint_id::sequence(stale) + 1) & constraint_id::SEQUENCE_MASK) &&
                 constraints::get(scene.constraints, stale) == nullptr && constraints::get(scene.constraints, churned) != nullptr;
    }
    CHECK(ctx, bumped);
    CHECK(ctx, constraint_id::sequence(churned) == 1);
    CHECK(ctx, constraints::get(scene.constraints, kept) != nullptr && constraint_id::sequence(kept) == 1);

    // a fresh slot starts at 1 whatever the pool's other slots have been through, as does another type's pool
    const ConstraintId fresh = point_constraint::create(scene.constraints, scene.bodies, a, b, {});
    CHECK(ctx, constraint_id::index(fresh) == 2 && constraint_id::sequence(fresh) == 1);
    const ConstraintId other_type = fixed_constraint::create(scene.constraints, scene.bodies, a, b, {});
    CHECK(ctx, constraint_id::index(other_type) == 0 && constraint_id::sequence(other_type) == 1);
    release_constraint_scene(scene);
}

void test_constraints_destroy_body_constraints(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *hub = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *spokes[3];
    ConstraintId ids[3];
    for (uint32_t i = 0; i < 3; i++) {
        spokes[i] = add_body(scene, motion_type::MOTION_DYNAMIC, {(float)i + 1.0f, 0.0f, 0.0f});
        ids[i] = point_constraint::create(scene.constraints, scene.bodies, hub, spokes[i], {});
    }
    const ConstraintId between = fixed_constraint::create(scene.constraints, scene.bodies, spokes[0], spokes[1], {});

    constraints::destroy_body_constraints(scene.constraints, scene.bodies, *hub);
    CHECK(ctx, hub->constraint_ids.length == 0);
    for (uint32_t i = 0; i < 3; i++) {
        CHECK(ctx, constraints::get(scene.constraints, ids[i]) == nullptr);
        CHECK(ctx, !has_id(spokes[i], ids[i]));
    }
    // a constraint between two other bodies is untouched
    CHECK(ctx, constraints::get(scene.constraints, between) != nullptr);
    CHECK(ctx, spokes[0]->constraint_ids.length == 1 && spokes[1]->constraint_ids.length == 1 && spokes[2]->constraint_ids.length == 0);
    release_constraint_scene(scene);
}

void test_constraints_overrides_and_sort(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    const ConstraintId p0 = point_constraint::create(scene.constraints, scene.bodies, a, b, {.constraint_priority = 2});
    const ConstraintId p1 = point_constraint::create(scene.constraints, scene.bodies, a, b, {.constraint_priority = 0});
    const ConstraintId d0 = distance_constraint::create(
        scene.constraints, scene.bodies, a, b,
        {.constraint_priority = 1, .num_velocity_steps_override = 12, .num_position_steps_override = 4});
    const ConstraintId f0 = fixed_constraint::create(scene.constraints, scene.bodies, a, b, {.constraint_priority = 2});
    const ConstraintId c0 = cone_constraint::create(scene.constraints, scene.bodies, a, b, {.constraint_priority = -1});
    const ConstraintId p2 = point_constraint::create(scene.constraints, scene.bodies, a, b, {.constraint_priority = 0});

    constraints::ConstraintIterationOverrides overrides;
    constraints::get_constraint_iteration_overrides(overrides, scene.constraints, d0);
    CHECK(ctx, overrides.velocity == 12 && overrides.position == 4);
    constraints::get_constraint_iteration_overrides(overrides, scene.constraints, p0);
    CHECK(ctx, overrides.velocity == 0 && overrides.position == 0);
    // a type with no pool reads zeros
    overrides = {5, 5};
    constraints::get_constraint_iteration_overrides(overrides, scene.constraints, constraint_id::ser(0, constraint_id::USER_1, 0));
    CHECK(ctx, overrides.velocity == 0 && overrides.position == 0);

    constraints::ConstraintSortFields fields;
    constraints::get_constraint_sort_fields(fields, scene.constraints, p2);
    CHECK(ctx, fields.priority == 0 && fields.index == 2);

    // lower priority first (the highest is solved last, so most likely satisfied), then by index;
    // p0 and f0 tie on priority 2 and index 0 and keep their order
    collections::Vec<ConstraintId> ids = {};
    ids.push(f0);
    ids.push(p0);
    ids.push(p2);
    ids.push(d0);
    ids.push(c0);
    ids.push(p1);
    constraints::sort_constraint_ids(scene.constraints, ids);
    CHECK(ctx, ids[0] == c0 && ids[1] == p1 && ids[2] == p2 && ids[3] == d0 && ids[4] == f0 && ids[5] == p0);
    ids.release();
    release_constraint_scene(scene);
}

void test_constraints_sleeping_and_disabled(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *a = add_body(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    RigidBody *c = add_body(scene, motion_type::MOTION_DYNAMIC, {2.0f, 0.0f, 0.0f});
    const ConstraintId ab = point_constraint::create(scene.constraints, scene.bodies, a, b, {});
    const ConstraintId bc = point_constraint::create(scene.constraints, scene.bodies, b, c, {});
    const ConstraintId ac = point_constraint::create(scene.constraints, scene.bodies, a, c, {});

    // sleeping only when both bodies sleep
    a->sleeping = true;
    b->sleeping = true;
    constraints::update_sleeping(scene.constraints, scene.bodies);
    CHECK(ctx, constraints::get(scene.constraints, ab)->sleeping);
    CHECK(ctx, !constraints::get(scene.constraints, bc)->sleeping);
    CHECK(ctx, !constraints::get(scene.constraints, ac)->sleeping);

    // setup skips sleeping and disabled constraints, so their parts stay inactive
    constraints::get(scene.constraints, ac)->enabled = false;
    constraints::setup_velocity_constraints(scene.constraints, scene.bodies, 1.0f / 60.0f);
    CHECK(ctx, !point_constraint_part::is_active(point_constraint::get(scene.constraints, ab)->point_constraint_part));
    CHECK(ctx, point_constraint_part::is_active(point_constraint::get(scene.constraints, bc)->point_constraint_part));
    CHECK(ctx, !point_constraint_part::is_active(point_constraint::get(scene.constraints, ac)->point_constraint_part));

    // warm start likewise: only bc's impulse reaches the bodies
    point_constraint::get(scene.constraints, ab)->point_constraint_part.total_lambda = vec3f::make(1.0f, 0.0f, 0.0f);
    point_constraint::get(scene.constraints, bc)->point_constraint_part.total_lambda = vec3f::make(0.0f, 1.0f, 0.0f);
    collections::Vec<ConstraintId> all = {};
    all.push(ab);
    all.push(bc);
    all.push(ac);
    constraints::warm_start_velocity_constraints(scene.constraints, scene.bodies, all, 1.0f);
    all.release();
    CHECK(ctx, a->motion_properties.linear_velocity.x == 0.0f && a->motion_properties.linear_velocity.y == 0.0f);
    CHECK(ctx, b->motion_properties.linear_velocity.y != 0.0f && b->motion_properties.linear_velocity.x == 0.0f);
    release_constraint_scene(scene);
}

void test_constraints_island_solve(TestCtx &ctx) {
    // a chain of three balls hung from a static pivot by point constraints, solved through the island loops
    ConstraintScene scene = make_constraint_scene(ctx);
    RigidBody *pivot = add_body(scene, motion_type::MOTION_STATIC, {0.0f, 10.0f, 0.0f});
    RigidBody *links[3];
    RigidBody *previous = pivot;
    for (uint32_t i = 0; i < 3; i++) {
        links[i] = add_body(scene, motion_type::MOTION_DYNAMIC, {(float)i + 1.0f, 10.0f, 0.0f});
        const Float3 joint = {(float)i + 0.5f, 10.0f, 0.0f};
        scene.ids.push(point_constraint::create(scene.constraints, scene.bodies, previous, links[i], {.point_a = joint, .point_b = joint}));
        previous = links[i];
    }
    constraints::sort_constraint_ids(scene.constraints, scene.ids);

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 60; i++) step_constraint_scene(scene, dt, vec3f::make(0.0f, -9.81f, 0.0f));

    // every joint holds and the chain has fallen from horizontal
    float worst = 0.0f;
    for (uint32_t i = 0; i < 3; i++) {
        const point_constraint::PointConstraint *joint = point_constraint::get(scene.constraints, scene.ids[i]);
        const RigidBody *body_a = scene.bodies.pool[joint->base.body_index_a];
        const RigidBody *body_b = scene.bodies.pool[joint->base.body_index_b];
        worst = scalar::max(worst, vec3f::distance(world_point(body_a, joint->local_space_position1),
                                                   world_point(body_b, joint->local_space_position2)));
    }
    CHECK(ctx, worst < 2e-2f);
    CHECK(ctx, links[2]->center_of_mass_position.y < 8.0f);

    // an empty island applies nothing
    collections::Vec<ConstraintId> none = {};
    CHECK(ctx, !constraints::solve_velocity_constraints_for_island(scene.constraints, scene.bodies, none, dt));
    CHECK(ctx, !constraints::solve_position_constraints_for_island(scene.constraints, scene.bodies, none, 0.2f, dt));
    release_constraint_scene(scene);
}
