#include "tumble/physics/islands.h"

#include "contact_test_scene.h"
#include "test_math.h"
#include "tumble/physics/body/sleep.h"
#include "tumble/physics/constraints/point_constraint.h"

using rigid_body::RigidBody;

namespace {

/** islands built by hand over the world's contact constraints and user constraints */
struct IslandScene {
    ContactScene contact;
};

IslandScene make_island_scene() { return {make_contact_scene()}; }

void release_island_scene(IslandScene &scene) { release_contact_scene(scene.contact); }

RigidBody *add_sphere(IslandScene &scene, motion_type::MotionType motion_type, const Float3 &position) {
    return add_contact_body(scene.contact, sphere::create({.radius = 0.5f}), motion_type, position);
}

/** a touching contact between two unit spheres side by side along x */
void touch(TestCtx &ctx, IslandScene &scene, RigidBody *a, RigidBody *b) {
    Float3 point;
    vec3f::store(point, vec3f::lerp(vec3f::load(a->position), vec3f::load(b->position), 0.5f));
    Float3 normal;
    vec3f::store(normal, unit(vec3f::subtract(vec3f::load(b->position), vec3f::load(a->position))));
    CHECK(ctx, add_contact(scene.contact, a, b, touching_manifold(&point, 1, normal)));
}

/** the islands of this step's contact constraints and, with `link_user`, the user constraints */
void build(IslandScene &scene, bool link_user) {
    islands::prepare(scene.contact.world->islands, scene.contact.world->bodies, scene.contact.world->contact_constraints.count);
    islands::link_contact_constraints(scene.contact.world->islands, scene.contact.world->contact_constraints, scene.contact.world->contacts, scene.contact.world->bodies);
    if (link_user) islands::link_user_constraints(scene.contact.world->islands, scene.contact.world->constraints, scene.contact.world->bodies);
    islands::finalize(scene.contact.world->islands, scene.contact.world->bodies, scene.contact.world->constraints, scene.contact.world->settings);
}

/** the island holding the body, by its active index; -1 when none does */
int32_t island_of(const IslandScene &scene, const RigidBody *body) {
    for (uint32_t i = 0; i < scene.contact.world->islands.islands.length; i++) {
        const islands::Island &island = scene.contact.world->islands.islands[i];
        for (uint32_t b = 0; b < island.body_indices.length; b++) {
            if (island.body_indices[b] == body->active_index) return (int32_t)i;
        }
    }
    return -1;
}

}  // namespace

void test_islands_link_bodies_unions_to_the_lowest(TestCtx &ctx) {
    IslandScene scene = make_island_scene();
    for (uint32_t i = 0; i < 5; i++) add_sphere(scene, motion_type::MOTION_DYNAMIC, {(float)i * 3.0f, 0.0f, 0.0f});
    islands::prepare(scene.contact.world->islands, scene.contact.world->bodies, 0);
    CHECK(ctx, scene.contact.world->islands.body_links.length == 5);
    for (uint32_t i = 0; i < 5; i++) CHECK(ctx, scene.contact.world->islands.body_links[i] == i);

    islands::link_bodies(scene.contact.world->islands, scene.contact.world->bodies, 3, 4);
    CHECK(ctx, scene.contact.world->islands.body_links[4] == 3);
    islands::link_bodies(scene.contact.world->islands, scene.contact.world->bodies, 2, 1);
    CHECK(ctx, scene.contact.world->islands.body_links[2] == 1);
    // the roots 1 and 3 join under 1, and the bodies passed in are pointed straight at it
    islands::link_bodies(scene.contact.world->islands, scene.contact.world->bodies, 4, 2);
    CHECK(ctx, scene.contact.world->islands.body_links[3] == 1);
    CHECK(ctx, scene.contact.world->islands.body_links[4] == 1 && scene.contact.world->islands.body_links[2] == 1);
    CHECK(ctx, scene.contact.world->islands.body_links[0] == 0);

    // an index past the active bodies (a static body's) links nothing
    islands::link_bodies(scene.contact.world->islands, scene.contact.world->bodies, 0, sleep::INACTIVE_BODY_INDEX);
    CHECK(ctx, scene.contact.world->islands.body_links[0] == 0);

    islands::finalize(scene.contact.world->islands, scene.contact.world->bodies, scene.contact.world->constraints, scene.contact.world->settings);
    CHECK(ctx, scene.contact.world->islands.islands.length == 2);
    CHECK(ctx, scene.contact.world->islands.islands[0].body_indices.length + scene.contact.world->islands.islands[1].body_indices.length == 5);

    release_island_scene(scene);
}

void test_islands_three_touching_bodies_one_island(TestCtx &ctx) {
    IslandScene scene = make_island_scene();
    RigidBody *ground =
        add_contact_body(scene.contact, box::create({.half_extents = {20.0f, 0.5f, 20.0f}}), motion_type::MOTION_STATIC, {0.0f, -0.5f, 0.0f});
    RigidBody *a = add_sphere(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.5f, 0.0f});
    RigidBody *b = add_sphere(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.5f, 0.0f});
    RigidBody *c = add_sphere(scene, motion_type::MOTION_DYNAMIC, {2.0f, 0.5f, 0.0f});
    RigidBody *distant = add_sphere(scene, motion_type::MOTION_DYNAMIC, {10.0f, 0.5f, 0.0f});

    begin_contact_step(scene.contact);
    touch(ctx, scene, a, b);
    touch(ctx, scene, b, c);
    const Float3 under_c = {2.0f, 0.0f, 0.0f};
    CHECK(ctx, add_contact(scene.contact, ground, c, touching_manifold(&under_c, 1, {0.0f, 1.0f, 0.0f})));
    const Float3 under_distant = {10.0f, 0.0f, 0.0f};
    CHECK(ctx, add_contact(scene.contact, ground, distant, touching_manifold(&under_distant, 1, {0.0f, 1.0f, 0.0f})));
    build(scene, false);

    // the static ground joins no island and links nothing: the far sphere is on its own
    CHECK(ctx, ground->active_index == sleep::INACTIVE_BODY_INDEX && ground->island_index == -1);
    CHECK(ctx, scene.contact.world->islands.islands.length == 2);

    // the bigger island first: a, b, c with their two contacts and c's ground contact
    const islands::Island &chain = scene.contact.world->islands.islands[0];
    CHECK(ctx, island_of(scene, a) == 0 && island_of(scene, b) == 0 && island_of(scene, c) == 0);
    CHECK(ctx, chain.body_indices.length == 3 && chain.contact_indices.length == 3);
    CHECK(ctx, a->island_index == b->island_index && b->island_index == c->island_index);

    const islands::Island &alone = scene.contact.world->islands.islands[1];
    CHECK(ctx, island_of(scene, distant) == 1);
    CHECK(ctx, alone.body_indices.length == 1 && alone.contact_indices.length == 1);
    CHECK(ctx, alone.index == 1 && chain.index == 0);
    CHECK(ctx, distant->island_index != a->island_index);

    // each island solves the world's default counts
    CHECK(ctx, chain.num_velocity_steps == scene.contact.world->settings.solver.velocity_iterations);
    CHECK(ctx, chain.num_position_steps == scene.contact.world->settings.solver.position_iterations);

    release_island_scene(scene);
}

void test_islands_sleeping_body_excluded(TestCtx &ctx) {
    IslandScene scene = make_island_scene();
    RigidBody *awake = add_sphere(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.5f, 0.0f});
    RigidBody *asleep = add_sphere(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.5f, 0.0f});
    rigid_body::sleep(*scene.contact.world, asleep);
    CHECK(ctx, asleep->active_index == sleep::INACTIVE_BODY_INDEX && scene.contact.world->bodies.active_body_count == 1);

    begin_contact_step(scene.contact);
    touch(ctx, scene, awake, asleep);
    build(scene, false);

    // the contact goes with the awake body; the sleeping one is in no island
    CHECK(ctx, scene.contact.world->islands.islands.length == 1);
    CHECK(ctx, scene.contact.world->islands.islands[0].body_indices.length == 1 && scene.contact.world->islands.islands[0].contact_indices.length == 1);
    CHECK(ctx, island_of(scene, awake) == 0 && awake->island_index == 0);
    CHECK(ctx, asleep->island_index == -1);
    CHECK(ctx, scene.contact.world->islands.contact_links[0] == awake->active_index);

    release_island_scene(scene);
}

void test_islands_user_constraint_links_two_islands(TestCtx &ctx) {
    IslandScene scene = make_island_scene();
    RigidBody *a = add_sphere(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_sphere(scene, motion_type::MOTION_DYNAMIC, {5.0f, 0.0f, 0.0f});
    RigidBody *anchor = add_sphere(scene, motion_type::MOTION_STATIC, {0.0f, 10.0f, 0.0f});

    begin_contact_step(scene.contact);
    build(scene, false);
    CHECK(ctx, scene.contact.world->islands.islands.length == 2);

    // a joint between them makes one island holding the constraint
    const constraint_id::ConstraintId joint =
        point_constraint::create(*scene.contact.world, a, b, {.point_a = {2.5f, 0.0f, 0.0f}, .point_b = {2.5f, 0.0f, 0.0f}});
    build(scene, true);
    CHECK(ctx, scene.contact.world->islands.islands.length == 1);
    const islands::Island &island = scene.contact.world->islands.islands[0];
    CHECK(ctx, island.body_indices.length == 2);
    CHECK(ctx, island.constraint_ids.length == 1 && island.constraint_ids[0] == joint);
    CHECK(ctx, scene.contact.world->islands.constraint_ids.length == 1 && scene.contact.world->islands.constraint_links[0] == 0);

    // a joint to a static anchor goes with the dynamic body's island and links no static body in
    const constraint_id::ConstraintId hang = point_constraint::create(*scene.contact.world, anchor, b,
                                                                      {.point_a = {5.0f, 10.0f, 0.0f}, .point_b = {5.0f, 10.0f, 0.0f}});
    build(scene, true);
    CHECK(ctx, scene.contact.world->islands.islands.length == 1 && scene.contact.world->islands.islands[0].constraint_ids.length == 2);
    CHECK(ctx, scene.contact.world->islands.islands[0].body_indices.length == 2);
    CHECK(ctx, anchor->island_index == -1);

    // a disabled joint links nothing
    constraints::get(scene.contact.world->constraints, joint)->enabled = false;
    constraints::get(scene.contact.world->constraints, hang)->enabled = false;
    build(scene, true);
    CHECK(ctx, scene.contact.world->islands.islands.length == 2);
    CHECK(ctx, scene.contact.world->islands.constraint_ids.length == 0);

    release_island_scene(scene);
}

void test_islands_iteration_counts(TestCtx &ctx) {
    IslandScene scene = make_island_scene();
    scene.contact.world->settings.solver.velocity_iterations = 10;
    scene.contact.world->settings.solver.position_iterations = 5;

    // a default body touching one asking for 20 / 15, and a body alone asking for 8 / 3
    RigidBody *body_default = add_sphere(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *body_custom = add_sphere(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    body_custom->motion_properties.num_velocity_steps_override = 20;
    body_custom->motion_properties.num_position_steps_override = 15;
    RigidBody *body_other = add_sphere(scene, motion_type::MOTION_DYNAMIC, {10.0f, 0.0f, 0.0f});
    body_other->motion_properties.num_velocity_steps_override = 8;
    body_other->motion_properties.num_position_steps_override = 3;

    begin_contact_step(scene.contact);
    touch(ctx, scene, body_default, body_custom);
    build(scene, true);

    // max(default 10, 20) and max(default 5, 15): the default body asks for the world's counts with 0
    const islands::Island &pair = scene.contact.world->islands.islands[(uint32_t)island_of(scene, body_default)];
    CHECK(ctx, island_of(scene, body_default) == island_of(scene, body_custom));
    CHECK(ctx, pair.num_velocity_steps == 20 && pair.num_position_steps == 15);

    // nothing in the lone island asks for the default, so its own counts stand
    const islands::Island &alone = scene.contact.world->islands.islands[(uint32_t)island_of(scene, body_other)];
    CHECK(ctx, alone.num_velocity_steps == 8 && alone.num_position_steps == 3);

    // a constraint asking for 12 velocity steps and the default position steps raises the lone island's counts
    RigidBody *anchor = add_sphere(scene, motion_type::MOTION_STATIC, {10.0f, 5.0f, 0.0f});
    point_constraint::create(*scene.contact.world, anchor, body_other,
                             {.point_a = {10.0f, 5.0f, 0.0f}, .point_b = {10.0f, 5.0f, 0.0f}, .num_velocity_steps_override = 12});
    build(scene, true);
    const islands::Island &raised = scene.contact.world->islands.islands[(uint32_t)island_of(scene, body_other)];
    CHECK(ctx, raised.num_velocity_steps == 12 && raised.num_position_steps == 5);

    release_island_scene(scene);
}

void test_islands_sorted_largest_first_and_reused(TestCtx &ctx) {
    IslandScene scene = make_island_scene();
    RigidBody *lone = add_sphere(scene, motion_type::MOTION_DYNAMIC, {-10.0f, 0.0f, 0.0f});
    RigidBody *pair_a = add_sphere(scene, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *pair_b = add_sphere(scene, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    RigidBody *row[3];
    for (uint32_t i = 0; i < 3; i++) row[i] = add_sphere(scene, motion_type::MOTION_DYNAMIC, {10.0f + (float)i, 0.0f, 0.0f});

    begin_contact_step(scene.contact);
    touch(ctx, scene, pair_a, pair_b);
    touch(ctx, scene, row[0], row[1]);
    touch(ctx, scene, row[1], row[2]);
    build(scene, false);

    // by contacts plus constraints, largest first: the row (2), the pair (1), the lone body (0)
    CHECK(ctx, scene.contact.world->islands.islands.length == 3);
    CHECK(ctx, island_of(scene, row[0]) == 0 && island_of(scene, pair_a) == 1 && island_of(scene, lone) == 2);
    for (uint32_t i = 0; i < 3; i++) CHECK(ctx, scene.contact.world->islands.islands[i].index == i);

    // each body's island index names the sorted island that lists it, not its island in build order
    const RigidBody *all[] = {lone, pair_a, pair_b, row[0], row[1], row[2]};
    for (const RigidBody *body : all) CHECK(ctx, body->island_index == island_of(scene, body));

    // the next step with fewer islands reuses the records and drops the rest
    begin_contact_step(scene.contact);
    touch(ctx, scene, lone, pair_a);
    touch(ctx, scene, pair_a, pair_b);
    touch(ctx, scene, pair_b, row[0]);
    touch(ctx, scene, row[0], row[1]);
    touch(ctx, scene, row[1], row[2]);
    build(scene, false);
    CHECK(ctx, scene.contact.world->islands.islands.length == 1);
    CHECK(ctx, scene.contact.world->islands.islands[0].body_indices.length == 6 && scene.contact.world->islands.islands[0].contact_indices.length == 5);

    release_island_scene(scene);
}
