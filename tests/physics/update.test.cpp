#include "tumble/physics/update.h"

#include "tumble/physics/body/sleep.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"
#include "world_test_scene.h"
#include "test_math.h"

using rigid_body::RigidBody;

namespace {

/** bitwise equality, so -0 and 0 or two NaNs are told apart */
bool bits_equal(const void *a, const void *b, uint32_t bytes) {
    const uint8_t *bytes_a = static_cast<const uint8_t *>(a);
    const uint8_t *bytes_b = static_cast<const uint8_t *>(b);
    for (uint32_t i = 0; i < bytes; i++) {
        if (bytes_a[i] != bytes_b[i]) return false;
    }
    return true;
}

/** the island holding the body's active index, or null */
const islands::Island *island_of(const world::World &world, const RigidBody &body) {
    if (body.active_index == sleep::INACTIVE_BODY_INDEX) return nullptr;
    for (uint32_t i = 0; i < world.islands.islands.length; i++) {
        const islands::Island &island = world.islands.islands[i];
        for (uint32_t j = 0; j < island.body_indices.length; j++) {
            if (island.body_indices[j] == body.active_index) return &island;
        }
    }
    return nullptr;
}

RigidBody *add_floor(WorldScene &scene, float half_extent = 10.0f) {
    return add_body(scene, box::create({.half_extents = {half_extent, 0.5f, half_extent}}), motion_type::MOTION_STATIC, {0.0f, -0.5f, 0.0f});
}

RigidBody *add_unit_box(WorldScene &scene, const Float3 &position) {
    return add_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), motion_type::MOTION_DYNAMIC, position);
}

/** a mesh shape over flat xyz positions and triangle indices */
shapes::Shape *make_mesh(const float *positions, uint32_t num_positions, const uint32_t *indices, uint32_t num_indices) {
    triangle_mesh_builder::TriangleMeshBuilderSettings settings;
    settings.positions = positions;
    settings.num_positions = num_positions;
    settings.indices = indices;
    settings.num_indices = num_indices;
    return triangle_mesh::create({.data = triangle_mesh_builder::build(settings)});
}

/** the step with the caller rejecting every pair between find_pairs and the narrowphase */
void step_rejecting_pairs(world::World &world) {
    update::step_begin(world, WORLD_TEST_DELTA_TIME);
    update::find_pairs(world);
    for (uint32_t i = 0; i < world.answers.pair_reject.length; i++) world.answers.pair_reject[i] = ~0u;
    update::narrowphase(world);
    update::solve(world);
    update::step_end(world);
}

/** the step with the caller answering every contact with `answer` between the narrowphase and the solve */
void step_answering_contacts(world::World &world, void (*answer)(events::ContactOverride &out, const events::ContactEvent &event)) {
    update::step_begin(world, WORLD_TEST_DELTA_TIME);
    update::find_pairs(world);
    update::narrowphase(world);
    for (uint32_t i = 0; i < world.answers.contact_override.length; i++) {
        answer(world.answers.contact_override[i], events::contact_event(world.events, i));
    }
    update::solve(world);
    update::step_end(world);
}

}  // namespace


void test_update_position_and_aabb_follow_center_of_mass(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *body = add_body(scene, sphere::create({.radius = 0.5f}), motion_type::MOTION_DYNAMIC, {1.0f, 10.0f, 2.0f});
    step_world(*scene.world);
    CHECK(ctx, body->center_of_mass_position.y < 10.0f);
    CHECK_NEAR(ctx, body->position.y, body->center_of_mass_position.y, 1e-6f);
    CHECK_NEAR(ctx, body->aabb.min.y, body->position.y - 0.5f, 1e-5f);
    CHECK_NEAR(ctx, body->aabb.max.y, body->position.y + 0.5f, 1e-5f);
    // one step of gravity from rest: v = g dt (damped), x = v dt
    CHECK_NEAR(ctx, body->motion_properties.linear_velocity.y, -9.81f * WORLD_TEST_DELTA_TIME * (1.0f - 0.05f * WORLD_TEST_DELTA_TIME), 1e-5f);
    CHECK_NEAR(ctx, body->position.y, 10.0f + body->motion_properties.linear_velocity.y * WORLD_TEST_DELTA_TIME, 1e-5f);
    release_world_scene(scene);
}

void test_update_forces_consumed_by_step_and_zero_step(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *body = add_unit_box(scene, {0.0f, 10.0f, 0.0f});
    rigid_body::add_force(*scene.world, body, vec3f::make(3.0f, 0.0f, 0.0f), true);
    step_world(*scene.world);
    CHECK(ctx, body->motion_properties.force.x == 0.0f && body->motion_properties.force.y == 0.0f && body->motion_properties.force.z == 0.0f);
    // a = F/m over one step (the unit box of density 1000 has m = 1000)
    CHECK_NEAR(ctx, body->motion_properties.linear_velocity.x, 3.0f / 1000.0f * WORLD_TEST_DELTA_TIME * (1.0f - 0.05f * WORLD_TEST_DELTA_TIME),
               1e-9f);

    rigid_body::add_force(*scene.world, body, vec3f::make(3.0f, 0.0f, 0.0f), true);
    const Float3 position = body->position;
    step_world(*scene.world, 0.0f);
    CHECK(ctx, body->motion_properties.force.x == 0.0f);
    CHECK(ctx, body->position.x == position.x && body->position.y == position.y);
    release_world_scene(scene);
}


void test_update_islands_group_contacts_and_exclude_static(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    shapes::Shape *shape = sphere::create({.radius = 0.5f});
    RigidBody *body_1 = add_body(scene, shape, motion_type::MOTION_DYNAMIC, {0.0f, 0.5f, 0.0f});
    RigidBody *body_2 = add_body(scene, shape, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *static_body = add_body(scene, shape, motion_type::MOTION_STATIC, {5.0f, 0.0f, 0.0f});
    RigidBody *body_3 = add_body(scene, shape, motion_type::MOTION_DYNAMIC, {10.0f, 0.0f, 0.0f});

    step_world(*scene.world);

    CHECK(ctx, static_body->active_index == sleep::INACTIVE_BODY_INDEX);
    CHECK(ctx, static_body->island_index == -1);
    CHECK(ctx, body_1->island_index >= 0 && body_1->island_index == body_2->island_index);
    CHECK(ctx, body_3->island_index >= 0 && body_3->island_index != body_1->island_index);
    CHECK(ctx, scene.world->islands.islands.length == 2);
    CHECK(ctx, island_of(*scene.world, *body_1) == island_of(*scene.world, *body_2));
    CHECK(ctx, island_of(*scene.world, *body_1)->contact_indices.length == 1);
    CHECK(ctx, island_of(*scene.world, *body_3)->contact_indices.length == 0);
    for (uint32_t i = 0; i < scene.world->bodies.active_body_count; i++) {
        CHECK(ctx, scene.world->bodies.active_body_indices[i] != static_body->index);
    }
    release_world_scene(scene);
}

void test_update_island_iterations_from_overrides(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    scene.world->settings.solver.velocity_iterations = 10;
    scene.world->settings.solver.position_iterations = 5;
    shapes::Shape *shape = sphere::create({.radius = 0.5f});
    RigidBody *body_default = add_body(scene, shape, motion_type::MOTION_DYNAMIC, {0.0f, 0.6f, 0.0f});
    RigidBody *body_custom = add_body(scene, shape, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    body_custom->motion_properties.num_velocity_steps_override = 20;
    body_custom->motion_properties.num_position_steps_override = 15;
    RigidBody *body_other = add_body(scene, shape, motion_type::MOTION_DYNAMIC, {10.0f, 0.0f, 0.0f});
    body_other->motion_properties.num_velocity_steps_override = 8;
    body_other->motion_properties.num_position_steps_override = 3;

    step_world(*scene.world);

    const islands::Island *island_1 = island_of(*scene.world, *body_default);
    CHECK(ctx, island_1 != nullptr && island_1 == island_of(*scene.world, *body_custom));
    // a default body asks for the world's 10 and 5, the custom one for 20 and 15: the max
    CHECK(ctx, island_1->num_velocity_steps == 20);
    CHECK(ctx, island_1->num_position_steps == 15);
    // alone, with no body asking for the default
    const islands::Island *island_2 = island_of(*scene.world, *body_other);
    CHECK(ctx, island_2 != nullptr && island_2 != island_1);
    CHECK(ctx, island_2->num_velocity_steps == 8);
    CHECK(ctx, island_2->num_position_steps == 3);
    release_world_scene(scene);
}

void test_update_island_index_cleared_on_sleep_and_reassigned_on_wake(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *ground = add_floor(scene);
    RigidBody *resting = add_unit_box(scene, {0.0f, 0.5f, 0.0f});

    step_world(*scene.world);
    CHECK(ctx, resting->island_index >= 0);
    CHECK(ctx, ground->island_index == -1);

    for (uint32_t i = 0; i < 240 && !resting->sleeping; i++) step_world(*scene.world);
    CHECK(ctx, resting->sleeping);
    CHECK(ctx, resting->island_index == -1);

    // a ball dropped on it wakes it
    RigidBody *ball = add_body(scene, sphere::create({.radius = 0.25f}), motion_type::MOTION_DYNAMIC, {0.0f, 1.3f, 0.0f});
    rigid_body::set_linear_velocity(*scene.world, ball, vec3f::make(0.0f, -5.0f, 0.0f));
    for (uint32_t i = 0; i < 30 && resting->sleeping; i++) step_world(*scene.world);
    CHECK(ctx, !resting->sleeping);
    CHECK(ctx, resting->island_index >= 0);
    CHECK(ctx, resting->island_index == ball->island_index);
    CHECK(ctx, ground->island_index == -1);
    release_world_scene(scene);
}


void test_update_sleeping_body_woken_by_collision(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *sleeping_body = add_unit_box(scene, {0.0f, 0.5f, 0.0f});
    rigid_body::sleep(*scene.world, sleeping_body);
    RigidBody *active_body = add_unit_box(scene, {0.0f, 1.5f, 0.0f});
    active_body->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    CHECK(ctx, sleeping_body->sleeping && !active_body->sleeping);

    step_world(*scene.world);
    CHECK(ctx, !sleeping_body->sleeping);
    release_world_scene(scene);
}

void test_update_sleeping_body_on_floor_woken_by_falling_body(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene);
    RigidBody *sleeping_body = add_unit_box(scene, {0.0f, 0.5f, 0.0f});
    rigid_body::sleep(*scene.world, sleeping_body);
    RigidBody *falling_body = add_unit_box(scene, {0.0f, 2.0f, 0.0f});
    falling_body->motion_properties.linear_velocity = {0.0f, -5.0f, 0.0f};

    for (uint32_t i = 0; i < 30 && sleeping_body->sleeping; i++) step_world(*scene.world);
    CHECK(ctx, !sleeping_body->sleeping);
    release_world_scene(scene);
}

void test_update_falling_body_lands_on_sleeping_body(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene);
    RigidBody *sleeping_body = add_body(scene, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}), motion_type::MOTION_DYNAMIC, {0.0f, 1.0f, 0.0f});
    rigid_body::sleep(*scene.world, sleeping_body);
    RigidBody *falling_body = add_unit_box(scene, {0.0f, 3.0f, 0.0f});

    for (uint32_t i = 0; i < 120; i++) step_world(*scene.world);

    // resting on the sleeping body's top at y = 2, which stayed on the floor
    CHECK(ctx, falling_body->position.y > 2.0f);
    CHECK_NEAR(ctx, falling_body->position.y, 2.5f, 0.03f);
    CHECK_NEAR(ctx, sleeping_body->position.y, 1.0f, 0.03f);
    release_world_scene(scene);
}

namespace {

/** fifty capsules in a 10 x 5 grid 2 m apart on a floor */
void add_capsule_grid(WorldScene &scene, RigidBody **out, float half_height, float radius, float spacing, float y,
                      const Float3 &grid_offset) {
    for (uint32_t i = 0; i < 50; i++) {
        rigid_body::RigidBodySettings settings =
            body_settings(capsule::create({.half_height_of_cylinder = half_height, .radius = radius}), scene.moving, motion_type::MOTION_DYNAMIC,
                          {(float)(i % 10) * spacing + grid_offset.x, y, (float)(i / 10) * spacing + grid_offset.z});
        settings.restitution = 0.0f;
        out[i] = rigid_body::create(*scene.world, settings);
    }
}

uint32_t count_sleeping(RigidBody *const *bodies, uint32_t count) {
    uint32_t sleeping = 0;
    for (uint32_t i = 0; i < count; i++) sleeping += bodies[i]->sleeping ? 1 : 0;
    return sleeping;
}

}  // namespace

void test_update_capsule_falls_onto_sleeping_capsule(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene, 20.0f);
    RigidBody *capsules[50];
    add_capsule_grid(scene, capsules, 0.5f, 0.3f, 2.0f, 0.8f, {-9.0f, 0.0f, -4.0f});
    for (uint32_t i = 0; i < 300; i++) step_world(*scene.world);
    CHECK(ctx, count_sleeping(capsules, 50) > 40);

    RigidBody *target = nullptr;
    for (uint32_t i = 0; i < 50 && target == nullptr; i++) {
        if (capsules[i]->sleeping && scalar::abs(capsules[i]->position.x) < 2.0f && scalar::abs(capsules[i]->position.z) < 2.0f) {
            target = capsules[i];
        }
    }
    CHECK(ctx, target != nullptr);
    if (target == nullptr) return;

    RigidBody *falling = add_body(scene, capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f}), motion_type::MOTION_DYNAMIC,
                                  {target->position.x, target->position.y + 3.0f, target->position.z});
    for (uint32_t i = 0; i < 120; i++) step_world(*scene.world);
    CHECK(ctx, falling->position.y > 1.0f);
    release_world_scene(scene);
}

void test_update_capsule_falls_onto_sleeping_capsule_after_respawns(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene, 20.0f);
    RigidBody *capsules[50];
    add_capsule_grid(scene, capsules, 0.5f, 0.3f, 2.0f, 0.8f, {-9.0f, 0.0f, -4.0f});
    for (uint32_t i = 0; i < 300; i++) step_world(*scene.world);

    // every fifth body removed and spawned again high up, reusing the slots
    for (uint32_t r = 0; r < 10; r++) {
        const uint32_t index = r * 5;
        CHECK(ctx, rigid_body::remove(*scene.world, capsules[index]));
        capsules[index] =
            add_body(scene, capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f}), motion_type::MOTION_DYNAMIC,
                     {(float)(index % 10) * 2.0f - 9.0f, 5.0f, (float)(index / 10) * 2.0f - 4.0f});
    }
    for (uint32_t i = 0; i < 300; i++) step_world(*scene.world);

    RigidBody *target = nullptr;
    for (uint32_t i = 0; i < 50 && target == nullptr; i++) {
        if (capsules[i]->sleeping) target = capsules[i];
    }
    CHECK(ctx, target != nullptr);
    if (target == nullptr) return;

    RigidBody *falling = add_body(scene, capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f}), motion_type::MOTION_DYNAMIC,
                                  {target->position.x, target->position.y + 3.0f, target->position.z});
    for (uint32_t i = 0; i < 120; i++) step_world(*scene.world);
    CHECK(ctx, falling->position.y > 1.0f);
    release_world_scene(scene);
}

void test_update_large_capsule_falls_onto_sleeping_capsules(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene, 50.0f);
    RigidBody *capsules[50];
    for (uint32_t i = 0; i < 50; i++) {
        rigid_body::RigidBodySettings settings =
            body_settings(capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f}), scene.moving, motion_type::MOTION_DYNAMIC,
                          {(float)(i % 10) * 5.0f - 22.5f, 3.0f, (float)(i / 10) * 5.0f - 10.0f});
        settings.restitution = 0.0f;
        settings.friction = 0.5f;
        capsules[i] = rigid_body::create(*scene.world, settings);
    }
    for (uint32_t i = 0; i < 400; i++) step_world(*scene.world);
    CHECK(ctx, count_sleeping(capsules, 50) > 40);

    RigidBody *target = nullptr;
    for (uint32_t i = 0; i < 50 && target == nullptr; i++) {
        if (capsules[i]->sleeping) target = capsules[i];
    }
    CHECK(ctx, target != nullptr);
    if (target == nullptr) return;

    rigid_body::RigidBodySettings settings =
        body_settings(capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f}), scene.moving, motion_type::MOTION_DYNAMIC,
                      {target->position.x, 25.0f, target->position.z});
    settings.restitution = 0.0f;
    settings.friction = 0.5f;
    RigidBody *falling = rigid_body::create(*scene.world, settings);
    for (uint32_t i = 0; i < 180; i++) step_world(*scene.world);
    // stacked capsule centres are at least radius + radius apart
    CHECK(ctx, falling->position.y > 3.0f);
    release_world_scene(scene);
}

void test_update_body_at_rest_sleeps_and_impulse_wakes(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    scene.world->settings.gravity_enabled = false;
    RigidBody *body = add_body(scene, sphere::create({.radius = 1.0f}), motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    CHECK(ctx, !body->sleeping);

    // time_before_sleep (0.5 s) and a margin
    const uint32_t steps = (uint32_t)__builtin_ceilf((scene.world->settings.sleeping.time_before_sleep + 0.1f) * 60.0f);
    for (uint32_t i = 0; i < steps; i++) step_world(*scene.world);
    CHECK(ctx, body->sleeping);

    rigid_body::add_impulse(*scene.world, body, vec3f::make(0.0f, 5.0f, 0.0f));
    step_world(*scene.world);
    CHECK(ctx, !body->sleeping);
    CHECK(ctx, body->position.y > 0.0f);
    release_world_scene(scene);
}


void test_update_contacts_removed_once_on_sleep_and_added_on_wake(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *floor = add_floor(scene);
    RigidBody *box_body = add_unit_box(scene, {0.0f, 0.5f, 0.0f});

    uint32_t steps = 0;
    for (; steps < 600 && !box_body->sleeping; steps++) step_world(*scene.world);
    CHECK(ctx, box_body->sleeping);
    // asleep, but the sweep that destroys the chain runs on the next step
    const uint32_t contacts_at_sleep = pair_contact_count(*scene.world, *box_body, *floor);
    CHECK(ctx, contacts_at_sleep > 0);

    // the first sleeping step: both bodies inactive, so the chain goes, each contact reported once
    step_world(*scene.world);
    CHECK(ctx, scene.world->events.contacts_removed.length == contacts_at_sleep);
    CHECK(ctx, box_body->sleeping);
    CHECK(ctx, pair_contact_count(*scene.world, *box_body, *floor) == 0);

    // steady state: nothing more
    uint32_t added = 0;
    uint32_t removed = 0;
    for (uint32_t i = 0; i < 15; i++) {
        step_world(*scene.world);
        added += scene.world->events.contacts_added.length;
        removed += scene.world->events.contacts_removed.length;
    }
    CHECK(ctx, added == 0 && removed == 0);

    // wake: a clean narrowphase re-adds, not a cache hit into zero contacts
    const float y_before = box_body->position.y;
    rigid_body::wake(*scene.world, box_body);
    step_world(*scene.world);
    CHECK(ctx, !box_body->sleeping);
    CHECK(ctx, scene.world->events.contacts_added.length > 0);
    CHECK(ctx, pair_contact_count(*scene.world, *box_body, *floor) > 0);
    CHECK_NEAR(ctx, box_body->position.y, y_before, 0.05f);
    release_world_scene(scene);
}


namespace {

/** the points of this step's added contacts, on the given body and on the other one */
uint32_t added_contact_points(const world::World &world, const RigidBody &body, Float3 *out_on_body, Float3 *out_on_other, uint32_t max_points) {
    uint32_t count = 0;
    for (uint32_t i = 0; i < world.events.contacts_added.length; i++) {
        const events::ContactEvent &event = world.events.contacts_added[i];
        const bool body_is_a = event.body_index_a == body.index;
        for (uint32_t j = 0; j < event.num_points && count < max_points; j++, count++) {
            out_on_body[count] = body_is_a ? event.points_a[j] : event.points_b[j];
            out_on_other[count] = body_is_a ? event.points_b[j] : event.points_a[j];
        }
    }
    return count;
}

}  // namespace

void test_update_capsule_lying_on_box_gets_two_points(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // a 2 m static box with its top at y = 1, and a capsule (radius 0.5, cylinder half height 0.5) lying along x with its
    // bottom line 1 cm into the box's top
    RigidBody *box_body = add_body(scene, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    rigid_body::RigidBodySettings settings = body_settings(capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.5f}), scene.moving,
                                                           motion_type::MOTION_DYNAMIC, {0.0f, 1.49f, 0.0f});
    settings.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI / 2.0f);
    rigid_body::create(*scene.world, settings);

    step_world(*scene.world);

    Float3 on_box[8];
    Float3 on_capsule[8];
    const uint32_t count = added_contact_points(*scene.world, *box_body, on_box, on_capsule, 8);
    CHECK(ctx, count == 2);
    if (count != 2) return;
    const float min_x = scalar::min(on_box[0].x, on_box[1].x);
    const float max_x = scalar::max(on_box[0].x, on_box[1].x);
    CHECK_NEAR(ctx, min_x, -0.5f, 1e-3f);
    CHECK_NEAR(ctx, max_x, 0.5f, 1e-3f);
    for (uint32_t i = 0; i < 2; i++) {
        CHECK_NEAR(ctx, on_box[i].y, 1.0f, 1e-3f);
        CHECK_NEAR(ctx, on_box[i].z, 0.0f, 1e-3f);
        CHECK_NEAR(ctx, on_capsule[i].x, on_box[i].x, 1e-3f);
        CHECK(ctx, on_capsule[i].y < 1.0f && on_capsule[i].y > 0.98f);
    }
    release_world_scene(scene);
}

void test_update_box_resting_along_capsule_gets_two_points(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // a static capsule lying along x with its top line at y = 0.5, and a dynamic box whose bottom face sits 1 cm into it
    rigid_body::RigidBodySettings settings = body_settings(capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.5f}),
                                                           scene.not_moving, motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    settings.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI / 2.0f);
    RigidBody *capsule_body = rigid_body::create(*scene.world, settings);
    add_body(scene, box::create({.half_extents = {1.0f, 0.5f, 0.5f}}), motion_type::MOTION_DYNAMIC, {0.0f, 0.99f, 0.0f});

    step_world(*scene.world);

    Float3 on_capsule[8];
    Float3 on_box[8];
    const uint32_t count = added_contact_points(*scene.world, *capsule_body, on_capsule, on_box, 8);
    CHECK(ctx, count == 2);
    if (count != 2) return;
    CHECK_NEAR(ctx, scalar::min(on_capsule[0].x, on_capsule[1].x), -0.5f, 1e-3f);
    CHECK_NEAR(ctx, scalar::max(on_capsule[0].x, on_capsule[1].x), 0.5f, 1e-3f);
    for (uint32_t i = 0; i < 2; i++) {
        CHECK_NEAR(ctx, on_capsule[i].y, 0.5f, 1e-3f);
        CHECK_NEAR(ctx, on_capsule[i].z, 0.0f, 1e-3f);
        CHECK_NEAR(ctx, on_box[i].x, on_capsule[i].x, 1e-3f);
        CHECK(ctx, on_box[i].y < 0.5f && on_box[i].y > 0.48f);
    }
    release_world_scene(scene);
}

void test_update_box_on_tessellated_mesh_prunes_to_quad(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // a 2 m square of 0.1 m cells, 800 triangles: the box touches many, each adding its clipped points to one manifold
    // that is pruned past 32 points before the normals are averaged
    constexpr uint32_t CELLS = 20;
    float positions[(CELLS + 1) * (CELLS + 1) * 3];
    uint32_t indices[CELLS * CELLS * 6];
    uint32_t p = 0;
    for (uint32_t z = 0; z <= CELLS; z++) {
        for (uint32_t x = 0; x <= CELLS; x++) {
            positions[p++] = -1.0f + (float)x * 0.1f;
            positions[p++] = 0.0f;
            positions[p++] = -1.0f + (float)z * 0.1f;
        }
    }
    uint32_t n = 0;
    for (uint32_t z = 0; z < CELLS; z++) {
        for (uint32_t x = 0; x < CELLS; x++) {
            const uint32_t i = z * (CELLS + 1) + x;
            const uint32_t quad[6] = {i, i + CELLS + 1, i + 1, i + 1, i + CELLS + 1, i + CELLS + 2};
            for (uint32_t k = 0; k < 6; k++) indices[n++] = quad[k];
        }
    }
    add_body(scene, make_mesh(positions, (CELLS + 1) * (CELLS + 1), indices, CELLS * CELLS * 6), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    RigidBody *box_body = add_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), motion_type::MOTION_DYNAMIC, {0.0f, 0.49f, 0.0f});

    uint32_t max_points = 0;
    for (uint32_t i = 0; i < 30; i++) {
        step_world(*scene.world);
        for (uint32_t j = 0; j < scene.world->events.contacts_added.length; j++) {
            if (scene.world->events.contacts_added[j].num_points > max_points) max_points = scene.world->events.contacts_added[j].num_points;
        }
    }
    CHECK(ctx, max_points == 4);
    CHECK_NEAR(ctx, box_body->position.y, 0.5f, 0.05f);
    CHECK(ctx, scalar::abs(box_body->position.x) < 0.01f);
    CHECK(ctx, scalar::abs(box_body->position.z) < 0.01f);
    release_world_scene(scene);
}

// the answers the caller writes between the phases

namespace {

void reduction_off(world_settings::WorldSettings &settings) { settings.narrowphase.use_manifold_reduction = false; }

/** a sphere sunk 0.5 into a 4 x 4 two-triangle floor over its diagonal, so it overlaps both triangles */
RigidBody *add_sunk_sphere_over_floor(WorldScene &scene) {
    static const float FLOOR_POSITIONS[] = {-2.0f, 0.0f, -2.0f, 2.0f, 0.0f, -2.0f, 2.0f, 0.0f, 2.0f, -2.0f, 0.0f, 2.0f};
    static const uint32_t FLOOR_INDICES[] = {0, 2, 1, 0, 3, 2};
    add_body(scene, make_mesh(FLOOR_POSITIONS, 4, FLOOR_INDICES, 6), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    return add_body(scene, sphere::create({.radius = 1.0f}), motion_type::MOTION_DYNAMIC, {0.0f, 0.5f, 0.0f});
}

void check_rejected_pair_never_contacts(TestCtx &ctx, WorldScene &scene) {
    RigidBody *ball = add_sunk_sphere_over_floor(scene);

    // the pair is found, rejected, and never reaches the narrowphase
    update::step_begin(*scene.world, WORLD_TEST_DELTA_TIME);
    update::find_pairs(*scene.world);
    CHECK(ctx, scene.world->events.pairs.length == 1);
    CHECK(ctx, scene.world->answers.pair_reject.length == 1 && scene.world->answers.pair_reject[0] == 0);
    events::reject_pair(scene.world->answers, 0);
    update::narrowphase(*scene.world);
    CHECK(ctx, scene.world->events.contacts_added.length == 0);
    CHECK(ctx, scene.world->contact_constraints.count == 0);
    CHECK(ctx, scene.world->answers.contact_override.length == 0);
    update::solve(*scene.world);
    update::step_end(*scene.world);

    // rejected every step, the ball falls through the floor as if it were not there
    float previous_y = ball->position.y;
    for (uint32_t i = 0; i < 60; i++) {
        step_rejecting_pairs(*scene.world);
        CHECK(ctx, scene.world->events.contacts_added.length == 0 && scene.world->events.contacts_persisted.length == 0);
        CHECK(ctx, ball->position.y < previous_y);
        previous_y = ball->position.y;
    }
    CHECK(ctx, body_contact_count(*scene.world, *ball) == 0);
    CHECK(ctx, ball->position.y < -3.0f);
}

}  // namespace

void test_update_pair_reject_with_manifold_reduction(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    check_rejected_pair_never_contacts(ctx, scene);
    release_world_scene(scene);
}

void test_update_pair_reject_without_manifold_reduction(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx, reduction_off);
    check_rejected_pair_never_contacts(ctx, scene);
    release_world_scene(scene);
}

void test_update_pair_reject_drops_existing_contacts(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *floor = add_floor(scene);
    RigidBody *box_body = add_unit_box(scene, {0.0f, 0.5f, 0.0f});
    step_world(*scene.world);
    const uint32_t contacts = pair_contact_count(*scene.world, *floor, *box_body);
    CHECK(ctx, contacts > 0);

    // rejecting a pair with contacts reports them removed in the narrowphase
    step_rejecting_pairs(*scene.world);
    CHECK(ctx, scene.world->events.contacts_removed.length == contacts);
    CHECK(ctx, pair_contact_count(*scene.world, *floor, *box_body) == 0);

    // accepted again, the contacts come back as added
    step_world(*scene.world);
    CHECK(ctx, scene.world->events.contacts_added.length == contacts);
    release_world_scene(scene);
}

namespace {

void reject_contact(events::ContactOverride &out, const events::ContactEvent &) {
    out.written = true;
    out.reject = true;
}

void conveyor_contact(events::ContactOverride &out, const events::ContactEvent &event) {
    out.written = true;
    out.reject = false;
    out.settings = event.settings;
    out.settings.combined_friction = 1.0f;
    // body 2's surface relative to body 1's: the floor (made first, so body a) moving +x at 2 m/s gives -2
    out.settings.relative_linear_surface_velocity = {-2.0f, 0.0f, 0.0f};
}

}  // namespace

void test_update_contact_override_reject_falls_through_floor(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene);
    RigidBody *box_body = add_unit_box(scene, {0.0f, 0.5f, 0.0f});
    RigidBody *control = add_unit_box(scene, {5.0f, 0.5f, 0.0f});

    // the boxes rest on the floor until the caller rejects the first one's contacts
    step_world(*scene.world);
    CHECK_NEAR(ctx, box_body->position.y, 0.5f, 0.01f);

    for (uint32_t i = 0; i < 60; i++) {
        step_answering_contacts(*scene.world, [](events::ContactOverride &out, const events::ContactEvent &event) {
            // only the first box's contacts (body index 1; the floor is 0, the control 2)
            if (event.body_index_a == 1 || event.body_index_b == 1) reject_contact(out, event);
        });
    }
    // fell freely through the floor, while the control box still rests on it
    CHECK(ctx, box_body->position.y < -3.0f);
    CHECK_NEAR(ctx, control->position.y, 0.5f, 0.02f);
    release_world_scene(scene);
}

void test_update_contact_override_conveyor_moves_resting_box(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene);
    RigidBody *box_body = add_unit_box(scene, {0.0f, 0.5f, 0.0f});

    // settled first, still
    for (uint32_t i = 0; i < 10; i++) step_world(*scene.world);
    CHECK(ctx, scalar::abs(box_body->motion_properties.linear_velocity.x) < 1e-3f);

    for (uint32_t i = 0; i < 60; i++) step_answering_contacts(*scene.world, conveyor_contact);
    // carried along with the belt's surface, still resting on it
    CHECK_NEAR(ctx, box_body->motion_properties.linear_velocity.x, 2.0f, 0.02f);
    CHECK(ctx, box_body->position.x > 1.0f);
    CHECK_NEAR(ctx, box_body->position.y, 0.5f, 0.02f);
    CHECK(ctx, scalar::abs(box_body->position.z) < 1e-3f);
    release_world_scene(scene);
}

void test_update_answers_sized_per_phase_and_cleared_at_begin(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene);
    add_unit_box(scene, {0.0f, 0.5f, 0.0f});
    add_unit_box(scene, {3.0f, 0.5f, 0.0f});
    step_world(*scene.world);

    world::World &world = *scene.world;
    update::step_begin(world, WORLD_TEST_DELTA_TIME);
    CHECK(ctx, world.events.pairs.length == 0 && world.events.contacts_added.length == 0 && world.events.contacts_persisted.length == 0);
    CHECK(ctx, world.events.moved.length == 0 && world.answers.pair_reject.length == 0 && world.answers.contact_override.length == 0);
    update::find_pairs(world);
    CHECK(ctx, world.events.pairs.length == 2);
    CHECK(ctx, world.answers.pair_reject.length == 1 && world.answers.pair_reject[0] == 0);
    update::narrowphase(world);
    CHECK(ctx, world.events.contacts_persisted.length == 2);
    CHECK(ctx, world.answers.contact_override.length == 2);
    CHECK(ctx, !world.answers.contact_override[0].written && !world.answers.contact_override[1].written);
    // each event names the step's bodies, lower id first
    for (uint32_t i = 0; i < world.events.contacts_persisted.length; i++) {
        const events::ContactEvent &event = world.events.contacts_persisted[i];
        CHECK(ctx, event.body_id_a < event.body_id_b);
        CHECK_NEAR(ctx, event.normal.y, 1.0f, 1e-4f);
    }
    update::solve(world);
    update::step_end(world);
    release_world_scene(scene);
}

// the step as a whole

void test_update_stack_of_ten_stands(TestCtx &ctx) {
    // a stack of ten unit boxes, built after k bodies came and went so the ids (and with them the solve order) differ:
    // every one stands, each box on the one below, sunk at most the slop per contact, none sliding off. an exactly
    // aligned stack takes a sideways kick from the first step's sequential impulses and in some orders creeps by up to
    // tens of cm over 15 s, never sleeping, as a solver of this kind does on these layouts: the ids and so the solve order are
    // these, it tracks this solver to 1e-5 for the first 30 to 300 steps, and its layout 6 never sleeps either
    for (uint32_t k = 0; k < 8; k++) {
        WorldScene scene = make_world_scene(ctx);
        add_floor(scene);
        RigidBody *scratch[8];
        for (uint32_t j = 0; j < k; j++) scratch[j] = add_unit_box(scene, {50.0f + (float)j * 3.0f, 0.5f, 0.0f});
        for (uint32_t j = 0; j < k; j++) rigid_body::remove(*scene.world, scratch[j]);
        RigidBody *stack[10];
        for (uint32_t i = 0; i < 10; i++) stack[i] = add_unit_box(scene, {0.0f, 0.5f + (float)i, 0.0f});
        for (uint32_t step = 0; step < 900 && count_sleeping(stack, 10) < 10; step++) step_world(*scene.world);
        for (uint32_t i = 0; i < 10; i++) {
            CHECK_NEAR(ctx, stack[i]->position.y, 0.5f + (float)i, 0.02f * (float)(i + 1));
            CHECK(ctx, scalar::abs(stack[i]->position.x) < 0.5f && scalar::abs(stack[i]->position.z) < 0.5f);
        }
        release_world_scene(scene);
    }
}

namespace {

/** a pile of boxes, spheres and capsules dropped onto a floor at slight offsets */
WorldScene build_pile(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene);
    for (uint32_t i = 0; i < 12; i++) {
        const float offset_x = 0.07f * (float)(i % 3) - 0.05f;
        const float offset_z = 0.03f * (float)(i % 4) - 0.04f;
        const Float3 position = {offset_x, 0.6f + 1.1f * (float)i, offset_z};
        shapes::Shape *shape;
        if (i % 3 == 0) shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
        else if (i % 3 == 1) shape = sphere::create({.radius = 0.45f});
        else shape = capsule::create({.half_height_of_cylinder = 0.3f, .radius = 0.3f});
        rigid_body::RigidBodySettings settings = body_settings(shape, scene.moving, motion_type::MOTION_DYNAMIC, position);
        settings.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.1f * (float)i);
        rigid_body::create(*scene.world, settings);
    }
    return scene;
}

}  // namespace

// warm start across a change of step length

void test_update_landed_box_not_launched_by_a_longer_step(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene);
    add_unit_box(scene, {0.0f, 0.5f, 0.0f});
    RigidBody *top = add_unit_box(scene, {0.0f, 3.0f, 0.0f});

    // short steps until the step that stops the falling box: its contact now holds the large stopping impulse
    const float short_step = 1.0f / 240.0f;
    float previous_velocity = 0.0f;
    for (uint32_t i = 0; i < 400; i++) {
        step_world(*scene.world, short_step);
        const float velocity = top->motion_properties.linear_velocity.y;
        if (previous_velocity < -2.0f && scalar::abs(velocity) < 0.5f) break;
        previous_velocity = velocity;
    }
    CHECK_NEAR(ctx, top->position.y, 1.5f, 0.05f);

    // 32 times longer steps: the warm start scales by at most 4, which the solver takes back, so the box stays put
    float highest = top->position.y;
    for (uint32_t i = 0; i < 30; i++) {
        step_world(*scene.world, short_step * 32.0f);
        highest = scalar::max(highest, top->position.y);
    }
    CHECK(ctx, highest < 1.51f);
    release_world_scene(scene);
}

void test_update_new_shape_not_given_the_old_shapes_cached_contacts(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx, [](world_settings::WorldSettings &settings) { settings.sleeping.allow_sleeping = false; });
    add_floor(scene);
    RigidBody *box = add_unit_box(scene, {0.0f, 0.5f, 0.0f});
    for (uint32_t i = 0; i < 60; i++) step_world(*scene.world);
    CHECK_NEAR(ctx, box->position.y, 0.5f, 0.01f);

    // twice as tall, its bottom a half metre into the floor: the pair barely moved, but its cached contacts are the old
    // shape's, so it runs the narrowphase and is pushed out
    rigid_body::set_shape(*scene.world, box, box::create({.half_extents = {0.5f, 1.0f, 0.5f}}));
    for (uint32_t i = 0; i < 120; i++) step_world(*scene.world);
    CHECK_NEAR(ctx, box->position.y, 1.0f, 0.03f);
    release_world_scene(scene);
}

// friction lever arms

void test_update_speculative_friction_acts_at_each_bodys_point(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx, [](world_settings::WorldSettings &settings) { settings.gravity_enabled = false; });
    const float radius = 0.5f;
    const float gap = 0.01f;
    // the sphere first, so it is body a
    RigidBody *ball = add_body(scene, sphere::create({.radius = radius}), motion_type::MOTION_DYNAMIC, {0.0f, radius + gap, 0.0f});
    add_floor(scene);
    ball->motion_properties.linear_velocity = {1.0f, 0.0f, 0.0f};

    step_world(*scene.world);

    CHECK(ctx, scene.world->contact_constraints.count == 1);
    const contact_constraints::ContactConstraint &constraint = scene.world->contact_constraints.pool[0];
    CHECK(ctx, constraint.body_index_a == ball->index);
    // the ball's lever arm for both friction directions is its radius, not halfway across the gap: r x t with r straight
    // down, length radius
    CHECK_NEAR(ctx, vec3f::length(vec3f::load(constraint.friction_constraint_1.r1_plus_u_x_axis)), radius, 1e-5f);
    CHECK_NEAR(ctx, vec3f::length(vec3f::load(constraint.friction_constraint_2.r1_plus_u_x_axis)), radius, 1e-5f);
    release_world_scene(scene);
}

// waking

namespace {

/**
 * one step as step_world makes it, counting the find_pairs and narrowphase rounds and checking no round repeats the
 * last's work: every pair record and every contact appears once in the step's events
 */
uint32_t step_world_counting_rounds(TestCtx &ctx, world::World &world) {
    update::step_begin(world, WORLD_TEST_DELTA_TIME);
    uint32_t rounds = 0;
    do {
        const uint32_t first_pair = world.events.pairs.length;
        update::find_pairs(world);
        // a later round adds only the woken bodies' pairs
        CHECK(ctx, rounds == 0 || world.events.pairs.length > first_pair);
        update::narrowphase(world);
        rounds++;
    } while (world.events.woken > 0);

    for (uint32_t i = 0; i < world.events.pairs.length; i++) {
        for (uint32_t j = i + 1; j < world.events.pairs.length; j++) CHECK(ctx, world.events.pairs[i].pair_record != world.events.pairs[j].pair_record);
    }
    const uint32_t added = world.events.contacts_added.length;
    const uint32_t total = added + world.events.contacts_persisted.length;
    for (uint32_t i = 0; i < total; i++) {
        const int32_t contact_i = i < added ? world.events.contacts_added[i].contact_index : world.events.contacts_persisted[i - added].contact_index;
        for (uint32_t j = i + 1; j < total; j++) {
            const int32_t contact_j = j < added ? world.events.contacts_added[j].contact_index : world.events.contacts_persisted[j - added].contact_index;
            CHECK(ctx, contact_i != contact_j);
        }
    }
    CHECK(ctx, world.answers.contact_override.length == total && world.answers.pair_reject.length == (world.events.pairs.length + 31) / 32);

    update::solve(world);
    update::step_end(world);
    return rounds;
}

}  // namespace

void test_update_body_hitting_sleeping_stack_wakes_it_whole(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_floor(scene);
    RigidBody *stack[4];
    for (uint32_t i = 0; i < 4; i++) stack[i] = add_unit_box(scene, {0.0f, 0.5f + (float)i, 0.0f});
    for (uint32_t i = 0; i < 300 && !(stack[0]->sleeping && stack[1]->sleeping && stack[2]->sleeping && stack[3]->sleeping); i++) {
        step_world(*scene.world);
    }
    CHECK(ctx, stack[0]->sleeping && stack[1]->sleeping && stack[2]->sleeping && stack[3]->sleeping);

    // a quiet step: the narrowphase wakes nothing, so the caller makes one round
    CHECK(ctx, step_world_counting_rounds(ctx, *scene.world) == 1);
    CHECK(ctx, scene.world->events.woken == 0);

    // the sphere's contact wakes the top box; each later round finds the pairs of the box woken last, whose contact wakes
    // the box below, until the bottom box's floor contact wakes nothing
    add_body(scene, sphere::create({.radius = 0.4f}), motion_type::MOTION_DYNAMIC, {0.0f, 5.0f, 0.0f});
    uint32_t rounds = 1;
    for (uint32_t i = 0; i < 120 && stack[3]->sleeping; i++) rounds = step_world_counting_rounds(ctx, *scene.world);
    CHECK(ctx, rounds == 5);

    // the contact that woke the top box found the pairs below it in the same step, down to the floor
    CHECK(ctx, !stack[3]->sleeping);
    CHECK(ctx, !stack[0]->sleeping && !stack[1]->sleeping && !stack[2]->sleeping);
    release_world_scene(scene);
}

// sleeping

void test_update_weight_hanging_from_static_anchor_stays_asleep(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *anchor = add_body(scene, sphere::create({.radius = 0.1f}), motion_type::MOTION_STATIC, {0.0f, 10.0f, 0.0f});
    RigidBody *weight = add_body(scene, sphere::create({.radius = 0.5f}), motion_type::MOTION_DYNAMIC, {0.0f, 7.0f, 0.0f});
    distance_constraint::create(*scene.world, weight, anchor,
                                {.point_a = {0.0f, 7.0f, 0.0f}, .point_b = {0.0f, 10.0f, 0.0f}});

    bool slept = false;
    for (uint32_t i = 0; i < 300 && !slept; i++) {
        step_world(*scene.world);
        slept = weight->sleeping;
    }
    CHECK(ctx, slept);

    // the static anchor is never active, so nothing wakes the weight again
    for (uint32_t i = 0; i < 120; i++) step_world(*scene.world);
    CHECK(ctx, weight->sleeping);
    CHECK(ctx, vec3f::length(vec3f::load(weight->motion_properties.linear_velocity)) == 0.0f);
    release_world_scene(scene);
}

void test_update_determinism_two_worlds_bit_identical(TestCtx &ctx) {
    WorldScene scene_a = build_pile(ctx);
    for (uint32_t i = 0; i < 200; i++) step_world(*scene_a.world);
    // the second world lives alongside the first, so its bodies are at other addresses; nothing may depend on them
    WorldScene scene_b = {};
    {
        world_settings::WorldSettings settings = make_world_test_settings(scene_b.moving, scene_b.not_moving);
        scene_b.world = world::create(settings);
        world_settings::release(settings);
        WorldScene &scene = scene_b;
        add_floor(scene);
        for (uint32_t i = 0; i < 12; i++) {
            const float offset_x = 0.07f * (float)(i % 3) - 0.05f;
            const float offset_z = 0.03f * (float)(i % 4) - 0.04f;
            const Float3 position = {offset_x, 0.6f + 1.1f * (float)i, offset_z};
            shapes::Shape *shape;
            if (i % 3 == 0) shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
            else if (i % 3 == 1) shape = sphere::create({.radius = 0.45f});
            else shape = capsule::create({.half_height_of_cylinder = 0.3f, .radius = 0.3f});
            rigid_body::RigidBodySettings settings_body = body_settings(shape, scene.moving, motion_type::MOTION_DYNAMIC, position);
            settings_body.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.1f * (float)i);
            rigid_body::create(*scene.world, settings_body);
        }
    }
    for (uint32_t i = 0; i < 200; i++) step_world(*scene_b.world);

    const bodies::Bodies &bodies_a = scene_a.world->bodies;
    const bodies::Bodies &bodies_b = scene_b.world->bodies;
    CHECK(ctx, bodies_a.pool.length == bodies_b.pool.length && bodies_a.pool.length == 13);
    bool identical = true;
    bool any_moved = false;
    for (uint32_t i = 0; i < bodies_a.pool.length; i++) {
        const RigidBody &a = *bodies_a.pool[i];
        const RigidBody &b = *bodies_b.pool[i];
        identical = identical && bits_equal(&a.position, &b.position, sizeof(Float3)) &&
                    bits_equal(&a.quaternion, &b.quaternion, sizeof(Float4)) &&
                    bits_equal(&a.motion_properties.linear_velocity, &b.motion_properties.linear_velocity, sizeof(Float3));
        any_moved = any_moved || a.position.y < 0.6f + 1.1f * (float)(i - 1) - 0.5f;
    }
    CHECK(ctx, identical);
    // the pile actually fell and collided
    CHECK(ctx, any_moved);
    CHECK(ctx, bodies_a.pool[12]->position.y < 10.0f);
    release_world_scene(scene_a);
    release_world_scene(scene_b);
}

void test_update_moved_lists_exactly_the_bodies_that_moved(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *floor = add_floor(scene);
    RigidBody *falling = add_unit_box(scene, {0.0f, 5.0f, 0.0f});
    RigidBody *sleeping_box = add_unit_box(scene, {4.0f, 0.5f, 0.0f});
    rigid_body::sleep(*scene.world, sleeping_box);
    RigidBody *kinematic = add_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), motion_type::MOTION_KINEMATIC, {-4.0f, 3.0f, 0.0f});
    kinematic->motion_properties.linear_velocity = {1.0f, 0.0f, 0.0f};
    RigidBody *all[4] = {floor, falling, sleeping_box, kinematic};

    Float3 before[4];
    for (uint32_t i = 0; i < 4; i++) before[i] = all[i]->position;
    step_world(*scene.world);

    // each moved body once, and a body is listed iff its pose changed
    CHECK(ctx, scene.world->events.moved.length == 2);
    for (uint32_t i = 0; i < 4; i++) {
        const bool changed = !vec3f::exact_equals(vec3f::load(before[i]), vec3f::load(all[i]->position));
        CHECK(ctx, changed == moved_this_step(*scene.world, all[i]->index));
    }
    CHECK(ctx, moved_this_step(*scene.world, falling->index) && moved_this_step(*scene.world, kinematic->index));

    // a step of 0 moves nothing
    step_world(*scene.world, 0.0f);
    CHECK(ctx, scene.world->events.moved.length == 0);

    // the step a body falls asleep in lists it; the steps after do not
    scene.world->settings.gravity_enabled = false;
    falling->motion_properties.linear_velocity = {0.0f, 0.0f, 0.0f};
    kinematic->motion_properties.linear_velocity = {0.0f, 0.0f, 0.0f};
    rigid_body::set_motion_type(*scene.world, kinematic, motion_type::MOTION_STATIC, false);
    bool slept_listed = false;
    for (uint32_t i = 0; i < 120 && !falling->sleeping; i++) {
        step_world(*scene.world);
        slept_listed = moved_this_step(*scene.world, falling->index);
    }
    CHECK(ctx, falling->sleeping && slept_listed);
    step_world(*scene.world);
    CHECK(ctx, scene.world->events.moved.length == 0);
    release_world_scene(scene);
}


namespace {

WorldScene make_single_layer_world() {
    dispatch::init();
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t layer = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, layer, layer);
    WorldScene scene = {};
    scene.world = world::create(settings);
    world_settings::release(settings);
    scene.moving = layer;
    scene.not_moving = layer;
    return scene;
}

RigidBody *make_pair_box(WorldScene &scene, const Float3 &position, motion_type::MotionType motion_type = motion_type::MOTION_DYNAMIC, float gravity_factor = 1.0f) {
    rigid_body::RigidBodySettings settings = body_settings(box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), scene.moving, motion_type, position);
    if (motion_type == motion_type::MOTION_DYNAMIC) settings.mass = 1.0f;
    settings.gravity_factor = gravity_factor;
    return rigid_body::create(*scene.world, settings);
}

}  // namespace

void test_update_removal_purge_then_steps(TestCtx &ctx) {
    WorldScene scene = make_single_layer_world();
    RigidBody *a = make_pair_box(scene, {0.0f, 0.0f, 0.0f});
    RigidBody *b = make_pair_box(scene, {0.4f, 0.0f, 0.0f});
    step_world(*scene.world);
    CHECK(ctx, scene.world->pairs.record_count == 1);

    CHECK(ctx, rigid_body::remove(*scene.world, b));
    CHECK(ctx, scene.world->pairs.record_count == 0);
    for (uint32_t i = 0; i < 5; i++) {
        step_world(*scene.world);
        CHECK(ctx, scene.world->events.pairs.length == 0);
    }
    CHECK(ctx, bodies::is_valid(scene.world->bodies, a->id));
    release_world_scene(scene);
}

void test_update_hub_with_spokes_then_steps(TestCtx &ctx) {
    WorldScene scene = make_single_layer_world();
    shapes::Shape *spoke_shape = box::create({.half_extents = {0.1f, 0.1f, 0.1f}});
    RigidBody *hub = make_pair_box(scene, {0.0f, 0.0f, 0.0f}, motion_type::MOTION_DYNAMIC, 0.0f);
    RigidBody *spokes[4];
    const Float3 spoke_positions[4] = {{0.58f, 0.0f, 0.0f}, {-0.58f, 0.0f, 0.0f}, {0.0f, 0.58f, 0.0f}, {0.0f, -0.58f, 0.0f}};
    for (uint32_t i = 0; i < 4; i++) {
        rigid_body::RigidBodySettings settings = body_settings(spoke_shape, scene.moving, motion_type::MOTION_DYNAMIC, spoke_positions[i]);
        settings.gravity_factor = 0.0f;
        spokes[i] = rigid_body::create(*scene.world, settings);
    }
    step_world(*scene.world);
    CHECK(ctx, scene.world->pairs.record_count == 4);
    CHECK(ctx, scene.world->events.pairs.length == 4);

    CHECK(ctx, rigid_body::remove(*scene.world, spokes[0]));
    CHECK(ctx, rigid_body::remove(*scene.world, spokes[1]));
    CHECK(ctx, scene.world->pairs.record_count == 2);

    rigid_body::RigidBodySettings settings = body_settings(spoke_shape, scene.moving, motion_type::MOTION_DYNAMIC, spoke_positions[0]);
    settings.gravity_factor = 0.0f;
    rigid_body::create(*scene.world, settings);
    for (uint32_t i = 0; i < 5; i++) {
        step_world(*scene.world);
        CHECK(ctx, scene.world->pairs.record_count == 3);
    }
    CHECK(ctx, bodies::is_valid(scene.world->bodies, hub->id));
    release_world_scene(scene);
}

void test_update_determinism_stack_60_steps(TestCtx &ctx) {
    Float3 positions[2][4];
    for (uint32_t run = 0; run < 2; run++) {
        WorldScene scene = make_single_layer_world();
        make_pair_box(scene, {0.0f, -1.0f, 0.0f}, motion_type::MOTION_STATIC);
        make_pair_box(scene, {0.0f, 0.2f, 0.0f});
        make_pair_box(scene, {0.05f, 1.4f, 0.03f});
        make_pair_box(scene, {-0.03f, 2.6f, 0.02f});
        for (uint32_t i = 0; i < 60; i++) step_world(*scene.world);
        for (uint32_t i = 0; i < 4; i++) positions[run][i] = scene.world->bodies.pool[i]->position;
        release_world_scene(scene);
    }
    for (uint32_t i = 0; i < 4; i++) CHECK(ctx, bits_equal(&positions[0][i], &positions[1][i], sizeof(Float3)));
}

void test_update_bodies_drifting_within_fat_bounds_form_contact(TestCtx &ctx) {
    WorldScene scene = make_single_layer_world();
    // a 0.08 gap between faces: outside tight + speculative reach (0.07) at creation, inside fat + speculative (0.12)
    RigidBody *body_a = make_pair_box(scene, {-0.54f, 0.0f, 0.0f}, motion_type::MOTION_DYNAMIC, 0.0f);
    RigidBody *body_b = make_pair_box(scene, {0.54f, 0.0f, 0.0f}, motion_type::MOTION_DYNAMIC, 0.0f);
    // 0.5 m/s each closes the gap to speculative range in about 4 steps, under the 0.05 escape margin
    rigid_body::set_linear_velocity(*scene.world, body_a, vec3f::make(0.5f, 0.0f, 0.0f));
    rigid_body::set_linear_velocity(*scene.world, body_b, vec3f::make(-0.5f, 0.0f, 0.0f));
    for (uint32_t i = 0; i < 6; i++) step_world(*scene.world);
    CHECK(ctx, body_contact_count(*scene.world, *body_a) > 0);
    CHECK(ctx, body_contact_count(*scene.world, *body_b) > 0);
    release_world_scene(scene);
}
