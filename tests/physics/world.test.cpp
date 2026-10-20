#include "tumble/physics/world.h"

#include "tumble/util/alloc.h"
#include "tumble/util/object.h"
#include "world_test_scene.h"
#include "test_math.h"

using rigid_body::RigidBody;

void test_world_create_copies_settings_and_destroy_frees(TestCtx &ctx) {
    dispatch::init();
    const size_t bytes_before = util::allocated_bytes();
    const size_t blocks_before = util::allocated_blocks();

    uint32_t moving, not_moving;
    world_settings::WorldSettings settings = make_world_test_settings(moving, not_moving);
    settings.solver.velocity_iterations = 7;
    world::World *world = world::create(settings);
    world_settings::release(settings);
    CHECK(ctx, world != nullptr);

    // the defaults, the tuned value, and the layers, copied: the settings were released before any of this
    CHECK(ctx, world->settings.solver.velocity_iterations == 7);
    CHECK(ctx, world->settings.solver.position_iterations == 2);
    CHECK(ctx, world->settings.gravity.y == -9.81f && world->settings.gravity_enabled);
    CHECK(ctx, world->settings.narrowphase.speculative_contact_distance == 0.02f);
    CHECK(ctx, world->settings.narrowphase.internal_edge_removal_vertex_tolerance_sq == 1e-8f);
    CHECK(ctx, world->settings.layers.object_layers == 2);
    CHECK(ctx, layers::object_layer_collides_with_object_layer(world->settings.layers, moving, not_moving));
    CHECK(ctx, !layers::object_layer_collides_with_object_layer(world->settings.layers, not_moving, not_moving));
    CHECK(ctx, world->broadphase.trees.length == 2);
    CHECK(ctx, world->previous_time_step == 0.0f && world->delta_time == 0.0f);

    // bodies, contacts, constraints and events in use, then every part freed
    shapes::Shape *floor_shape = box::create({.half_extents = {5.0f, 0.5f, 5.0f}});
    shapes::Shape *box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    rigid_body::create(*world, body_settings(floor_shape, not_moving, motion_type::MOTION_STATIC, {0.0f, -0.5f, 0.0f}));
    rigid_body::create(*world, body_settings(box_shape, moving, motion_type::MOTION_DYNAMIC, {0.0f, 0.5f, 0.0f}));
    for (uint32_t i = 0; i < 5; i++) step_world(*world);
    CHECK(ctx, world->contacts.contacts.length > 0 && world->events.contacts_persisted.length > 0);
    world::destroy(world);
    box::destroy(floor_shape);
    box::destroy(box_shape);

    CHECK(ctx, util::allocated_blocks() == blocks_before);
    CHECK(ctx, util::allocated_bytes() == bytes_before);
}


void test_world_stale_contacts_removed_when_bodies_move_apart(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    shapes::Shape *shape = sphere::create({.radius = 1.0f});
    RigidBody *body_a = add_body(scene, shape, motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    RigidBody *body_b = add_body(scene, shape, motion_type::MOTION_DYNAMIC, {0.5f, 0.0f, 0.0f});

    step_world(world);
    CHECK(ctx, world.events.contacts_added.length > 0);
    if (world.events.contacts_added.length > 0) {
        CHECK(ctx, world.events.contacts_added[0].body_id_a == body_a->id);
        CHECK(ctx, world.events.contacts_added[0].body_id_b == body_b->id);
    }
    CHECK(ctx, body_contact_count(world, *body_a) > 0 && body_contact_count(world, *body_b) > 0);

    // still touching: persisted
    step_world(world);
    CHECK(ctx, world.events.contacts_added.length == 0);
    CHECK(ctx, world.events.contacts_persisted.length > 0);
    if (world.events.contacts_persisted.length > 0) {
        CHECK(ctx, world.events.contacts_persisted[0].body_id_a == body_a->id);
        CHECK(ctx, world.events.contacts_persisted[0].body_id_b == body_b->id);
    }

    // far apart, beyond the broadphase: removed
    rigid_body::set_transform(world, body_a, vec3f::make(0.0f, 0.0f, 0.0f), quatf::identity(), false);
    rigid_body::set_transform(world, body_b, vec3f::make(100.0f, 0.0f, 0.0f), quatf::identity(), false);
    step_world(world);
    CHECK(ctx, world.events.contacts_removed.length > 0);
    if (world.events.contacts_removed.length > 0) {
        CHECK(ctx, world.events.contacts_removed[0].body_id_a == body_a->id);
        CHECK(ctx, world.events.contacts_removed[0].body_id_b == body_b->id);
    }
    CHECK(ctx, body_contact_count(world, *body_a) == 0 && body_contact_count(world, *body_b) == 0);
    release_world_scene(scene);
}

void test_world_stale_contacts_removed_when_shapes_stop_colliding(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    RigidBody *body_a = add_body(scene, sphere::create({.radius = 0.5f}), motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    add_body(scene, sphere::create({.radius = 0.5f}), motion_type::MOTION_STATIC, {0.5f, 0.0f, 0.0f});

    step_world(world);
    CHECK(ctx, body_contact_count(world, *body_a) > 0);

    // in broadphase reach, no longer colliding
    rigid_body::set_transform(world, body_a, vec3f::make(5.0f, 0.0f, 0.0f), quatf::identity(), false);
    step_world(world);
    CHECK(ctx, world.events.contacts_removed.length > 0);
    CHECK(ctx, body_contact_count(world, *body_a) == 0);
    release_world_scene(scene);
}

void test_world_contacts_removed_the_step_after_body_removal(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    shapes::Shape *shape = sphere::create({.radius = 0.5f});
    RigidBody *body_a = add_body(scene, shape, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *body_b = add_body(scene, shape, motion_type::MOTION_DYNAMIC, {0.5f, 0.0f, 0.0f});

    step_world(world);
    CHECK(ctx, body_contact_count(world, *body_a) > 0);
    CHECK(ctx, world.events.contacts_removed.length == 0);

    // the removal queues the report; the buffers the caller reads after this step are untouched
    const body_id::BodyId body_b_id = body_b->id;
    CHECK(ctx, rigid_body::remove(world, body_b));
    CHECK(ctx, world.events.contacts_removed.length == 0);
    CHECK(ctx, body_contact_count(world, *body_a) == 0);

    // the next step reports it, with the dead body's id, from step_begin on
    update::step_begin(world, WORLD_TEST_DELTA_TIME);
    CHECK(ctx, world.events.contacts_removed.length == 1);
    if (world.events.contacts_removed.length == 1) {
        CHECK(ctx, world.events.contacts_removed[0].body_id_a == body_a->id);
        CHECK(ctx, world.events.contacts_removed[0].body_id_b == body_b_id);
    }
    update::find_pairs(world);
    update::narrowphase(world);
    update::solve(world);
    update::step_end(world);
    CHECK(ctx, world.events.contacts_removed.length == 1);

    // exactly once
    step_world(world);
    CHECK(ctx, world.events.contacts_removed.length == 0);
    release_world_scene(scene);
}

void test_world_one_settings_makes_two_worlds(TestCtx &ctx) {
    dispatch::init();
    uint32_t moving, not_moving;
    world_settings::WorldSettings settings = make_world_test_settings(moving, not_moving);
    world::World *first = world::create(settings);
    world_settings::disable_collision(settings, moving, not_moving);
    world::World *second = world::create(settings);
    world_settings::release(settings);

    // each world kept the layers as they were when it was made
    CHECK(ctx, layers::object_layer_collides_with_object_layer(first->settings.layers, moving, not_moving));
    CHECK(ctx, !layers::object_layer_collides_with_object_layer(second->settings.layers, moving, not_moving));
    CHECK(ctx, first->settings.layers.object_layer_pairs.data != second->settings.layers.object_layer_pairs.data);
    world::destroy(first);
    world::destroy(second);
}

/** the combined friction of the first contact the box makes with the floor, the floor's shape naming material 0 */
static float first_contact_friction(TestCtx &ctx, const contact_constraints::Material *materials, uint32_t count) {
    WorldScene scene = make_world_scene(ctx);
    CHECK(ctx, world::set_materials(*scene.world, materials, count));
    shapes::Shape *floor_shape = box::create({.half_extents = {10.0f, 0.5f, 10.0f}, .material_id = 0});
    rigid_body::RigidBodySettings floor = body_settings(floor_shape, scene.not_moving, motion_type::MOTION_STATIC, {0.0f, -0.5f, 0.0f});
    floor.friction = 0.2f;
    rigid_body::create(*scene.world, floor);
    shapes::Shape *box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    rigid_body::RigidBodySettings box = body_settings(box_shape, scene.moving, motion_type::MOTION_DYNAMIC, {0.0f, 0.5f, 0.0f});
    box.friction = 0.25f;
    rigid_body::create(*scene.world, box);
    step_world(*scene.world);
    const float friction = scene.world->events.contacts_added.length > 0 ? scene.world->events.contacts_added[0].settings.combined_friction : -1.0f;
    release_world_scene(scene);
    box::destroy(box_shape);
    box::destroy(floor_shape);
    return friction;
}

void test_world_materials_stand_in_for_the_body_on_their_side(TestCtx &ctx) {
    // geometric means: the box's 0.25 against the material's 0.81, then against the floor body's own 0.2 with no table
    const contact_constraints::Material rough = {0.81f, 0.0f};
    CHECK_NEAR(ctx, first_contact_friction(ctx, &rough, 1), 0.45f, 1e-6f);
    CHECK_NEAR(ctx, first_contact_friction(ctx, nullptr, 0), scalar::sqrt(0.25f * 0.2f), 1e-6f);
}
