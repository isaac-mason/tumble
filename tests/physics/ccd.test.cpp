#include "tumble/physics/ccd.h"
#include "tumble/math/aabox.h"

#include "world_test_scene.h"
#include "test_math.h"

using rigid_body::RigidBody;

namespace {

/** a radius 0.2 bullet at x = -2 (or the given x) moving 300 m/s along +x, 5 m in one step */
RigidBody *add_bullet(WorldScene &scene, float x = -2.0f) {
    rigid_body::RigidBodySettings settings = body_settings(sphere::create({.radius = 0.2f}), scene.moving, motion_type::MOTION_DYNAMIC, {x, 0.0f, 0.0f});
    settings.motion_quality = motion_properties::MOTION_QUALITY_LINEAR_CAST;
    RigidBody *bullet = add_body(*scene.world, settings);
    bullet->motion_properties.linear_velocity = {300.0f, 0.0f, 0.0f};
    return bullet;
}

/** a thin static wall spanning x in [-0.1, 0.1] */
RigidBody *add_thin_wall(WorldScene &scene) {
    return add_body(scene, box::create({.half_extents = {0.1f, 5.0f, 5.0f}}), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
}

}  // namespace

void test_ccd_reset_and_swept_aabb(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    ccd::CCDBody ccd_body;
    ccd_body.fraction = 0.3f;
    ccd_body.contact_settings.inv_mass_scale_1 = 0.0f;
    ccd::reset_ccd_body(ccd_body);
    CHECK(ctx, ccd_body.hit_body_index == ~0u && ccd_body.fraction == 1.0f && ccd_body.fraction_plus_slop == 1.0f);
    CHECK(ctx, ccd_body.contact_settings.inv_mass_scale_1 == 1.0f && ccd_body.contact_settings.combined_friction == 0.0f);

    // the union of the start bounds and the same moved by the displacement
    rigid_body::RigidBody body = {};
    body.aabb = {{-1.0f, -2.0f, -3.0f}, {1.0f, 2.0f, 3.0f}};
    Box3f swept;
    aabox::store(swept, ccd::compute_swept_aabb(body, vec3f::make(4.0f, -1.0f, 0.5f)));
    CHECK(ctx, swept.min.x == -1.0f && swept.min.y == -3.0f && swept.min.z == -3.0f);
    CHECK(ctx, swept.max.x == 5.0f && swept.max.y == 2.0f && swept.max.z == 3.5f);
}


void test_ccd_fast_sphere_stopped_by_thin_wall(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *wall = add_thin_wall(scene);
    RigidBody *bullet = add_bullet(scene);

    step_world(*scene.world);

    // stopped at the near face (-0.1 - radius, plus at most the allowed penetration), not tunnelled through
    CHECK(ctx, scene.world->ccd.ccd_bodies.length == 1);
    CHECK(ctx, scene.world->ccd.ccd_bodies[0].hit_body_index == wall->index);
    CHECK(ctx, bullet->position.x > -2.0f && bullet->position.x < 0.0f);
    CHECK_NEAR(ctx, bullet->position.x, -0.3f, 0.025f);
    // the contact killed the approach velocity (no restitution)
    CHECK(ctx, bullet->motion_properties.linear_velocity.x < 1.0f);

    // and it stays on the near side
    for (uint32_t i = 0; i < 30; i++) step_world(*scene.world);
    CHECK(ctx, bullet->position.x < -0.1f);
    release_world_scene(scene);
}

void test_ccd_without_linear_cast_tunnels(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_thin_wall(scene);
    RigidBody *bullet = add_body(scene, sphere::create({.radius = 0.2f}), motion_type::MOTION_DYNAMIC, {-2.0f, 0.0f, 0.0f});
    bullet->motion_properties.linear_velocity = {300.0f, 0.0f, 0.0f};
    step_world(*scene.world);
    // the discrete step jumps the wall
    CHECK(ctx, scene.world->ccd.ccd_bodies.length == 0);
    CHECK(ctx, bullet->position.x > 2.9f);
    release_world_scene(scene);
}

void test_ccd_contact_reported_on_impact_step(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *wall = add_thin_wall(scene);
    RigidBody *bullet = add_bullet(scene);

    step_world(*scene.world);

    // no discrete contact can exist at the start of the step: the sweep reported it
    CHECK(ctx, scene.world->events.contacts_added.length == 1);
    if (scene.world->events.contacts_added.length == 1) {
        const events::ContactEvent &event = scene.world->events.contacts_added[0];
        CHECK(ctx, (event.body_id_a == wall->id && event.body_id_b == bullet->id) || (event.body_id_a == bullet->id && event.body_id_b == wall->id));
        CHECK(ctx, event.body_id_a < event.body_id_b);
        // along the travel axis, from a to b: the wall (a) to the bullet (b) points back along -x
        CHECK(ctx, scalar::abs(event.normal.x) > 0.9f);
        CHECK(ctx, event.normal.x < 0.0f);
        CHECK(ctx, event.num_points >= 1);
    }
    CHECK(ctx, bullet->position.x < 0.0f);
    release_world_scene(scene);
}

void test_ccd_sweeps_target_moved_into_path_this_step(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // a kinematic plate whose bottom face starts at y = 0.245, above the bullet, and ends at y = 0.185, inside its path
    RigidBody *plate = add_body(scene, box::create({.half_extents = {0.1f, 0.5f, 5.0f}}), motion_type::MOTION_KINEMATIC, {0.0f, 0.745f, 0.0f});
    plate->motion_properties.linear_velocity = {0.0f, -3.6f, 0.0f};
    RigidBody *bullet = add_bullet(scene);

    step_world(*scene.world);

    CHECK(ctx, plate->position.y - 0.5f < 0.2f);
    CHECK(ctx, bullet->position.x < 0.0f);
    release_world_scene(scene);
}


void test_ccd_contact_creates_pair_record(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *wall = add_thin_wall(scene);
    // far enough that no fat bounds overlap at the start of the step: the broadphase never finds the pair
    RigidBody *bullet = add_bullet(scene, -5.0f);
    CHECK(ctx, pairs::find_pair_record(scene.world->pairs, *wall, *bullet) == pairs::INVALID_PAIR_KEY);

    step_world(*scene.world);

    CHECK(ctx, scene.world->events.pairs.length == 0);
    CHECK(ctx, scene.world->events.contacts_added.length == 1);
    CHECK(ctx, pairs::find_pair_record(scene.world->pairs, *wall, *bullet) != pairs::INVALID_PAIR_KEY);
    CHECK(ctx, pair_contact_count(*scene.world, *wall, *bullet) == 1);
    CHECK(ctx, bullet->position.x < 0.0f);

    // the next step keeps the record through the usual sweep
    step_world(*scene.world);
    CHECK(ctx, pairs::find_pair_record(scene.world->pairs, *wall, *bullet) != pairs::INVALID_PAIR_KEY);
    CHECK(ctx, bullet->position.x < 0.0f);
    release_world_scene(scene);
}


namespace {

void gravity_off(world_settings::WorldSettings &settings) { settings.gravity_enabled = false; }

}  // namespace

void test_ccd_body_index_only_for_ccd_bodies(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx, gravity_off);
    RigidBody *wall = add_body(scene, box::create({.half_extents = {0.5f, 5.0f, 5.0f}}), motion_type::MOTION_STATIC, {20.0f, 0.0f, 0.0f});
    rigid_body::RigidBodySettings settings = body_settings(sphere::create({.radius = 0.1f}), scene.moving, motion_type::MOTION_DYNAMIC, {0.0f, 0.0f, 0.0f});
    settings.motion_quality = motion_properties::MOTION_QUALITY_LINEAR_CAST;
    RigidBody *bullet = add_body(*scene.world, settings);
    rigid_body::set_linear_velocity(scene.world->bodies, bullet, vec3f::make(200.0f, 0.0f, 0.0f));
    RigidBody *bystander = add_body(scene, sphere::create({.radius = 0.1f}), motion_type::MOTION_DYNAMIC, {0.0f, 5.0f, 0.0f});

    step_world(*scene.world);

    // the fast body keeps its record until the next step clears it
    CHECK(ctx, scene.world->ccd.ccd_bodies.length == 1);
    CHECK(ctx, bullet->ccd_body_index == 0);
    CHECK(ctx, bystander->ccd_body_index == -1);
    CHECK(ctx, wall->ccd_body_index == -1);
    // no hit: the whole displacement
    CHECK_NEAR(ctx, bullet->position.x, 200.0f * WORLD_TEST_DELTA_TIME * (1.0f - 0.05f * WORLD_TEST_DELTA_TIME), 1e-3f);

    rigid_body::set_linear_velocity(scene.world->bodies, bullet, vec3f::make(0.0f, 0.0f, 0.0f));
    step_world(*scene.world);
    CHECK(ctx, scene.world->ccd.ccd_bodies.length == 0);
    CHECK(ctx, bullet->ccd_body_index == -1);
    CHECK(ctx, bystander->ccd_body_index == -1);
    CHECK(ctx, wall->ccd_body_index == -1);
    release_world_scene(scene);
}

void test_ccd_two_bullets_in_time_of_impact_order(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx, gravity_off);
    RigidBody *wall = add_thin_wall(scene);
    // the later one first in the ccd list: the resolution runs by impact time, and each body keeps its own record
    RigidBody *far_bullet = add_bullet(scene, -4.0f);
    rigid_body::RigidBodySettings settings = body_settings(sphere::create({.radius = 0.2f}), scene.moving, motion_type::MOTION_DYNAMIC, {-2.0f, 2.0f, 0.0f});
    settings.motion_quality = motion_properties::MOTION_QUALITY_LINEAR_CAST;
    RigidBody *near_bullet = add_body(*scene.world, settings);
    near_bullet->motion_properties.linear_velocity = {300.0f, 0.0f, 0.0f};

    step_world(*scene.world);

    CHECK(ctx, scene.world->ccd.ccd_bodies.length == 2);
    CHECK(ctx, scene.world->ccd.ccd_bodies[(uint32_t)far_bullet->ccd_body_index].body_index == far_bullet->index);
    CHECK(ctx, scene.world->ccd.ccd_bodies[(uint32_t)near_bullet->ccd_body_index].body_index == near_bullet->index);
    CHECK(ctx, scene.world->ccd.ccd_bodies[(uint32_t)near_bullet->ccd_body_index].hit_body_index == wall->index);
    CHECK(ctx, scene.world->ccd.ccd_bodies[(uint32_t)near_bullet->ccd_body_index].fraction <
               scene.world->ccd.ccd_bodies[(uint32_t)far_bullet->ccd_body_index].fraction);
    // both stopped at the wall's near face
    CHECK_NEAR(ctx, far_bullet->position.x, -0.3f, 0.025f);
    CHECK_NEAR(ctx, near_bullet->position.x, -0.3f, 0.025f);
    release_world_scene(scene);
}


namespace {

RigidBody *add_linear_cast_sphere(WorldScene &scene, float radius, float x, float y, float velocity_x, int32_t material_id = -1) {
    rigid_body::RigidBodySettings settings =
        body_settings(sphere::create({.radius = radius, .material_id = material_id}), scene.moving, motion_type::MOTION_DYNAMIC, {x, y, 0.0f});
    settings.motion_quality = motion_properties::MOTION_QUALITY_LINEAR_CAST;
    RigidBody *body = add_body(*scene.world, settings);
    body->motion_properties.linear_velocity = {velocity_x, 0.0f, 0.0f};
    return body;
}

const ccd::CCDBody &ccd_body_of(const WorldScene &scene, const RigidBody &body) {
    return scene.world->ccd.ccd_bodies[(uint32_t)body.ccd_body_index];
}

}  // namespace

void test_ccd_body_hit_earlier_by_another_ccd_body_travels_only_to_the_impact(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // a sweeps b (the lower index does the pair) and b finds nothing, so a's hit is what shortens b's travel
    RigidBody *a = add_linear_cast_sphere(scene, 0.2f, -3.0f, 0.0f, 300.0f);
    RigidBody *b = add_linear_cast_sphere(scene, 0.2f, 3.0f, 0.0f, -300.0f);

    step_world(*scene.world);

    // they meet at the origin, 5.6 of the 10 units closing: b stops at its side, not 5 units on through a
    CHECK(ctx, ccd_body_of(scene, *a).hit_body_index == b->index);
    CHECK_NEAR(ctx, b->position.x, 0.2f, 0.005f);
    CHECK(ctx, a->position.x < b->position.x);
    // head on with equal masses and no restitution, the impulse stops both
    CHECK(ctx, scalar::abs(a->motion_properties.linear_velocity.x) < 1.0f);
    CHECK(ctx, scalar::abs(b->motion_properties.linear_velocity.x) < 1.0f);
    release_world_scene(scene);
}

void test_ccd_target_stopped_earlier_against_something_else_takes_no_response(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // a chases b; b reaches the wall at 0.28 of the step, a reaches b's swept path at 0.35 and the wall at 0.43
    RigidBody *a = add_linear_cast_sphere(scene, 0.5f, -4.05f, 0.0f, 480.0f);
    RigidBody *b = add_linear_cast_sphere(scene, 0.5f, -2.0f, 0.0f, 300.0f);
    RigidBody *wall = add_thin_wall(scene);

    step_world(*scene.world);

    CHECK(ctx, ccd_body_of(scene, *b).hit_body_index == wall->index);
    CHECK(ctx, ccd_body_of(scene, *a).hit_body_index == b->index);
    // b resolved first and stopped at the wall; a's hit on b's swept path never happened, so b is not pushed on
    CHECK_NEAR(ctx, b->position.x, -0.6f, 0.05f);
    CHECK(ctx, scalar::abs(b->motion_properties.linear_velocity.x) < 1.0f);
    CHECK(ctx, a->motion_properties.linear_velocity.x > 470.0f);
    release_world_scene(scene);
}

void test_ccd_kinematic_linear_cast_target_does_not_shift_the_contact(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // the wall slides 1 unit along +y this step; it is kinematic, so it moves during integration like any other
    rigid_body::RigidBodySettings settings =
        body_settings(box::create({.half_extents = {0.1f, 5.0f, 5.0f}}), scene.moving, motion_type::MOTION_KINEMATIC, {0.0f, 0.0f, 0.0f});
    settings.motion_quality = motion_properties::MOTION_QUALITY_LINEAR_CAST;
    RigidBody *wall = add_body(*scene.world, settings);
    wall->motion_properties.linear_velocity = {0.0f, 60.0f, 0.0f};
    add_bullet(scene);

    step_world(*scene.world);

    // the sphere meets the wall's face on its own line of travel, y = 0, not 0.34 units up the wall
    CHECK(ctx, scene.world->events.contacts_added.length == 1);
    if (scene.world->events.contacts_added.length == 1) {
        const events::ContactEvent &event = scene.world->events.contacts_added[0];
        CHECK(ctx, event.body_id_a == wall->id);
        CHECK(ctx, event.num_points == 1);
        CHECK_NEAR(ctx, event.points_a[0].y, 0.0f, 0.005f);
    }
    release_world_scene(scene);
}

void test_ccd_contact_with_bodies_swapped_negates_normal_and_swaps_materials(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // the wall is created first, so it has the lower id and comes first in the report
    RigidBody *wall = add_body(scene, box::create({.half_extents = {0.1f, 5.0f, 5.0f}, .material_id = 7}), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    RigidBody *bullet = add_linear_cast_sphere(scene, 0.2f, -2.0f, 0.0f, 300.0f, 3);

    step_world(*scene.world);

    CHECK(ctx, scene.world->events.contacts_added.length == 1);
    if (scene.world->events.contacts_added.length == 1) {
        const events::ContactEvent &event = scene.world->events.contacts_added[0];
        CHECK(ctx, event.body_id_a == wall->id && event.body_id_b == bullet->id);
        // from a (the wall) to b (the bullet): back along -x
        CHECK(ctx, event.normal.x < -0.9f);
        CHECK(ctx, event.material_id_a == 7 && event.material_id_b == 3);
    }
    CHECK(ctx, bullet->position.x < 0.0f);
    release_world_scene(scene);
}

void test_ccd_bodies_resolved_out_of_creation_order_find_their_targets_records(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // a and b converge as above; c, created last, hits its own wall first and sorts ahead of both
    RigidBody *a = add_linear_cast_sphere(scene, 0.2f, -3.0f, 0.0f, 300.0f);
    RigidBody *b = add_linear_cast_sphere(scene, 0.2f, 3.0f, 0.0f, -300.0f);
    RigidBody *c = add_linear_cast_sphere(scene, 0.2f, -2.0f, 20.0f, 300.0f);
    add_body(scene, box::create({.half_extents = {0.1f, 1.0f, 1.0f}}), motion_type::MOTION_STATIC, {0.0f, 20.0f, 0.0f});

    step_world(*scene.world);

    // resolved by impact time, c first, while every body still points at its own record
    CHECK(ctx, scene.world->ccd.order.length == 3);
    if (scene.world->ccd.order.length == 3) {
        CHECK(ctx, scene.world->ccd.ccd_bodies[scene.world->ccd.order[0]].body_index == c->index);
        CHECK(ctx, scene.world->ccd.ccd_bodies[scene.world->ccd.order[1]].body_index == a->index);
        CHECK(ctx, scene.world->ccd.ccd_bodies[scene.world->ccd.order[2]].body_index == b->index);
    }
    CHECK(ctx, ccd_body_of(scene, *a).body_index == a->index);
    CHECK(ctx, ccd_body_of(scene, *b).body_index == b->index);
    CHECK(ctx, ccd_body_of(scene, *c).body_index == c->index);
    CHECK_NEAR(ctx, c->position.x, -0.3f, 0.025f);
    CHECK_NEAR(ctx, b->position.x, 0.2f, 0.005f);
    CHECK(ctx, a->position.x < b->position.x);
    release_world_scene(scene);
}

// a grazes b, which is discrete and has already moved this step, up and away from a's path
// (0.045, inside its broadphase leaf). at the time of impact the normal is nearly along +y, where b moves away faster
// than a follows, so the bodies are separating. the ccd contact is solved anyway, with an unbounded normal impulse,
// so they leave with no relative normal velocity
void test_ccd_contact_whose_bodies_are_separating_is_still_solved(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // a small a keeps its allowed penetration (a quarter of its radius) small, so the grazing hit is kept
    RigidBody *a = add_linear_cast_sphere(scene, 0.02f, 0.0f, 0.0f, 15.0f);
    RigidBody *b = add_body(scene, sphere::create({.radius = 0.5f}), motion_type::MOTION_DYNAMIC, {0.28f, 0.47f, 0.0f});
    b->motion_properties.linear_velocity = {0.0f, 2.7f, 0.0f};

    step_world(*scene.world);

    const ccd::CCDBody &ccd_body_a = ccd_body_of(scene, *a);
    CHECK(ctx, ccd_body_a.hit_body_index == b->index);
    CHECK(ctx, vec3f::y(ccd_body_a.contact_normal) > 0.95f);
    const Vec3f relative_velocity = vec3f::subtract(vec3f::load(b->motion_properties.linear_velocity), vec3f::load(a->motion_properties.linear_velocity));
    CHECK_NEAR(ctx, vec3f::dot(relative_velocity, ccd_body_a.contact_normal), 0.0f, 5e-4f);
    release_world_scene(scene);
}

// b is linear cast but moves 0.04 this step, under its 0.05 threshold, so it has already moved.
// its motion still counts against a's sweep: a sweeps 1 - 0.04 relative to b and stops short of b's face, which the
// full 1 would have reached (at 0.97, inside the step even with a's 0.02 allowed penetration)
void test_ccd_linear_cast_target_below_its_threshold_still_shortens_the_sweep(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *a = add_linear_cast_sphere(scene, 0.1f, 0.0f, 0.0f, 60.0f);
    RigidBody *b = add_linear_cast_sphere(scene, 1.0f, 2.03f, 0.0f, 2.4f);

    step_world(*scene.world);

    // b's face ends at 1.07, a's leading point at 1.1 after a full step and 1.06 after the relative one
    CHECK(ctx, b->ccd_body_index == -1);
    CHECK(ctx, ccd_body_of(scene, *a).hit_body_index == ~0u);
    CHECK_NEAR(ctx, a->position.x, 1.0f, 0.005f);
    CHECK(ctx, a->motion_properties.linear_velocity.x > 59.0f);
    release_world_scene(scene);
}
