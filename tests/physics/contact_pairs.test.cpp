#include "tumble/physics/contact_pairs.h"

#include "tumble/physics/shapes/box.h"
#include "world_test_scene.h"

using contact_pairs::ContactPair;
using contact_pairs::ContactPairs;
using rigid_body::RigidBody;

namespace {

/** a world with a static ground, its top at y = 1, that observes nothing */
struct GroundScene {
    WorldScene scene;
    RigidBody *ground;
};

GroundScene make_scene(TestCtx &ctx) {
    GroundScene s = {};
    s.scene = make_world_scene(ctx);
    s.ground = add_body(s.scene, box::create({.half_extents = {10.0f, 0.5f, 10.0f}}), motion_type::MOTION_STATIC, {0.0f, 0.5f, 0.0f});
    return s;
}

void release_scene(GroundScene &s) { release_world_scene(s.scene); }

/** a dynamic box that never sleeps, observing its contacts unless `observes` is false */
RigidBody *add_box(GroundScene &s, float half_extent, const Float3 &position, uint64_t user_data, bool observes = true) {
    rigid_body::RigidBodySettings settings =
        body_settings(box::create({.half_extents = {half_extent, half_extent, half_extent}}), s.scene.moving, motion_type::MOTION_DYNAMIC, position);
    settings.user_data = user_data;
    settings.allow_sleeping = false;
    settings.observes_contacts = observes;
    return rigid_body::create(*s.scene.world, settings);
}

/** the active pairs whose sides are these two bodies, either way round */
uint32_t count_pairs(const ContactPairs &pairs, const RigidBody &first, const RigidBody &second) {
    uint32_t count = 0;
    for (uint32_t i = 0; i < pairs.active.length; i++) {
        const ContactPair &pair = pairs.pairs[pairs.active[i]];
        if ((pair.a.body_id == first.id && pair.b.body_id == second.id) || (pair.a.body_id == second.id && pair.b.body_id == first.id)) count++;
    }
    return count;
}

/** a step with the pairs recorded after the phases that append contact events */
void step(world::World &world) {
    contact_pairs::begin(world);
    update::step_begin(world, WORLD_TEST_DELTA_TIME);
    do {
        update::find_pairs(world);
        update::narrowphase(world);
    } while (world.events.woken > 0);
    const uint32_t added = world.events.contacts_added.length;
    const uint32_t persisted = world.events.contacts_persisted.length;
    contact_pairs::record_world_contacts(world, 0, 0);
    update::solve(world);
    update::step_end(world);
    contact_pairs::record_world_contacts(world, added, persisted);
    contact_pairs::end(world);
}

const ContactPair &active_pair(const ContactPairs &pairs, uint32_t i) { return pairs.pairs[pairs.active[i]]; }

}  // namespace

void test_contact_pairs_a_resting_body_keeps_its_handle_while_it_persists(TestCtx &ctx) {
    // a box on the ground: one pair, added, then the same handle each step, persisted, none removed
    GroundScene s = make_scene(ctx);
    RigidBody *box = add_box(s, 0.4f, {0.5f, 1.39f, 0.5f}, 7);
    step(*s.scene.world);
    const ContactPairs &pairs = s.scene.world->contact_pairs;
    CHECK(ctx, pairs.active.length == 1 && active_pair(pairs, 0).added && count_pairs(pairs, *box, *s.ground) == 1);
    const uint32_t first = pairs.active[0];
    for (uint32_t n = 0; n < 5; n++) {
        step(*s.scene.world);
        CHECK(ctx, pairs.active.length == 1 && pairs.removed.length == 0);
        CHECK(ctx, pairs.active[0] == first && !active_pair(pairs, 0).added);
    }
    release_scene(s);
}

void test_contact_pairs_sides_carry_their_bodies_and_user_data(TestCtx &ctx) {
    // a box of user data above 2^32 on the ground: its side names its body, sub shape and user data; the ground's its own
    GroundScene s = make_scene(ctx);
    RigidBody *box = add_box(s, 0.4f, {0.5f, 1.39f, 0.5f}, 0x200000009ull);
    step(*s.scene.world);
    const ContactPairs &pairs = s.scene.world->contact_pairs;
    CHECK(ctx, pairs.active.length == 1);
    const ContactPair &pair = active_pair(pairs, 0);
    const contact_pairs::Side &box_side = pair.a.body_id == box->id ? pair.a : pair.b;
    const contact_pairs::Side &ground_side = pair.a.body_id == box->id ? pair.b : pair.a;
    CHECK(ctx, box_side.user_data == 0x200000009ull && sub_shape::is_empty(box_side.sub_shape_id) && !box_side.is_sensor);
    CHECK(ctx, ground_side.body_id == s.ground->id && ground_side.user_data == 0);
    // the normal runs a to b, so it points from the box down when the box is a
    CHECK(ctx, pair.a.body_id == box->id ? pair.normal.y < -0.9f : pair.normal.y > 0.9f);
    release_scene(s);
}

void test_contact_pairs_removed_in_the_order_added_then_freed_a_step_later(TestCtx &ctx) {
    // two boxes land, then are lifted clear: every pair is removed that step, in the order added, and freed by the next
    GroundScene s = make_scene(ctx);
    RigidBody *left = add_box(s, 0.4f, {-3.5f, 1.39f, 0.5f}, 7);
    step(*s.scene.world);
    RigidBody *right = add_box(s, 0.4f, {3.5f, 1.39f, 0.5f}, 8);
    step(*s.scene.world);
    const ContactPairs &pairs = s.scene.world->contact_pairs;
    CHECK(ctx, pairs.active.length == 2);
    const uint32_t left_handle = active_pair(pairs, 0).a.body_id == left->id || active_pair(pairs, 0).b.body_id == left->id ? pairs.active[0]
                                                                                                                              : pairs.active[1];
    const uint32_t right_handle = left_handle == pairs.active[0] ? pairs.active[1] : pairs.active[0];

    // lifted out of reach, in the reverse order of adding
    rigid_body::set_transform(*s.scene.world, right, vec3f::make(3.5f, 20.0f, 0.5f), quatf::identity(), false);
    rigid_body::set_transform(*s.scene.world, left, vec3f::make(-3.5f, 20.0f, 0.5f), quatf::identity(), false);
    step(*s.scene.world);
    CHECK(ctx, pairs.active.length == 0 && pairs.removed.length == 2);
    CHECK(ctx, pairs.removed[0] == left_handle && pairs.removed[1] == right_handle);
    CHECK(ctx, pairs.live == 2);

    // freed by the next begin; a pair added after takes a freed handle
    step(*s.scene.world);
    CHECK(ctx, pairs.removed.length == 0 && pairs.live == 0 && pairs.free.length == 2);
    rigid_body::set_transform(*s.scene.world, left, vec3f::make(-3.5f, 1.39f, 0.5f), quatf::identity(), false);
    step(*s.scene.world);
    CHECK(ctx, pairs.active.length == 1 && active_pair(pairs, 0).added);
    CHECK(ctx, pairs.active[0] == left_handle || pairs.active[0] == right_handle);
    release_scene(s);
}

void test_contact_pairs_a_pair_recorded_twice_in_a_step_is_one(TestCtx &ctx) {
    // the step's events recorded a second time, as a ccd contact repeats one: still one pair
    GroundScene s = make_scene(ctx);
    add_box(s, 0.4f, {0.5f, 1.39f, 0.5f}, 7);
    step(*s.scene.world);
    contact_pairs::begin(*s.scene.world);
    contact_pairs::record_world_contacts(*s.scene.world, 0, 0);
    contact_pairs::record_world_contacts(*s.scene.world, 0, 0);
    contact_pairs::end(*s.scene.world);
    const ContactPairs &pairs = s.scene.world->contact_pairs;
    CHECK(ctx, pairs.active.length == 1 && pairs.removed.length == 0 && pairs.live == 1);
    release_scene(s);
}

void test_contact_pairs_only_a_pair_a_side_observes_is_recorded(TestCtx &ctx) {
    // a box on the ground and one on it, neither observing: no pairs. the upper observing: its pair with the lower, not the
    // lower's with the ground. the lower too: both. both cleared: every pair removed
    GroundScene s = make_scene(ctx);
    RigidBody *lower = add_box(s, 0.4f, {0.5f, 1.39f, 0.5f}, 7, false);
    RigidBody *upper = add_box(s, 0.4f, {0.5f, 2.18f, 0.5f}, 8, false);
    const ContactPairs &pairs = s.scene.world->contact_pairs;
    step(*s.scene.world);
    CHECK(ctx, pairs.active.length == 0 && pairs.live == 0);

    upper->observes_contacts = true;
    step(*s.scene.world);
    CHECK(ctx, pairs.active.length == 1 && count_pairs(pairs, *lower, *upper) == 1 && active_pair(pairs, 0).added);

    lower->observes_contacts = true;
    step(*s.scene.world);
    CHECK(ctx, pairs.active.length == 2 && count_pairs(pairs, *lower, *upper) == 1 && count_pairs(pairs, *lower, *s.ground) == 1);

    upper->observes_contacts = false;
    lower->observes_contacts = false;
    step(*s.scene.world);
    CHECK(ctx, pairs.active.length == 0 && pairs.removed.length == 2);
    release_scene(s);
}

void test_contact_pairs_a_body_made_again_is_a_new_pair(TestCtx &ctx) {
    // pairs are keyed by body id: a box removed and made again in its place is a new body, so its old pair is removed
    // and the new one added, even with the same user data
    GroundScene s = make_scene(ctx);
    RigidBody *box = add_box(s, 0.4f, {0.5f, 1.39f, 0.5f}, 7);
    step(*s.scene.world);
    const ContactPairs &pairs = s.scene.world->contact_pairs;
    CHECK(ctx, pairs.active.length == 1);
    const body_id::BodyId old_id = box->id;
    CHECK(ctx, rigid_body::remove(*s.scene.world, box));
    RigidBody *again = add_box(s, 0.4f, {0.5f, 1.39f, 0.5f}, 7);
    CHECK(ctx, again->id != old_id);
    step(*s.scene.world);
    CHECK(ctx, pairs.active.length == 1 && active_pair(pairs, 0).added && count_pairs(pairs, *again, *s.ground) == 1);
    CHECK(ctx, pairs.removed.length == 1);
    release_scene(s);
}
