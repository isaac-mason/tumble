#include "tumble/physics/pairs.h"

#include "tumble/physics/body/sleep.h"
#include "tumble/physics/contacts.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "test.h"
#include "test_math.h"

using rigid_body::RigidBody;

namespace {

constexpr float SPEC = 0.02f;

struct World {
    layers::Layers layers;
    bodies::Bodies bodies;
    broadphase::Broadphase broadphase;
    pairs::Pairs pairs;
    contacts::Contacts contacts;
    events::PhysicsEvents events;
};

/** a world over `layers`, which it takes */
World make_world(const layers::Layers &layers) {
    World world = {};
    world.layers = layers;
    world.bodies = bodies::init();
    world.broadphase = broadphase::init(world.layers);
    world.pairs = pairs::create();
    world.contacts = contacts::create();
    world.events = events::create_events();
    return world;
}

/** a world with a single object layer that collides with itself */
World make_single_layer_world(uint32_t &layer) {
    layers::Layers layers = layers::create();
    const uint32_t broadphase_layer = layers::add_broadphase_layer(layers);
    layer = layers::add_object_layer(layers, broadphase_layer);
    layers::enable_collision(layers, layer, layer);
    return make_world(layers);
}

void release_world(World &world) {
    events::release(world.events);
    contacts::release(world.contacts);
    pairs::release(world.pairs);
    broadphase::release(world.broadphase);
    bodies::release(world.bodies);
    layers::release(world.layers);
}

struct BoxOptions {
    motion_type::MotionType motion_type = motion_type::MOTION_DYNAMIC;
    bool sensor = false;
    bool collide_kinematic_vs_non_dynamic = false;
    uint64_t user_data = 0;
};

/** in the broadphase and marked moved */
RigidBody *add_body(World &world, shapes::Shape *shape, uint32_t layer, const Float3 &position, const BoxOptions &options = {}) {
    RigidBody *body = rigid_body::create(world.bodies, world.layers,
                                         {.shape = shape,
                                          .object_layer = layer,
                                          .motion_type = options.motion_type,
                                          .user_data = options.user_data,
                                          .position = position,
                                          .sensor = options.sensor,
                                          .collide_kinematic_vs_non_dynamic = options.collide_kinematic_vs_non_dynamic,
                                          .mass = options.motion_type == motion_type::MOTION_DYNAMIC ? 1.0f : -1.0f});
    broadphase::add_body(world.broadphase, body, world.layers);
    pairs::mark_moved(world.pairs, *body);
    return body;
}

/** pairs purged, out of the broadphase, slot freed */
void remove_body(World &world, RigidBody *body) {
    pairs::purge_body_pairs(world.pairs, world.contacts, world.bodies, *body, world.events);
    broadphase::remove_body(world.broadphase, body);
    rigid_body::remove(world.bodies, body);
}

/** the leaf refreshed, the body marked moved if it escaped */
void set_position(World &world, RigidBody *body, const Float3 &position, bool wake) {
    rigid_body::set_position(world.bodies, body, vec3f::load(position), wake);
    if (broadphase::update_body(world.broadphase, body)) pairs::mark_moved(world.pairs, *body);
}

void find_pairs(World &world) {
    world.events.pairs.length = 0;
    pairs::find_colliding_pairs(world.pairs, world.contacts, world.broadphase, world.layers, world.bodies, SPEC, world.events);
}

/** whether the emitted pairs hold (a, b) in either order */
bool emitted(const World &world, const RigidBody *a, const RigidBody *b) {
    for (uint32_t i = 0; i < world.pairs.colliding_pairs.length; i++) {
        const int32_t x = (int32_t)world.pairs.colliding_pairs[i].body_index_a;
        const int32_t y = (int32_t)world.pairs.colliding_pairs[i].body_index_b;
        if ((x == (int32_t)a->index && y == (int32_t)b->index) || (x == (int32_t)b->index && y == (int32_t)a->index)) return true;
    }
    return false;
}

/** the number of records in the body's intrusive pair list */
uint32_t pair_list_length(const World &world, const RigidBody *body) {
    uint32_t count = 0;
    int32_t edge_key = body->head_pair_key;
    while (edge_key != pairs::INVALID_PAIR_KEY) {
        count++;
        const int32_t record = pairs::get_pair_edge_record(edge_key);
        const int32_t side = pairs::get_pair_edge_side(edge_key);
        edge_key = world.pairs.records[(uint32_t)record].edges[side].next;
    }
    return count;
}

struct PairBodySpec {
    motion_type::MotionType motion_type;
    bool sleeping;
    bool sensor;
    bool opt_in;
};

/** the pair-reporting contract, written from the intended behaviour, not from the implementation */
bool reference_should_report(const PairBodySpec &a, const PairBodySpec &b, bool layers_collide) {
    // at least one body must query the broadphase (awake and non-static)
    const bool queries_a = a.motion_type != motion_type::MOTION_STATIC && !a.sleeping;
    const bool queries_b = b.motion_type != motion_type::MOTION_STATIC && !b.sleeping;
    if (!queries_a && !queries_b) return false;

    // object layer pair table must allow the pair
    if (!layers_collide) return false;

    // motion type / sensor gate: opt-in, or a dynamic body, or kinematic-meets-sensor
    return a.opt_in || b.opt_in || a.motion_type == motion_type::MOTION_DYNAMIC || b.motion_type == motion_type::MOTION_DYNAMIC ||
           (a.motion_type == motion_type::MOTION_KINEMATIC && b.sensor) || (b.motion_type == motion_type::MOTION_KINEMATIC && a.sensor);
}

uint32_t run_combo(shapes::Shape *shape, const PairBodySpec &a, const PairBodySpec &b, bool layers_collide) {
    layers::Layers layers = layers::create();
    const uint32_t broadphase_layer = layers::add_broadphase_layer(layers);
    const uint32_t layer_a = layers::add_object_layer(layers, broadphase_layer);
    const uint32_t layer_b = layers::add_object_layer(layers, broadphase_layer);
    if (layers_collide) layers::enable_collision(layers, layer_a, layer_b);
    World world = make_world(layers);

    RigidBody *body_a = add_body(world, shape, layer_a, {0, 0, 0}, {a.motion_type, a.sensor, a.opt_in});
    RigidBody *body_b = add_body(world, shape, layer_b, {0.4f, 0, 0}, {b.motion_type, b.sensor, b.opt_in});
    if (a.sleeping) rigid_body::sleep(world.bodies, body_a);
    if (b.sleeping) rigid_body::sleep(world.bodies, body_b);

    find_pairs(world);
    const uint32_t count = world.pairs.colliding_pairs.length;
    release_world(world);
    return count;
}

}  // namespace

void test_pairs_edge_keys(TestCtx &ctx) {
    CHECK(ctx, pairs::pair_edge_key(0, 0) == 0 && pairs::pair_edge_key(0, 1) == 1 && pairs::pair_edge_key(5, 1) == 11);
    CHECK(ctx, pairs::get_pair_edge_record(pairs::pair_edge_key(123, 1)) == 123);
    CHECK(ctx, pairs::get_pair_edge_side(pairs::pair_edge_key(123, 1)) == 1);
    CHECK(ctx, pairs::get_pair_edge_side(pairs::pair_edge_key(123, 0)) == 0);
}

void test_pairs_reporting_truth_table(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    const motion_type::MotionType motion_types[] = {motion_type::MOTION_STATIC, motion_type::MOTION_KINEMATIC, motion_type::MOTION_DYNAMIC};
    uint32_t combos = 0;
    uint32_t mismatches = 0;
    for (motion_type::MotionType motion_a : motion_types) {
        for (motion_type::MotionType motion_b : motion_types) {
            for (uint32_t bits = 0; bits < 128; bits++) {
                const PairBodySpec a = {motion_a, (bits & 1) != 0, (bits & 4) != 0, (bits & 16) != 0};
                const PairBodySpec b = {motion_b, (bits & 2) != 0, (bits & 8) != 0, (bits & 32) != 0};
                const bool layers_collide = (bits & 64) != 0;
                // sleeping is only reachable for dynamic bodies
                if ((a.sleeping && motion_a != motion_type::MOTION_DYNAMIC) || (b.sleeping && motion_b != motion_type::MOTION_DYNAMIC)) continue;

                const uint32_t expected = reference_should_report(a, b, layers_collide) ? 1 : 0;
                if (run_combo(shape, a, b, layers_collide) != expected) mismatches++;
                combos++;
            }
        }
    }
    CHECK(ctx, mismatches == 0);
    CHECK(ctx, combos > 500);

    box::destroy(shape);
}

void test_pairs_cluster_records_and_events(TestCtx &ctx) {
    dispatch::init();
    uint32_t layer;
    World world = make_single_layer_world(layer);
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    // three mutually overlapping boxes and a fourth far away
    RigidBody *a = add_body(world, shape, layer, {0, 0, 0}, {.user_data = 101});
    RigidBody *b = add_body(world, shape, layer, {0.4f, 0, 0}, {.user_data = 102});
    RigidBody *c = add_body(world, shape, layer, {0.2f, 0.4f, 0}, {.user_data = 103});
    RigidBody *far = add_body(world, shape, layer, {20, 0, 0}, {.user_data = 104});
    CHECK(ctx, world.pairs.moved.length == 4 && a->in_move_set && far->in_move_set);

    find_pairs(world);

    // one record and one emitted pair per overlapping pair; the move set consumed
    CHECK(ctx, world.pairs.record_count == 3 && world.pairs.colliding_pairs.length == 3);
    CHECK(ctx, emitted(world, a, b) && emitted(world, a, c) && emitted(world, b, c));
    CHECK(ctx, pair_list_length(world, a) == 2 && pair_list_length(world, b) == 2 && pair_list_length(world, c) == 2);
    CHECK(ctx, pair_list_length(world, far) == 0);
    CHECK(ctx, world.pairs.moved.length == 0 && !a->in_move_set && !far->in_move_set);

    // one event per emitted pair, same order: the record, the bodies (lower id first), their ids and nodes
    CHECK(ctx, world.events.pairs.length == 3);
    int32_t records[3];
    for (uint32_t i = 0; i < world.events.pairs.length; i++) {
        const events::PairEvent &event = world.events.pairs[i];
        const RigidBody *body_a = world.bodies.pool[event.body_index_a];
        const RigidBody *body_b = world.bodies.pool[event.body_index_b];
        records[i] = event.pair_record;
        CHECK(ctx, event.pair_record == world.pairs.colliding_pairs[i].pair_record);
        CHECK(ctx, event.pair_record == pairs::find_pair_record(world.pairs, *body_a, *body_b));
        CHECK(ctx, event.body_id_a == body_a->id && event.body_id_b == body_b->id && event.body_id_a < event.body_id_b);
        CHECK(ctx, event.user_data_a == body_a->user_data && event.user_data_b == body_b->user_data);
        CHECK(ctx, event.user_data_a >= 101 && event.user_data_a <= 103 && event.user_data_b > event.user_data_a && event.user_data_b <= 103);
    }

    // the next step finds the same records, and none new
    find_pairs(world);
    CHECK(ctx, world.pairs.record_count == 3 && world.events.pairs.length == 3);
    for (uint32_t i = 0; i < world.events.pairs.length; i++) CHECK(ctx, world.events.pairs[i].pair_record == records[i]);

    box::destroy(shape);
    release_world(world);
}

void test_pairs_record_survives_while_overlapping(TestCtx &ctx) {
    dispatch::init();
    uint32_t layer;
    World world = make_single_layer_world(layer);
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    RigidBody *a = add_body(world, shape, layer, {0, 0, 0});
    RigidBody *b = add_body(world, shape, layer, {0.4f, 0, 0});
    find_pairs(world);
    const int32_t record = pairs::find_pair_record(world.pairs, *a, *b);
    CHECK(ctx, record != pairs::INVALID_PAIR_KEY && world.pairs.record_count == 1);

    // b slides along y, escaping its fat leaf more than once, while the boxes keep overlapping: the same record, every step
    bool same_record = true;
    bool escaped = false;
    for (uint32_t step = 1; step <= 20; step++) {
        set_position(world, b, {0.4f, 0.04f * (float)step, 0}, false);
        escaped = escaped || b->in_move_set;
        find_pairs(world);
        same_record = same_record && world.pairs.record_count == 1 && pairs::find_pair_record(world.pairs, *a, *b) == record &&
                      pairs::find_pair_record(world.pairs, *b, *a) == record && world.pairs.colliding_pairs.length == 1 &&
                      world.events.pairs.length == 1 && world.events.pairs[0].pair_record == record;
    }
    CHECK(ctx, escaped);
    CHECK(ctx, same_record);

    // once they part beyond the fat boxes the record goes
    set_position(world, b, {5, 0, 0}, false);
    find_pairs(world);
    CHECK(ctx, world.pairs.record_count == 0 && world.pairs.colliding_pairs.length == 0 && world.events.pairs.length == 0);
    CHECK(ctx, a->head_pair_key == pairs::INVALID_PAIR_KEY && b->head_pair_key == pairs::INVALID_PAIR_KEY);

    box::destroy(shape);
    release_world(world);
}

void test_pairs_persist_through_sleep(TestCtx &ctx) {
    dispatch::init();
    uint32_t layer;
    World world = make_single_layer_world(layer);
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    RigidBody *a = add_body(world, shape, layer, {0, 0, 0});
    RigidBody *b = add_body(world, shape, layer, {0.4f, 0, 0});

    // the first pass discovers and emits the pair, and the record persists
    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 1 && world.pairs.record_count == 1);

    // both asleep: the record persists but is not emitted (both have INACTIVE_BODY_INDEX)
    rigid_body::sleep(world.bodies, a);
    rigid_body::sleep(world.bodies, b);
    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 0 && world.pairs.record_count == 1 && world.events.pairs.length == 0);

    // waking one (without moving it) re-emits the persisted pair
    rigid_body::wake(world.bodies, a);
    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 1 && world.events.pairs.length == 1);

    box::destroy(shape);
    release_world(world);
}

void test_pairs_teleport_sleeping_destroys_record(TestCtx &ctx) {
    dispatch::init();
    uint32_t layer;
    World world = make_single_layer_world(layer);
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    RigidBody *a = add_body(world, shape, layer, {0, 0, 0});
    RigidBody *b = add_body(world, shape, layer, {0.4f, 0, 0});
    find_pairs(world);
    CHECK(ctx, world.pairs.record_count == 1);

    rigid_body::sleep(world.bodies, a);
    rigid_body::sleep(world.bodies, b);
    find_pairs(world);
    CHECK(ctx, world.pairs.record_count == 1 && world.pairs.colliding_pairs.length == 0);

    // a sleeping body teleported away without waking: the moved flag routes the pair past the sleeping-pair skip, so the
    // separated record is destroyed this step
    set_position(world, a, {100, 0, 0}, false);
    find_pairs(world);
    CHECK(ctx, world.pairs.record_count == 0);
    CHECK(ctx, world.pairs.free_records.length == 1 && world.pairs.records[0].body_index[0] == pairs::INVALID_PAIR_KEY);

    box::destroy(shape);
    release_world(world);
}

void test_pairs_teleport_sleeping_discovers_pair(TestCtx &ctx) {
    dispatch::init();
    uint32_t layer;
    World world = make_single_layer_world(layer);
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    RigidBody *a = add_body(world, shape, layer, {0, 0, 0});
    RigidBody *b = add_body(world, shape, layer, {50, 0, 0});
    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 0 && world.pairs.record_count == 0);
    rigid_body::sleep(world.bodies, a);
    rigid_body::sleep(world.bodies, b);

    // a sleeping body teleported onto another without waking still discovers the overlap
    set_position(world, a, {50.4f, 0, 0}, false);
    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 0 && world.pairs.record_count == 1);

    // waking one emits it
    rigid_body::wake(world.bodies, a);
    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 1);

    box::destroy(shape);
    release_world(world);
}

void test_pairs_removal_purges(TestCtx &ctx) {
    dispatch::init();
    uint32_t layer;
    World world = make_single_layer_world(layer);
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    RigidBody *a = add_body(world, shape, layer, {0, 0, 0});
    RigidBody *b = add_body(world, shape, layer, {0.4f, 0, 0});
    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 1 && world.pairs.record_count == 1);

    // removing one body of the pair purges its record from both lists
    remove_body(world, b);
    CHECK(ctx, world.pairs.record_count == 0 && a->head_pair_key == pairs::INVALID_PAIR_KEY);

    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 0 && world.events.pairs.length == 0);
    CHECK(ctx, world.bodies.sequences[a->index] != bodies::SLOT_FREE);

    // a body marked moved and then removed in the same step is skipped by discovery
    RigidBody *c = add_body(world, shape, layer, {-0.4f, 0, 0});
    remove_body(world, c);
    find_pairs(world);
    CHECK(ctx, world.pairs.record_count == 0 && world.pairs.moved.length == 0);

    box::destroy(shape);
    release_world(world);
}

void test_pairs_recycled_bodies_stay_distinct(TestCtx &ctx) {
    dispatch::init();
    uint32_t layer;
    World world = make_single_layer_world(layer);
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    // recycle three slots so the ids carry sequences above the index bits
    RigidBody *scratch[3] = {add_body(world, shape, layer, {100, 0, 0}), add_body(world, shape, layer, {103, 0, 0}),
                             add_body(world, shape, layer, {106, 0, 0})};
    for (RigidBody *body : scratch) remove_body(world, body);

    // all three pairwise overlapping: three distinct pairs
    RigidBody *cluster[3] = {add_body(world, shape, layer, {0, 0, 0}), add_body(world, shape, layer, {0.4f, 0, 0}),
                             add_body(world, shape, layer, {-0.4f, 0, 0})};
    bool high_ids = true;
    for (RigidBody *body : cluster) high_ids = high_ids && body->id > body_id::MAX_BODY_INDEX;
    CHECK(ctx, high_ids);

    find_pairs(world);
    CHECK(ctx, world.pairs.record_count == 3 && world.pairs.colliding_pairs.length == 3);

    box::destroy(shape);
    release_world(world);
}

void test_pairs_hub_with_spokes(TestCtx &ctx) {
    dispatch::init();
    uint32_t layer;
    World world = make_single_layer_world(layer);
    shapes::Shape *hub_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    shapes::Shape *spoke_shape = box::create({.half_extents = {0.1f, 0.1f, 0.1f}});

    RigidBody *hub = add_body(world, hub_shape, layer, {0, 0, 0});
    // four spokes overlapping only the hub, pairwise far apart even with fat margins: one record each
    RigidBody *spokes[4] = {add_body(world, spoke_shape, layer, {0.58f, 0, 0}), add_body(world, spoke_shape, layer, {-0.58f, 0, 0}),
                            add_body(world, spoke_shape, layer, {0, 0.58f, 0}), add_body(world, spoke_shape, layer, {0, -0.58f, 0})};

    find_pairs(world);
    CHECK(ctx, world.pairs.record_count == 4 && world.pairs.colliding_pairs.length == 4 && pair_list_length(world, hub) == 4);

    // removing two spokes unlinks their records from the hub's list (one the head, one interior)
    remove_body(world, spokes[0]);
    remove_body(world, spokes[1]);
    CHECK(ctx, world.pairs.record_count == 2 && pair_list_length(world, hub) == 2 && world.pairs.free_records.length == 2);

    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 2 && emitted(world, hub, spokes[2]) && emitted(world, hub, spokes[3]));

    // a new overlapping body reuses a freed record slot
    const uint32_t record_slots = world.pairs.records.length;
    RigidBody *replacement = add_body(world, spoke_shape, layer, {0.58f, 0, 0});
    find_pairs(world);
    CHECK(ctx, world.pairs.record_count == 3 && world.pairs.colliding_pairs.length == 3 && emitted(world, hub, replacement));
    CHECK(ctx, world.pairs.records.length == record_slots && world.pairs.free_records.length == 1);
    CHECK(ctx, pair_list_length(world, hub) == 3);

    box::destroy(hub_shape);
    box::destroy(spoke_shape);
    release_world(world);
}

void test_pairs_pose_cache(TestCtx &ctx) {
    dispatch::init();
    uint32_t layer;
    World world = make_single_layer_world(layer);
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    RigidBody *a = add_body(world, shape, layer, {0, 0, 0});
    RigidBody *b = add_body(world, shape, layer, {0.4f, 0, 0});
    const int32_t record = pairs::add_pair_record(world.pairs, *a, *b);
    const pairs::PoseCache &cache = world.pairs.pose_cache[(uint32_t)record];
    CHECK(ctx, !cache.valid && world.pairs.first_contact[(uint32_t)record] == contacts::INVALID_CONTACT_KEY);

    pairs::set_cache(world.pairs, record, vec3f::make(1, 2, 3), quatf::make(0, 0, 0.6f, 0.8f));
    CHECK(ctx, cache.valid);
    CHECK(ctx, cache.delta_position.x == 1 && cache.delta_position.y == 2 && cache.delta_position.z == 3);
    CHECK(ctx, cache.delta_rotation.z == 0.6f && cache.delta_rotation.w == 0.8f);

    // a reused record starts invalid
    remove_body(world, b);
    RigidBody *c = add_body(world, shape, layer, {0.4f, 0, 0});
    const int32_t reused = pairs::add_pair_record(world.pairs, *a, *c);
    CHECK(ctx, reused == record && !world.pairs.pose_cache[(uint32_t)reused].valid);
    CHECK(ctx, pairs::find_pair_record(world.pairs, *c, *a) == reused);

    box::destroy(shape);
    release_world(world);
}
