#include "tumble/physics/contacts.h"

#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

using contacts::Contact;
using contacts::INVALID_CONTACT_KEY;
using rigid_body::RigidBody;

namespace {

/** a moving and a not-moving layer, each with its own broadphase layer */
struct World {
    layers::Layers layers;
    bodies::Bodies bodies;
    broadphase::Broadphase broadphase;
    pairs::Pairs pairs;
    contacts::Contacts contacts;
    events::PhysicsEvents events;
    uint32_t moving;
    uint32_t not_moving;
    shapes::Shape *sphere;
};

World make_world() {
    dispatch::init();
    World world = {};
    const uint32_t broadphase_moving = layers::add_broadphase_layer(world.layers);
    const uint32_t broadphase_not_moving = layers::add_broadphase_layer(world.layers);
    world.moving = layers::add_object_layer(world.layers, broadphase_moving);
    world.not_moving = layers::add_object_layer(world.layers, broadphase_not_moving);
    layers::enable_collision(world.layers, world.moving, world.moving);
    layers::enable_collision(world.layers, world.moving, world.not_moving);
    world.bodies = bodies::init();
    world.broadphase = broadphase::init(world.layers);
    world.pairs = pairs::create();
    world.contacts = contacts::create();
    world.events = events::create_events();
    world.sphere = sphere::create({.radius = 1});
    return world;
}

void release_world(World &world) {
    sphere::destroy(world.sphere);
    events::release(world.events);
    contacts::release(world.contacts);
    pairs::release(world.pairs);
    broadphase::release(world.broadphase);
    bodies::release(world.bodies);
    layers::release(world.layers);
}

/** a unit sphere, dynamic, in the moving layer, in the broadphase and marked moved */
RigidBody *make_body(World &world, const Float3 &position = {0, 0, 0}, motion_type::MotionType motion_type = motion_type::MOTION_DYNAMIC) {
    RigidBody *body = rigid_body::create(world.bodies, world.layers,
                                         {.shape = world.sphere,
                                          .object_layer = motion_type == motion_type::MOTION_STATIC ? world.not_moving : world.moving,
                                          .motion_type = motion_type,
                                          .position = position});
    broadphase::add_body(world.broadphase, body, world.layers);
    pairs::mark_moved(world.pairs, *body);
    return body;
}

uint32_t active_contact_count(const contacts::Contacts &contacts) { return contacts.contacts.length - contacts.free_indices.length; }

/** the contact indices of a record's chain, head first */
uint32_t chain(const World &world, int32_t record, int32_t *out, uint32_t capacity) {
    uint32_t count = 0;
    int32_t contact_index = world.pairs.first_contact[(uint32_t)record];
    while (contact_index != INVALID_CONTACT_KEY) {
        if (count < capacity) out[count] = contact_index;
        count++;
        contact_index = world.contacts.contacts[(uint32_t)contact_index].next_in_pair;
    }
    return count;
}

/** every chain link reciprocal, every contact pointing at its record, the head's prev empty, no freed contact in a chain */
bool chains_consistent(const World &world, const int32_t *records, uint32_t num_records) {
    for (uint32_t r = 0; r < num_records; r++) {
        int32_t contact_index = world.pairs.first_contact[(uint32_t)records[r]];
        int32_t previous = INVALID_CONTACT_KEY;
        while (contact_index != INVALID_CONTACT_KEY) {
            const Contact &contact = world.contacts.contacts[(uint32_t)contact_index];
            if (contact.contact_index != contact_index || contact.pair_record != records[r] || contact.prev_in_pair != previous) return false;
            for (uint32_t f = 0; f < world.contacts.free_indices.length; f++) {
                if (world.contacts.free_indices[f] == contact_index) return false;
            }
            previous = contact_index;
            contact_index = contact.next_in_pair;
        }
    }
    return true;
}

bool removed_event_is(const events::ContactRemovedEvent &event, const RigidBody *a, const RigidBody *b, uint32_t sub_shape_a,
                      uint32_t sub_shape_b) {
    return event.body_id_a == a->id && event.body_id_b == b->id && event.sub_shape_id_a == sub_shape_a && event.sub_shape_id_b == sub_shape_b;
}

void find_pairs(World &world) {
    world.events.pairs.length = 0;
    pairs::find_colliding_pairs(world.pairs, world.contacts, world.broadphase, world.layers, world.bodies, 0.02f, world.events);
}

}  // namespace

void test_contacts_create_empty(TestCtx &ctx) {
    const contacts::Contacts contacts = contacts::create();
    CHECK(ctx, contacts.contacts.length == 0 && contacts.free_indices.length == 0);
    CHECK(ctx, contacts.frame_stamp == 0 && contacts.read_index == 0 && active_contact_count(contacts) == 0);
}

void test_contacts_create_nested_under_record(TestCtx &ctx) {
    World world = make_world();
    RigidBody *a = make_body(world);
    RigidBody *b = make_body(world);
    const int32_t record = pairs::add_pair_record(world.pairs, *a, *b);

    Contact *contact = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 0, 0);
    CHECK(ctx, contact != nullptr && contact->contact_index == 0);
    CHECK(ctx, contact->body_id_a == a->id && contact->body_id_b == b->id);
    CHECK(ctx, contact->body_index_a == a->index && contact->body_index_b == b->index);
    CHECK(ctx, contact->sub_shape_id_a == 0 && contact->sub_shape_id_b == 0 && contact->last_processed_frame == 0);
    CHECK(ctx, contacts::get_read_manifold(*contact, world.contacts).num_contact_points == 0);
    CHECK(ctx, contacts::get_write_manifold(*contact, world.contacts).num_contact_points == 0);
    CHECK(ctx, world.contacts.contacts.length == 1 && active_contact_count(world.contacts) == 1);

    // nested under the record: the chain's head, empty links, the record set
    CHECK(ctx, world.pairs.first_contact[(uint32_t)record] == contact->contact_index && contact->pair_record == record);
    CHECK(ctx, contact->prev_in_pair == INVALID_CONTACT_KEY && contact->next_in_pair == INVALID_CONTACT_KEY);

    release_world(world);
}

void test_contacts_destroy_reports_and_empties_chain(TestCtx &ctx) {
    World world = make_world();
    RigidBody *a = make_body(world);
    RigidBody *b = make_body(world);
    const int32_t record = pairs::add_pair_record(world.pairs, *a, *b);

    Contact *contact = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 3, 7);
    contacts::destroy_contact(world.contacts, world.pairs, *contact, world.events);

    CHECK(ctx, world.pairs.first_contact[(uint32_t)record] == INVALID_CONTACT_KEY);
    CHECK(ctx, contact->contact_index == INVALID_CONTACT_KEY && contact->pair_record == pairs::INVALID_PAIR_KEY);
    CHECK(ctx, active_contact_count(world.contacts) == 0);
    CHECK(ctx, world.events.contacts_removed.length == 1 && removed_event_is(world.events.contacts_removed[0], a, b, 3, 7));

    release_world(world);
}

void test_contacts_reuse_freed_index(TestCtx &ctx) {
    World world = make_world();
    RigidBody *a = make_body(world);
    RigidBody *b = make_body(world);
    const int32_t record = pairs::add_pair_record(world.pairs, *a, *b);

    Contact *first = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 0, 0);
    const int32_t first_index = first->contact_index;
    contacts::get_write_manifold(*first, world.contacts).num_contact_points = 3;
    contacts::get_write_manifold(*first, world.contacts).contact_points[0].normal_lambda = 5;
    contacts::destroy_contact(world.contacts, world.pairs, *first, world.events);

    // the freed slot comes back, its manifolds reset
    Contact *second = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 1, 1);
    CHECK(ctx, second->contact_index == first_index && world.contacts.contacts.length == 1);
    CHECK(ctx, contacts::get_write_manifold(*second, world.contacts).num_contact_points == 0);
    CHECK(ctx, contacts::get_write_manifold(*second, world.contacts).contact_points[0].normal_lambda == 0);

    release_world(world);
}

void test_contacts_manifold_double_buffer(TestCtx &ctx) {
    World world = make_world();
    RigidBody *a = make_body(world);
    RigidBody *b = make_body(world);
    const int32_t record = pairs::add_pair_record(world.pairs, *a, *b);
    Contact *contact = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 0, 0);

    // this step's write is the next step's read
    contacts::get_write_manifold(*contact, world.contacts).num_contact_points = 2;
    CHECK(ctx, contacts::get_read_manifold(*contact, world.contacts).num_contact_points == 0);
    contacts::flip_manifold_cache(world.contacts);
    CHECK(ctx, world.contacts.read_index == 1);
    CHECK(ctx, contacts::get_read_manifold(*contact, world.contacts).num_contact_points == 2);
    CHECK(ctx, &contacts::get_write_manifold(*contact, world.contacts) == &contact->manifolds[0]);

    release_world(world);
}

void test_contacts_chain_middle_destroy(TestCtx &ctx) {
    World world = make_world();
    RigidBody *a = make_body(world);
    RigidBody *b = make_body(world);
    const int32_t record = pairs::add_pair_record(world.pairs, *a, *b);

    const int32_t c0 = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 0, 0)->contact_index;
    const int32_t c1 = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 1, 1)->contact_index;
    const int32_t c2 = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 2, 2)->contact_index;

    // head first: c2 -> c1 -> c0
    int32_t links[4];
    CHECK(ctx, chain(world, record, links, 4) == 3 && links[0] == c2 && links[1] == c1 && links[2] == c0);

    contacts::destroy_contact(world.contacts, world.pairs, world.contacts.contacts[(uint32_t)c1], world.events);
    CHECK(ctx, chain(world, record, links, 4) == 2 && links[0] == c2 && links[1] == c0);
    CHECK(ctx, world.contacts.contacts[(uint32_t)c2].next_in_pair == c0 && world.contacts.contacts[(uint32_t)c0].prev_in_pair == c2);

    release_world(world);
}

void test_contacts_find_in_pair(TestCtx &ctx) {
    World world = make_world();
    RigidBody *a = make_body(world);
    RigidBody *b = make_body(world);
    const int32_t record = pairs::add_pair_record(world.pairs, *a, *b);

    CHECK(ctx, contacts::find_contact_in_pair(world.contacts, world.pairs, record, 5, 10) == nullptr);
    const int32_t c0 = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 0, 0)->contact_index;
    const int32_t c1 = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 1, 1)->contact_index;
    const int32_t c2 = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 5, 10)->contact_index;

    Contact *found = contacts::find_contact_in_pair(world.contacts, world.pairs, record, 5, 10);
    CHECK(ctx, found != nullptr && found->contact_index == c2);
    found = contacts::find_contact_in_pair(world.contacts, world.pairs, record, 0, 0);
    CHECK(ctx, found != nullptr && found->contact_index == c0);
    found = contacts::find_contact_in_pair(world.contacts, world.pairs, record, 1, 1);
    CHECK(ctx, found != nullptr && found->contact_index == c1);
    CHECK(ctx, contacts::find_contact_in_pair(world.contacts, world.pairs, record, 2, 2) == nullptr);
    CHECK(ctx, contacts::find_contact_in_pair(world.contacts, world.pairs, record, 10, 5) == nullptr);

    release_world(world);
}

void test_contacts_destroy_stale_in_pair(TestCtx &ctx) {
    World world = make_world();
    world.contacts.frame_stamp = 5;
    RigidBody *a = make_body(world);
    RigidBody *b = make_body(world);
    const int32_t record = pairs::add_pair_record(world.pairs, *a, *b);

    Contact *fresh = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 0, 0);
    fresh->last_processed_frame = 5;
    const int32_t fresh_index = fresh->contact_index;
    Contact *stale = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 1, 1);
    stale->last_processed_frame = 4;
    const int32_t stale_index = stale->contact_index;

    contacts::destroy_stale_contacts_in_pair(world.contacts, world.pairs, record, world.events);

    int32_t links[2];
    CHECK(ctx, chain(world, record, links, 2) == 1 && links[0] == fresh_index);
    CHECK(ctx, world.contacts.contacts[(uint32_t)stale_index].contact_index == INVALID_CONTACT_KEY);
    CHECK(ctx, world.events.contacts_removed.length == 1 && removed_event_is(world.events.contacts_removed[0], a, b, 1, 1));

    release_world(world);
}

void test_contacts_destroy_pair_chain_immediate_and_queued(TestCtx &ctx) {
    World world = make_world();
    RigidBody *a = make_body(world);
    RigidBody *b = make_body(world);
    const int32_t record = pairs::add_pair_record(world.pairs, *a, *b);

    // reported now
    contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 0, 0);
    contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 1, 1);
    contacts::destroy_pair_chain(world.contacts, world.pairs, record, world.events, false);
    CHECK(ctx, world.pairs.first_contact[(uint32_t)record] == INVALID_CONTACT_KEY && active_contact_count(world.contacts) == 0);
    CHECK(ctx, world.events.contacts_removed.length == 2 && world.contacts.pending_contact_removed.length == 0);

    // queued: nothing reported, one entry pending, reported by the next flush
    world.events.contacts_removed.length = 0;
    contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 2, 3);
    contacts::destroy_pair_chain(world.contacts, world.pairs, record, world.events, true);
    CHECK(ctx, world.events.contacts_removed.length == 0 && world.contacts.pending_contact_removed.length == 4);
    contacts::flush_pending_contact_removed(world.contacts, world.events);
    CHECK(ctx, world.contacts.pending_contact_removed.length == 0);
    CHECK(ctx, world.events.contacts_removed.length == 1 && removed_event_is(world.events.contacts_removed[0], a, b, 2, 3));

    release_world(world);
}

void test_contacts_chain_integrity_many_pairs(TestCtx &ctx) {
    World world = make_world();
    RigidBody *hub = make_body(world);
    RigidBody *spokes[6];
    int32_t records[6];
    for (uint32_t i = 0; i < 6; i++) {
        spokes[i] = make_body(world);
        records[i] = pairs::add_pair_record(world.pairs, *hub, *spokes[i]);
    }

    // two or three contacts per pair
    int32_t created[18];
    uint32_t num_created = 0;
    for (uint32_t i = 0; i < 6; i++) {
        for (uint32_t s = 0; s < 2 + (i % 2); s++) {
            created[num_created++] = contacts::create_contact(world.contacts, world.pairs, records[i], *hub, *spokes[i], s, s)->contact_index;
        }
    }
    CHECK(ctx, num_created == 15 && chains_consistent(world, records, 6));

    // destroy heads, middles and tails
    const int32_t to_destroy[] = {created[0], created[4], created[7], created[num_created - 1]};
    for (int32_t contact_index : to_destroy) {
        contacts::destroy_contact(world.contacts, world.pairs, world.contacts.contacts[(uint32_t)contact_index], world.events);
    }
    CHECK(ctx, chains_consistent(world, records, 6) && active_contact_count(world.contacts) == 11);

    // recreate: the four freed slots come back first, then two new ones
    const uint32_t before_slots = world.contacts.contacts.length;
    for (uint32_t i = 0; i < 6; i++) contacts::create_contact(world.contacts, world.pairs, records[i], *hub, *spokes[i], 9, 9);
    CHECK(ctx, chains_consistent(world, records, 6));
    CHECK(ctx, world.contacts.contacts.length == before_slots + 2 && world.contacts.free_indices.length == 0);

    release_world(world);
}

void test_contacts_lifecycle_under_found_pair(TestCtx &ctx) {
    World world = make_world();
    RigidBody *floor = make_body(world, {0, -1, 0}, motion_type::MOTION_STATIC);
    RigidBody *ball = make_body(world, {0, 0.9f, 0});

    // the broadphase finds the pair; the narrowphase's part is played by hand: a contact per sub shape pair, ids in body
    // id order
    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 1 && world.events.pairs.length == 1);
    const int32_t record = world.events.pairs[0].pair_record;
    const RigidBody *a = floor->id < ball->id ? floor : ball;
    const RigidBody *b = a == floor ? ball : floor;

    world.contacts.frame_stamp++;
    Contact *contact = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 0, 0);
    contact->last_processed_frame = world.contacts.frame_stamp;
    contact = contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 1, 0);
    contact->last_processed_frame = world.contacts.frame_stamp;
    contacts::destroy_stale_contacts_in_pair(world.contacts, world.pairs, record, world.events);
    CHECK(ctx, active_contact_count(world.contacts) == 2 && world.events.contacts_removed.length == 0);

    // next step: the pair is found again and only one contact is touched; the other is swept as stale, reported now
    find_pairs(world);
    CHECK(ctx, world.events.pairs.length == 1 && world.events.pairs[0].pair_record == record);
    world.contacts.frame_stamp++;
    contact = contacts::find_contact_in_pair(world.contacts, world.pairs, record, 0, 0);
    CHECK(ctx, contact != nullptr);
    contact->last_processed_frame = world.contacts.frame_stamp;
    CHECK(ctx, contacts::find_contact_in_pair(world.contacts, world.pairs, record, 1, 0) != nullptr);
    contacts::destroy_stale_contacts_in_pair(world.contacts, world.pairs, record, world.events);
    CHECK(ctx, active_contact_count(world.contacts) == 1);
    CHECK(ctx, contacts::find_contact_in_pair(world.contacts, world.pairs, record, 1, 0) == nullptr);
    CHECK(ctx, world.events.contacts_removed.length == 1 && removed_event_is(world.events.contacts_removed[0], a, b, 1, 0));

    // the ball falls asleep: the record is kept but not emitted, so the sweep destroys its chain, reported now
    world.events.contacts_removed.length = 0;
    rigid_body::sleep(world.bodies, ball);
    find_pairs(world);
    CHECK(ctx, world.pairs.record_count == 1 && world.pairs.colliding_pairs.length == 0 && world.events.pairs.length == 0);
    CHECK(ctx, world.pairs.first_contact[(uint32_t)record] == INVALID_CONTACT_KEY && active_contact_count(world.contacts) == 0);
    CHECK(ctx, world.events.contacts_removed.length == 1 && removed_event_is(world.events.contacts_removed[0], a, b, 0, 0));
    CHECK(ctx, world.contacts.pending_contact_removed.length == 0);

    // woken, with a contact again; then the ball is removed: the purge queues the removal for the next step
    world.events.contacts_removed.length = 0;
    rigid_body::wake(world.bodies, ball);
    find_pairs(world);
    CHECK(ctx, world.pairs.colliding_pairs.length == 1 && world.events.pairs[0].pair_record == record);
    contacts::create_contact(world.contacts, world.pairs, record, *a, *b, 0, 0);
    pairs::purge_body_pairs(world.pairs, world.contacts, world.bodies, *ball, world.events);
    broadphase::remove_body(world.broadphase, ball);
    rigid_body::remove(world.bodies, ball);
    CHECK(ctx, world.pairs.record_count == 0 && active_contact_count(world.contacts) == 0);
    CHECK(ctx, world.events.contacts_removed.length == 0 && world.contacts.pending_contact_removed.length == 4);

    // the next step's flush reports it, with the removed body's id
    contacts::flush_pending_contact_removed(world.contacts, world.events);
    CHECK(ctx, world.events.contacts_removed.length == 1 && removed_event_is(world.events.contacts_removed[0], a, b, 0, 0));
    CHECK(ctx, world.contacts.pending_contact_removed.length == 0);

    // a flush with nothing queued reports nothing
    contacts::flush_pending_contact_removed(world.contacts, world.events);
    CHECK(ctx, world.events.contacts_removed.length == 1);

    release_world(world);
}
