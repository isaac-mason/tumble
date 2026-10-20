#include "physics/contact_pairs.h"

#include "physics/body/rigid_body.h"
#include "physics/events.h"
#include "physics/world.h"

namespace contact_pairs {

using rigid_body::RigidBody;

ContactPairs create() {
    ContactPairs pairs = {};
    pairs.head = NO_PAIR;
    pairs.tail = NO_PAIR;
    return pairs;
}

void release(ContactPairs &pairs) {
    pairs.pairs.release();
    pairs.free.release();
    pairs.index.release();
    pairs.active.release();
    pairs.removed.release();
    pairs = create();
}

/* keys */

static bool side_key_less(const SideKey &a, const SideKey &b) {
    if (a.body_id != b.body_id) return a.body_id < b.body_id;
    return a.sub_shape_id < b.sub_shape_id;
}

static PairKey pair_key(const Side &a, const Side &b) {
    const SideKey key_a = {a.body_id, a.sub_shape_id};
    const SideKey key_b = {b.body_id, b.sub_shape_id};
    PairKey key;
    key.first = side_key_less(key_b, key_a) ? key_b : key_a;
    key.second = side_key_less(key_b, key_a) ? key_a : key_b;
    return key;
}

/* the live list, in the order pairs were added */

static void link_at_tail(ContactPairs &pairs, uint32_t handle) {
    ContactPair &pair = pairs.pairs[handle];
    pair.previous = pairs.tail;
    pair.next = NO_PAIR;
    if (pairs.tail == NO_PAIR) pairs.head = (int32_t)handle;
    else pairs.pairs[(uint32_t)pairs.tail].next = (int32_t)handle;
    pairs.tail = (int32_t)handle;
}

static void unlink(ContactPairs &pairs, uint32_t handle) {
    const ContactPair &pair = pairs.pairs[handle];
    if (pair.previous == NO_PAIR) pairs.head = pair.next;
    else pairs.pairs[(uint32_t)pair.previous].next = pair.next;
    if (pair.next == NO_PAIR) pairs.tail = pair.previous;
    else pairs.pairs[(uint32_t)pair.next].previous = pair.previous;
}

/** the pair of these sides, recorded this step: found, or added; repeat records this step return the same one. null when the memory cannot grow */
static ContactPair *record(ContactPairs &pairs, const Side &a, const Side &b) {
    const PairKey key = pair_key(a, b);
    const uint32_t *found = pairs.index.get(key);
    uint32_t handle;
    if (found != nullptr) {
        handle = *found;
        ContactPair &pair = pairs.pairs[handle];
        if (pair.seen_stamp != pairs.stamp) {
            if (!pairs.active.push(handle)) return nullptr;
            pair.seen_stamp = pairs.stamp;
            pair.added = false;
            pairs.persisted++;
        }
    } else {
        const bool reused = pairs.free.length > 0;
        if (reused) {
            handle = pairs.free[pairs.free.length - 1];
        } else {
            handle = pairs.pairs.length;
            if (!pairs.pairs.push(ContactPair())) return nullptr;
        }
        if (!pairs.active.push(handle)) {
            if (!reused) pairs.pairs.length--;
            return nullptr;
        }
        if (!pairs.index.set(key, handle)) {
            pairs.active.length--;
            if (!reused) pairs.pairs.length--;
            return nullptr;
        }
        if (reused) pairs.free.length--;
        ContactPair &pair = pairs.pairs[handle];
        pair.key = key;
        pair.seen_stamp = pairs.stamp;
        pair.added = true;
        link_at_tail(pairs, handle);
        pairs.live++;
    }
    ContactPair &pair = pairs.pairs[handle];
    pair.a = a;
    pair.b = b;
    return &pair;
}

void begin(world::World &world) {
    ContactPairs &pairs = world.contact_pairs;
    for (uint32_t i = 0; i < pairs.removed.length; i++) {
        const uint32_t handle = pairs.removed[i];
        unlink(pairs, handle);
        pairs.index.remove(pairs.pairs[handle].key);
        pairs.free.push(handle);
        pairs.live--;
    }
    pairs.removed.length = 0;
    pairs.active.length = 0;
    pairs.persisted = 0;
    pairs.live_at_begin = pairs.live;
    pairs.stamp++;
}

void end(world::World &world) {
    ContactPairs &pairs = world.contact_pairs;
    // every pair live at the begin that was not recorded again is removed; none is the usual step, which skips the walk
    if (pairs.live_at_begin == pairs.persisted) return;
    for (int32_t handle = pairs.head; handle != NO_PAIR; handle = pairs.pairs[(uint32_t)handle].next) {
        if (pairs.pairs[(uint32_t)handle].seen_stamp != pairs.stamp) pairs.removed.push((uint32_t)handle);
    }
}

/* the world's contacts */

static Side side_of(const RigidBody &body, sub_shape::SubShapeId sub_shape_id) {
    Side side;
    side.body_id = body.id;
    side.sub_shape_id = sub_shape_id;
    side.user_data = body.user_data;
    side.is_sensor = body.sensor;
    return side;
}

static void record_world_contact(world::World &world, const events::ContactEvent &event) {
    const RigidBody &body_a = *world.bodies.pool[event.body_index_a];
    const RigidBody &body_b = *world.bodies.pool[event.body_index_b];
    if (!body_a.observes_contacts && !body_b.observes_contacts) return;
    ContactPair *pair = record(world.contact_pairs, side_of(body_a, event.sub_shape_id_a), side_of(body_b, event.sub_shape_id_b));
    if (pair == nullptr) return;
    pair->point = event.points_a[0];
    pair->normal = event.normal;
    pair->penetration_depth = event.penetration;
    const Float3 &velocity_a = body_a.motion_properties.linear_velocity;
    const Float3 &velocity_b = body_b.motion_properties.linear_velocity;
    pair->relative_velocity = {velocity_b.x - velocity_a.x, velocity_b.y - velocity_a.y, velocity_b.z - velocity_a.z};
}

void record_world_contacts(world::World &world, uint32_t first_added, uint32_t first_persisted) {
    const events::PhysicsEvents &events = world.events;
    for (uint32_t i = first_added; i < events.contacts_added.length; i++) record_world_contact(world, events.contacts_added[i]);
    for (uint32_t i = first_persisted; i < events.contacts_persisted.length; i++) record_world_contact(world, events.contacts_persisted[i]);
}

}  // namespace contact_pairs
