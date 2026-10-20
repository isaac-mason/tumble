#pragma once

// the contact pairs a user observes: every pair of body sub shapes in contact, keyed by its two sides, with a stable
// handle while it lives and added / persisted / removed each step. a pair is recorded only when a side's body observes
// contacts (RigidBody::observes_contacts). the world's contact events are recorded into it after the phases that
// append them. keyed so, not by the world's contact cache: world.h says why.

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

namespace world {
struct World;
}

namespace contact_pairs {

/** one side of a pair: a body's sub shape */
struct Side {
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
    uint64_t user_data;
    bool is_sensor;
};

/** what names a side */
struct SideKey {
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
};

/** the two sides' keys, the lesser first, so a pair is one key whichever way round it is recorded */
struct PairKey {
    SideKey first;
    SideKey second;
};

inline bool operator==(const SideKey &a, const SideKey &b) { return a.body_id == b.body_id && a.sub_shape_id == b.sub_shape_id; }

inline bool operator==(const PairKey &a, const PairKey &b) { return a.first == b.first && a.second == b.second; }

inline uint32_t hash_of(const SideKey &key) { return key.body_id * 0x85ebca6bu ^ key.sub_shape_id * 0x165667b1u; }

inline uint32_t hash_of(const PairKey &key) {
    uint32_t h = hash_of(key.first) ^ (hash_of(key.second) * 0x9e3779b1u);
    h ^= h >> 15;
    h *= 0x2c1b3c6du;
    h ^= h >> 12;
    return h;
}

/** no pair: the ends of the live list */
constexpr int32_t NO_PAIR = -1;

/** a pair, its fields as its last record this step left them; a removed pair keeps last step's */
struct ContactPair {
    Side a;
    Side b;
    /** the first contact point on a */
    Float3 point;
    /** unit, a to b */
    Float3 normal;
    float penetration_depth;
    /** b's linear velocity less a's when recorded */
    Float3 relative_velocity;
    /** first recorded this step, not there the step before */
    bool added;
    PairKey key;
    /** ContactPairs::stamp of the step that last recorded it */
    uint32_t seen_stamp;
    /** the live pairs in the order they were added */
    int32_t previous;
    int32_t next;
};

struct ContactPairs {
    /** by handle, live and free */
    collections::Vec<ContactPair> pairs;
    collections::Vec<uint32_t> free;
    collections::Map<PairKey, uint32_t> index;
    /** this step's pairs, in the order first recorded; ContactPair::added tells added from persisted */
    collections::Vec<uint32_t> active;
    /** there last step and not recorded this one, in the order they were added; freed by the next begin */
    collections::Vec<uint32_t> removed;
    int32_t head;
    int32_t tail;
    uint32_t live;
    uint32_t live_at_begin;
    uint32_t persisted;
    uint32_t stamp;
};

ContactPairs create();
void release(ContactPairs &pairs);

/** a step's bookkeeping begins: last step's removed pairs freed, their handles reusable */
void begin(world::World &world);
/** the world's contact events from these indices of contacts_added and contacts_persisted on */
void record_world_contacts(world::World &world, uint32_t first_added, uint32_t first_persisted);
/** the step's bookkeeping ends: the live pairs it did not record are removed */
void end(world::World &world);

}  // namespace contact_pairs

TUMBLE_NAMESPACE_END
