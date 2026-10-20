#pragma once

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
namespace kcc {
struct KCC;
}

// the pairs of body sub shapes in contact that a body observing contacts is a side of: a stable handle while a pair
// lives, added / persisted / removed each frame. a frame is begin, then the records of everything that moved in it
// (each world step's contact events, each character's move), then end
namespace contact_pairs {

struct Side {
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
    uint64_t user_data;
    bool is_sensor;
};

struct SideKey {
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
};

/** the lesser side first, so a pair has one key whichever way round it is recorded */
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

constexpr int32_t NO_PAIR = -1;

/** its fields are its last record's; a removed pair keeps last step's */
struct ContactPair {
    Side a;
    Side b;
    /** the first contact point on a */
    Float3 point;
    /** a to b */
    Float3 normal;
    float penetration_depth;
    /** b's linear velocity less a's */
    Float3 relative_velocity;
    /** not there the step before */
    bool added;
    PairKey key;
    /** the stamp of the step that last recorded it */
    uint32_t seen_stamp;
    /** the live list, in the order pairs were added */
    int32_t previous;
    int32_t next;
};

struct ContactPairs {
    /** by handle, live and free */
    collections::Vec<ContactPair> pairs;
    collections::Vec<uint32_t> free;
    collections::Map<PairKey, uint32_t> index;
    /** this step's pairs, in the order first recorded */
    collections::Vec<uint32_t> active;
    /** live last step and not recorded this one, in the order added; freed by the next begin */
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
/**
 * the character's collisions from its last move or refresh, its inner body as side a (sub shape empty), the normal from
 * the character into the body. a character without an inner body has no id in the world and records nothing
 */
void record_character_contacts(world::World &world, const kcc::KCC &character);
/** the step's bookkeeping ends: the live pairs it did not record are removed */
void end(world::World &world);

}  // namespace contact_pairs

TUMBLE_NAMESPACE_END
