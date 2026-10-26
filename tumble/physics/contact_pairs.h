#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct World;
struct KCC;

namespace contact_pairs {

constexpr int32_t NO_PAIR = -1;

}  // namespace contact_pairs

// the pairs of body sub shapes in contact that a body observing contacts is a side of: a stable handle while a pair
// lives, added / persisted / removed each frame. a frame is begin, then the records of everything that moved in it
// (each world step's contact events, each character's move), then end

/** its fields are its last record's; a removed pair keeps last step's */
struct ContactPair {
    struct SideKey {
        BodyId body_id;
        SubShapeId sub_shape_id;
    };

    /** the lesser side first, so a pair has one key whichever way round it is recorded */
    struct Key {
        SideKey first;
        SideKey second;
    };

    struct Side {
        BodyId body_id;
        SubShapeId sub_shape_id;
        uint64_t user_data;
        bool is_sensor;
    };

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
    Key key;
    /** the stamp of the step that last recorded it */
    uint32_t seen_stamp;
    /** the live list, in the order pairs were added */
    int32_t previous;
    int32_t next;
};

inline bool operator==(const ContactPair::SideKey &a, const ContactPair::SideKey &b) { return a.body_id == b.body_id && a.sub_shape_id == b.sub_shape_id; }

inline bool operator==(const ContactPair::Key &a, const ContactPair::Key &b) { return a.first == b.first && a.second == b.second; }

inline uint32_t hash_of(const ContactPair::SideKey &key) { return key.body_id * 0x85ebca6bu ^ key.sub_shape_id * 0x165667b1u; }

inline uint32_t hash_of(const ContactPair::Key &key) {
    uint32_t h = hash_of(key.first) ^ (hash_of(key.second) * 0x9e3779b1u);
    h ^= h >> 15;
    h *= 0x2c1b3c6du;
    h ^= h >> 12;
    return h;
}

struct ContactPairs {
    /** by handle, live and free */
    collections::Array<ContactPair> pairs;
    collections::Array<uint32_t> free;
    collections::Map<ContactPair::Key, uint32_t> index;
    /** this step's pairs, in the order first recorded */
    collections::Array<uint32_t> active;
    /** live last step and not recorded this one, in the order added; freed by the next begin */
    collections::Array<uint32_t> removed;
    int32_t head;
    int32_t tail;
    uint32_t live;
    uint32_t live_at_begin;
    uint32_t persisted;
    uint32_t stamp;
};

namespace contact_pairs {

ContactPairs create();

/** a step's bookkeeping begins: last step's removed pairs freed, their handles reusable */
void begin(World &world);
/** the world's contact events from these indices of contacts_added and contacts_persisted on, but the rejected */
void record_world_contacts(World &world, uint32_t first_added, uint32_t first_persisted);
/**
 * the character's collisions from its last move or refresh, its inner body as side a (sub shape empty), the normal from
 * the character into the body. a character without an inner body has no id in the world and records nothing
 */
void record_character_contacts(World &world, const KCC &character);
/** the step's bookkeeping ends: the live pairs it did not record are removed */
void end(World &world);

}  // namespace contact_pairs

TUMBLE_NAMESPACE_END
