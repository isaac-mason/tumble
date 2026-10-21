#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// how a character treats the bodies it meets: a policy per body, with defaults for the rest. one policy can serve any
// number of characters
namespace kcc_policy {

struct BodyPolicy {
    /** the character passes through it, as if it were not there */
    bool ignore = false;
    /** false: it blocks the character but does not move it */
    bool can_push_character = true;
    /** false: the character does not push it */
    bool can_receive_impulses = true;
    /** false: no sliding along a still, walkable surface of it */
    bool slide_on_walkable = true;
    /** added to the body's velocities where the character meets it, as a conveyor's belt moves */
    Float3 surface_linear_velocity = {0.0f, 0.0f, 0.0f};
    Float3 surface_angular_velocity = {0.0f, 0.0f, 0.0f};
};

struct Key {
    body_id::BodyId body_id;
};

inline bool operator==(const Key &a, const Key &b) { return a.body_id == b.body_id; }

inline uint32_t hash_of(const Key &key) {
    uint32_t h = key.body_id * 0x9e3779b1u;
    h ^= h >> 16;
    return h;
}

struct Policy {
    /** for every body without one of its own */
    BodyPolicy defaults;
    collections::Map<Key, BodyPolicy> bodies;
};

/** every body at the defaults */
inline Policy create() { return {}; }
void release(Policy &policy);

/** the body's own policy, by its id: a removed body's stays, naming nothing, until removed; false (error set) when the allocator fails */
bool set(Policy &policy, body_id::BodyId body_id, const BodyPolicy &body_policy);
/** back to the defaults; whether it had one */
bool remove(Policy &policy, body_id::BodyId body_id);
/** the body's policy, or the defaults; a null policy is the BodyPolicy defaults for every body */
const BodyPolicy &get(const Policy *policy, body_id::BodyId body_id);

}  // namespace kcc_policy

TUMBLE_NAMESPACE_END
