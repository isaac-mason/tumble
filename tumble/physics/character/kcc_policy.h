#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/character/kcc.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// how a character treats the bodies it meets: a policy per body, with defaults for the rest. one policy can serve any
// number of characters

struct KCC::BodyPolicy {
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

struct KCC::Policy {
    struct Key {
        BodyId body_id;
    };

    /** for every body without one of its own */
    BodyPolicy defaults;
    collections::Map<Key, BodyPolicy> bodies;
};

inline bool operator==(const KCC::Policy::Key &a, const KCC::Policy::Key &b) { return a.body_id == b.body_id; }
inline uint32_t hash_of(const KCC::Policy::Key &key) {
    uint32_t h = key.body_id * 0x9e3779b1u;
    h ^= h >> 16;
    return h;
}

namespace kcc_policy {

/** every body at the defaults */
inline KCC::Policy create() { return {}; }

/** the body's own policy, by its id: a removed body's stays, naming nothing, until removed */
void set(KCC::Policy &policy, BodyId body_id, const KCC::BodyPolicy &body_policy);
/** back to the defaults; whether it had one */
bool remove(KCC::Policy &policy, BodyId body_id);
/** the body's policy, or the defaults; a null policy is the BodyPolicy defaults for every body */
const KCC::BodyPolicy &get(const KCC::Policy *policy, BodyId body_id);

/**
 * a listener that treats each body as the policy says, reading it at each callback, so changes to it apply at once.
 * it borrows the policy, which outlives it
 */
KCC::Listener listener(const KCC::Policy &policy);

}  // namespace kcc_policy

TUMBLE_NAMESPACE_END
