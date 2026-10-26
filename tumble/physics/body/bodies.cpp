#include "tumble/physics/body/bodies.h"

#include "tumble/util/error.h"

#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/util/assert.h"
#include "tumble/util/mem.h"
#include "tumble/util/object.h"

TUMBLE_NAMESPACE_BEGIN

Bodies::~Bodies() {
    for (uint32_t i = 0; i < pool.length; i++) util::destroy(pool[i]);
}

namespace bodies {

BodyId add_slot(Bodies &bodies) {
    uint32_t index;
    if (bodies.free_indices.length > 0) {
        // reuse a pooled slot
        index = bodies.free_indices[bodies.free_indices.length - 1];
        bodies.free_indices.length--;
    } else {
        // expand pool, as far as the id has index bits for
        index = bodies.pool.length;
        if (index > body_id::MAX_BODY_INDEX) {
            util::set_error(Error::QUOTA);
            return body_id::INVALID_BODY_ID;
        }
        bodies.pool.push(nullptr);
        bodies.sequences.push(SLOT_FREE);
        bodies.slot_sequences.push(0);
        bodies.active_body_indices.push(0);
        // reserved to the pool's capacity, so remove_slot never has to grow it
        bodies.free_indices.reserve(bodies.pool.capacity);
    }

    // the slot's sequence, bumped: an id the slot's last body had stops naming this one
    const uint32_t sequence = (uint8_t)(bodies.slot_sequences[index] + 1);
    bodies.slot_sequences[index] = (uint8_t)sequence;

    bodies.sequences[index] = sequence;
    return body_id::ser(index, sequence);
}

bool remove_slot(Bodies &bodies, BodyId id) {
    if (!is_valid(bodies, id)) return false;
    const uint32_t index = body_id::index(id);
    TUMBLE_ASSERT(bodies.free_indices.length < bodies.free_indices.capacity);
    bodies.free_indices.data[bodies.free_indices.length++] = index;
    bodies.sequences[index] = SLOT_FREE;
    return true;
}

bool is_valid(const Bodies &bodies, BodyId id) {
    const uint32_t index = body_id::index(id);
    if (index >= bodies.sequences.length) return false;
    // check sequence matches to catch stale references
    return bodies.sequences[index] == body_id::sequence(id);
}

RigidBody *get(const Bodies &bodies, BodyId id) {
    if (!is_valid(bodies, id)) return nullptr;
    return bodies.pool[body_id::index(id)];
}

}  // namespace bodies

TUMBLE_NAMESPACE_END
