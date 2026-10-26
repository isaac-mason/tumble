#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct BodyActivationListener;
typedef void (*OnBodyActivated)(BodyActivationListener &listener, BodyId body_id, uint64_t user_data);
typedef void (*OnBodyDeactivated)(BodyActivationListener &listener, BodyId body_id, uint64_t user_data);

/**
 * hears a body join or leave the active list, whatever moved it: a step's contacts, constraints and islands, or a call
 * between steps. called as it happens, by the code changing the world; the body is not to be activated, deactivated,
 * added or removed from the callback. either callback may be null
 */
struct BodyActivationListener {
    OnBodyActivated on_body_activated;
    OnBodyDeactivated on_body_deactivated;
    void *context;
};

struct BodyActivationEvent {
    BodyId body_id;
    uint64_t user_data;
    /** false for a deactivation */
    bool activated;
};

/** a listener that keeps what it hears, in order, for flush to hand on later */
struct BodyActivationRecorder {
    BodyActivationListener listener;
    collections::Array<BodyActivationEvent> events;
};

namespace body_activation {

/** a recorder whose listener records into it; it must stay where it is while the listener is set */
void init_recorder(BodyActivationRecorder &recorder);

/**
 * every event recorded since the last flush, to the listener in the order they happened, then none; a null listener only
 * empties the recorder. the listener may change the world: what that records is handed on in the same flush. an event
 * the allocator could not record is lost
 */
void flush(BodyActivationRecorder &recorder, BodyActivationListener *listener);

}  // namespace body_activation

TUMBLE_NAMESPACE_END
