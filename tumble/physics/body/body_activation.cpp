#include "tumble/physics/body/body_activation.h"

TUMBLE_NAMESPACE_BEGIN

namespace body_activation {

static void record(BodyActivationListener &listener, BodyId body_id, uint64_t user_data, bool activated) {
    BodyActivationRecorder &recorder = *static_cast<BodyActivationRecorder *>(listener.context);
    BodyActivationEvent event;
    event.body_id = body_id;
    event.user_data = user_data;
    event.activated = activated;
    recorder.events.push(event);
}

static void record_activated(BodyActivationListener &listener, BodyId body_id, uint64_t user_data) {
    record(listener, body_id, user_data, true);
}

static void record_deactivated(BodyActivationListener &listener, BodyId body_id, uint64_t user_data) {
    record(listener, body_id, user_data, false);
}

void init_recorder(BodyActivationRecorder &recorder) {
    recorder = {};
    recorder.listener.on_body_activated = record_activated;
    recorder.listener.on_body_deactivated = record_deactivated;
    recorder.listener.context = &recorder;
}

void flush(BodyActivationRecorder &recorder, BodyActivationListener *listener) {
    if (listener != nullptr) {
        // a copy: a callback may change the world, recording more, which can move the events; those are handed on too
        for (uint32_t i = 0; i < recorder.events.length; i++) {
            const BodyActivationEvent event = recorder.events[i];
            if (event.activated) {
                if (listener->on_body_activated != nullptr) listener->on_body_activated(*listener, event.body_id, event.user_data);
            } else if (listener->on_body_deactivated != nullptr) {
                listener->on_body_deactivated(*listener, event.body_id, event.user_data);
            }
        }
    }
    recorder.events.length = 0;
}

}  // namespace body_activation

TUMBLE_NAMESPACE_END
