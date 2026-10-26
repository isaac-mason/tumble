#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/layers.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct RigidBody;

/** a body filter: true to keep the body. context is the caller's. */
typedef bool (*BodyFilter)(const RigidBody *body, void *context);

struct Filter {
    /** one byte per object layer, 1 when enabled */
    collections::Array<uint8_t> enabled_object_layers;
    /** one byte per broadphase layer, 1 when enabled */
    collections::Array<uint8_t> enabled_broadphase_layers;
    uint32_t collision_mask;
    uint32_t collision_groups;
    BodyFilter body_filter;
    void *body_filter_context;
};

namespace filter {

/** a filter with every layer of `layers` enabled and the mask and group wide open. */
Filter create(const Layers &layers);
/** a filter with no layers. */
inline Filter create_empty() { return {}; }

/** every layer enabled, mask and group wide open, no body filter. */
void set_all_enabled(Filter &filter, const Layers &layers);
void enable_all_layers(Filter &filter, const Layers &layers);
void disable_all_layers(Filter &filter, const Layers &layers);
/** enables an object layer and the broadphase layer it belongs to. */
void enable_object_layer(Filter &filter, const Layers &layers, uint32_t object_layer);
/** disables an object layer, and its broadphase layer when no other enabled object layer uses it. */
void disable_object_layer(Filter &filter, const Layers &layers, uint32_t object_layer);
/** enables a broadphase layer and every object layer in it. */
void enable_broadphase_layer(Filter &filter, const Layers &layers, uint32_t broadphase_layer);
/** disables a broadphase layer and every object layer in it. */
void disable_broadphase_layer(Filter &filter, const Layers &layers, uint32_t broadphase_layer);

bool filter_object_layer(const Filter &filter, uint32_t object_layer);
bool filter_broadphase_layer(const Filter &filter, uint32_t broadphase_layer);

/** the layers that collide with a body's object layer, and its group and mask (rigid_body.h wraps this for a body). */
void set_from_layer(Filter &filter, const Layers &layers, uint32_t object_layer, uint32_t collision_groups, uint32_t collision_mask);
void copy(Filter &out, const Filter &source);

/** two bodies collide when each is in the other's mask. */
inline bool should_pair_collide(uint32_t group_a, uint32_t mask_a, uint32_t group_b, uint32_t mask_b) {
    return (group_a & mask_b) != 0 && (group_b & mask_a) != 0;
}

}  // namespace filter

TUMBLE_NAMESPACE_END
