#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct Layers {
    uint32_t broadphase_layers;
    uint32_t object_layers;
    /** object layer -> broadphase layer */
    collections::Array<uint32_t> object_layer_to_broadphase_layer;
    /** object layer pairs: i * object_layers + j is 1 when i collides with j */
    collections::Array<uint8_t> object_layer_pairs;
    /** object layer vs broadphase layer: i * broadphase_layers + j */
    collections::Array<uint8_t> object_vs_broadphase;
    /** broadphase layer pairs: i * broadphase_layers + j */
    collections::Array<uint8_t> broadphase_pairs;
};

namespace layers {

/** an empty set of layers; a zeroed struct is the same. */
inline Layers create() { return {}; }

/** adds a broadphase layer and returns its index. */
uint32_t add_broadphase_layer(Layers &layers);
/** adds an object layer in `broadphase_layer` and returns its index. */
uint32_t add_object_layer(Layers &layers, uint32_t broadphase_layer);
/** lets two object layers collide, both ways, and their broadphase layers with them. */
void enable_collision(Layers &layers, uint32_t object_layer_a, uint32_t object_layer_b);
/** stops two object layers colliding, both ways. */
void disable_collision(Layers &layers, uint32_t object_layer_a, uint32_t object_layer_b);

bool object_layer_collides_with_broadphase_layer(const Layers &layers, uint32_t object_layer, uint32_t broadphase_layer);
bool broadphase_layer_collides_with_broadphase_layer(const Layers &layers, uint32_t broadphase_layer_a, uint32_t broadphase_layer_b);
bool object_layer_collides_with_object_layer(const Layers &layers, uint32_t object_layer_a, uint32_t object_layer_b);

}  // namespace layers

TUMBLE_NAMESPACE_END
