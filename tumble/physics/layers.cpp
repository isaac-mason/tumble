#include "tumble/physics/layers.h"

TUMBLE_NAMESPACE_BEGIN

namespace layers {

/** grows a table to at least `size` entries, the new ones zero. */
template <typename T>
static void grow_table(collections::Vec<T> &table, uint32_t size) {
    table.reserve(size);
    while (table.length < size) table.push(0);
}

void release(Layers &layers) {
    layers.object_layer_to_broadphase_layer.release();
    layers.object_layer_pairs.release();
    layers.object_vs_broadphase.release();
    layers.broadphase_pairs.release();
    layers.broadphase_layers = 0;
    layers.object_layers = 0;
}

/** the tables are row-major on the current counts; a new row or column means rebuilding them. */
static void resize_tables(Layers &layers, uint32_t object_layers, uint32_t broadphase_layers) {
    const uint32_t old_objects = layers.object_layers;
    const uint32_t old_broadphase = layers.broadphase_layers;
    collections::Vec<uint8_t> object_pairs = {};
    collections::Vec<uint8_t> object_vs_broadphase = {};
    collections::Vec<uint8_t> broadphase_pairs = {};
    grow_table(object_pairs, object_layers * object_layers);
    grow_table(object_vs_broadphase, object_layers * broadphase_layers);
    grow_table(broadphase_pairs, broadphase_layers * broadphase_layers);
    for (uint32_t i = 0; i < old_objects; i++) {
        for (uint32_t j = 0; j < old_objects; j++) object_pairs[i * object_layers + j] = layers.object_layer_pairs[i * old_objects + j];
        for (uint32_t j = 0; j < old_broadphase; j++)
            object_vs_broadphase[i * broadphase_layers + j] = layers.object_vs_broadphase[i * old_broadphase + j];
    }
    for (uint32_t i = 0; i < old_broadphase; i++) {
        for (uint32_t j = 0; j < old_broadphase; j++)
            broadphase_pairs[i * broadphase_layers + j] = layers.broadphase_pairs[i * old_broadphase + j];
    }
    layers.object_layer_pairs.release();
    layers.object_vs_broadphase.release();
    layers.broadphase_pairs.release();
    layers.object_layer_pairs = object_pairs;
    layers.object_vs_broadphase = object_vs_broadphase;
    layers.broadphase_pairs = broadphase_pairs;
    layers.object_layers = object_layers;
    layers.broadphase_layers = broadphase_layers;
}

uint32_t add_broadphase_layer(Layers &layers) {
    const uint32_t index = layers.broadphase_layers;
    resize_tables(layers, layers.object_layers, layers.broadphase_layers + 1);
    return index;
}

uint32_t add_object_layer(Layers &layers, uint32_t broadphase_layer) {
    const uint32_t index = layers.object_layers;
    layers.object_layer_to_broadphase_layer.push(broadphase_layer);
    resize_tables(layers, layers.object_layers + 1, layers.broadphase_layers);
    // an object layer always queries its own broadphase layer
    layers.object_vs_broadphase[index * layers.broadphase_layers + broadphase_layer] = 1;
    return index;
}

void enable_collision(Layers &layers, uint32_t object_layer_a, uint32_t object_layer_b) {
    const uint32_t objects = layers.object_layers;
    layers.object_layer_pairs[object_layer_a * objects + object_layer_b] = 1;
    layers.object_layer_pairs[object_layer_b * objects + object_layer_a] = 1;

    const uint32_t bp_a = layers.object_layer_to_broadphase_layer[object_layer_a];
    const uint32_t bp_b = layers.object_layer_to_broadphase_layer[object_layer_b];
    const uint32_t broadphase = layers.broadphase_layers;
    layers.broadphase_pairs[bp_a * broadphase + bp_b] = 1;
    layers.broadphase_pairs[bp_b * broadphase + bp_a] = 1;
    // each may query the other's broadphase layer
    layers.object_vs_broadphase[object_layer_a * broadphase + bp_b] = 1;
    layers.object_vs_broadphase[object_layer_b * broadphase + bp_a] = 1;
}

void disable_collision(Layers &layers, uint32_t object_layer_a, uint32_t object_layer_b) {
    const uint32_t objects = layers.object_layers;
    layers.object_layer_pairs[object_layer_a * objects + object_layer_b] = 0;
    layers.object_layer_pairs[object_layer_b * objects + object_layer_a] = 0;
}

bool object_layer_collides_with_broadphase_layer(const Layers &layers, uint32_t object_layer, uint32_t broadphase_layer) {
    return layers.object_vs_broadphase[object_layer * layers.broadphase_layers + broadphase_layer] == 1;
}

bool broadphase_layer_collides_with_broadphase_layer(const Layers &layers, uint32_t broadphase_layer_a, uint32_t broadphase_layer_b) {
    return layers.broadphase_pairs[broadphase_layer_a * layers.broadphase_layers + broadphase_layer_b] == 1;
}

bool object_layer_collides_with_object_layer(const Layers &layers, uint32_t object_layer_a, uint32_t object_layer_b) {
    return layers.object_layer_pairs[object_layer_a * layers.object_layers + object_layer_b] == 1;
}

}  // namespace layers

TUMBLE_NAMESPACE_END
