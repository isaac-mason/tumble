#include "tumble/physics/filter.h"

TUMBLE_NAMESPACE_BEGIN

namespace filter {

/** sets a table to `size` entries of `value`. */
static void fill_table(collections::Vec<uint8_t> &table, uint32_t size, uint8_t value) {
    table.length = 0;
    table.reserve(size);
    for (uint32_t i = 0; i < size; i++) table.push(value);
}

Filter create(const layers::Layers &layers) {
    Filter filter = {};
    set_all_enabled(filter, layers);
    return filter;
}

void release(Filter &filter) {
    filter.enabled_object_layers.release();
    filter.enabled_broadphase_layers.release();
}

void set_all_enabled(Filter &filter, const layers::Layers &layers) {
    fill_table(filter.enabled_object_layers, layers.object_layers, 1);
    fill_table(filter.enabled_broadphase_layers, layers.broadphase_layers, 1);
    filter.collision_mask = ~0u;
    filter.collision_groups = ~0u;
    filter.body_filter = nullptr;
    filter.body_filter_context = nullptr;
}

void enable_all_layers(Filter &filter, const layers::Layers &layers) {
    fill_table(filter.enabled_object_layers, layers.object_layers, 1);
    fill_table(filter.enabled_broadphase_layers, layers.broadphase_layers, 1);
}

void disable_all_layers(Filter &filter, const layers::Layers &layers) {
    fill_table(filter.enabled_object_layers, layers.object_layers, 0);
    fill_table(filter.enabled_broadphase_layers, layers.broadphase_layers, 0);
}

void enable_object_layer(Filter &filter, const layers::Layers &layers, uint32_t object_layer) {
    filter.enabled_object_layers[object_layer] = 1;
    filter.enabled_broadphase_layers[layers.object_layer_to_broadphase_layer[object_layer]] = 1;
}

void disable_object_layer(Filter &filter, const layers::Layers &layers, uint32_t object_layer) {
    filter.enabled_object_layers[object_layer] = 0;
    const uint32_t broadphase_layer = layers.object_layer_to_broadphase_layer[object_layer];
    for (uint32_t i = 0; i < layers.object_layers; i++) {
        if (i != object_layer && filter.enabled_object_layers[i] == 1 && layers.object_layer_to_broadphase_layer[i] == broadphase_layer) return;
    }
    filter.enabled_broadphase_layers[broadphase_layer] = 0;
}

void enable_broadphase_layer(Filter &filter, const layers::Layers &layers, uint32_t broadphase_layer) {
    filter.enabled_broadphase_layers[broadphase_layer] = 1;
    for (uint32_t i = 0; i < filter.enabled_object_layers.length; i++) {
        if (layers.object_layer_to_broadphase_layer[i] == broadphase_layer) filter.enabled_object_layers[i] = 1;
    }
}

void disable_broadphase_layer(Filter &filter, const layers::Layers &layers, uint32_t broadphase_layer) {
    filter.enabled_broadphase_layers[broadphase_layer] = 0;
    for (uint32_t i = 0; i < filter.enabled_object_layers.length; i++) {
        if (layers.object_layer_to_broadphase_layer[i] == broadphase_layer) filter.enabled_object_layers[i] = 0;
    }
}

bool filter_object_layer(const Filter &filter, uint32_t object_layer) {
    return object_layer < filter.enabled_object_layers.length && filter.enabled_object_layers[object_layer] == 1;
}

bool filter_broadphase_layer(const Filter &filter, uint32_t broadphase_layer) {
    return broadphase_layer < filter.enabled_broadphase_layers.length && filter.enabled_broadphase_layers[broadphase_layer] == 1;
}

void set_from_layer(Filter &filter, const layers::Layers &layers, uint32_t object_layer, uint32_t collision_groups, uint32_t collision_mask) {
    disable_all_layers(filter, layers);
    for (uint32_t i = 0; i < layers.object_layers; i++) {
        if (layers.object_layer_pairs[object_layer * layers.object_layers + i] == 1) filter.enabled_object_layers[i] = 1;
    }
    for (uint32_t i = 0; i < layers.broadphase_layers; i++) {
        if (layers.object_vs_broadphase[object_layer * layers.broadphase_layers + i] == 1) filter.enabled_broadphase_layers[i] = 1;
    }
    filter.collision_groups = collision_groups;
    filter.collision_mask = collision_mask;
}

void copy(Filter &out, const Filter &source) {
    out.enabled_object_layers.length = 0;
    for (uint32_t i = 0; i < source.enabled_object_layers.length; i++) out.enabled_object_layers.push(source.enabled_object_layers[i]);
    out.enabled_broadphase_layers.length = 0;
    for (uint32_t i = 0; i < source.enabled_broadphase_layers.length; i++)
        out.enabled_broadphase_layers.push(source.enabled_broadphase_layers[i]);
    out.collision_groups = source.collision_groups;
    out.collision_mask = source.collision_mask;
    out.body_filter = source.body_filter;
    out.body_filter_context = source.body_filter_context;
}

}  // namespace filter

TUMBLE_NAMESPACE_END
