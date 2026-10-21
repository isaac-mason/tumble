#define CGLTF_IMPLEMENTATION
#include "app/model.h"

#include <math.h>
#include <stdio.h>

#include "cgltf.h"

using namespace tumble;

namespace model {

static void append_primitive(Model &model, const cgltf_primitive &primitive, const float *world) {
    if (primitive.type != cgltf_primitive_type_triangles) return;
    const cgltf_accessor *positions = nullptr;
    for (cgltf_size i = 0; i < primitive.attributes_count; i++) {
        if (primitive.attributes[i].type == cgltf_attribute_type_position) positions = primitive.attributes[i].data;
    }
    if (!positions) return;

    Float3 color = {0.8f, 0.8f, 0.8f};
    if (primitive.material && primitive.material->has_pbr_metallic_roughness) {
        const cgltf_float *factor = primitive.material->pbr_metallic_roughness.base_color_factor;
        color = {factor[0], factor[1], factor[2]};
    }

    const uint32_t base = model.positions.length / 3;
    const uint32_t vertex_count = (uint32_t)positions->count;
    for (uint32_t i = 0; i < vertex_count; i++) {
        float local[3] = {0, 0, 0};
        cgltf_accessor_read_float(positions, i, local, 3);
        const Float3 p = {
            world[0] * local[0] + world[4] * local[1] + world[8] * local[2] + world[12],
            world[1] * local[0] + world[5] * local[1] + world[9] * local[2] + world[13],
            world[2] * local[0] + world[6] * local[1] + world[10] * local[2] + world[14],
        };
        model.positions.push(p.x);
        model.positions.push(p.y);
        model.positions.push(p.z);
        model.colors.push(color.x);
        model.colors.push(color.y);
        model.colors.push(color.z);
        model.bounds.min = {fminf(model.bounds.min.x, p.x), fminf(model.bounds.min.y, p.y), fminf(model.bounds.min.z, p.z)};
        model.bounds.max = {fmaxf(model.bounds.max.x, p.x), fmaxf(model.bounds.max.y, p.y), fmaxf(model.bounds.max.z, p.z)};
    }
    const uint32_t index_count = primitive.indices ? (uint32_t)primitive.indices->count : vertex_count;
    for (uint32_t i = 0; i < index_count; i++) {
        model.indices.push(base + (primitive.indices ? (uint32_t)cgltf_accessor_read_index(primitive.indices, i) : i));
    }
}

static void append_node(Model &model, const cgltf_node &node) {
    if (node.mesh) {
        float world[16];
        cgltf_node_transform_world(&node, world);
        for (cgltf_size i = 0; i < node.mesh->primitives_count; i++) append_primitive(model, node.mesh->primitives[i], world);
    }
    for (cgltf_size i = 0; i < node.children_count; i++) append_node(model, *node.children[i]);
}

Model load(const char *name) {
    char path[512];
#if defined(TUMBLE_LABS_ASSETS_DIR)
    snprintf(path, sizeof(path), "%s/%s", TUMBLE_LABS_ASSETS_DIR, name);
#else
    snprintf(path, sizeof(path), "/assets/%s", name);
#endif
    Model model = {};
    model.bounds = {{INFINITY, INFINITY, INFINITY}, {-INFINITY, -INFINITY, -INFINITY}};
    cgltf_options options = {};
    cgltf_data *data = nullptr;
    if (cgltf_parse_file(&options, path, &data) != cgltf_result_success) {
        fprintf(stderr, "labs: could not parse %s\n", path);
        return model;
    }
    if (cgltf_load_buffers(&options, data, path) != cgltf_result_success) {
        fprintf(stderr, "labs: could not load the buffers of %s\n", path);
        cgltf_free(data);
        return model;
    }
    const cgltf_scene *scene = data->scene ? data->scene : (data->scenes_count > 0 ? &data->scenes[0] : nullptr);
    if (scene) {
        for (cgltf_size i = 0; i < scene->nodes_count; i++) append_node(model, *scene->nodes[i]);
    }
    cgltf_free(data);
    model.loaded = model.indices.length > 0;
    return model;
}

void release(Model &model) {
    model.positions.release();
    model.colors.release();
    model.indices.release();
    model.loaded = false;
}

}
