#include "physics/shapes/utils/triangle_mesh_data.h"

#include "util/alloc.h"
#include "util/assert.h"
#include "util/mem.h"

namespace triangle_mesh_data {

static constexpr uint32_t ARRAY_ALIGN = 16;

struct Layout {
    uint32_t positions;
    uint32_t triangles;
    uint32_t triangle_aabbs;
    uint32_t nodes;
    uint32_t bytes;
};

/** each array's byte offset from the start of the block */
static Layout layout_of(uint32_t vertex_count, uint32_t triangle_count, uint32_t node_count) {
    Layout layout;
    layout.positions = util::align_up(sizeof(TriangleMeshData), ARRAY_ALIGN);
    layout.triangles = util::align_up(layout.positions + vertex_count * 3 * sizeof(float), ARRAY_ALIGN);
    layout.triangle_aabbs = util::align_up(layout.triangles + triangle_count * sizeof(Triangle), ARRAY_ALIGN);
    layout.nodes = util::align_up(layout.triangle_aabbs + triangle_count * 6 * sizeof(float), ARRAY_ALIGN);
    layout.bytes = layout.nodes + node_count * sizeof(bvh::Node);
    return layout;
}

uint32_t byte_size(uint32_t vertex_count, uint32_t triangle_count, uint32_t node_count) {
    return layout_of(vertex_count, triangle_count, node_count).bytes;
}

TriangleMeshData *create(uint32_t vertex_count, uint32_t triangle_count, uint32_t node_count) {
    const Layout layout = layout_of(vertex_count, triangle_count, node_count);
    uint8_t *block = static_cast<uint8_t *>(util::alloc_bytes(layout.bytes));
    if (block == nullptr) return nullptr;
    util::zero(block, layout.bytes);

    TriangleMeshData *data = reinterpret_cast<TriangleMeshData *>(block);
    data->vertex_count = vertex_count;
    data->triangle_count = triangle_count;
    data->node_count = node_count;
    data->positions = reinterpret_cast<float *>(block + layout.positions);
    data->triangles = reinterpret_cast<Triangle *>(block + layout.triangles);
    data->triangle_aabbs = reinterpret_cast<float *>(block + layout.triangle_aabbs);
    data->nodes = reinterpret_cast<bvh::Node *>(block + layout.nodes);
    return data;
}

void destroy(TriangleMeshData *data) { util::free(data); }

void get_triangle_indices(uint32_t out[3], const TriangleMeshData &data, uint32_t tri_idx) {
    const Triangle &triangle = data.triangles[tri_idx];
    out[0] = triangle.index_a;
    out[1] = triangle.index_b;
    out[2] = triangle.index_c;
}

void get_triangle_vertices(Vec3f &out_a, Vec3f &out_b, Vec3f &out_c, const TriangleMeshData &data, uint32_t tri_idx) {
    const Triangle &triangle = data.triangles[tri_idx];
    out_a = vec3f::load(data.positions, triangle.index_a * 3);
    out_b = vec3f::load(data.positions, triangle.index_b * 3);
    out_c = vec3f::load(data.positions, triangle.index_c * 3);
}

void precompute_triangle_aabbs(TriangleMeshData &data) {
    const uint32_t count = data.triangle_count;
    float *aabbs = data.triangle_aabbs;
    Box3f aabb;
    for (uint32_t tri_idx = 0; tri_idx < count; tri_idx++) {
        calculate_triangle_aabb(aabb, data, tri_idx);
        float *out = aabbs + tri_idx * 6;
        out[0] = aabb.min.x;
        out[1] = aabb.min.y;
        out[2] = aabb.min.z;
        out[3] = aabb.max.x;
        out[4] = aabb.max.y;
        out[5] = aabb.max.z;
    }
}

Box3f &calculate_triangle_aabb(Box3f &out, const TriangleMeshData &data, uint32_t tri_idx) {
    const Triangle &triangle = data.triangles[tri_idx];
    const float *positions = data.positions;

    // get vertices
    const float *a = positions + triangle.index_a * 3;
    const float *b = positions + triangle.index_b * 3;
    const float *c = positions + triangle.index_c * 3;

    // compute aabb
    float min_x = a[0], min_y = a[1], min_z = a[2];
    float max_x = a[0], max_y = a[1], max_z = a[2];

    if (b[0] < min_x) min_x = b[0];
    if (b[1] < min_y) min_y = b[1];
    if (b[2] < min_z) min_z = b[2];
    if (b[0] > max_x) max_x = b[0];
    if (b[1] > max_y) max_y = b[1];
    if (b[2] > max_z) max_z = b[2];

    if (c[0] < min_x) min_x = c[0];
    if (c[1] < min_y) min_y = c[1];
    if (c[2] < min_z) min_z = c[2];
    if (c[0] > max_x) max_x = c[0];
    if (c[1] > max_y) max_y = c[1];
    if (c[2] > max_z) max_z = c[2];

    out.min = {min_x, min_y, min_z};
    out.max = {max_x, max_y, max_z};
    return out;
}

Vec3f calculate_triangle_centroid(const TriangleMeshData &data, uint32_t tri_idx) {
    const Triangle &triangle = data.triangles[tri_idx];
    const Vec3f a = vec3f::load(data.positions, triangle.index_a * 3);
    const Vec3f b = vec3f::load(data.positions, triangle.index_b * 3);
    const Vec3f c = vec3f::load(data.positions, triangle.index_c * 3);
    return vec3f::divide(vec3f::add(vec3f::add(a, b), c), vec3f::splat(3.0f));
}

void swap_triangles(TriangleMeshData &data, uint32_t index_a, uint32_t index_b) {
    if (index_a == index_b) return;
    CC_ASSERT(index_a < data.triangle_count && index_b < data.triangle_count);

    const Triangle tmp = data.triangles[index_a];
    data.triangles[index_a] = data.triangles[index_b];
    data.triangles[index_b] = tmp;
}

}  // namespace triangle_mesh_data
