#pragma once

// crashcat's triangle-mesh-data.ts: a triangle mesh's vertices, triangles and bvh, in one block.
//
// `create` makes one allocation: the header, then the arrays back to back, each 16-aligned, so js
// can size a mesh, create it and write the arrays at the addresses the header holds:
// - positions: f32, 3 per vertex [x, y, z], deduplicated
// - triangles: TRIANGLE_STRIDE words per triangle (see Triangle), in bvh order
// - triangle_aabbs: f32, 6 per triangle [min_x, min_y, min_z, max_x, max_y, max_z]
// - nodes: bvh::NODE_STRIDE words per node, pre-order, the root at 0

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/utils/bvh.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace triangle_mesh_data {

/** triangle buffer stride, in 32-bit words */
constexpr uint32_t TRIANGLE_STRIDE = 8;

// offsets within each triangle's stride
constexpr uint32_t OFFSET_INDEX_A = 0;
constexpr uint32_t OFFSET_INDEX_B = 1;
constexpr uint32_t OFFSET_INDEX_C = 2;
constexpr uint32_t OFFSET_NORMAL_X = 3;
constexpr uint32_t OFFSET_NORMAL_Y = 4;
constexpr uint32_t OFFSET_NORMAL_Z = 5;
constexpr uint32_t OFFSET_ACTIVE_EDGES = 6;
constexpr uint32_t OFFSET_MATERIAL_ID = 7;

/** one triangle's stride of the interleaved triangle buffer */
struct Triangle {
    /** vertex indices into positions */
    uint32_t index_a;
    uint32_t index_b;
    uint32_t index_c;
    /** unit, from the counter clockwise winding a, b, c */
    Float3 normal;
    /** bit 0 = edge AB, bit 1 = edge BC, bit 2 = edge CA */
    uint32_t active_edges;
    /** -1 for no material */
    int32_t material_id;
};

static_assert(sizeof(Triangle) == TRIANGLE_STRIDE * 4);
static_assert(__builtin_offsetof(Triangle, index_a) == OFFSET_INDEX_A * 4);
static_assert(__builtin_offsetof(Triangle, index_b) == OFFSET_INDEX_B * 4);
static_assert(__builtin_offsetof(Triangle, index_c) == OFFSET_INDEX_C * 4);
static_assert(__builtin_offsetof(Triangle, normal) == OFFSET_NORMAL_X * 4);
static_assert(__builtin_offsetof(Triangle, active_edges) == OFFSET_ACTIVE_EDGES * 4);
static_assert(__builtin_offsetof(Triangle, material_id) == OFFSET_MATERIAL_ID * 4);

/** a built triangle mesh: one allocation from create, shareable between shapes. */
struct TriangleMeshData {
    uint32_t vertex_count;
    uint32_t triangle_count;
    uint32_t node_count;
    /** deduplicated vertex positions, 3 floats per vertex */
    float *positions;
    Triangle *triangles;
    /**
     * per-triangle mesh-local aabbs, 6 floats per triangle. derived at build time (after the bvh
     * build finalizes triangle order) so cast queries read them instead of recomputing from the
     * vertex indices per query.
     */
    float *triangle_aabbs;
    /** the bvh over the triangles; none when there are no triangles */
    bvh::Node *nodes;
};

/** the bytes create allocates for a mesh of these counts, header included. */
uint32_t byte_size(uint32_t vertex_count, uint32_t triangle_count, uint32_t node_count);

/** a zeroed mesh with room for the counts, or null (error set) when the memory cannot grow. */
TriangleMeshData *create(uint32_t vertex_count, uint32_t triangle_count, uint32_t node_count);

void destroy(TriangleMeshData *data);

/** get triangle vertex indices */
void get_triangle_indices(uint32_t out[3], const TriangleMeshData &data, uint32_t tri_idx);

/** get triangle vertex positions */
void get_triangle_vertices(Vec3f &out_a, Vec3f &out_b, Vec3f &out_c, const TriangleMeshData &data, uint32_t tri_idx);

/** get triangle normal */
inline Vec3f get_triangle_normal(const TriangleMeshData &data, uint32_t tri_idx) { return vec3f::load(data.triangles[tri_idx].normal); }

/** set triangle normal */
inline void set_triangle_normal(TriangleMeshData &data, uint32_t tri_idx, Vec3f normal) { vec3f::store(data.triangles[tri_idx].normal, normal); }

/** get active edges bitmask (bit 0 = edge AB, bit 1 = edge BC, bit 2 = edge CA) */
inline uint32_t get_active_edges(const TriangleMeshData &data, uint32_t tri_idx) { return data.triangles[tri_idx].active_edges; }

/** set active edges */
inline void set_active_edges(TriangleMeshData &data, uint32_t tri_idx, uint32_t active_edges) {
    data.triangles[tri_idx].active_edges = active_edges;
}

/** get material id */
inline int32_t get_material_id(const TriangleMeshData &data, uint32_t tri_idx) { return data.triangles[tri_idx].material_id; }

/** fill data.triangle_aabbs from the (final, post-bvh-build) triangles */
void precompute_triangle_aabbs(TriangleMeshData &data);

/** calculate triangle aabb from vertices */
Box3f &calculate_triangle_aabb(Box3f &out, const TriangleMeshData &data, uint32_t tri_idx);

/** calculate triangle centroid from vertices */
Vec3f calculate_triangle_centroid(const TriangleMeshData &data, uint32_t tri_idx);

/** swap two triangles */
void swap_triangles(TriangleMeshData &data, uint32_t index_a, uint32_t index_b);

}  // namespace triangle_mesh_data

TUMBLE_NAMESPACE_END
