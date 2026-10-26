#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"

#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/active_edges.h"
#include "tumble/util/assert.h"
#include "tumble/util/collections.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

namespace triangle_mesh_builder {


/** 1 / 1e-8 */
static constexpr float WELD_GRID_INV = 1e8f;

static inline uint32_t mix(uint32_t h) {
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

static inline uint32_t combine(uint32_t h, uint32_t value) { return mix(h ^ (value * 0x9e3779b1u)); }

/** a vertex position snapped to the weld grid */
struct WeldKey {
    float x, y, z;
    bool operator==(const WeldKey &o) const { return x == o.x && y == o.y && z == o.z; }
};

static inline uint32_t hash_of(const WeldKey &key) {
    return combine(combine(mix(__builtin_bit_cast(uint32_t, key.x)), __builtin_bit_cast(uint32_t, key.y)),
                   __builtin_bit_cast(uint32_t, key.z));
}

/** a triangle's indices rotated so the lowest comes first, and its material */
struct TriangleKey {
    uint32_t a, b, c;
    int32_t material_id;
    bool operator==(const TriangleKey &o) const { return a == o.a && b == o.b && c == o.c && material_id == o.material_id; }
};

static inline uint32_t hash_of(const TriangleKey &key) {
    return combine(combine(combine(mix(key.a), key.b), key.c), (uint32_t)key.material_id);
}

/** an undirected edge: its two vertex indices, lowest first */
struct EdgeKey {
    uint32_t lo, hi;
    bool operator==(const EdgeKey &o) const { return lo == o.lo && hi == o.hi; }
};

static inline uint32_t hash_of(const EdgeKey &key) { return combine(mix(key.lo), key.hi); }

/**
 * an edge's triangle references: up to 2 inline. when a 3rd
 * triangle uses the edge, it is marked non-manifold immediately without storing more references.
 */
struct EdgeRefs {
    uint32_t count;
    /** packed (tri_idx << 3 | edge_bit) */
    uint32_t tri0;
    /** packed (tri_idx << 3 | edge_bit) */
    uint32_t tri1;
};

/** Math.round: halves round up, and -0 becomes 0 so it welds with 0 */
static inline float round_half_up(float x) {
    float rounded = scalar::floor(x);
    if (x - rounded >= 0.5f) rounded += 1.0f;
    return rounded + 0.0f;
}

static void deduplicate_vertices(collections::Array<float> &out_positions, collections::Array<uint32_t> &out_index_map, const float *input_positions,
                                 uint32_t vertex_count) {
    collections::Map<WeldKey, uint32_t> vertex_map = {};
    out_index_map.reserve(vertex_count);
    out_index_map.length = vertex_count;
    uint32_t unique_count = 0;

    for (uint32_t i = 0; i < vertex_count; i++) {
        const float *p = input_positions + i * 3;
        const WeldKey key = {round_half_up(p[0] * WELD_GRID_INV), round_half_up(p[1] * WELD_GRID_INV), round_half_up(p[2] * WELD_GRID_INV)};

        const uint32_t *found = vertex_map.get(key);
        uint32_t idx;
        if (found == nullptr) {
            idx = unique_count++;
            vertex_map.set(key, idx);
            out_positions.reserve(unique_count * 3);
            out_positions.push(p[0]);
            out_positions.push(p[1]);
            out_positions.push(p[2]);
        } else {
            idx = *found;
        }
        out_index_map[i] = idx;
    }
}

/**
 * compute a unique key for an edge, independent of its direction.
 */
static inline EdgeKey edge_key(uint32_t ia, uint32_t ib) { return ia < ib ? EdgeKey{ia, ib} : EdgeKey{ib, ia}; }

/** process an edge during the build phase */
static void process_edge(TriangleMeshShape::Data &data, collections::Map<EdgeKey, EdgeRefs> &edge_map, uint32_t ia, uint32_t ib, uint32_t tri_idx,
                         uint32_t edge_bit) {
    const EdgeKey key = edge_key(ia, ib);
    EdgeRefs *refs = edge_map.get(key);

    if (refs == nullptr) {
        // first triangle using this edge
        edge_map.set(key, {1, (tri_idx << 3) | edge_bit, 0});
        return;
    }
    if (refs->count == 1) {
        // second triangle: store it
        refs->tri1 = (tri_idx << 3) | edge_bit;
        refs->count = 2;
    } else {
        // third or more triangle: non-manifold edge, mark as active immediately without storing the reference
        data.triangles[tri_idx].active_edges |= edge_bit;
        refs->count++;
    }
}

/*
 * compute which edges are "active" (sharp) based on dihedral angles.
 *
 * an edge is active if:
 * - it's a boundary edge (only one triangle uses it)
 * - it's a non-manifold edge (more than 2 triangles use it)
 * - the dihedral angle threshold is disabled (cos_threshold < 0)
 * - the edge is convex and the dihedral angle between adjacent triangles exceeds the threshold (cos(angle) < cos_threshold)
 *
 * concave edges between two triangles are never active.
 *
 * active edges are important for collision detection to avoid smoothing at sharp features.
 */
static void compute_active_edges(TriangleMeshShape::Data &data, float cos_threshold) {
    // early exit: if threshold is negative, all edges remain active
    if (cos_threshold < 0.0f) return;

    // build edge-to-triangles map
    collections::Map<EdgeKey, EdgeRefs> edge_map = {};

    // phase 1: build - collect edges and mark non-manifold edges immediately
    for (uint32_t tri_idx = 0; tri_idx < data.triangle_count; tri_idx++) {
        const TriangleMeshShape::Triangle &triangle = data.triangles[tri_idx];

        // process three edges: AB (bit 0), BC (bit 1), CA (bit 2)
        process_edge(data, edge_map, triangle.index_a, triangle.index_b, tri_idx, 0b001);
        process_edge(data, edge_map, triangle.index_b, triangle.index_c, tri_idx, 0b010);
        process_edge(data, edge_map, triangle.index_c, triangle.index_a, tri_idx, 0b100);
    }

    // phase 2: process - check manifold edges for smoothness
    for (uint32_t slot = 0; slot < edge_map.capacity; slot++) {
        if (!edge_map.slots[slot].used) continue;
        const EdgeRefs &refs = edge_map.slots[slot].value;

        // only process manifold edges (exactly 2 triangles share this edge); boundary edges
        // (count 1) and non-manifold edges (count > 2) remain active
        if (refs.count != 2) continue;

        // unpack triangle indices and edge masks
        const uint32_t tri_idx0 = refs.tri0 >> 3;
        const uint32_t edge_mask0 = refs.tri0 & 0b111;
        const uint32_t tri_idx1 = refs.tri1 >> 3;
        const uint32_t edge_mask1 = refs.tri1 & 0b111;

        TriangleMeshShape::Triangle &triangle0 = data.triangles[tri_idx0];
        TriangleMeshShape::Triangle &triangle1 = data.triangles[tri_idx1];

        // edge direction as wound by the first triangle: AB (bit 0), BC (bit 1), CA (bit 2)
        const uint32_t edge_start_index =
            edge_mask0 == 0b001 ? triangle0.index_a : edge_mask0 == 0b010 ? triangle0.index_b : triangle0.index_c;
        const uint32_t edge_end_index =
            edge_mask0 == 0b001 ? triangle0.index_b : edge_mask0 == 0b010 ? triangle0.index_c : triangle0.index_a;
        const Vec3f edge_direction = vec3f::subtract(vec3f::load(data.positions, edge_end_index * 3), vec3f::load(data.positions, edge_start_index * 3));

        // concave edges are never active, convex edges are active past the threshold angle
        if (!active_edges::is_edge_active(vec3f::load(triangle0.normal), vec3f::load(triangle1.normal), edge_direction, cos_threshold)) {
            // mark edge as inactive in both triangles
            triangle0.active_edges &= ~edge_mask0;
            triangle1.active_edges &= ~edge_mask1;
        }
        // else: sharp convex edge, edges remain active (initialized to 0b111)
    }
}

namespace {

/** what build works in, freed with it */
struct Scratch {
    collections::Array<float> positions;
    collections::Array<uint32_t> index_map;
    collections::Array<TriangleMeshShape::Triangle> triangles;
    collections::Map<TriangleKey, uint32_t> seen;
    collections::Array<BvhNode> nodes;
};

}  // namespace

TriangleMeshShape::Data *build(const TriangleMeshShapeSettings &settings) {
    const float tolerance = settings.degenerate_tolerance;
    Scratch scratch = {};

    /* deduplicate vertices */
    deduplicate_vertices(scratch.positions, scratch.index_map, settings.positions, settings.num_positions);
    const float *positions = scratch.positions.data;

    /* create initial triangles */
    const uint32_t raw_triangle_count = settings.num_indices / 3;
    scratch.triangles.reserve(raw_triangle_count);
    collections::Array<TriangleMeshShape::Triangle> &triangles = scratch.triangles;

    for (uint32_t i = 0; i < raw_triangle_count; i++) {
        // get vertex indices from input
        const uint32_t ia_input = settings.indices[i * 3 + 0];
        const uint32_t ib_input = settings.indices[i * 3 + 1];
        const uint32_t ic_input = settings.indices[i * 3 + 2];

        // map to deduplicated indices
        const uint32_t ia = scratch.index_map[ia_input];
        const uint32_t ib = scratch.index_map[ib_input];
        const uint32_t ic = scratch.index_map[ic_input];

        // get vertex positions
        const Vec3f a = vec3f::load(positions, ia * 3);
        const Vec3f b = vec3f::load(positions, ib * 3);
        const Vec3f c = vec3f::load(positions, ic * 3);

        // check for degeneracy
        const Vec3f edge1 = vec3f::subtract(b, a);
        const Vec3f edge2 = vec3f::subtract(c, a);
        const Vec3f cross_product = vec3f::cross(edge1, edge2);

        const float cross_magnitude = vec3f::length(cross_product);
        if (cross_magnitude < tolerance) {
            // degenerate triangle, skip
            continue;
        }

        // compute and normalize normal
        TriangleMeshShape::Triangle triangle;
        triangle.index_a = ia;
        triangle.index_b = ib;
        triangle.index_c = ic;
        vec3f::store(triangle.normal, vec3f::normalize(cross_product));
        // all active, updated by compute_active_edges
        triangle.active_edges = 0b111;
        triangle.material_id = settings.material_ids != nullptr ? settings.material_ids[i] : -1;
        triangles.push(triangle);
    }

    /* remove duplicate triangles */
    const uint32_t valid_triangle_count = triangles.length;
    uint32_t write_idx = 0;

    for (uint32_t read_idx = 0; read_idx < valid_triangle_count; read_idx++) {
        const TriangleMeshShape::Triangle &triangle = triangles[read_idx];
        const uint32_t ia = triangle.index_a;
        const uint32_t ib = triangle.index_b;
        const uint32_t ic = triangle.index_c;

        // rotate indices so lowest comes first (preserves winding order)
        TriangleKey key;
        if (ia < ib) {
            key = ia < ic ? TriangleKey{ia, ib, ic, 0} : TriangleKey{ic, ia, ib, 0};
        } else {
            key = ib < ic ? TriangleKey{ib, ic, ia, 0} : TriangleKey{ic, ia, ib, 0};
        }
        key.material_id = triangle.material_id;

        if (scratch.seen.get(key) == nullptr) {
            scratch.seen.set(key, 1);
            if (write_idx != read_idx) triangles[write_idx] = triangles[read_idx];
            write_idx++;
        }
    }
    triangles.length = write_idx;

    // the mesh over the scratch arrays, until the bvh fixes the triangle order and node count
    TriangleMeshShape::Data working = {};
    working.vertex_count = scratch.positions.length / 3;
    working.triangle_count = triangles.length;
    working.positions = scratch.positions.data;
    working.triangles = triangles.data;

    /* compute active edges based on dihedral angles */
    compute_active_edges(working, settings.active_edge_cos_threshold_angle);

    /* build bvh with cleaned triangles */
    const TriangleMeshBvhBuildSettings bvh_settings = {settings.bvh_split_strategy, settings.bvh_max_leaf_tris};
    triangle_mesh_bvh::build(scratch.nodes, working, bvh_settings);

    TriangleMeshShape::Data *data = triangle_mesh_data::create(working.vertex_count, working.triangle_count, scratch.nodes.length);
    if (working.vertex_count > 0) util::copy(data->positions, working.positions, working.vertex_count * 3 * sizeof(float));
    if (working.triangle_count > 0) util::copy(data->triangles, working.triangles, working.triangle_count * sizeof(TriangleMeshShape::Triangle));
    if (scratch.nodes.length > 0) util::copy(data->nodes, scratch.nodes.data, scratch.nodes.length * sizeof(BvhNode));

    /* precompute per-triangle aabbs (after the bvh build, which reorders triangles) */
    triangle_mesh_data::precompute_triangle_aabbs(*data);
    return data;
}

}  // namespace triangle_mesh_builder

TUMBLE_NAMESPACE_END
