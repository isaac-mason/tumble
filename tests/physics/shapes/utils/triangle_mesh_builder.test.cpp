#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"

#include "tumble/math/scalar.h"
#include "test.h"
#include "test_math.h"
#include "tumble/util/alloc.h"
#include "tumble/util/bits.h"

using triangle_mesh_data::get_active_edges;

namespace builder = triangle_mesh_builder;

static constexpr float ACTIVE_EDGE_COS_THRESHOLD_ANGLE = 0.996195f;

/** sah, two triangles per leaf */
template <uint32_t NUM_FLOATS, uint32_t NUM_INDICES>
static TriangleMeshShape::Data *build_mesh(const float (&positions)[NUM_FLOATS], const uint32_t (&indices)[NUM_INDICES],
                                    float degenerate_tolerance = 1e-6f, float cos_threshold = ACTIVE_EDGE_COS_THRESHOLD_ANGLE) {
    TriangleMeshShapeSettings settings;
    settings.positions = positions;
    settings.num_positions = NUM_FLOATS / 3;
    settings.indices = indices;
    settings.num_indices = NUM_INDICES;
    settings.degenerate_tolerance = degenerate_tolerance;
    settings.bvh_split_strategy = TriangleMeshShape::BvhSplitStrategy::SAH;
    settings.bvh_max_leaf_tris = 2;
    settings.active_edge_cos_threshold_angle = cos_threshold;
    return builder::build(settings);
}

/** the triangle count of the built mesh, which is then destroyed */
template <uint32_t NUM_FLOATS, uint32_t NUM_INDICES>
static uint32_t built_triangle_count(const float (&positions)[NUM_FLOATS], const uint32_t (&indices)[NUM_INDICES],
                                     float degenerate_tolerance = 1e-6f) {
    TriangleMeshShape::Data *data = build_mesh(positions, indices, degenerate_tolerance);
    const uint32_t count = data->triangle_count;
    triangle_mesh_data::destroy(data);
    return count;
}

void test_triangle_mesh_builder_basic(TestCtx &ctx) {

    // builds valid non-degenerate triangle
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 1, 2};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data != nullptr);
        CHECK(ctx, data->triangle_count == 1 && data->vertex_count == 3);
        CHECK(ctx, data->node_count == 1);
        // the normal is unit, from the counter clockwise winding
        CHECK(ctx, data->triangles[0].normal.x == 0 && data->triangles[0].normal.y == 0 && data->triangles[0].normal.z == 1);
        CHECK(ctx, data->triangles[0].material_id == -1);
        triangle_mesh_data::destroy(data);
    }

    // builds mesh with multiple valid triangles
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1};
        const uint32_t indices[] = {0, 1, 2, 1, 2, 3};
        CHECK(ctx, built_triangle_count(positions, indices) == 2);
    }
}

void test_triangle_mesh_builder_degenerate_removal(TestCtx &ctx) {

    // removes collinear triangle (zero area)
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 3, 4, 5};
        CHECK(ctx, built_triangle_count(positions, indices) == 1);
    }

    // removes triangle with repeated vertices
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 0, 1};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 0 && data->node_count == 0);
        triangle_mesh_data::destroy(data);
    }

    // removes multiple degenerate triangles
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 2, 3, 4, 5, 6, 7};
        CHECK(ctx, built_triangle_count(positions, indices) == 1);
    }

    // respects custom degenerate tolerance: the cross product's length is 1e-10
    {
        const float positions[] = {0, 0, 0, 0.00001f, 0, 0, 0, 0.00001f, 0};
        const uint32_t indices[] = {0, 1, 2};
        // in f32 the length lands just under 1e-10, where the ts's doubles land just over it
        CHECK(ctx, built_triangle_count(positions, indices, 1e-11f) == 1);
        CHECK(ctx, built_triangle_count(positions, indices, 1e-2f) == 0);
    }
}

void test_triangle_mesh_builder_duplicate_removal(TestCtx &ctx) {

    // removes exact duplicate triangles (same vertex order)
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 0, 1, 2};
        CHECK(ctx, built_triangle_count(positions, indices) == 1);
    }

    // removes duplicate triangles with rotated vertex order
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 1, 2, 0};
        CHECK(ctx, built_triangle_count(positions, indices) == 1);
    }

    // keeps the reversed winding: it faces the other way
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 0, 2, 1};
        CHECK(ctx, built_triangle_count(positions, indices) == 2);
    }

    // keeps unique triangles
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 1, 2, 3};
        CHECK(ctx, built_triangle_count(positions, indices) == 2);
    }

    // removes multiple duplicates of same triangle
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 0, 1, 2, 1, 2, 0};
        CHECK(ctx, built_triangle_count(positions, indices) == 1);
    }

    // handles floating point precision in duplicate detection: vertices within the weld grid merge
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0.000000001f, 0.000000001f, 0.000000001f,
                                   1.000000001f, 0, 0, 0, 1.000000001f, 0};
        const uint32_t indices[] = {0, 1, 2, 3, 4, 5};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 1 && data->vertex_count == 3);
        triangle_mesh_data::destroy(data);
    }

    // -0 welds with 0
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, -0.0f, -0.0f, -0.0f};
        const uint32_t indices[] = {0, 1, 2, 3, 1, 2};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 1 && data->vertex_count == 3);
        triangle_mesh_data::destroy(data);
    }

    // the same triangle with different materials is kept twice
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 0, 1, 2, 1, 2, 0};
        const int32_t material_ids[] = {3, 4, 3};
        TriangleMeshShapeSettings settings;
        settings.positions = positions;
        settings.num_positions = 3;
        settings.indices = indices;
        settings.num_indices = 9;
        settings.material_ids = material_ids;
        TriangleMeshShape::Data *data = builder::build(settings);
        CHECK(ctx, data->triangle_count == 2);
        const int32_t a = triangle_mesh_data::get_material_id(*data, 0), b = triangle_mesh_data::get_material_id(*data, 1);
        CHECK(ctx, (a == 3 && b == 4) || (a == 4 && b == 3));
        triangle_mesh_data::destroy(data);
    }
}

void test_triangle_mesh_builder_combined_removal(TestCtx &ctx) {

    // removes both degenerate and duplicate triangles
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 0, 0, 1, 1, 1};
        const uint32_t indices[] = {0, 1, 2, 0, 1, 3, 1, 2, 4};
        CHECK(ctx, built_triangle_count(positions, indices) == 2);
    }

    // removes both degenerate and duplicate in same mesh
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 0, 0};
        const uint32_t indices[] = {0, 1, 2, 0, 1, 3, 0, 1, 2};
        CHECK(ctx, built_triangle_count(positions, indices) == 1);
    }

    // sequential triangles after cleanup: welded copies, a degenerate and a duplicate leave two
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1};
        const uint32_t indices[] = {0, 1, 2, 0, 1, 3, 4, 5, 6, 1, 2, 7};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 2 && data->vertex_count == 5);
        triangle_mesh_data::destroy(data);
    }

    // all triangles removed except one
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
        CHECK(ctx, built_triangle_count(positions, indices) == 1);
    }
}

void test_triangle_mesh_builder_bvh(TestCtx &ctx) {

    // builds valid bvh after cleanup: the root bounds every triangle
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1};
        const uint32_t indices[] = {0, 1, 2, 1, 2, 3};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->node_count >= 1);
        const Box3f &root = data->nodes[0].bounds;
        CHECK(ctx, root.min.x == 0 && root.min.y == 0 && root.min.z == 0 && root.max.x == 1 && root.max.y == 1 && root.max.z == 1);
        triangle_mesh_data::destroy(data);
    }

    // builds bvh with multiple triangles after degenerate removal
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 0, 0, 1, 1, 1, 2, 1, 1};
        const uint32_t indices[] = {0, 1, 2, 0, 1, 3, 1, 2, 4};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 2);
        CHECK(ctx, data->node_count >= 1);
        triangle_mesh_data::destroy(data);
    }
}

void test_triangle_mesh_builder_edge_cases(TestCtx &ctx) {
    const size_t before = util::allocated_bytes();

    // handles empty mesh
    {
        TriangleMeshShapeSettings settings;
        TriangleMeshShape::Data *data = builder::build(settings);
        CHECK(ctx, data != nullptr);
        CHECK(ctx, data->triangle_count == 0 && data->vertex_count == 0 && data->node_count == 0);
        triangle_mesh_data::destroy(data);
    }

    // handles mesh with single vertex (degenerate)
    {
        const float positions[] = {0, 0, 0};
        const uint32_t indices[] = {0, 0, 0};
        CHECK(ctx, built_triangle_count(positions, indices) == 0);
    }

    // handles very large mesh: 100 separate triangles, no shared vertices to weld
    {
        float positions[100 * 9];
        uint32_t indices[100 * 3];
        uint32_t written = 0;
        for (uint32_t x = 0; x < 10; x++) {
            for (uint32_t y = 0; y < 10; y++) {
                const float corner[9] = {(float)x, (float)y, 0, (float)x + 1, (float)y, 0, (float)x, (float)y + 1, 0};
                for (uint32_t i = 0; i < 9; i++) positions[written * 9 + i] = corner[i];
                for (uint32_t i = 0; i < 3; i++) indices[written * 3 + i] = written * 3 + i;
                written++;
            }
        }
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 100);
        // a full binary tree with at most 2 triangles per leaf: at least 50 leaves, so at least 99 nodes
        CHECK(ctx, data->node_count >= 99);
        triangle_mesh_data::destroy(data);
    }

    // everything the builds allocated was given back
    CHECK(ctx, util::allocated_bytes() == before);
}

void test_triangle_mesh_builder_active_edges(TestCtx &ctx) {

    // single triangle has all edges active: they are boundary edges
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        const uint32_t indices[] = {0, 1, 2};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 1);
        CHECK(ctx, get_active_edges(*data, 0) == 0b111);
        triangle_mesh_data::destroy(data);
    }

    // smooth adjacent triangles have shared edge inactive: (0, 1, 2) and (1, 3, 2) are coplanar,
    // sharing 1-2 as the first's BC and the second's CA
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 1, 3, 2};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 2);
        CHECK(ctx, (get_active_edges(*data, 0) & 0b010) == 0);
        CHECK(ctx, (get_active_edges(*data, 1) & 0b100) == 0);
        // other edges remain active
        CHECK(ctx, (get_active_edges(*data, 0) & 0b101) == 0b101);
        CHECK(ctx, (get_active_edges(*data, 1) & 0b011) == 0b011);
        triangle_mesh_data::destroy(data);
    }

    // sharp adjacent triangles have shared edge active (a 90 degree convex fold)
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1};
        const uint32_t indices[] = {0, 1, 2, 1, 2, 3};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 2);
        CHECK(ctx, (get_active_edges(*data, 0) & 0b010) == 0b010);
        CHECK(ctx, (get_active_edges(*data, 1) & 0b001) == 0b001);
        triangle_mesh_data::destroy(data);
    }

    // back to back triangles keep the shared edge active
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -1, 0};
        const uint32_t indices[] = {0, 1, 2, 0, 1, 3};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 2);
        CHECK(ctx, (get_active_edges(*data, 0) & 0b001) == 0b001);
        CHECK(ctx, (get_active_edges(*data, 1) & 0b001) == 0b001);
        triangle_mesh_data::destroy(data);
    }

    // non-manifold edges remain active: three coplanar triangles on the edge 0-1
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0.5f, 1, 0, 0.5f, -1, 0, 0.5f, 2, 0};
        const uint32_t indices[] = {0, 1, 2, 1, 0, 3, 0, 1, 4};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 3);
        bool all_active = true;
        for (uint32_t t = 0; t < 3; t++) all_active = all_active && get_active_edges(*data, t) == 0b111;
        CHECK(ctx, all_active);
        triangle_mesh_data::destroy(data);
    }

    // mixed smooth and sharp edges: a pyramid whose base is two coplanar triangles
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0.5f, 0.5f, 1};
        const uint32_t indices[] = {0, 1, 2, 0, 2, 3, 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 6);
        bool in_range = true;
        bool has_inactive_edges = false;
        for (uint32_t t = 0; t < data->triangle_count; t++) {
            in_range = in_range && get_active_edges(*data, t) <= 0b111;
            has_inactive_edges = has_inactive_edges || get_active_edges(*data, t) != 0b111;
        }
        CHECK(ctx, in_range);
        CHECK(ctx, has_inactive_edges);
        triangle_mesh_data::destroy(data);
    }

    // all edges with extreme angle difference: a sharp convex ridge on the shared edge (0, 1)
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0.5f, 0.1f, 0, 0.5f, -0.1f, -0.5f};
        const uint32_t indices[] = {0, 1, 2, 1, 0, 3};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 2);
        CHECK(ctx, (get_active_edges(*data, 0) & 0b001) == 0b001);
        CHECK(ctx, (get_active_edges(*data, 1) & 0b001) == 0b001);
        triangle_mesh_data::destroy(data);
    }

    // preserves active edges through the pipeline: a degenerate and a duplicate removed, the
    // remaining pair sharing the edge 0-2 (the first's CA, the second's AB)
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 2, 0, 0, 3, 0, 0, 3.5f, 0, 0};
        const uint32_t indices[] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 0, 1, 2};
        TriangleMeshShape::Data *data = build_mesh(positions, indices);
        CHECK(ctx, data->triangle_count == 2);
        // (0, 2, 3) winds clockwise seen from +z, so the two normals are opposed: back to back, active
        CHECK(ctx, get_active_edges(*data, 0) == 0b111);
        CHECK(ctx, get_active_edges(*data, 1) == 0b111);
        triangle_mesh_data::destroy(data);
    }

    // a negative threshold keeps every edge active
    {
        const float positions[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0};
        const uint32_t indices[] = {0, 1, 2, 1, 3, 2};
        TriangleMeshShape::Data *data = build_mesh(positions, indices, 1e-6f, -1.0f);
        CHECK(ctx, get_active_edges(*data, 0) == 0b111 && get_active_edges(*data, 1) == 0b111);
        triangle_mesh_data::destroy(data);
    }
}

/**
 * the active edges by fold angle: triangle 0 = (0, 1, 2) lies in z = 0 with normal +z, triangle
 * 1 = (1, 0, 3) shares the edge from (0, 0, 0) to (1, 0, 0) as its AB edge too. the fold is the
 * angle between the two normals: folding triangle 1 below the plane makes a convex ridge, folding
 * it above makes a concave valley.
 */
static TriangleMeshShape::Data *build_fold(float fold_degrees, bool valley) {
    const float fold = fold_degrees * scalar::PI / 180.0f;
    const float z = valley ? scalar::sin(fold) : -scalar::sin(fold);
    const float positions[] = {0, 0, 0, 1, 0, 0, 0.5f, 1, 0, 0.5f, -scalar::cos(fold), z};
    const uint32_t indices[] = {0, 1, 2, 1, 0, 3};
    return build_mesh(positions, indices, 1e-6f, scalar::cos(10.0f * scalar::PI / 180.0f));
}

/** the shared edge is AB on both triangles, the other edges are boundaries and stay active */
static void check_shared_edge(TestCtx &ctx, TriangleMeshShape::Data *data, bool active) {
    CHECK(ctx, data->triangle_count == 2);
    const uint32_t expected = active ? 0b111 : 0b110;
    CHECK(ctx, get_active_edges(*data, 0) == expected);
    CHECK(ctx, get_active_edges(*data, 1) == expected);
    triangle_mesh_data::destroy(data);
}

void test_triangle_mesh_builder_active_edges_by_fold_angle(TestCtx &ctx) {

    // the shared edge of a flat quad is inactive
    check_shared_edge(ctx, build_fold(0, false), false);
    // a convex ridge past the threshold is active on both triangles
    check_shared_edge(ctx, build_fold(30, false), true);
    // a concave valley of the same angle is inactive
    check_shared_edge(ctx, build_fold(30, true), false);
    // a convex fold just under the threshold is inactive
    check_shared_edge(ctx, build_fold(9, false), false);
}

/** whether the edge from a to b runs along the border of the square [0, side]^2 in z = 0 */
static bool on_border(const float *a, const float *b, float side) {
    return (a[0] == 0 && b[0] == 0) || (a[0] == side && b[0] == side) || (a[1] == 0 && b[1] == 0) || (a[1] == side && b[1] == side);
}

void test_triangle_mesh_builder_flat_grid(TestCtx &ctx) {

    // a flat 10 x 10 grid in z = 0, every triangle with its own three vertices, counter clockwise from +z
    constexpr uint32_t SIDE = 10;
    constexpr uint32_t TRIANGLES = SIDE * SIDE * 2;
    float positions[TRIANGLES * 9];
    uint32_t indices[TRIANGLES * 3];
    int32_t material_ids[TRIANGLES];
    uint32_t written = 0;
    for (uint32_t y = 0; y < SIDE; y++) {
        for (uint32_t x = 0; x < SIDE; x++) {
            const float x0 = (float)x, y0 = (float)y, x1 = x0 + 1, y1 = y0 + 1;
            const float quad[2][9] = {{x0, y0, 0, x1, y0, 0, x1, y1, 0}, {x0, y0, 0, x1, y1, 0, x0, y1, 0}};
            for (uint32_t k = 0; k < 2; k++) {
                for (uint32_t i = 0; i < 9; i++) positions[written * 9 + i] = quad[k][i];
                for (uint32_t i = 0; i < 3; i++) indices[written * 3 + i] = written * 3 + i;
                material_ids[written] = (int32_t)written;
                written++;
            }
        }
    }

    TriangleMeshShapeSettings settings;
    settings.positions = positions;
    settings.num_positions = TRIANGLES * 3;
    settings.indices = indices;
    settings.num_indices = TRIANGLES * 3;
    settings.material_ids = material_ids;
    TriangleMeshShape::Data *data = builder::build(settings);

    // the copies weld into the grid's vertices, and every triangle survives
    CHECK(ctx, data->vertex_count == (SIDE + 1) * (SIDE + 1));
    CHECK(ctx, data->triangle_count == TRIANGLES);

    // only the border edges are active; every interior edge joins two coplanar triangles
    bool edges_ok = true;
    bool normals_ok = true;
    bool aabbs_ok = true;
    bool materials_ok = true;
    bool seen[TRIANGLES] = {};
    for (uint32_t t = 0; t < data->triangle_count; t++) {
        const TriangleMeshShape::Triangle &triangle = data->triangles[t];
        const float *a = data->positions + triangle.index_a * 3;
        const float *b = data->positions + triangle.index_b * 3;
        const float *c = data->positions + triangle.index_c * 3;
        const uint32_t expected =
            (on_border(a, b, SIDE) ? 0b001u : 0u) | (on_border(b, c, SIDE) ? 0b010u : 0u) | (on_border(c, a, SIDE) ? 0b100u : 0u);
        edges_ok = edges_ok && triangle.active_edges == expected;
        normals_ok = normals_ok && triangle.normal.x == 0 && triangle.normal.y == 0 && triangle.normal.z == 1;

        // the precomputed aabbs follow the final triangle order
        Box3f aabb;
        triangle_mesh_data::calculate_triangle_aabb(aabb, *data, t);
        const float *stored = data->triangle_aabbs + t * 6;
        aabbs_ok = aabbs_ok && stored[0] == aabb.min.x && stored[1] == aabb.min.y && stored[2] == aabb.min.z && stored[3] == aabb.max.x &&
                   stored[4] == aabb.max.y && stored[5] == aabb.max.z;

        // each material id once: the triangles were moved whole
        const int32_t material_id = triangle.material_id;
        materials_ok = materials_ok && material_id >= 0 && material_id < (int32_t)TRIANGLES && !seen[material_id];
        if (materials_ok) seen[material_id] = true;
    }
    CHECK(ctx, edges_ok);
    CHECK(ctx, normals_ok);
    CHECK(ctx, aabbs_ok);
    CHECK(ctx, materials_ok);

    // the border is 4 * SIDE unit edges, each on exactly one triangle
    uint32_t active_edge_count = 0;
    for (uint32_t t = 0; t < data->triangle_count; t++) active_edge_count += bits::count_ones(data->triangles[t].active_edges);
    CHECK(ctx, active_edge_count == 4 * SIDE);

    // the default settings: a center-split bvh, at most 8 triangles a leaf, the root over the grid
    const Box3f &root = data->nodes[0].bounds;
    CHECK(ctx, root.min.x == 0 && root.min.y == 0 && root.max.x == (float)SIDE && root.max.y == (float)SIDE && root.max.z == 0);
    const TriangleMeshBvhStats stats = triangle_mesh_bvh::stats(data->nodes, data->node_count);
    CHECK(ctx, stats.node_count == data->node_count && stats.leaf_count >= TRIANGLES / 8);

    triangle_mesh_data::destroy(data);
}
