#include "tumble/physics/shapes/utils/triangle_mesh_bvh.h"
#include "tumble/math/aabox.h"

#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "test.h"
#include "test_math.h"
#include "tumble/util/alloc.h"
#include "tumble/util/collections.h"


namespace mesh_bvh = triangle_mesh_bvh;

/** cells per side of the test grid; two triangles per cell */
static constexpr uint32_t GRID = 12;
static constexpr uint32_t GRID_TRIANGLES = GRID * GRID * 2;

/**
 * a GRID x GRID heightfield over x, z in [0, GRID], heights within +-0.25. cell (i, j) holds
 * triangle 2k = (v00, v01, v11), covering local z > x, and 2k + 1 = (v00, v11, v10), covering
 * local x > z, both facing +y, with k = j * GRID + i. each triangle's material id is its index
 * here, so it can be found after the build reorders them.
 */
static TriangleMeshShape::Data *make_grid() {
    const uint32_t side = GRID + 1;
    TriangleMeshShape::Data *data = triangle_mesh_data::create(side * side, GRID_TRIANGLES, 0);
    for (uint32_t j = 0; j < side; j++) {
        for (uint32_t i = 0; i < side; i++) {
            float *p = data->positions + (j * side + i) * 3;
            p[0] = (float)i;
            p[1] = 0.25f * scalar::sin(0.7f * (float)i) * scalar::cos(0.5f * (float)j);
            p[2] = (float)j;
        }
    }
    for (uint32_t j = 0; j < GRID; j++) {
        for (uint32_t i = 0; i < GRID; i++) {
            const uint32_t v00 = j * side + i, v10 = v00 + 1, v01 = v00 + side, v11 = v01 + 1;
            const uint32_t k = j * GRID + i;
            data->triangles[2 * k] = {v00, v01, v11, {0, 1, 0}, 0b111, (int32_t)(2 * k)};
            data->triangles[2 * k + 1] = {v00, v11, v10, {0, 1, 0}, 0b111, (int32_t)(2 * k + 1)};
        }
    }
    return data;
}

/** the grid triangle under (x, z), by its original index */
static uint32_t grid_triangle_under(float x, float z) {
    const uint32_t i = (uint32_t)scalar::floor(x);
    const uint32_t j = (uint32_t)scalar::floor(z);
    const float u = x - (float)i, w = z - (float)j;
    return (j * GRID + i) * 2 + (w > u ? 0 : 1);
}

/** moller-trumbore, both sides: whether the ray hits within [0, length] */
static bool ray_hits_triangle(Vec3f origin, Vec3f direction, float length, Vec3f a, Vec3f b, Vec3f c) {
    const Vec3f e1 = vec3f::subtract(b, a);
    const Vec3f e2 = vec3f::subtract(c, a);
    const Vec3f p = vec3f::cross(direction, e2);
    const float det = vec3f::dot(e1, p);
    if (scalar::abs(det) < 1e-12f) return false;
    const float inv_det = 1.0f / det;
    const Vec3f s = vec3f::subtract(origin, a);
    const float u = vec3f::dot(s, p) * inv_det;
    if (u < 0.0f || u > 1.0f) return false;
    const Vec3f q = vec3f::cross(s, e1);
    const float v = vec3f::dot(direction, q) * inv_det;
    if (v < 0.0f || u + v > 1.0f) return false;
    const float t = vec3f::dot(e2, q) * inv_det;
    return t >= 0.0f && t <= length;
}

/** marks hits[original index] for every triangle in a leaf the box overlaps whose own aabb overlaps the box */
static void query_box(bool *hits, const BvhNode *nodes, const TriangleMeshShape::Data &data, const Box3f &box) {
    int32_t stack[128];
    uint32_t size = 0;
    stack[size++] = 0;
    while (size > 0) {
        const int32_t node = stack[--size];
        if (!bvh::node_intersects_box(nodes, node, aabox::load(box))) continue;
        if (bvh::node_is_leaf(nodes, node)) {
            const uint32_t start = mesh_bvh::node_tri_start(nodes, node);
            const uint32_t count = mesh_bvh::node_tri_count(nodes, node);
            for (uint32_t t = start; t < start + count; t++) {
                Box3f aabb;
                triangle_mesh_data::calculate_triangle_aabb(aabb, data, t);
                if (aabox::intersects(aabox::load(aabb), aabox::load(box))) hits[data.triangles[t].material_id] = true;
            }
        } else {
            stack[size++] = bvh::node_right(nodes, node);
            stack[size++] = bvh::node_left(node);
        }
    }
}

/** marks hits[original index] for every triangle the ray hits, found through the tree */
static void query_ray(bool *hits, const BvhNode *nodes, const TriangleMeshShape::Data &data, Vec3f origin, Vec3f direction, float length) {
    int32_t stack[128];
    uint32_t size = 0;
    stack[size++] = 0;
    while (size > 0) {
        const int32_t node = stack[--size];
        if (!bvh::node_intersects_ray(nodes, node, origin, direction, 0.0f, length)) continue;
        if (bvh::node_is_leaf(nodes, node)) {
            const uint32_t start = mesh_bvh::node_tri_start(nodes, node);
            const uint32_t count = mesh_bvh::node_tri_count(nodes, node);
            for (uint32_t t = start; t < start + count; t++) {
                Vec3f a, b, c;
                triangle_mesh_data::get_triangle_vertices(a, b, c, data, t);
                if (ray_hits_triangle(origin, direction, length, a, b, c)) hits[data.triangles[t].material_id] = true;
            }
        } else {
            stack[size++] = bvh::node_right(nodes, node);
            stack[size++] = bvh::node_left(node);
        }
    }
}

/** checks the tree is well formed over data; strict_leaves when every leaf must be within max_leaf_tris */
static void check_tree(TestCtx &ctx, const collections::Array<BvhNode> &tree, const TriangleMeshShape::Data &data, const TriangleMeshShape::Data &original,
                       uint32_t max_leaf_tris, bool strict_leaves) {
    const BvhNode *nodes = tree.data;
    uint32_t covered[GRID_TRIANGLES] = {};
    bool bounds_ok = true;
    bool leaves_ok = true;
    uint32_t visited = 0;

    int32_t stack[128];
    uint32_t size = 0;
    stack[size++] = 0;
    while (size > 0) {
        const int32_t node = stack[--size];
        visited++;
        const Box3f &bounds = nodes[node].bounds;
        if (bvh::node_is_leaf(nodes, node)) {
            const uint32_t start = mesh_bvh::node_tri_start(nodes, node);
            const uint32_t count = mesh_bvh::node_tri_count(nodes, node);
            leaves_ok = leaves_ok && count >= 1 && start + count <= data.triangle_count;
            if (strict_leaves) leaves_ok = leaves_ok && count <= max_leaf_tris;
            for (uint32_t t = start; t < start + count && t < data.triangle_count; t++) {
                covered[t]++;
                Box3f aabb;
                triangle_mesh_data::calculate_triangle_aabb(aabb, data, t);
                bounds_ok = bounds_ok && aabox::contains(aabox::load(bounds), aabox::load(aabb));
            }
        } else {
            const int32_t left = bvh::node_left(node);
            const int32_t right = bvh::node_right(nodes, node);
            bounds_ok = bounds_ok && aabox::contains(aabox::load(bounds), aabox::load(nodes[left].bounds)) && aabox::contains(aabox::load(bounds), aabox::load(nodes[right].bounds));
            bounds_ok = bounds_ok && bvh::node_split_axis(nodes, node) >= 0 && bvh::node_split_axis(nodes, node) <= 2;
            stack[size++] = right;
            stack[size++] = left;
        }
    }

    // every node reachable, every triangle in exactly one leaf
    CHECK(ctx, visited == tree.length);
    bool each_once = true;
    for (uint32_t t = 0; t < data.triangle_count; t++) each_once = each_once && covered[t] == 1;
    CHECK(ctx, each_once);
    CHECK(ctx, bounds_ok);
    CHECK(ctx, leaves_ok);

    // the triangles were permuted whole: each original appears once, with its own indices
    uint32_t seen[GRID_TRIANGLES] = {};
    bool permuted = true;
    for (uint32_t t = 0; t < data.triangle_count; t++) {
        const TriangleMeshShape::Triangle &triangle = data.triangles[t];
        const TriangleMeshShape::Triangle &source = original.triangles[triangle.material_id];
        seen[triangle.material_id]++;
        permuted = permuted && triangle.index_a == source.index_a && triangle.index_b == source.index_b &&
                   triangle.index_c == source.index_c;
    }
    for (uint32_t t = 0; t < data.triangle_count; t++) permuted = permuted && seen[t] == 1;
    CHECK(ctx, permuted);

    // the root bounds the whole grid
    const Box3f &root = nodes[0].bounds;
    CHECK(ctx, root.min.x == 0 && root.min.z == 0 && root.max.x == (float)GRID && root.max.z == (float)GRID);
}

static void check_queries(TestCtx &ctx, const collections::Array<BvhNode> &tree, const TriangleMeshShape::Data &data) {
    const BvhNode *nodes = tree.data;

    // boxes spanning every height over a range of cells: exactly the triangles of the cells they overlap
    const float boxes[][4] = {
        {0.2f, 0.3f, 0.4f, 0.6f},    // inside one cell
        {2.5f, 3.5f, 6.25f, 8.75f},  // five columns, six rows
        {-3.0f, -3.0f, 0.5f, 20.0f}, // the first column, hanging off the grid
        {4.1f, 4.1f, 7.9f, 7.9f},    // a 4 x 4 block
        {0.0f, 0.0f, 12.0f, 12.0f},  // everything
        {12.5f, 0.0f, 14.0f, 3.0f},  // off the grid
        {5.5f, 0.0f, 5.75f, 12.0f},  // one column end to end
    };
    bool boxes_ok = true;
    for (const float *b : boxes) {
        const Box3f box = {{b[0], -1.0f, b[1]}, {b[2], 1.0f, b[3]}};
        bool hits[GRID_TRIANGLES] = {};
        query_box(hits, nodes, data, box);
        for (uint32_t j = 0; j < GRID; j++) {
            for (uint32_t i = 0; i < GRID; i++) {
                const bool under = (float)i <= b[2] && (float)(i + 1) >= b[0] && (float)j <= b[3] && (float)(j + 1) >= b[1];
                const uint32_t k = j * GRID + i;
                boxes_ok = boxes_ok && hits[2 * k] == under && hits[2 * k + 1] == under;
            }
        }
    }
    CHECK(ctx, boxes_ok);

    // a box above the surface finds nothing
    {
        bool hits[GRID_TRIANGLES] = {};
        query_box(hits, nodes, data, {{0, 0.5f, 0}, {12, 2, 12}});
        bool none = true;
        for (bool hit : hits) none = none && !hit;
        CHECK(ctx, none);
    }

    // straight down: exactly the one triangle under the point
    const float points[][2] = {{0.3f, 0.7f}, {0.7f, 0.3f}, {5.137f, 9.811f}, {11.9f, 0.05f}, {6.62f, 6.41f}, {2.01f, 10.97f}};
    bool down_ok = true;
    for (const float *point : points) {
        bool hits[GRID_TRIANGLES] = {};
        query_ray(hits, nodes, data, vec3f::make(point[0], 5.0f, point[1]), vec3f::make(0, -1, 0), 10.0f);
        const uint32_t expected = grid_triangle_under(point[0], point[1]);
        for (uint32_t t = 0; t < GRID_TRIANGLES; t++) down_ok = down_ok && hits[t] == (t == expected);
    }
    CHECK(ctx, down_ok);

    // a ray beside the grid, and one too short to reach it, hit nothing
    {
        bool hits[GRID_TRIANGLES] = {};
        query_ray(hits, nodes, data, vec3f::make(-0.5f, 5.0f, 3.3f), vec3f::make(0, -1, 0), 10.0f);
        query_ray(hits, nodes, data, vec3f::make(3.3f, 5.0f, 3.3f), vec3f::make(0, -1, 0), 4.0f);
        bool none = true;
        for (bool hit : hits) none = none && !hit;
        CHECK(ctx, none);
    }

    // grazing rays through the bumps: the same triangles as testing all of them
    const float rays[][6] = {
        {-1.0f, 0.05f, 0.37f, 1.0f, -0.01f, 0.93f},
        {13.0f, 0.1f, 2.71f, -1.0f, -0.003f, 0.41f},
        {6.13f, 0.2f, -1.0f, 0.07f, -0.02f, 1.0f},
        {0.5f, 3.0f, 0.5f, 1.0f, -0.4f, 1.1f},
    };
    bool grazing_ok = true;
    uint32_t grazing_hits = 0;
    for (const float *r : rays) {
        const Vec3f origin = vec3f::make(r[0], r[1], r[2]);
        const Vec3f direction = vec3f::normalize(vec3f::make(r[3], r[4], r[5]));
        const float length = 30.0f;
        bool hits[GRID_TRIANGLES] = {};
        query_ray(hits, nodes, data, origin, direction, length);
        for (uint32_t t = 0; t < data.triangle_count; t++) {
            Vec3f a, b, c;
            triangle_mesh_data::get_triangle_vertices(a, b, c, data, t);
            const bool expected = ray_hits_triangle(origin, direction, length, a, b, c);
            grazing_ok = grazing_ok && hits[data.triangles[t].material_id] == expected;
            if (expected) grazing_hits++;
        }
    }
    CHECK(ctx, grazing_ok);
    CHECK(ctx, grazing_hits >= 4);
}

void test_triangle_mesh_bvh_grid(TestCtx &ctx) {
    TriangleMeshShape::Data *original = make_grid();

    const TriangleMeshShape::BvhSplitStrategy strategies[] = {TriangleMeshShape::BvhSplitStrategy::CENTER, TriangleMeshShape::BvhSplitStrategy::AVERAGE,
                                                     TriangleMeshShape::BvhSplitStrategy::SAH};
    const uint32_t leaf_sizes[] = {2, 4, 8};
    for (TriangleMeshShape::BvhSplitStrategy strategy : strategies) {
        for (uint32_t max_leaf_tris : leaf_sizes) {
            TriangleMeshShape::Data *data = make_grid();
            collections::Array<BvhNode> tree = {};
            mesh_bvh::build(tree, *data, {strategy, max_leaf_tris});
            CHECK(ctx, tree.length > 1);

            check_tree(ctx, tree, *data, *original, max_leaf_tris, strategy != TriangleMeshShape::BvhSplitStrategy::SAH);
            check_queries(ctx, tree, *data);

            // a full binary tree: one fewer internal node than leaves
            const TriangleMeshBvhStats stats = mesh_bvh::stats(tree.data, tree.length);
            CHECK(ctx, stats.node_count == tree.length);
            CHECK(ctx, stats.leaf_count * 2 - 1 == stats.node_count);
            CHECK(ctx, (float)stats.min_depth <= stats.avg_depth && stats.avg_depth <= (float)stats.max_depth);
            // midpoint splits of a uniform grid stay near balanced
            if (strategy == TriangleMeshShape::BvhSplitStrategy::CENTER) CHECK(ctx, stats.max_depth <= 12);

            tree.release();
            triangle_mesh_data::destroy(data);
        }
    }

    triangle_mesh_data::destroy(original);
}

void test_triangle_mesh_bvh_small(TestCtx &ctx) {

    // no triangles: no nodes, empty stats
    TriangleMeshShape::Data *empty = triangle_mesh_data::create(0, 0, 0);
    collections::Array<BvhNode> tree = {};
    mesh_bvh::build(tree, *empty, {});
    CHECK(ctx, tree.length == 0);
    const TriangleMeshBvhStats empty_stats = mesh_bvh::stats(tree.data, tree.length);
    CHECK(ctx, empty_stats.node_count == 0 && empty_stats.leaf_count == 0 && empty_stats.max_depth == 0);
    triangle_mesh_data::destroy(empty);

    // one triangle: a single leaf bounding it
    TriangleMeshShape::Data *single = triangle_mesh_data::create(3, 1, 0);
    const float positions[9] = {0, 0, 0, 1, 0, 0, 0, 1, 2};
    for (uint32_t i = 0; i < 9; i++) single->positions[i] = positions[i];
    single->triangles[0] = {0, 1, 2, {0, 0, 1}, 0b111, -1};
    mesh_bvh::build(tree, *single, {TriangleMeshShape::BvhSplitStrategy::SAH, 2});
    CHECK(ctx, tree.length == 1);
    CHECK(ctx, bvh::node_is_leaf(tree.data, 0));
    CHECK(ctx, mesh_bvh::node_tri_start(tree.data, 0) == 0 && mesh_bvh::node_tri_count(tree.data, 0) == 1);
    const Box3f &bounds = tree[0].bounds;
    CHECK(ctx, bounds.min.x == 0 && bounds.min.y == 0 && bounds.min.z == 0 && bounds.max.x == 1 && bounds.max.y == 1 && bounds.max.z == 2);
    const TriangleMeshBvhStats single_stats = mesh_bvh::stats(tree.data, tree.length);
    CHECK(ctx, single_stats.node_count == 1 && single_stats.leaf_count == 1 && single_stats.max_depth == 0);
    triangle_mesh_data::destroy(single);

    // stacked copies of one triangle cannot be split: one leaf over all of them, whatever the leaf size
    TriangleMeshShape::Data *stacked = triangle_mesh_data::create(3, 5, 0);
    for (uint32_t i = 0; i < 9; i++) stacked->positions[i] = positions[i];
    for (uint32_t t = 0; t < 5; t++) stacked->triangles[t] = {0, 1, 2, {0, 0, 1}, 0b111, (int32_t)t};
    const TriangleMeshShape::BvhSplitStrategy strategies[] = {TriangleMeshShape::BvhSplitStrategy::CENTER, TriangleMeshShape::BvhSplitStrategy::AVERAGE,
                                                     TriangleMeshShape::BvhSplitStrategy::SAH};
    for (TriangleMeshShape::BvhSplitStrategy strategy : strategies) {
        mesh_bvh::build(tree, *stacked, {strategy, 1});
        CHECK(ctx, tree.length == 1 && mesh_bvh::node_tri_count(tree.data, 0) == 5);
    }
    triangle_mesh_data::destroy(stacked);

    // two triangles far apart along z split on z, nearest first
    TriangleMeshShape::Data *pair = triangle_mesh_data::create(6, 2, 0);
    const float pair_positions[18] = {0, 0, 10, 1, 0, 10, 0, 1, 10, 0, 0, 0, 1, 0, 0, 0, 1, 0};
    for (uint32_t i = 0; i < 18; i++) pair->positions[i] = pair_positions[i];
    pair->triangles[0] = {0, 1, 2, {0, 0, 1}, 0b111, 0};
    pair->triangles[1] = {3, 4, 5, {0, 0, 1}, 0b111, 1};
    for (TriangleMeshShape::BvhSplitStrategy strategy : strategies) {
        mesh_bvh::build(tree, *pair, {strategy, 1});
        CHECK(ctx, tree.length == 3);
        CHECK(ctx, !bvh::node_is_leaf(tree.data, 0) && bvh::node_split_axis(tree.data, 0) == 2);
        CHECK(ctx, bvh::node_right(tree.data, 0) == 2);
        CHECK(ctx, tree[1].bounds.max.z == 0 && tree[2].bounds.min.z == 10);
        CHECK(ctx, pair->triangles[0].material_id == 1 && pair->triangles[1].material_id == 0);
    }
    triangle_mesh_data::destroy(pair);

    tree.release();
}

/** unit right triangles (x, 0, 0), (x + 1, 0, 0), (x, 1, 0) in the xy plane, one per x */
static TriangleMeshShape::Data *make_strip(const float *xs, uint32_t count) {
    TriangleMeshShape::Data *data = triangle_mesh_data::create(count * 3, count, 0);
    for (uint32_t i = 0; i < count; i++) {
        float *p = data->positions + i * 9;
        const float x = xs[i];
        const float vertices[9] = {x, 0, 0, x + 1, 0, 0, x, 1, 0};
        for (uint32_t k = 0; k < 9; k++) p[k] = vertices[k];
        data->triangles[i] = {i * 3, i * 3 + 1, i * 3 + 2, {0, 0, 1}, 0b111, (int32_t)i};
    }
    return data;
}

void test_triangle_mesh_bvh_sah_small_mode_own_candidates(TestCtx &ctx) {

    // five unit triangles at x = 0, 10, 20, 30, 40: aabb centers x = 0.5, 10.5, 20.5, 30.5, 40.5, y = 0.5,
    // z = 0. the center extent is zero on y and z, so only x is tried. surface areas of flat boxes are
    // 2 * dx * dy, all boxes have dy = 1; the leaf cost is 1.25 * n.
    //
    // root: bounds x [0, 41], area 82, leaf cost 6.25. candidates (left | right, cost = 1 + 1.25 * sum / 82):
    //   0.5:  {} | {0..4}, all right, cost 1 + 1.25 * 5 = 7.25
    //   10.5: {0} [0,1] area 2 | {1..4} [10,41] area 62, sum 2 * 1 + 62 * 4 = 250, cost 4.81
    //   20.5: {0,1} [0,11] area 22 | {2,3,4} [20,41] area 42, sum 44 + 126 = 170, cost 3.59
    //   30.5: {0,1,2} [0,21] area 42 | {3,4} [30,41] area 22, sum 126 + 44 = 170, cost 3.59
    //   40.5: {0..3} [0,31] area 62 | {4} area 2, sum 250, cost 4.81
    // 20.5 and 30.5 tie; the first strictly lower cost wins, so the root splits at 20.5:
    // {0,1} is a leaf (2 <= max_leaf_tris), {2,3,4} splits again, with the root's stale bins still
    // holding candidates below its own.
    //
    // {2,3,4}: bounds x [20, 41], area 42, leaf cost 3.75:
    //   20.5: all right, cost 4.75
    //   30.5: {2} area 2 | {3,4} area 22, sum 2 + 44 = 46, cost 1 + 1.25 * 46 / 42 = 2.37
    //   40.5: {2,3} area 22 | {4} area 2, sum 46, cost 2.37
    // it splits at 30.5 into leaves {2} and {3,4}.
    //
    // pre-order: root, leaf {0,1}, node, leaf {2}, leaf {3,4}
    const float xs[5] = {0, 10, 20, 30, 40};
    TriangleMeshShape::Data *data = make_strip(xs, 5);
    collections::Array<BvhNode> tree = {};
    mesh_bvh::build(tree, *data, {TriangleMeshShape::BvhSplitStrategy::SAH, 2});
    CHECK(ctx, tree.length == 5);
    if (tree.length == 5) {
        const BvhNode *nodes = tree.data;
        CHECK(ctx, !bvh::node_is_leaf(nodes, 0) && bvh::node_split_axis(nodes, 0) == 0 && bvh::node_right(nodes, 0) == 2);
        CHECK(ctx, nodes[0].bounds.min.x == 0 && nodes[0].bounds.max.x == 41);
        CHECK(ctx, bvh::node_is_leaf(nodes, 1) && mesh_bvh::node_tri_count(nodes, 1) == 2);
        CHECK(ctx, nodes[1].bounds.min.x == 0 && nodes[1].bounds.max.x == 11);
        CHECK(ctx, !bvh::node_is_leaf(nodes, 2) && bvh::node_split_axis(nodes, 2) == 0 && bvh::node_right(nodes, 2) == 4);
        CHECK(ctx, nodes[2].bounds.min.x == 20 && nodes[2].bounds.max.x == 41);
        CHECK(ctx, bvh::node_is_leaf(nodes, 3) && mesh_bvh::node_tri_count(nodes, 3) == 1);
        CHECK(ctx, nodes[3].bounds.min.x == 20 && nodes[3].bounds.max.x == 21);
        CHECK(ctx, bvh::node_is_leaf(nodes, 4) && mesh_bvh::node_tri_count(nodes, 4) == 2);
        CHECK(ctx, nodes[4].bounds.min.x == 30 && nodes[4].bounds.max.x == 41);
    }

    tree.release();
    triangle_mesh_data::destroy(data);
}

void test_triangle_mesh_bvh_exact_bounds(TestCtx &ctx) {

    // a 9 x 9 vertex grid whose coordinates are sums and products of values with no exact binary form
    const uint32_t side = 9;
    const uint32_t triangle_count = (side - 1) * (side - 1) * 2;
    TriangleMeshShape::Data *original = triangle_mesh_data::create(side * side, triangle_count, 0);
    for (uint32_t j = 0; j < side; j++) {
        for (uint32_t i = 0; i < side; i++) {
            float *p = original->positions + (j * side + i) * 3;
            p[0] = (float)i * 0.1f * 3.0f + (float)j * 0.7e-3f * 7.0f;
            p[1] = 0.1f * scalar::sin(0.7f * (float)i + 0.3f) * scalar::cos(1.1f * (float)j) + 1e-3f * 13.0f * (float)(i % 3);
            p[2] = (float)j * 0.7f * 0.3f - (float)i * 1e-3f * 11.0f;
        }
    }
    for (uint32_t j = 0; j < side - 1; j++) {
        for (uint32_t i = 0; i < side - 1; i++) {
            const uint32_t v00 = j * side + i, v10 = v00 + 1, v01 = v00 + side, v11 = v01 + 1;
            const uint32_t k = j * (side - 1) + i;
            original->triangles[2 * k] = {v00, v01, v11, {0, 1, 0}, 0b111, (int32_t)(2 * k)};
            original->triangles[2 * k + 1] = {v00, v11, v10, {0, 1, 0}, 0b111, (int32_t)(2 * k + 1)};
        }
    }

    const TriangleMeshShape::BvhSplitStrategy strategies[] = {TriangleMeshShape::BvhSplitStrategy::CENTER, TriangleMeshShape::BvhSplitStrategy::AVERAGE,
                                                     TriangleMeshShape::BvhSplitStrategy::SAH};
    const uint32_t leaf_sizes[] = {1, 2, 3, 4, 8};
    for (TriangleMeshShape::BvhSplitStrategy strategy : strategies) {
        for (uint32_t max_leaf_tris : leaf_sizes) {
            TriangleMeshShape::Data *data = triangle_mesh_data::create(side * side, triangle_count, 0);
            for (uint32_t v = 0; v < side * side * 3; v++) data->positions[v] = original->positions[v];
            for (uint32_t t = 0; t < triangle_count; t++) data->triangles[t] = original->triangles[t];

            collections::Array<BvhNode> tree = {};
            mesh_bvh::build(tree, *data, {strategy, max_leaf_tris});
            const BvhNode *nodes = tree.data;

            // every leaf contains its triangles' aabbs and every internal node its children, with no tolerance
            bool leaves_contain = true;
            bool nodes_contain = true;
            uint32_t covered = 0;
            int32_t stack[128];
            uint32_t size = 0;
            stack[size++] = 0;
            while (size > 0) {
                const int32_t node = stack[--size];
                const Box3f &bounds = nodes[node].bounds;
                if (bvh::node_is_leaf(nodes, node)) {
                    const uint32_t start = mesh_bvh::node_tri_start(nodes, node);
                    const uint32_t count = mesh_bvh::node_tri_count(nodes, node);
                    covered += count;
                    for (uint32_t t = start; t < start + count; t++) {
                        Box3f aabb;
                        triangle_mesh_data::calculate_triangle_aabb(aabb, *data, t);
                        leaves_contain = leaves_contain && aabox::contains(aabox::load(bounds), aabox::load(aabb));
                    }
                } else {
                    const int32_t left = bvh::node_left(node);
                    const int32_t right = bvh::node_right(nodes, node);
                    nodes_contain = nodes_contain && aabox::contains(aabox::load(bounds), aabox::load(nodes[left].bounds)) &&
                                    aabox::contains(aabox::load(bounds), aabox::load(nodes[right].bounds));
                    stack[size++] = right;
                    stack[size++] = left;
                }
            }
            CHECK(ctx, covered == triangle_count);
            CHECK(ctx, leaves_contain);
            CHECK(ctx, nodes_contain);

            tree.release();
            triangle_mesh_data::destroy(data);
        }
    }

    triangle_mesh_data::destroy(original);
}
