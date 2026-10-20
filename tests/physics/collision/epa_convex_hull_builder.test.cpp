#include "instance.h"
#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/collision/epa_convex_hull_builder.h"
#include "test.h"
#include "test_math.h"
#include "util/collections.h"

namespace hull = epa_convex_hull_builder;

static void push_position(collections::Vec<float> &positions, float x, float y, float z) {
    positions.push(x);
    positions.push(y);
    positions.push(z);
}

/** the positions as the builder reads them, in `out` (64 at most) */
static const Vec3f *as_points(Vec3f *out, const collections::Vec<float> &positions) {
    for (uint32_t i = 0; i < positions.length / 3; i++) out[i] = vec3f::load(positions.data, i * 3);
    return out;
}

static bool is_removed(const hull::EpaConvexHullBuilderState &state, int32_t tri) { return (state.triangles[tri].flags & hull::TRI_FLAG_REMOVED) != 0; }

/** the distinct position indices the live triangles use. */
static uint32_t count_vertices(const hull::EpaConvexHullBuilderState &state) {
    bool used[64] = {};
    uint32_t count = 0;
    for (int32_t tri = 0; tri < state.triangle_high_watermark; tri++) {
        if (is_removed(state, tri)) continue;
        for (int32_t edge = 0; edge < 3; edge++) {
            const int32_t index = state.triangles[tri].edges[edge].start_index;
            if (!used[index]) count++;
            used[index] = true;
        }
    }
    return count;
}

static bool is_free(const hull::EpaConvexHullBuilderState &state, int32_t tri) {
    for (int32_t t = state.triangle_free_head; t != -1; t = state.triangles[t].next_free) {
        if (t == tri) return true;
    }
    return false;
}

static int32_t start_index(const hull::EpaConvexHullBuilderState &state, int32_t tri, int32_t edge) {
    return state.triangles[tri].edges[edge].start_index;
}

static uint32_t count_live(const hull::EpaConvexHullBuilderState &state) {
    uint32_t live = 0;
    for (int32_t tri = 0; tri < state.triangle_high_watermark; tri++) {
        if (!is_removed(state, tri)) live++;
    }
    return live;
}

/**
 * closed means every directed edge (a, b) of a live triangle appears exactly once and its reverse
 * (b, a) exactly once, so each undirected edge is shared by two faces. counted by brute force.
 */
static bool hull_is_closed(const hull::EpaConvexHullBuilderState &state) {
    for (int32_t tri = 0; tri < state.triangle_high_watermark; tri++) {
        if (is_removed(state, tri)) continue;
        for (int32_t edge = 0; edge < 3; edge++) {
            const int32_t a = start_index(state, tri, edge);
            const int32_t b = start_index(state, tri, (edge + 1) % 3);
            uint32_t same = 0, reverse = 0;
            for (int32_t other = 0; other < state.triangle_high_watermark; other++) {
                if (is_removed(state, other)) continue;
                for (int32_t other_edge = 0; other_edge < 3; other_edge++) {
                    const int32_t c = start_index(state, other, other_edge);
                    const int32_t d = start_index(state, other, (other_edge + 1) % 3);
                    if (c == a && d == b) same++;
                    if (c == b && d == a) reverse++;
                }
            }
            if (same != 1 || reverse != 1) return false;
        }
    }
    return true;
}

static Float3 unit_normal(const hull::EpaConvexHullBuilderState &state, int32_t tri) {
    Float3 out;
    return vec3f::store(out, vec3f::normalize(state.triangles[tri].normal));
}

/** the closest live triangle in the queue, popping removed ones as epa does. */
static int32_t closest_live(hull::EpaConvexHullBuilderState &state) {
    while (hull::has_next_triangle(state)) {
        const int32_t tri = hull::peek_closest_triangle_in_queue(state);
        if (!is_removed(state, tri)) return tri;
        hull::pop_closest_triangle_from_queue(state);
        hull::free_triangle(state, tri);
    }
    return -1;
}

/** the tetrahedron: its base in the plane z = -0.5 around the origin, the apex above. */
static void tetrahedron(collections::Vec<float> &positions) {
    push_position(positions, -1.0f, -1.0f, -0.5f);
    push_position(positions, 2.0f, -1.0f, -0.5f);
    push_position(positions, -1.0f, 2.0f, -0.5f);
    push_position(positions, 0.0f, 0.0f, 2.0f);
}

void test_epa_hull_initialize(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    collections::Vec<float> positions = {};
    tetrahedron(positions);
    static hull::EpaConvexHullBuilderState state;
    state = {};
    Vec3f points[64];
    state.positions = as_points(points, positions);
    hull::initialize(state, 0, 1, 2);

    // two back to back triangles, both queued, linked to each other on all three edges
    CHECK(ctx, state.triangle_high_watermark == 2);
    CHECK(ctx, state.queue_size == 2);
    CHECK(ctx, hull::validate_triangles(state));
    CHECK(ctx, hull_is_closed(state));
    const Float3 n0 = unit_normal(state, 0);
    const Float3 n1 = unit_normal(state, 1);
    CHECK_NEAR(ctx, n0.z, 1.0f, 1e-6f);
    CHECK_NEAR(ctx, n1.z, -1.0f, 1e-6f);

    // the plane is 0.5 from the origin: the signed squared distance is positive on the side facing away
    CHECK_NEAR(ctx, state.triangles[0].closest_length_sq, -0.25f, 1e-6f);
    CHECK_NEAR(ctx, state.triangles[1].closest_length_sq, 0.25f, 1e-6f);
    // the heap puts the negative one first
    CHECK(ctx, hull::peek_closest_triangle_in_queue(state) == 0);

    // re-initialising frees everything en masse
    hull::initialize(state, 0, 1, 2);
    CHECK(ctx, state.triangle_high_watermark == 2 && state.queue_size == 2 && state.triangle_free_head == -1);

    positions.release();
}

void test_epa_hull_tetrahedron_closest_face(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    collections::Vec<float> positions = {};
    tetrahedron(positions);
    static hull::EpaConvexHullBuilderState state;
    state = {};
    Vec3f points[64];
    state.positions = as_points(points, positions);
    hull::NewTriangles new_triangles = {};
    hull::initialize(state, 0, 1, 2);

    // the apex sees only the base triangle facing it, 2.5 above the base plane
    float best_dist_sq = -1.0f;
    const int32_t facing = hull::find_facing_triangle(state, state.positions[3], best_dist_sq);
    CHECK(ctx, facing == 0);
    CHECK_NEAR(ctx, best_dist_sq, 6.25f, 1e-5f);
    CHECK(ctx, hull::add_point(state, facing, 3, __FLT_MAX__, new_triangles));
    CHECK(ctx, new_triangles.size == 3);

    // a closed tetrahedron: four live faces, each edge shared by two
    CHECK(ctx, count_live(state) == 4);
    CHECK(ctx, is_removed(state, 0));
    CHECK(ctx, hull_is_closed(state));
    CHECK(ctx, hull::validate_triangles(state));
    for (uint32_t i = 0; i < new_triangles.size; i++) CHECK(ctx, start_index(state, new_triangles.values[i], 2) == 3);

    // every live face points away from the origin, which is inside
    for (int32_t tri = 0; tri < state.triangle_high_watermark; tri++) {
        if (is_removed(state, tri)) continue;
        CHECK(ctx, state.triangles[tri].closest_length_sq > 0.0f);
    }

    // the closest face is the base, 0.5 away, with outward normal -z
    const int32_t closest = closest_live(state);
    CHECK(ctx, closest == 1);
    const hull::Triangle &record = state.triangles[closest];
    CHECK_NEAR(ctx, record.closest_length_sq, 0.25f, 1e-6f);
    const Float3 n = unit_normal(state, closest);
    CHECK_NEAR(ctx, n.x, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, n.y, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, n.z, -1.0f, 1e-6f);

    // its closest point (0, 0, -0.5) is interior, and the barycentrics rebuild it
    const int32_t flags = record.flags;
    CHECK(ctx, (flags & hull::TRI_FLAG_CLOSEST_POINT_INTERIOR) != 0);
    const float *y0 = positions.data + start_index(state, closest, 0) * 3;
    const float *y1 = positions.data + start_index(state, closest, 1) * 3;
    const float *y2 = positions.data + start_index(state, closest, 2) * 3;
    const float l0 = record.lambda[0], l1 = record.lambda[1];
    Float3 point;
    if ((flags & hull::TRI_FLAG_LAMBDA_RELATIVE_TO_0) != 0) {
        point = {y0[0] + (y1[0] - y0[0]) * l0 + (y2[0] - y0[0]) * l1, y0[1] + (y1[1] - y0[1]) * l0 + (y2[1] - y0[1]) * l1,
                 y0[2] + (y1[2] - y0[2]) * l0 + (y2[2] - y0[2]) * l1};
    } else {
        point = {y1[0] + (y0[0] - y1[0]) * l0 + (y2[0] - y1[0]) * l1, y1[1] + (y0[1] - y1[1]) * l0 + (y2[1] - y1[1]) * l1,
                 y1[2] + (y0[2] - y1[2]) * l0 + (y2[2] - y1[2]) * l1};
    }
    CHECK_NEAR(ctx, point.x, 0.0f, 1e-5f);
    CHECK_NEAR(ctx, point.y, 0.0f, 1e-5f);
    CHECK_NEAR(ctx, point.z, -0.5f, 1e-5f);

    // the far faces, analytically: 2.5y - z + 2 = 0 (and its mirror) at 2 / sqrt(7.25), 2.5(x + y) + z = 2 at 2 / sqrt(13.5)
    uint32_t matched = 0;
    for (int32_t tri = 0; tri < state.triangle_high_watermark; tri++) {
        if (is_removed(state, tri) || tri == closest) continue;
        const float distance_sq = state.triangles[tri].closest_length_sq;
        if (scalar::abs(distance_sq - 4.0f / 7.25f) < 1e-5f || scalar::abs(distance_sq - 4.0f / 13.5f) < 1e-5f) matched++;
    }
    CHECK(ctx, matched == 3);

    positions.release();
}

void test_epa_hull_add_point_replaces_seen_faces(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    collections::Vec<float> positions = {};
    tetrahedron(positions);
    // below the base: sees only the base
    push_position(positions, 0.0f, 0.0f, -3.0f);
    // off the corner at (2, -1, -0.5), out past several faces
    push_position(positions, 4.0f, -3.0f, 0.5f);
    // far out on every side, to grow the hull past one 4-wide group of queued triangles
    push_position(positions, -4.0f, -4.0f, 1.0f);
    push_position(positions, -3.0f, 5.0f, -1.0f);
    push_position(positions, 1.0f, 1.0f, 6.0f);

    static hull::EpaConvexHullBuilderState state;
    state = {};
    Vec3f points[64];
    state.positions = as_points(points, positions);
    hull::NewTriangles new_triangles = {};
    hull::initialize(state, 0, 1, 2);

    uint32_t longest_queue = 0;
    for (uint32_t point = 3; point < positions.length / 3; point++) {
        if (state.queue_size > longest_queue) longest_queue = state.queue_size;
        // what the point sees before it is added: the live triangles it is in front of
        bool seen[hull::EPA_MAX_TRIANGLES];
        bool is_removed_before[hull::EPA_MAX_TRIANGLES];
        uint32_t seen_count = 0;
        const int32_t watermark_before = state.triangle_high_watermark;
        const uint32_t live_before = count_live(state);
        for (int32_t tri = 0; tri < watermark_before; tri++) {
            is_removed_before[tri] = is_removed(state, tri);
            seen[tri] = !is_removed_before[tri] && hull::triangle_is_facing(state, tri, state.positions[point]);
            if (seen[tri]) seen_count++;
        }
        if (point == 4) CHECK(ctx, seen_count == 1);
        if (point == 5) CHECK(ctx, seen_count >= 2);

        // the 4-wide pick matches a scalar scan of the queue in order
        float best_dist_sq = -1.0f;
        const int32_t facing = hull::find_facing_triangle(state, state.positions[point], best_dist_sq);
        int32_t expected = -1;
        float expected_dist_sq = 0.0f;
        for (uint32_t i = 0; i < state.queue_size; i++) {
            const int32_t tri = state.queue[i];
            if (is_removed(state, tri)) continue;
            const hull::Triangle &record = state.triangles[tri];
            const float abx = positions[point * 3] - vec3f::x(record.centroid);
            const float aby = positions[point * 3 + 1] - vec3f::y(record.centroid);
            const float abz = positions[point * 3 + 2] - vec3f::z(record.centroid);
            const float nx = vec3f::x(record.normal), ny = vec3f::y(record.normal), nz = vec3f::z(record.normal);
            const float dot = nx * abx + ny * aby + nz * abz;
            const float dist_sq = dot * dot / (nx * nx + ny * ny + nz * nz);
            if (dot > 0.0f && dist_sq > expected_dist_sq) {
                expected = tri;
                expected_dist_sq = dist_sq;
            }
        }
        CHECK(ctx, facing == expected);
        CHECK(ctx, best_dist_sq == expected_dist_sq);
        if (facing == -1) continue;
        CHECK(ctx, seen[facing]);

        new_triangles.size = 0;
        CHECK(ctx, hull::add_point(state, facing, static_cast<int32_t>(point), __FLT_MAX__, new_triangles));

        // exactly the seen faces went; every unseen one stayed
        bool exact = true;
        for (int32_t tri = 0; tri < watermark_before; tri++) {
            // a freed seen triangle may come straight back as one of the new ones
            bool reused = false;
            for (uint32_t i = 0; i < new_triangles.size; i++) reused = reused || new_triangles.values[i] == tri;
            const bool gone = is_removed(state, tri) || reused;
            if (!is_removed_before[tri] && seen[tri] != gone) exact = false;
        }
        CHECK(ctx, exact);
        // the fan from the horizon: as many new faces as horizon edges, each meeting at the new point
        CHECK(ctx, new_triangles.size == state.edges_size);
        for (uint32_t i = 0; i < new_triangles.size; i++) CHECK(ctx, start_index(state, new_triangles.values[i], 2) == static_cast<int32_t>(point));
        CHECK(ctx, count_live(state) == live_before - seen_count + new_triangles.size);
        // still closed, with v - e + f = 2
        CHECK(ctx, hull_is_closed(state));
        CHECK(ctx, hull::validate_triangles(state));
        const uint32_t faces = count_live(state);
        CHECK(ctx, count_vertices(state) + faces - faces * 3 / 2 == 2);
    }
    CHECK(ctx, longest_queue > 4);
    CHECK(ctx, count_live(state) == 2 * count_vertices(state) - 4);
    // convex: every position, swallowed ones included, is behind every live face
    bool convex = true;
    for (int32_t tri = 0; tri < state.triangle_high_watermark; tri++) {
        if (is_removed(state, tri)) continue;
        const Vec3f n = vec3f::normalize(state.triangles[tri].normal);
        const Vec3f centroid = state.triangles[tri].centroid;
        for (uint32_t point = 0; point < positions.length / 3; point++) {
            const Vec3f offset = vec3f::subtract(vec3f::load(positions.data, point * 3), centroid);
            if (vec3f::dot(n, offset) > 1e-5f) convex = false;
        }
    }
    CHECK(ctx, convex);

    // popping the queue yields non-decreasing signed distances
    float previous = -__builtin_inff();
    bool ordered = true;
    while (hull::has_next_triangle(state)) {
        const int32_t tri = hull::pop_closest_triangle_from_queue(state);
        const float distance_sq = state.triangles[tri].closest_length_sq;
        if (distance_sq < previous) ordered = false;
        previous = distance_sq;
    }
    CHECK(ctx, ordered);
    CHECK(ctx, hull::pop_closest_triangle_from_queue(state) == -1);
    CHECK(ctx, hull::peek_closest_triangle_in_queue(state) == -1);

    positions.release();
}

void test_epa_hull_free_list_reuse(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    collections::Vec<float> positions = {};
    tetrahedron(positions);
    push_position(positions, 0.0f, 0.0f, -3.0f);
    static hull::EpaConvexHullBuilderState state;
    state = {};
    Vec3f points[64];
    state.positions = as_points(points, positions);
    hull::NewTriangles new_triangles = {};
    hull::initialize(state, 0, 1, 2);

    float best_dist_sq;
    CHECK(ctx, hull::add_point(state, hull::find_facing_triangle(state, state.positions[3], best_dist_sq), 3, __FLT_MAX__, new_triangles));
    // the removed base triangle is still queued, so it is not freed until popped
    CHECK(ctx, state.triangle_free_head == -1);
    const int32_t removed = closest_live(state) == 1 ? 0 : -1;
    CHECK(ctx, removed == 0 && is_free(state, 0) && !is_free(state, 1));

    // the next triangle comes off the free list rather than the watermark
    const int32_t watermark = state.triangle_high_watermark;
    new_triangles.size = 0;
    CHECK(ctx, hull::add_point(state, hull::find_facing_triangle(state, state.positions[4], best_dist_sq), 4, __FLT_MAX__, new_triangles));
    CHECK(ctx, new_triangles.values[0] == 0);
    CHECK(ctx, state.triangle_high_watermark == watermark + 2);
    CHECK(ctx, hull_is_closed(state));

    positions.release();
}
