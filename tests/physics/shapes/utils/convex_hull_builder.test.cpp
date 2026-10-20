#include "tumble/instance.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/utils/convex_hull_builder.h"
#include "test.h"
#include "test_math.h"
#include "test_rng.h"

namespace hull = convex_hull_builder;

static constexpr float TOLERANCE = 1.0e-3f;
static constexpr int32_t NO_LIMIT = 1000000;

static uint32_t face_vertex_count(const hull::ConvexHullBuilder &builder, int32_t face) {
    const int32_t first_edge = builder.face_pool[face].first_edge;
    uint32_t count = 0;
    int32_t edge = first_edge;
    do {
        count++;
        edge = builder.edge_pool[edge].next_edge;
    } while (edge != first_edge);
    return count;
}

static Vec3f unit_normal(const hull::ConvexHullBuilder &builder, int32_t face) { return vec3f::normalize(builder.face_pool[face].normal); }

/** the signed distance of p above the face's plane. */
static float plane_distance(const hull::ConvexHullBuilder &builder, int32_t face, Vec3f p) {
    return vec3f::dot(unit_normal(builder, face), vec3f::subtract(p, builder.face_pool[face].centroid));
}

/** every position is behind or on every face plane, within tolerance. */
static bool contains_all(const hull::ConvexHullBuilder &builder, float tolerance) {
    for (uint32_t i = 0; i < builder.num_positions; i++) {
        for (uint32_t f = 0; f < builder.faces.length; f++) {
            if (plane_distance(builder, builder.faces[f], builder.positions[i]) > tolerance) return false;
        }
    }
    return true;
}

/** every hull vertex lies on its face's plane, within tolerance. */
static bool faces_are_planar(const hull::ConvexHullBuilder &builder, float tolerance) {
    for (uint32_t f = 0; f < builder.faces.length; f++) {
        const int32_t face = builder.faces[f];
        const int32_t first_edge = builder.face_pool[face].first_edge;
        int32_t edge = first_edge;
        do {
            if (scalar::abs(plane_distance(builder, face, builder.positions[builder.edge_pool[edge].start_idx])) > tolerance) return false;
            edge = builder.edge_pool[edge].next_edge;
        } while (edge != first_edge);
    }
    return true;
}

/**
 * closed: every edge of every face belongs to it, is linked to an edge of another face running the
 * other way, and that edge links back; and each directed vertex pair occurs once, its reverse once.
 */
static bool is_closed(const hull::ConvexHullBuilder &builder) {
    for (uint32_t f = 0; f < builder.faces.length; f++) {
        const int32_t face = builder.faces[f];
        if (builder.face_pool[face].removed) return false;
        const int32_t first_edge = builder.face_pool[face].first_edge;
        int32_t edge = first_edge;
        do {
            const hull::Edge &e = builder.edge_pool[edge];
            if (e.face != face || e.neighbour_edge == -1) return false;
            const hull::Edge &n = builder.edge_pool[e.neighbour_edge];
            if (n.neighbour_edge != edge || n.face == face) return false;
            if (e.start_idx != builder.edge_pool[n.next_edge].start_idx) return false;
            if (n.start_idx != builder.edge_pool[e.next_edge].start_idx) return false;

            // the directed pair (a, b) once, (b, a) once, over all faces
            const int32_t a = e.start_idx;
            const int32_t b = builder.edge_pool[e.next_edge].start_idx;
            uint32_t same = 0, reverse = 0;
            for (uint32_t g = 0; g < builder.faces.length; g++) {
                const int32_t other_first = builder.face_pool[builder.faces[g]].first_edge;
                int32_t other = other_first;
                do {
                    const int32_t c = builder.edge_pool[other].start_idx;
                    const int32_t d = builder.edge_pool[builder.edge_pool[other].next_edge].start_idx;
                    if (c == a && d == b) same++;
                    if (c == b && d == a) reverse++;
                    other = builder.edge_pool[other].next_edge;
                } while (other != other_first);
            }
            if (same != 1 || reverse != 1) return false;

            edge = e.next_edge;
        } while (edge != first_edge);
    }
    return true;
}

/** every face's normal points away from `inside`. */
static bool faces_point_out(const hull::ConvexHullBuilder &builder, const Float3 &inside) {
    for (uint32_t f = 0; f < builder.faces.length; f++) {
        if (plane_distance(builder, builder.faces[f], vec3f::load(inside)) >= 0.0f) return false;
    }
    return true;
}

/**
 * the hull's vertices, and for each how many faces use it; checks each face uses a vertex at most
 * once and returns false otherwise. `face_counts` is indexed by position.
 */
static bool count_vertex_faces(const hull::ConvexHullBuilder &builder, uint32_t *face_counts) {
    for (uint32_t i = 0; i < builder.num_positions; i++) face_counts[i] = 0;
    for (uint32_t f = 0; f < builder.faces.length; f++) {
        const int32_t first_edge = builder.face_pool[builder.faces[f]].first_edge;
        int32_t edge = first_edge;
        do {
            const int32_t start = builder.edge_pool[edge].start_idx;
            for (int32_t other = builder.edge_pool[edge].next_edge; other != first_edge; other = builder.edge_pool[other].next_edge) {
                if (builder.edge_pool[other].start_idx == start) return false;
            }
            face_counts[start]++;
            edge = builder.edge_pool[edge].next_edge;
        } while (edge != first_edge);
    }
    return true;
}

static bool has_face(const hull::ConvexHullBuilder &builder, int32_t a, int32_t b, int32_t c, int32_t d) {
    const int32_t indices[4] = {a, b, c, d};
    return hull::contains_face(builder, indices, 4);
}

void test_convex_hull_builder_degenerate(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    // too few points / coinciding points should be degenerate
    const Vec3f positions[4] = {
        vec3f::make(1, 2, 3), vec3f::make(1 + 0.5f * TOLERANCE, 2, 3), vec3f::make(1, 2 + 0.5f * TOLERANCE, 3), vec3f::make(1, 2, 3 + 0.5f * TOLERANCE)};

    hull::ConvexHullBuilder builder = hull::create(positions, 1);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::TOO_FEW_POINTS);
    hull::release(builder);

    builder = hull::create(positions, 2);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::TOO_FEW_POINTS);
    hull::release(builder);

    builder = hull::create(positions, 3);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::DEGENERATE);
    hull::release(builder);

    builder = hull::create(positions, 4);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::DEGENERATE);
    CHECK(ctx, builder.faces.length == 0);
    hull::release(builder);
}

void test_convex_hull_builder_line_is_degenerate(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    Vec3f positions[11];
    for (int32_t i = 0; i <= 10; i++) positions[i] = vec3f::make(0.1f * (float)i, 0.0f, 0.0f);

    hull::ConvexHullBuilder builder = hull::create(positions, 11);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::DEGENERATE);

    // a line off the axes too, its points exact in f32
    for (int32_t i = 0; i <= 10; i++) positions[i] = vec3f::make(1.0f + 0.25f * (float)i, -2.0f + 0.5f * (float)i, 0.5f - 0.75f * (float)i);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::DEGENERATE);

    // with steps f32 cannot hold exactly, rounding gives the best triangle more than MIN_TRIANGLE_AREA_SQ, so
    // the line passes the degenerate test; the sliver tetrahedron built from it then merges away to nothing
    for (int32_t i = 0; i <= 10; i++) positions[i] = vec3f::make(1.0f + 0.3f * (float)i, -2.0f + 0.2f * (float)i, 0.5f - 0.7f * (float)i);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::TOO_FEW_FACES);
    CHECK(ctx, builder.faces.length == 0);
    hull::release(builder);
}

void test_convex_hull_builder_triangle(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    const Vec3f positions[3] = {vec3f::make(-1, 0, -1), vec3f::make(1, 0, -1), vec3f::make(-1, 0, 1)};

    hull::ConvexHullBuilder builder = hull::create(positions, 3);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::SUCCESS);
    CHECK(ctx, hull::get_num_vertices_used(builder) == 3);
    CHECK(ctx, builder.faces.length == 2);
    const int32_t forward[3] = {0, 1, 2};
    const int32_t backward[3] = {2, 1, 0};
    CHECK(ctx, hull::contains_face(builder, forward, 3));
    CHECK(ctx, hull::contains_face(builder, backward, 3));
    CHECK(ctx, is_closed(builder));

    // the two faces are back to back along y
    CHECK_NEAR(ctx, scalar::abs(vec3f::y(unit_normal(builder, builder.faces[0]))), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(unit_normal(builder, builder.faces[0])), -vec3f::y(unit_normal(builder, builder.faces[1])), 1e-6f);
    hull::release(builder);
}

void test_convex_hull_builder_quad_with_interior_points(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    Vec3f positions[100];
    for (int32_t x = 0; x < 10; ++x) {
        for (int32_t z = 0; z < 10; ++z) positions[x * 10 + z] = vec3f::make(0.1f * (float)x, 0.0f, 0.2f * (float)z);
    }

    hull::ConvexHullBuilder builder = hull::create(positions, 100);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::SUCCESS);
    CHECK(ctx, hull::get_num_vertices_used(builder) == 4);
    CHECK(ctx, builder.faces.length == 2);
    CHECK(ctx, has_face(builder, 0, 9, 99, 90));
    CHECK(ctx, has_face(builder, 90, 99, 9, 0));
    CHECK(ctx, is_closed(builder));
    hull::release(builder);
}

void test_convex_hull_builder_disc_with_interior_points(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    Vec3f positions[100];
    for (int32_t r = 0; r < 10; ++r) {
        for (int32_t phi = 0; phi < 10; ++phi) {
            const float f_r = 2.0f * (float)r;
            const float f_phi = 2.0f * scalar::PI * (float)phi / 10.0f;
            positions[r * 10 + phi] = vec3f::make(f_r * scalar::cos(f_phi), f_r * scalar::sin(f_phi), 0.0f);
        }
    }

    hull::ConvexHullBuilder builder = hull::create(positions, 100);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::SUCCESS);
    CHECK(ctx, hull::get_num_vertices_used(builder) == 10);
    CHECK(ctx, builder.faces.length == 2);
    const int32_t outer[10] = {90, 91, 92, 93, 94, 95, 96, 97, 98, 99};
    const int32_t outer_reversed[10] = {99, 98, 97, 96, 95, 94, 93, 92, 91, 90};
    CHECK(ctx, hull::contains_face(builder, outer, 10));
    CHECK(ctx, hull::contains_face(builder, outer_reversed, 10));
    CHECK(ctx, is_closed(builder));
    hull::release(builder);
}

void test_convex_hull_builder_cube_with_interior_points(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    Vec3f positions[1000];
    for (int32_t x = 0; x < 10; ++x) {
        for (int32_t y = 0; y < 10; ++y) {
            for (int32_t z = 0; z < 10; ++z) positions[x * 100 + y * 10 + z] = vec3f::make(0.1f * (float)x, 1.0f + 0.2f * (float)y, 0.3f * (float)z);
        }
    }

    hull::ConvexHullBuilder builder = hull::create(positions, 1000);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::SUCCESS);
    CHECK(ctx, hull::get_num_vertices_used(builder) == 8);
    CHECK(ctx, builder.faces.length == 6);
    CHECK(ctx, has_face(builder, 0, 9, 99, 90));
    CHECK(ctx, has_face(builder, 0, 90, 990, 900));
    CHECK(ctx, has_face(builder, 900, 990, 999, 909));
    CHECK(ctx, has_face(builder, 9, 909, 999, 99));
    CHECK(ctx, has_face(builder, 90, 99, 999, 990));
    CHECK(ctx, has_face(builder, 0, 900, 909, 9));
    CHECK(ctx, is_closed(builder));
    CHECK(ctx, contains_all(builder, 1e-5f));
    hull::release(builder);
}

void test_convex_hull_builder_cube_corners_and_interior(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    // the eight corners of a box centred at (1, -2, 0.5) with interior points mixed in
    const Float3 center = {1.0f, -2.0f, 0.5f};
    const Float3 half = {1.0f, 0.5f, 2.0f};
    Vec3f positions[8 + 40];
    uint32_t rng = 11;
    uint32_t count = 0;
    for (int32_t i = 0; i < 48; i++) {
        if (i % 6 == 0) {
            const int32_t corner = i / 6;
            positions[count++] = vec3f::make(center.x + ((corner & 1) ? half.x : -half.x), center.y + ((corner & 2) ? half.y : -half.y),
                                  center.z + ((corner & 4) ? half.z : -half.z));
        } else {
            positions[count++] = vec3f::make(center.x + (mulberry32_unit(rng) * 1.8f - 0.9f) * half.x, center.y + (mulberry32_unit(rng) * 1.8f - 0.9f) * half.y,
                                  center.z + (mulberry32_unit(rng) * 1.8f - 0.9f) * half.z);
        }
    }

    hull::ConvexHullBuilder builder = hull::create(positions, count);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::SUCCESS);

    // six faces after the coplanar merge, four corners each, all of them corners
    CHECK(ctx, builder.faces.length == 6);
    CHECK(ctx, hull::get_num_vertices_used(builder) == 8);
    for (uint32_t f = 0; f < builder.faces.length; f++) CHECK(ctx, face_vertex_count(builder, builder.faces[f]) == 4);
    uint32_t face_counts[48];
    CHECK(ctx, count_vertex_faces(builder, face_counts));
    for (uint32_t i = 0; i < count; i++) CHECK(ctx, face_counts[i] == (i % 6 == 0 ? 3u : 0u));

    // outward axis-aligned planes at the box's sides
    for (uint32_t f = 0; f < builder.faces.length; f++) {
        const Vec3f n = unit_normal(builder, builder.faces[f]);
        const float axis = scalar::max(scalar::abs(vec3f::x(n)), scalar::max(scalar::abs(vec3f::y(n)), scalar::abs(vec3f::z(n))));
        CHECK_NEAR(ctx, axis, 1.0f, 1e-6f);
        const float side = scalar::abs(vec3f::x(n)) * half.x + scalar::abs(vec3f::y(n)) * half.y + scalar::abs(vec3f::z(n)) * half.z;
        CHECK_NEAR(ctx, -plane_distance(builder, builder.faces[f], vec3f::load(center)), side, 1e-5f);
    }
    CHECK(ctx, faces_point_out(builder, center));
    CHECK(ctx, is_closed(builder));
    CHECK(ctx, contains_all(builder, 1e-5f));
    hull::release(builder);
}

void test_convex_hull_builder_tetrahedron_and_octahedron(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    const Vec3f tetrahedron[4] = {vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), vec3f::make(0, 1, 0), vec3f::make(0, 0, 1)};
    hull::ConvexHullBuilder builder = hull::create(tetrahedron, 4);
    CHECK(ctx, hull::initialize(builder, 100, 1e-4f) == hull::SUCCESS);
    CHECK(ctx, builder.faces.length == 4);
    for (uint32_t f = 0; f < builder.faces.length; f++) CHECK(ctx, face_vertex_count(builder, builder.faces[f]) == 3);
    CHECK(ctx, is_closed(builder));
    CHECK(ctx, faces_point_out(builder, {0.2f, 0.2f, 0.2f}));
    CHECK(ctx, contains_all(builder, 1e-6f));
    // the slanted face: normal (1, 1, 1) / sqrt 3, a distance 1 / sqrt 3 from the origin
    bool found_slanted = false;
    for (uint32_t f = 0; f < builder.faces.length; f++) {
        const Vec3f n = unit_normal(builder, builder.faces[f]);
        if (vec3f::x(n) > 0.5f && vec3f::y(n) > 0.5f && vec3f::z(n) > 0.5f) {
            found_slanted = true;
            CHECK_NEAR(ctx, -plane_distance(builder, builder.faces[f], vec3f::zero()), 1.0f / scalar::sqrt(3.0f), 1e-6f);
        }
    }
    CHECK(ctx, found_slanted);
    hull::release(builder);

    // an octahedron with its center point: eight triangles, the center unused
    const Vec3f octahedron[7] = {vec3f::make(2, 0, 0), vec3f::make(-2, 0, 0), vec3f::make(0, 2, 0), vec3f::make(0, -2, 0), vec3f::make(0, 0, 2), vec3f::make(0, 0, -2), vec3f::make(0, 0, 0)};
    builder = hull::create(octahedron, 7);
    CHECK(ctx, hull::initialize(builder, 100, 1e-4f) == hull::SUCCESS);
    CHECK(ctx, builder.faces.length == 8);
    CHECK(ctx, hull::get_num_vertices_used(builder) == 6);
    for (uint32_t f = 0; f < builder.faces.length; f++) {
        CHECK(ctx, face_vertex_count(builder, builder.faces[f]) == 3);
        // each face is a distance 2 / sqrt 3 out along (+-1, +-1, +-1)
        const Vec3f n = unit_normal(builder, builder.faces[f]);
        CHECK_NEAR(ctx, scalar::abs(vec3f::x(n)), 1.0f / scalar::sqrt(3.0f), 1e-6f);
        CHECK_NEAR(ctx, -plane_distance(builder, builder.faces[f], vec3f::zero()), 2.0f / scalar::sqrt(3.0f), 1e-6f);
    }
    uint32_t face_counts[7];
    CHECK(ctx, count_vertex_faces(builder, face_counts));
    for (int32_t i = 0; i < 6; i++) CHECK(ctx, face_counts[i] == 4);
    CHECK(ctx, face_counts[6] == 0);
    CHECK(ctx, is_closed(builder));
    hull::release(builder);
}

void test_convex_hull_builder_sphere_with_interior_points(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    static Vec3f positions[1000];
    for (int32_t r = 0; r < 10; ++r) {
        for (int32_t phi = 0; phi < 10; ++phi) {
            for (int32_t theta = 0; theta < 10; ++theta) {
                const float f_r = 2.0f * (float)r;
                const float f_phi = 2.0f * scalar::PI * (float)phi / 10.0f;  // [0, 2 pi)
                const float f_theta = scalar::PI * (float)theta / 9.0f;      // [0, pi] (inclusive!)
                positions[r * 100 + phi * 10 + theta] = vec3f::make(f_r * scalar::sin(f_theta) * scalar::cos(f_phi), f_r * scalar::cos(f_theta),
                                                         f_r * scalar::sin(f_theta) * scalar::sin(f_phi));
            }
        }
    }

    hull::ConvexHullBuilder builder = hull::create(positions, 1000);
    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::SUCCESS);

    // the two ends of the sphere have 10 points that have the same position, so 82 remain
    const int32_t num_vertices = hull::get_num_vertices_used(builder);
    CHECK(ctx, num_vertices >= 80 && num_vertices <= 85);

    // too many faces, calculate the error instead
    hull::MaxError error;
    hull::determine_max_error(error, builder);
    CHECK(ctx, error.max_error < scalar::max(error.coplanar_distance, TOLERANCE));
    CHECK(ctx, is_closed(builder));
    CHECK(ctx, faces_point_out(builder, {0, 0, 0}));
    hull::release(builder);
}

void test_convex_hull_builder_nearly_coplanar_faces(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    // hull with 2 faces that are nearly coplanar
    const Vec3f two_faces[8] = {
        vec3f::make(-0.020472288f, -0.195635557f, 0.308015466f), vec3f::make(0.136248738f, 0.633286834f, 0.135366619f),
        vec3f::make(0.286418647f, -0.228475571f, 0.308084548f),  vec3f::make(-0.267285109f, 1.024676085f, 0.308042824f),
        vec3f::make(0.396568149f, -0.971658647f, 0.308055162f),  vec3f::make(0.321081549f, -1.024676085f, 0.308036327f),
        vec3f::make(0.034643859f, -0.404506862f, 0.308015764f),  vec3f::make(0.18922469f, -0.252762139f, 0.308060408f),
    };
    hull::ConvexHullBuilder builder = hull::create(two_faces, 8);
    CHECK(ctx, hull::initialize(builder, 0x7fffffff, TOLERANCE) == hull::SUCCESS);
    hull::MaxError error;
    hull::determine_max_error(error, builder);
    CHECK(ctx, error.max_error < scalar::max(error.coplanar_distance, 1.2f * TOLERANCE));
    CHECK(ctx, is_closed(builder));
    hull::release(builder);

    // nearly coplanar points
    const Vec3f nearly_coplanar[20] = {
        vec3f::make(0.917345762f, 0.157111734f, 1.650970459f),   vec3f::make(-0.098074198f, 0.157116055f, 0.664742708f),
        vec3f::make(1.777100325f, 0.157112047f, 1.238879442f),   vec3f::make(2.11432457f, 0.157112464f, 0.780688763f),
        vec3f::make(1.926570415f, 0.157114446f, 0.240761161f),   vec3f::make(-1.045998096f, 0.157108605f, 1.548911095f),
        vec3f::make(-1.820045233f, 0.157106474f, 1.050360918f),  vec3f::make(-1.918573976f, 0.157108605f, 0.039246202f),
        vec3f::make(0.042619467f, 0.157113969f, -1.405336142f),  vec3f::make(0.575986624f, 0.157114401f, -1.370834589f),
        vec3f::make(1.402592659f, 0.157115221f, -0.834864557f),  vec3f::make(1.110557318f, 0.157113969f, -1.336267948f),
        vec3f::make(1.689781666f, 0.157115355f, -0.308773756f),  vec3f::make(2.205337524f, 0.157113209f, -0.281754494f),
        vec3f::make(-1.346967936f, 0.157110974f, -0.978962541f), vec3f::make(-1.346967936f, 0.157110974f, -0.978962541f),
        vec3f::make(-2.085033417f, 0.157106936f, -0.506602883f), vec3f::make(-0.981224537f, 0.157110706f, -1.445893764f),
        vec3f::make(-0.481085658f, 0.157112658f, -1.426232934f), vec3f::make(-0.981224537f, 0.157110706f, -1.445893764f),
    };
    builder = hull::create(nearly_coplanar, 20);
    CHECK(ctx, hull::initialize(builder, 0x7fffffff, TOLERANCE) == hull::SUCCESS);
    hull::determine_max_error(error, builder);
    CHECK(ctx, error.max_error < scalar::max(error.coplanar_distance, 1.2f * TOLERANCE));
    CHECK(ctx, is_closed(builder));
    hull::release(builder);
}

void test_convex_hull_builder_random_cloud(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    uint32_t rng = 3;
    Vec3f positions[200];
    hull::ConvexHullBuilder builder = hull::create(positions, 200);
    for (uint32_t round = 0; round < 6; round++) {
        // a box cloud, then a ball cloud, alternating; the same builder each time
        for (uint32_t i = 0; i < 200; i++) {
            Float3 p;
            do {
                p = {mulberry32_unit(rng) * 2.0f - 1.0f, mulberry32_unit(rng) * 2.0f - 1.0f, mulberry32_unit(rng) * 2.0f - 1.0f};
            } while ((round & 1) && vec3f::squared_length(vec3f::load(p)) > 1.0f);
            positions[i] = vec3f::make(3.0f * p.x + 1.0f, 2.0f * p.y, 1.5f * p.z - 4.0f);
        }

        CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::SUCCESS);
        CHECK(ctx, builder.faces.length >= 4);
        CHECK(ctx, is_closed(builder));
        CHECK(ctx, faces_are_planar(builder, 2.0f * TOLERANCE));
        CHECK(ctx, contains_all(builder, 2.0f * TOLERANCE));
        CHECK(ctx, faces_point_out(builder, {1.0f, 0.0f, -4.0f}));

        hull::MaxError error;
        hull::determine_max_error(error, builder);
        CHECK(ctx, error.max_error < scalar::max(error.coplanar_distance, 2.0f * TOLERANCE));

        // euler for a closed convex polyhedron: v - e + f = 2, and every hull vertex is on at least 3 faces
        uint32_t face_counts[200];
        CHECK(ctx, count_vertex_faces(builder, face_counts));
        uint32_t vertices = 0, half_edges = 0;
        for (uint32_t i = 0; i < 200; i++) {
            if (face_counts[i] == 0) continue;
            vertices++;
            half_edges += face_counts[i];
            CHECK(ctx, face_counts[i] >= 3);
        }
        CHECK(ctx, (int32_t)vertices == hull::get_num_vertices_used(builder));
        CHECK(ctx, (int32_t)vertices - (int32_t)(half_edges / 2) + (int32_t)builder.faces.length == 2);

        // the hull's volume sits inside the cloud's box
        Vec3f center_of_mass;
        float volume;
        hull::get_center_of_mass_and_volume(builder, center_of_mass, volume);
        CHECK(ctx, volume > 0.0f && volume <= 6.0f * 4.0f * 3.0f);
        CHECK(ctx, vec3f::x(center_of_mass) > -2.0f && vec3f::x(center_of_mass) < 4.0f && vec3f::z(center_of_mass) > -5.5f &&
                       vec3f::z(center_of_mass) < -2.5f);
    }
    hull::release(builder);
}

void test_convex_hull_builder_max_vertices(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    // points on a sphere, capped at 12 hull vertices
    uint32_t rng = 5;
    Vec3f positions[100];
    for (uint32_t i = 0; i < 100; i++) {
        Float3 p;
        do {
            p = {mulberry32_unit(rng) * 2.0f - 1.0f, mulberry32_unit(rng) * 2.0f - 1.0f, mulberry32_unit(rng) * 2.0f - 1.0f};
        } while (vec3f::squared_length(vec3f::load(p)) > 1.0f || vec3f::squared_length(vec3f::load(p)) < 0.1f);
        positions[i] = vec3f::normalize(vec3f::load(p));
    }
    hull::ConvexHullBuilder builder = hull::create(positions, 100);
    CHECK(ctx, hull::initialize(builder, 12, TOLERANCE) == hull::MAX_VERTICES_REACHED);
    CHECK(ctx, hull::get_num_vertices_used(builder) <= 12);
    CHECK(ctx, is_closed(builder));

    CHECK(ctx, hull::initialize(builder, NO_LIMIT, TOLERANCE) == hull::SUCCESS);
    CHECK(ctx, hull::get_num_vertices_used(builder) > 12);
    hull::release(builder);
}

void test_convex_hull_builder_contains_face(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    const Vec3f positions[4] = {vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), vec3f::make(0, 1, 0), vec3f::make(0, 0, 1)};
    hull::ConvexHullBuilder builder = hull::create(positions, 4);
    CHECK(ctx, hull::initialize(builder, 100, 1e-4f) == hull::SUCCESS);

    // the base in the z = 0 plane faces down, so it winds clockwise seen from +z
    const int32_t base[3] = {0, 2, 1};
    const int32_t base_rotated[3] = {2, 1, 0};
    const int32_t base_inward[3] = {0, 1, 2};
    CHECK(ctx, hull::contains_face(builder, base, 3));
    CHECK(ctx, hull::contains_face(builder, base_rotated, 3));
    CHECK(ctx, !hull::contains_face(builder, base_inward, 3));

    // not a face with too many vertices, nor an empty one
    const int32_t too_many[5] = {0, 1, 2, 3, 4};
    CHECK(ctx, !hull::contains_face(builder, too_many, 5));
    CHECK(ctx, !hull::contains_face(builder, nullptr, 0));
    hull::release(builder);
}

void test_convex_hull_builder_center_of_mass_and_volume(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    // unit cube centered at origin
    const Vec3f cube[8] = {vec3f::make(-0.5f, -0.5f, -0.5f), vec3f::make(0.5f, -0.5f, -0.5f), vec3f::make(0.5f, 0.5f, -0.5f), vec3f::make(-0.5f, 0.5f, -0.5f),
                           vec3f::make(-0.5f, -0.5f, 0.5f),  vec3f::make(0.5f, -0.5f, 0.5f),  vec3f::make(0.5f, 0.5f, 0.5f),  vec3f::make(-0.5f, 0.5f, 0.5f)};
    hull::ConvexHullBuilder builder = hull::create(cube, 8);
    CHECK(ctx, hull::initialize(builder, 100, 1e-4f) == hull::SUCCESS);
    Vec3f center_of_mass;
    float volume;
    hull::get_center_of_mass_and_volume(builder, center_of_mass, volume);
    CHECK_NEAR(ctx, volume, 1.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::x(center_of_mass), 0.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::y(center_of_mass), 0.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::z(center_of_mass), 0.0f, 1e-5f);
    hull::release(builder);

    // the corner tetrahedron: volume 1/6, center of mass at the vertex average
    const Vec3f tetrahedron[4] = {vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), vec3f::make(0, 1, 0), vec3f::make(0, 0, 1)};
    builder = hull::create(tetrahedron, 4);
    CHECK(ctx, hull::initialize(builder, 100, 1e-4f) == hull::SUCCESS);
    hull::get_center_of_mass_and_volume(builder, center_of_mass, volume);
    CHECK_NEAR(ctx, volume, 1.0f / 6.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::x(center_of_mass), 0.25f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(center_of_mass), 0.25f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(center_of_mass), 0.25f, 1e-6f);
    hull::release(builder);

    // coplanar points: no volume, the center is the average of the two face centroids
    const Vec3f square[4] = {vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), vec3f::make(0, 1, 0), vec3f::make(1, 1, 0)};
    builder = hull::create(square, 4);
    CHECK(ctx, hull::initialize(builder, 100, 1e-4f) == hull::SUCCESS);
    CHECK(ctx, builder.faces.length == 2);
    CHECK(ctx, hull::get_num_vertices_used(builder) == 4);
    CHECK(ctx, is_closed(builder));
    hull::get_center_of_mass_and_volume(builder, center_of_mass, volume);
    CHECK_NEAR(ctx, volume, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::x(center_of_mass), 0.5f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(center_of_mass), 0.5f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(center_of_mass), 0.0f, 1e-6f);
    hull::release(builder);
}

void test_convex_hull_builder_determine_max_error(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    // a cube with an interior point: every point is on or inside the hull
    const Vec3f positions[9] = {vec3f::make(-1, -1, -1), vec3f::make(1, -1, -1), vec3f::make(1, 1, -1), vec3f::make(-1, 1, -1), vec3f::make(-1, -1, 1),
                                vec3f::make(1, -1, 1),   vec3f::make(1, 1, 1),   vec3f::make(-1, 1, 1),  vec3f::make(0, 0, 0)};
    hull::ConvexHullBuilder builder = hull::create(positions, 9);
    CHECK(ctx, hull::initialize(builder, 100, 1e-4f) == hull::SUCCESS);
    CHECK(ctx, builder.faces.length == 6);
    hull::MaxError error;
    hull::determine_max_error(error, builder);
    CHECK(ctx, error.max_error <= error.coplanar_distance * 2.0f);
    CHECK(ctx, error.max_error_position_idx >= -1 && error.max_error_position_idx < 9);
    // the coplanar distance is 3 eps (|x| + |y| + |z|) of the largest coordinates
    CHECK_NEAR(ctx, error.coplanar_distance, 3.0f * 1.1920929e-7f * 3.0f, 1e-12f);
    hull::release(builder);
}
