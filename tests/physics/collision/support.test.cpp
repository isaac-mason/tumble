#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/support.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/util/alloc.h"
#include "test.h"
#include "tumble/physics/shapes/dispatch.h"
#include "test_math.h"
#include "test_rng.h"

static constexpr float EPS = 1e-5f;

static Vec3f get(Support &s, Vec3f direction) { return s.get_support(s, direction); }

static Vec3f random_direction(uint32_t &rng) {
    Vec3f d;
    do {
        const float x = mulberry32_unit(rng) * 2.0f - 1.0f;
        const float y = mulberry32_unit(rng) * 2.0f - 1.0f;
        d = vec3f::make(x, y, mulberry32_unit(rng) * 2.0f - 1.0f);
    } while (vec3f::squared_length(d) < 0.01f);
    return d;
}

static Mat4f some_transform() {
    return mat4f::from_rotation_translation(axis_angle(vec3f::make(1.0f, 2.0f, -0.5f), 0.9f), vec3f::make(3.0f, -1.0f, 2.5f));
}

/** a polygon handed to set_polygon_support, as register points */
static Vec3f polygon_points[support::MAX_HULL_POINTS];

/** a transformed support equals the untransformed one in the local direction, moved by the transform. */
static void check_transformed(TestCtx &ctx, Support s, uint32_t &rng) {
    const Mat4f m = some_transform();
    for (uint32_t i = 0; i < 32; i++) {
        const Vec3f d = random_direction(rng);
        s.has_transform = false;
        const Vec3f expected = vec3f::transform_mat4(get(s, mat4f::multiply3x3_transposed_vec(m, d)), m);
        s.has_transform = true;
        s.transform = m;
        s.hull.last_vertex = -1;
        CHECK(ctx, near3(get(s, d), expected, 1e-4f));
    }
}

/** a support with add_radius equals the plain one pushed out by the radius along the local direction. */
static void check_add_radius(TestCtx &ctx, Support s, uint32_t &rng) {
    for (uint32_t i = 0; i < 32; i++) {
        const Vec3f d = random_direction(rng);
        s.add_radius = 0.0f;
        s.hull.last_vertex = -1;
        const Vec3f plain = get(s, d);
        s.add_radius = 0.25f;
        s.hull.last_vertex = -1;
        const Vec3f expected = vec3f::scale_and_add(plain, vec3f::normalize(d), 0.25f);
        CHECK(ctx, near3(get(s, d), expected, EPS));
    }
}

static const float CUBE[24] = {-1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1};
// corner i has bit 0 for +x, bit 1 for +y, bit 2 for +z; its neighbours differ in one bit
static const uint32_t CUBE_NEIGHBORS_START[9] = {0, 3, 6, 9, 12, 15, 18, 21, 24};
static const uint32_t CUBE_NEIGHBORS[24] = {1, 2, 4, 0, 3, 5, 3, 0, 6, 2, 1, 7, 5, 6, 0, 4, 7, 1, 7, 4, 2, 6, 5, 3};

static Support cube_hull(bool adjacency) {
    Support s = support::create();
    s.get_support = support::hull_support;
    s.hull.vertices = CUBE;
    s.hull.vertex_count = 8;
    if (adjacency) {
        s.hull.neighbors_start = CUBE_NEIGHBORS_START;
        s.hull.neighbors = CUBE_NEIGHBORS;
    }
    return s;
}

void test_support_create(TestCtx &ctx) {
    Support s = support::create();
    CHECK(ctx, s.get_support == support::sphere_support);
    CHECK(ctx, s.convex_radius == 0.0f && s.add_radius == 0.0f && !s.has_transform);
    CHECK(ctx, same_bits(s.transform, mat4f::identity()));
    CHECK(ctx, s.hull.output_scale == 1.0f && s.hull.last_vertex == -1);
    CHECK(ctx, near3(get(s, vec3f::make(1.0f, 2.0f, 3.0f)), 0.0f, 0.0f, 0.0f, EPS));
}

void test_support_sphere(TestCtx &ctx) {
    uint32_t rng = 1;
    Support s = support::create();
    s.sphere.radius = 1.5f;
    for (uint32_t i = 0; i < 64; i++) {
        const Vec3f d = random_direction(rng);
        const Vec3f p = get(s, d);
        // radius along the direction, whatever its length
        CHECK_NEAR(ctx, vec3f::length(p), 1.5f, EPS);
        const Vec3f unit = vec3f::normalize(d);
        CHECK_NEAR(ctx, vec3f::dot(p, unit), 1.5f, EPS);
    }
    // a zero direction has no support direction: the centre
    CHECK(ctx, near3(get(s, vec3f::make(0.0f, 0.0f, 0.0f)), 0.0f, 0.0f, 0.0f, EPS));
    // rotation cancels: only the translation moves it
    s.has_transform = true;
    s.transform = some_transform();
    CHECK(ctx, near3(get(s, vec3f::make(0.0f, 2.0f, 0.0f)), 3.0f, 0.5f, 2.5f, EPS));
    s.has_transform = false;
    check_transformed(ctx, s, rng);
    check_add_radius(ctx, s, rng);
}

void test_support_box(TestCtx &ctx) {
    uint32_t rng = 2;
    Support s = support::create();
    s.get_support = support::box_support;
    s.box.half_extents = vec3f::make(1.0f, 2.0f, 3.0f);
    for (uint32_t i = 0; i < 64; i++) {
        const Vec3f d = random_direction(rng);
        const Float3 expected = {vec3f::x(d) >= 0.0f ? 1.0f : -1.0f, vec3f::y(d) >= 0.0f ? 2.0f : -2.0f, vec3f::z(d) >= 0.0f ? 3.0f : -3.0f};
        CHECK(ctx, near3(get(s, d), expected, 0.0f));
    }
    // zero components pick the positive side
    CHECK(ctx, near3(get(s, vec3f::make(0.0f, -1.0f, 0.0f)), 1.0f, -2.0f, 3.0f, 0.0f));
    check_transformed(ctx, s, rng);
    check_add_radius(ctx, s, rng);
}

void test_support_capsule(TestCtx &ctx) {
    uint32_t rng = 3;
    Support s = support::create();
    s.get_support = support::capsule_support;
    s.capsule.half_height = 2.0f;
    s.capsule.radius = 0.5f;
    for (uint32_t i = 0; i < 64; i++) {
        const Vec3f d = random_direction(rng);
        const Vec3f unit = vec3f::normalize(d);
        // the near cap's centre plus the radius along the direction
        const Float3 cap = {0.0f, vec3f::y(d) > 0.0f ? 2.0f : -2.0f, 0.0f};
        const Vec3f expected = vec3f::scale_and_add(vec3f::load(cap), unit, 0.5f);
        CHECK(ctx, near3(get(s, d), expected, EPS));
    }
    // exclude mode: the bare segment
    s.capsule.radius = 0.0f;
    CHECK(ctx, near3(get(s, vec3f::make(1.0f, 0.1f, 0.0f)), 0.0f, 2.0f, 0.0f, 0.0f));
    CHECK(ctx, near3(get(s, vec3f::make(1.0f, 0.0f, 0.0f)), 0.0f, -2.0f, 0.0f, 0.0f));
    s.capsule.radius = 0.5f;
    check_transformed(ctx, s, rng);
    check_add_radius(ctx, s, rng);
}

void test_support_cylinder(TestCtx &ctx) {
    uint32_t rng = 4;
    Support s = support::create();
    s.get_support = support::cylinder_support;
    s.cylinder.radius = 2.0f;
    s.cylinder.half_height = 1.0f;
    // the rim: radius along the horizontal direction on the near cap
    CHECK(ctx, near3(get(s, vec3f::make(3.0f, -1.0f, 4.0f)), 1.2f, -1.0f, 1.6f, EPS));
    CHECK(ctx, near3(get(s, vec3f::make(1.0f, 1.0f, 0.0f)), 2.0f, 1.0f, 0.0f, EPS));
    // straight up or down: the cap's centre
    CHECK(ctx, near3(get(s, vec3f::make(0.0f, 1.0f, 0.0f)), 0.0f, 1.0f, 0.0f, 0.0f));
    CHECK(ctx, near3(get(s, vec3f::make(0.0f, -1.0f, 0.0f)), 0.0f, -1.0f, 0.0f, 0.0f));
    for (uint32_t i = 0; i < 64; i++) {
        const Vec3f d = random_direction(rng);
        const Vec3f p = get(s, d);
        CHECK_NEAR(ctx, scalar::sqrt(vec3f::x(p) * vec3f::x(p) + vec3f::z(p) * vec3f::z(p)), 2.0f, EPS);
        CHECK(ctx, vec3f::y(p) == (vec3f::y(d) >= 0.0f ? 1.0f : -1.0f));
    }
    check_transformed(ctx, s, rng);
    check_add_radius(ctx, s, rng);
}

void test_support_triangle(TestCtx &ctx) {
    uint32_t rng = 5;
    Support s = support::create();
    support::set_triangle_support(s, vec3f::make(0.0f, 0.0f, 0.0f), vec3f::make(2.0f, 0.0f, 0.0f), vec3f::make(0.0f, 3.0f, 0.0f), 0.3f);
    CHECK(ctx, s.get_support == support::triangle_support && s.convex_radius == 0.3f);
    CHECK(ctx, near3(get(s, vec3f::make(1.0f, 0.0f, 0.0f)), 2.0f, 0.0f, 0.0f, 0.0f));
    CHECK(ctx, near3(get(s, vec3f::make(0.0f, 1.0f, 0.0f)), 0.0f, 3.0f, 0.0f, 0.0f));
    CHECK(ctx, near3(get(s, vec3f::make(-1.0f, -1.0f, 0.0f)), 0.0f, 0.0f, 0.0f, 0.0f));
    // ties go to the last maximal vertex: every vertex dots 0 against +z
    CHECK(ctx, near3(get(s, vec3f::make(0.0f, 0.0f, 1.0f)), 0.0f, 3.0f, 0.0f, 0.0f));
    // the reported convex radius is never added
    for (uint32_t i = 0; i < 16; i++) {
        const Vec3f p = get(s, random_direction(rng));
        CHECK(ctx, vec3f::z(p) == 0.0f && vec3f::x(p) >= 0.0f && vec3f::y(p) >= 0.0f);
    }
    check_transformed(ctx, s, rng);
    check_add_radius(ctx, s, rng);
}

void test_support_point(TestCtx &ctx) {
    uint32_t rng = 6;
    Support s = support::create();
    support::set_point_support(s, vec3f::make(1.0f, -2.0f, 0.5f));
    CHECK(ctx, s.get_support == support::point_support && s.convex_radius == 0.0f);
    for (uint32_t i = 0; i < 16; i++) CHECK(ctx, near3(get(s, random_direction(rng)), 1.0f, -2.0f, 0.5f, 0.0f));
    check_transformed(ctx, s, rng);
    check_add_radius(ctx, s, rng);
}

void test_support_hull_cube(TestCtx &ctx) {
    Support cold = cube_hull(false);
    Support warm = cube_hull(true);
    for (uint32_t octant = 0; octant < 8; octant++) {
        const Vec3f d = vec3f::make((octant & 1) ? 0.3f : -0.3f, (octant & 2) ? 1.1f : -1.1f, (octant & 4) ? 0.7f : -0.7f);
        const Vec3f corner = vec3f::load(CUBE, octant * 3);
        // the brute scan picks the octant's corner and never seeds a hint without adjacency
        CHECK(ctx, near3(get(cold, d), corner, 0.0f));
        CHECK(ctx, cold.hull.last_vertex == -1);
        // a cold accelerated pair brute-seeds the hint with the corner's index
        warm.hull.last_vertex = -1;
        CHECK(ctx, near3(get(warm, d), corner, 0.0f));
        CHECK(ctx, warm.hull.last_vertex == (int32_t)octant);
        // and the hill climb from every start, adjacent or across the cube, agrees
        for (int32_t start = 0; start < 8; start++) {
            warm.hull.last_vertex = start;
            CHECK(ctx, near3(get(warm, d), corner, 0.0f));
            CHECK(ctx, warm.hull.last_vertex == (int32_t)octant);
        }
    }
    // ties go to the first maximal vertex: +x is a face of four corners, the first is corner 1
    CHECK(ctx, near3(get(cold, vec3f::make(1.0f, 0.0f, 0.0f)), 1.0f, -1.0f, -1.0f, 0.0f));
}

/** a random frustum capped by two apexes: two rings of `n` random angles and its csr 1-ring. */
struct RandomHull {
    float vertices[3 * 32];
    uint32_t neighbors_start[33];
    uint32_t neighbors[32 * 8];
    uint32_t vertex_count;
};

static void build_random_hull(RandomHull &hull, uint32_t &rng, uint32_t n) {
    float angles[15];
    for (uint32_t i = 0; i < n; i++) angles[i] = (float)i / (float)n * 2.0f * scalar::PI + mulberry32_unit(rng) * (2.0f * scalar::PI / (float)n) * 0.8f;
    const uint32_t top_apex = 2 * n;
    const uint32_t bottom_apex = 2 * n + 1;
    hull.vertex_count = 2 * n + 2;
    for (uint32_t i = 0; i < n; i++) {
        const float c = scalar::cos(angles[i]);
        const float s = scalar::sin(angles[i]);
        vec3f::store(hull.vertices, vec3f::make(c * 1.5f, 1.0f, s * 1.5f), i * 3);
        vec3f::store(hull.vertices, vec3f::make(c * 2.0f, -1.0f, s * 2.0f), (n + i) * 3);
    }
    vec3f::store(hull.vertices, vec3f::make(0.1f, 2.5f, -0.2f), top_apex * 3);
    vec3f::store(hull.vertices, vec3f::make(-0.1f, -3.0f, 0.2f), bottom_apex * 3);
    uint32_t k = 0;
    for (uint32_t ring = 0; ring < 2; ring++) {
        for (uint32_t i = 0; i < n; i++) {
            hull.neighbors_start[ring * n + i] = k;
            hull.neighbors[k++] = ring * n + (i + 1) % n;
            hull.neighbors[k++] = ring * n + (i + n - 1) % n;
            hull.neighbors[k++] = (1 - ring) * n + i;
            hull.neighbors[k++] = ring == 0 ? top_apex : bottom_apex;
        }
    }
    for (uint32_t apex = 0; apex < 2; apex++) {
        hull.neighbors_start[top_apex + apex] = k;
        for (uint32_t i = 0; i < n; i++) hull.neighbors[k++] = apex * n + i;
    }
    hull.neighbors_start[hull.vertex_count] = k;
}

/** the first vertex with the greatest dot, scalar, as the reference the 4-wide scan must match. */
static Vec3f reference_support(const float *vertices, uint32_t count, Vec3f d, float scale) {
    float best = -__builtin_inff();
    Vec3f out = vec3f::zero();
    for (uint32_t i = 0; i < count; i++) {
        const float dot = vertices[i * 3] * vec3f::x(d) + vertices[i * 3 + 1] * vec3f::y(d) + vertices[i * 3 + 2] * vec3f::z(d);
        if (dot > best) {
            best = dot;
            out = vec3f::load(vertices, i * 3);
        }
    }
    return vec3f::scale(out, scale);
}

void test_support_hull_random(TestCtx &ctx) {
    uint32_t rng = 7;
    RandomHull hull;
    for (uint32_t n = 4; n <= 14; n++) {
        build_random_hull(hull, rng, n);
        Support brute = support::create();
        support::set_polygon_support(brute, unflatten3(polygon_points, hull.vertices, hull.vertex_count), hull.vertex_count, 0.0f);
        Support climb = brute;
        climb.hull.neighbors_start = hull.neighbors_start;
        climb.hull.neighbors = hull.neighbors;
        for (uint32_t i = 0; i < 64; i++) {
            const Vec3f d = random_direction(rng);
            const Vec3f expected = reference_support(hull.vertices, hull.vertex_count, d, 1.0f);
            CHECK(ctx, near3(get(brute, d), expected, 0.0f));
            // warm across calls: the hint carries from the last direction
            CHECK(ctx, near3(get(climb, d), expected, 0.0f));
        }
        // every prefix of the points, so the scan meets every tail length after its four-wide blocks
        for (uint32_t count = 1; count <= hull.vertex_count; count++) {
            Support prefix = support::create();
            support::set_polygon_support(prefix, unflatten3(polygon_points, hull.vertices, count), count, 0.0f);
            const Vec3f d = random_direction(rng);
            CHECK(ctx, near3(get(prefix, d), reference_support(hull.vertices, count, d, 1.0f), 0.0f));
        }
        // every start agrees with a cold start
        const Vec3f d = random_direction(rng);
        for (uint32_t start = 0; start < hull.vertex_count; start++) {
            climb.hull.last_vertex = (int32_t)start;
            CHECK(ctx, near3(get(climb, d), get(brute, d), 0.0f));
        }
        check_transformed(ctx, climb, rng);
        check_add_radius(ctx, climb, rng);
    }
}

void test_support_hull_scaled(TestCtx &ctx) {
    uint32_t rng = 8;
    RandomHull hull;
    build_random_hull(hull, rng, 9);

    // uniform fast path: borrowed vertices scaled at output equal the vertices baked at that scale
    Support fast = support::create();
    support::set_polygon_support(fast, unflatten3(polygon_points, hull.vertices, hull.vertex_count), hull.vertex_count, 0.0f);
    fast.hull.output_scale = 1.75f;
    Support baked = support::create();
    for (uint32_t i = 0; i < hull.vertex_count * 3; i++) baked.hull.scratch[i] = hull.vertices[i] * 1.75f;
    support::set_polygon_support(baked, unflatten3(polygon_points, baked.hull.scratch, hull.vertex_count), hull.vertex_count, 0.0f);
    for (uint32_t i = 0; i < 64; i++) {
        const Vec3f d = random_direction(rng);
        CHECK(ctx, near3(get(fast, d), get(baked, d), 1e-5f));
        CHECK(ctx, near3(get(fast, d), reference_support(hull.vertices, hull.vertex_count, d, 1.75f), 1e-5f));
    }
}

/** a unit cube as convex_hull holds it: three faces per corner, planes n·p + c = 0. */
static const Plane3f CUBE_PLANES[6] = {{{-1, 0, 0}, -1}, {{1, 0, 0}, -1}, {{0, -1, 0}, -1},
                                       {{0, 1, 0}, -1},  {{0, 0, -1}, -1}, {{0, 0, 1}, -1}};
static const uint32_t CUBE_NUM_FACES[8] = {3, 3, 3, 3, 3, 3, 3, 3};
static const int32_t CUBE_FACES[24] = {0, 2, 4, 1, 2, 4, 0, 3, 4, 1, 3, 4, 0, 2, 5, 1, 2, 5, 0, 3, 5, 1, 3, 5};

void test_support_shrunk_hull(TestCtx &ctx) {
    collections::Array<float> shrunk = {};
    support::compute_shrunk_hull_points(shrunk, 8, CUBE, CUBE_NUM_FACES, CUBE_FACES, CUBE_PLANES, 0.1f);
    CHECK(ctx, shrunk.length == 24);
    // every corner moves in by the radius on all three axes
    for (uint32_t i = 0; i < 24; i++) CHECK_NEAR(ctx, shrunk[i], CUBE[i] * 0.9f, EPS);

    // a corner on two faces: in along both normals, held on the plane through it perpendicular to both
    const float edge_point[3] = {1.0f, 0.3f, 1.0f};
    const uint32_t two_faces[1] = {2};
    const int32_t edge_faces[3] = {1, 5, -1};
    collections::Array<float> edge = {};
    support::compute_shrunk_hull_points(edge, 1, edge_point, two_faces, edge_faces, CUBE_PLANES, 0.1f);
    CHECK(ctx, near3({edge[0], edge[1], edge[2]}, 0.9f, 0.3f, 0.9f, EPS));
    // a point on one face: back along its normal
    const uint32_t one_face[1] = {1};
    support::compute_shrunk_hull_points(edge, 1, edge_point, one_face, edge_faces, CUBE_PLANES, 0.1f);
    CHECK(ctx, near3({edge[0], edge[1], edge[2]}, 0.9f, 0.3f, 1.0f, EPS));
    edge.release();

    // scaled: a non-uniform box of half extents (2, 1, 0.5) shrunk by the radius at the smallest scale
    const Float3 scale = {2.0f, 1.0f, 0.5f};
    CHECK_NEAR(ctx, support::scale_convex_radius(0.1f, vec3f::load(scale)), 0.05f, EPS);
    CHECK_NEAR(ctx, support::scale_convex_radius(0.1f, vec3f::make(-3.0f, 0.4f, 1.0f)), 0.04f, EPS);
    float scaled[24];
    support::compute_scaled_shrunk_hull_points(scaled, 8, CUBE, CUBE_NUM_FACES, CUBE_FACES, CUBE_PLANES, 0.1f, vec3f::load(scale));
    for (uint32_t i = 0; i < 8; i++) {
        const Float3 expected = {CUBE[i * 3] * (2.0f - 0.05f), CUBE[i * 3 + 1] * (1.0f - 0.05f), CUBE[i * 3 + 2] * (0.5f - 0.05f)};
        CHECK(ctx, near3({scaled[i * 3], scaled[i * 3 + 1], scaled[i * 3 + 2]}, expected, EPS));
    }
    // mirrored: the same box, each corner landing on the mirrored side
    support::compute_scaled_shrunk_hull_points(scaled, 8, CUBE, CUBE_NUM_FACES, CUBE_FACES, CUBE_PLANES, 0.1f, vec3f::make(-2.0f, 1.0f, 0.5f));
    for (uint32_t i = 0; i < 8; i++) CHECK_NEAR(ctx, scaled[i * 3], -CUBE[i * 3] * 1.95f, EPS);

    // uniform: the scaled slow path equals the unscaled shrink at the scaled radius, scaled
    support::compute_scaled_shrunk_hull_points(scaled, 8, CUBE, CUBE_NUM_FACES, CUBE_FACES, CUBE_PLANES, 0.1f, vec3f::make(1.5f, 1.5f, 1.5f));
    support::compute_shrunk_hull_points(shrunk, 8, CUBE, CUBE_NUM_FACES, CUBE_FACES, CUBE_PLANES, 0.1f);
    for (uint32_t i = 0; i < 24; i++) CHECK_NEAR(ctx, scaled[i], shrunk[i] * 1.5f, EPS);

    // the slow path's scratch feeds the evaluator as a baked hull
    Support s = support::create();
    support::compute_scaled_shrunk_hull_points(s.hull.scratch, 8, CUBE, CUBE_NUM_FACES, CUBE_FACES, CUBE_PLANES, 0.1f, vec3f::load(scale));
    support::set_polygon_support(s, unflatten3(polygon_points, s.hull.scratch, 8), 8, 0.0f);
    CHECK(ctx, near3(get(s, vec3f::make(1.0f, -1.0f, 1.0f)), 1.95f, -0.95f, 0.45f, EPS));
    shrunk.release();
}

void test_support_polygon(TestCtx &ctx) {
    const float quad[12] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f};
    Support s = support::create();
    s.hull.last_vertex = 3;
    support::set_polygon_support(s, unflatten3(polygon_points, quad, 4), 4, 0.2f);
    CHECK(ctx, s.get_support == support::hull_support && s.convex_radius == 0.2f && s.hull.output_scale == 1.0f);
    CHECK(ctx, s.hull.neighbors_start == nullptr && s.hull.last_vertex == -1);
    CHECK(ctx, near3(get(s, vec3f::make(1.0f, 0.0f, 1.0f)), 1.0f, 0.0f, 1.0f, 0.0f));
    CHECK(ctx, near3(get(s, vec3f::make(-1.0f, 0.0f, 1.0f)), 0.0f, 0.0f, 1.0f, 0.0f));
    // the reported radius is not added, the evaluator's own add_radius is
    s.add_radius = 0.5f;
    CHECK(ctx, near3(get(s, vec3f::make(0.0f, 1.0f, 0.0f)), 0.0f, 0.5f, 0.0f, 0.0f));
    // an empty polygon supports at the origin
    support::set_polygon_support(s, unflatten3(polygon_points, quad, 0), 0, 0.0f);
    CHECK(ctx, near3(get(s, vec3f::make(1.0f, 0.0f, 0.0f)), 0.0f, 0.0f, 0.0f, 0.0f));
}

/** a file static as the shape fills hold them: it outlives the hull it was filled from */
static Support mirrored_support;

/** the cube as a hull keeping its whole convex radius (the corner shift allowance is loosened) */
static Shape *cube_hull(float convex_radius) {
    return convex_hull::create({.positions = CUBE, .num_positions = 8, .convex_radius = convex_radius, .max_error_convex_radius = 1.0f});
}

static void drop_hull(Ref<Shape> &shape) { shape = nullptr; }

/** takes the freed memory back and overwrites it */
static void scribble_freed(TestCtx &ctx) {
    constexpr uint32_t COUNT = 1u << 16;
    float *junk = util::alloc<float>(COUNT);
    CHECK(ctx, junk != nullptr);
    for (uint32_t i = 0; i < COUNT; i++) junk[i] = 1e9f;
    util::free(junk, COUNT);
}

void test_support_hull_scratch_outlives_its_hull(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> first = cube_hull(0.1f);
    CHECK(ctx, first != nullptr && shapes::as<ConvexHullShape>(first).data->convex_radius == 0.1f);
    // mirrored non-uniform scale, convex radius excluded: shrunk corners at (+-1.95, +-0.95, +-0.45)
    convex_hull::set_support(mirrored_support, first, SupportFunctionMode::EXCLUDE_CONVEX_RADIUS, vec3f::make(-2.0f, 1.0f, 0.5f));
    CHECK(ctx, mirrored_support.hull.vertices == mirrored_support.hull.scratch);
    CHECK_NEAR(ctx, mirrored_support.convex_radius, 0.05f, EPS);
    CHECK(ctx, near3(get(mirrored_support, vec3f::make(1.0f, -1.0f, 1.0f)), 1.95f, -0.95f, 0.45f, EPS));

    // the hull goes: the baked vertices hold none of its memory, so they survive
    drop_hull(first);
    scribble_freed(ctx);
    CHECK(ctx, near3(get(mirrored_support, vec3f::make(1.0f, -1.0f, 1.0f)), 1.95f, -0.95f, 0.45f, EPS));
    CHECK(ctx, near3(get(mirrored_support, vec3f::make(-1.0f, 1.0f, -1.0f)), -1.95f, 0.95f, -0.45f, EPS));

    // refilled from another hull, convex radius included: the raw corners mirrored in y
    Ref<Shape> second = cube_hull(0.1f);
    CHECK(ctx, second != nullptr);
    convex_hull::set_support(mirrored_support, second, SupportFunctionMode::INCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, -3.0f, 0.5f));
    CHECK(ctx, mirrored_support.hull.vertices == mirrored_support.hull.scratch && mirrored_support.convex_radius == 0.0f);
    CHECK(ctx, near3(get(mirrored_support, vec3f::make(1.0f, 1.0f, -1.0f)), 1.0f, 3.0f, -0.5f, EPS));

    drop_hull(second);
    scribble_freed(ctx);
    CHECK(ctx, near3(get(mirrored_support, vec3f::make(1.0f, 1.0f, -1.0f)), 1.0f, 3.0f, -0.5f, EPS));
    CHECK(ctx, near3(get(mirrored_support, vec3f::make(-1.0f, -1.0f, 1.0f)), -1.0f, -3.0f, 0.5f, EPS));
}
