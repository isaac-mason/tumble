#include "tumble/physics/shapes/convex_hull.h"

#include "tumble/instance.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"
#include "test_rng.h"

using shapes::Shape;

static const float CUBE_POINTS[24] = {-1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1};

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};

/** builds the hull and a shape over it; null (with the build's result in `out`) when the points make no hull */
static Shape *make(const convex_hull::ConvexHullBuildSettings &build, convex_hull::BuildResult *out = nullptr,
                   float density = shapes::DEFAULT_SHAPE_DENSITY, int32_t material_id = -1) {
    convex_hull::ConvexHullData *data = convex_hull::build(build, out);
    if (data == nullptr) return nullptr;
    return convex_hull::create({.data = data, .density = density, .material_id = material_id});
}

/** destroys a shape made by `make` and releases the hull it was the only user of */
static void drop(Shape *h) {
    convex_hull::ConvexHullData *data = h->convex_hull.data;
    convex_hull::destroy(h);
    convex_hull::release(data);
}

static Shape *hull(const float *positions, uint32_t num_positions, float convex_radius,
                   convex_hull::BakeSupportAdjacency bake = convex_hull::BAKE_SUPPORT_ADJACENCY_AUTO) {
    return make({.positions = positions, .num_positions = num_positions, .convex_radius = convex_radius, .bake_support_adjacency = bake});
}

/** the 8 corners of a box of these half extents */
static void box_corners(float *out, const Float3 &half_extents) {
    for (uint32_t i = 0; i < 8; i++) {
        out[i * 3] = CUBE_POINTS[i * 3] * half_extents.x;
        out[i * 3 + 1] = CUBE_POINTS[i * 3 + 1] * half_extents.y;
        out[i * 3 + 2] = CUBE_POINTS[i * 3 + 2] * half_extents.z;
    }
}

/** ~n points spread on a unit sphere, every one a hull vertex */
static void fibonacci_sphere(float *out, uint32_t n) {
    const float phi = scalar::PI * (3.0f - scalar::sqrt(5.0f));
    for (uint32_t i = 0; i < n; i++) {
        const float y = 1.0f - ((float)i / (float)(n - 1)) * 2.0f;
        const float r = scalar::sqrt(scalar::max(0.0f, 1.0f - y * y));
        const float theta = phi * (float)i;
        out[i * 3] = scalar::cos(theta) * r;
        out[i * 3 + 1] = y;
        out[i * 3 + 2] = scalar::sin(theta) * r;
    }
}

/** a thin slab with a 5x5 grid of points on each large face: many coplanar points; 50 points */
static void slab_grid(float *out) {
    uint32_t k = 0;
    for (int ix = 0; ix < 5; ix++) {
        for (int iz = 0; iz < 5; iz++) {
            const float x = (float)(ix - 2);
            const float z = (float)(iz - 2);
            out[k++] = x;
            out[k++] = 0.1f;
            out[k++] = z;
            out[k++] = x;
            out[k++] = -0.1f;
            out[k++] = z;
        }
    }
}

/** the ts tests' conversion: all 32 bits through a double */
static float random_f64(uint32_t &rng) { return (float)((double)mulberry32(rng) / 4294967296.0); }

static void random_directions(Vec3f *out, uint32_t count, uint32_t seed) {
    uint32_t rng = seed;
    uint32_t n = 0;
    while (n < count) {
        const float x = random_f64(rng) * 2.0f - 1.0f;
        const float y = random_f64(rng) * 2.0f - 1.0f;
        const Vec3f d = vec3f::make(x, y, random_f64(rng) * 2.0f - 1.0f);
        if (vec3f::length(d) > 1e-3f) out[n++] = d;
    }
}

/** a slowly rotating direction sequence, as gjk and epa evolve the query direction */
static void rotating_directions(Vec3f *out, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        const float t = ((float)i / (float)count) * scalar::PI * 2.0f;
        const float elevation = scalar::sin(t * 0.37f) * 0.9f;
        const float r = scalar::cos(elevation);
        out[i] = vec3f::make(scalar::cos(t) * r, elevation, scalar::sin(t) * r);
    }
}

static Vec3f support_point(support::Support &s, Vec3f direction) { return s.get_support(s, direction); }

/** the first vertex with the greatest dot, over flat xyz */
static Vec3f brute_force_max_dot(const float *positions, uint32_t count, Vec3f direction) {
    float best = -__builtin_inff();
    Vec3f result = vec3f::zero();
    for (uint32_t i = 0; i < count; i++) {
        const Vec3f p = vec3f::load(positions, i * 3);
        const float dot = vec3f::dot(p, direction);
        if (dot > best) {
            best = dot;
            result = p;
        }
    }
    return result;
}

/** the normal of the face's winding: (v1 - v0) x (v2 - v1), unnormalised */
static Vec3f face_winding(const face::Face &face) {
    const Vec3f *v = face.vertices;
    return vec3f::cross(vec3f::subtract(v[1], v[0]), vec3f::subtract(v[2], v[1]));
}

/** every vertex of a is a vertex of b and the counts agree */
static bool same_vertex_set(const face::Face &a, const face::Face &b, float eps) {
    if (a.num_vertices != b.num_vertices) return false;
    for (uint32_t i = 0; i < a.num_vertices; i++) {
        bool found = false;
        for (uint32_t j = 0; j < b.num_vertices && !found; j++) found = near3(a.vertices[i], b.vertices[j], eps);
        if (!found) return false;
    }
    return true;
}

/* create */

void test_convex_hull_create_box_corners(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    float corners[24];
    box_corners(corners, {1, 2, 3});
    convex_hull::BuildResult result = convex_hull::BUILD_OUT_OF_MEMORY;
    Shape *h = make({.positions = corners, .num_positions = 8, .convex_radius = 0.0f}, &result, 10.0f, 4);
    CHECK(ctx, h != nullptr && result == convex_hull::BUILD_SUCCESS);
    CHECK(ctx, h->type == shapes::SHAPE_CONVEX_HULL && h->material_id == 4);
    const convex_hull::ConvexHullData &data = *h->convex_hull.data;
    CHECK(ctx, data.num_points == 8 && data.point_positions.length == 24);
    CHECK(ctx, data.convex_radius == 0.0f && h->convex_hull.density == 10.0f);

    // six quads, each with its unit outward plane through the face
    CHECK(ctx, data.faces.length == 6 && data.planes.length == 6 && data.vertex_indices.length == 24);
    for (uint32_t f = 0; f < data.faces.length; f++) {
        CHECK(ctx, data.faces[f].num_vertices == 4);
        const Plane3f &plane = data.planes[f];
        const Vec3f normal = vec3f::load(plane.normal);
        CHECK_NEAR(ctx, vec3f::length(normal), 1.0f, 1e-6f);
        for (uint32_t v = 0; v < 4; v++) {
            const Vec3f p = vec3f::load(data.point_positions.data, data.vertex_indices[data.faces[f].first_vertex + v] * 3);
            CHECK_NEAR(ctx, vec3f::dot(normal, p) + plane.constant, 0.0f, 1e-5f);
        }
        // the ring winds counter clockwise seen from outside
        const uint32_t first = data.faces[f].first_vertex;
        const Vec3f p0 = vec3f::load(data.point_positions.data, data.vertex_indices[first] * 3);
        const Vec3f p1 = vec3f::load(data.point_positions.data, data.vertex_indices[first + 1] * 3);
        const Vec3f p2 = vec3f::load(data.point_positions.data, data.vertex_indices[first + 2] * 3);
        const Vec3f winding = vec3f::cross(vec3f::subtract(p1, p0), vec3f::subtract(p2, p1));
        CHECK(ctx, vec3f::dot(winding, normal) > 0.0f);
    }

    // every corner is on three faces, all three kept for the shrink
    for (uint32_t p = 0; p < 8; p++) {
        CHECK(ctx, data.point_num_faces[p] == 3);
        CHECK(ctx, data.point_faces[p * 3] >= 0 && data.point_faces[p * 3 + 1] >= 0 && data.point_faces[p * 3 + 2] >= 0);
    }

    // the box's bounds, volume and centre of mass
    CHECK(ctx, near3(h->aabb.min, -1, -2, -3, 0.0f) && near3(h->aabb.max, 1, 2, 3, 0.0f));
    CHECK_NEAR(ctx, h->volume, 48.0f, 1e-4f);
    CHECK(ctx, near3(h->center_of_mass, 0, 0, 0, 1e-6f));

    // update recomputes the same bounds
    h->aabb = {{0, 0, 0}, {0, 0, 0}};
    convex_hull::update(h);
    CHECK(ctx, near3(h->aabb.min, -1, -2, -3, 0.0f) && near3(h->aabb.max, 1, 2, 3, 0.0f));

    // 8 points: no adjacency by default, so the support brute scans
    CHECK(ctx, data.point_neighbors_start.length == 0 && data.point_neighbors.length == 0);
    // radius 0: no shrunk copy, the fill reads the points
    CHECK(ctx, data.shrunk_point_positions.length == 0);
    drop(h);
}

void test_convex_hull_create_rejects(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    convex_hull::BuildResult result = convex_hull::BUILD_SUCCESS;

    // a negative convex radius
    CHECK(ctx, make({.positions = CUBE_POINTS, .num_positions = 8, .convex_radius = -0.1f}, &result) == nullptr);
    CHECK(ctx, result == convex_hull::BUILD_INVALID_CONVEX_RADIUS);

    // too few points
    CHECK(ctx, make({.positions = CUBE_POINTS, .num_positions = 2}, &result) == nullptr);
    CHECK(ctx, result == convex_hull::BUILD_TOO_FEW_POINTS);

    // all at one spot
    const float same[12] = {1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3};
    CHECK(ctx, make({.positions = same, .num_positions = 4}, &result) == nullptr);
    CHECK(ctx, result == convex_hull::BUILD_DEGENERATE);

    // coplanar is not a rejection: a flat square is a two-sided hull with no volume
    const float square[12] = {0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1};
    Shape *flat = make({.positions = square, .num_positions = 4}, &result);
    CHECK(ctx, flat != nullptr && result == convex_hull::BUILD_SUCCESS);
    CHECK(ctx, flat->convex_hull.data->faces.length == 2 && flat->convex_hull.data->num_points == 4);
    CHECK(ctx, flat->volume == 0.0f && flat->convex_hull.data->convex_radius == 0.0f);
    CHECK(ctx, convex_hull::get_inner_radius(flat) == 0.0f);
    drop(flat);

    // the result pointer is optional
    CHECK(ctx, make({.positions = same, .num_positions = 4}) == nullptr);

    // a rejection leaks nothing: the heap holds what it held before
    util::HeapStats before, after;
    util::heap_stats(&before);
    make({.positions = same, .num_positions = 4}, &result);
    util::heap_stats(&after);
    CHECK(ctx, before.bytes_in_use == after.bytes_in_use && before.allocations == after.allocations);

    // nor does a full build with every array filled, once destroyed
    float points[100 * 3];
    fibonacci_sphere(points, 100);
    util::heap_stats(&before);
    Shape *h = make({.positions = points, .num_positions = 100, .bake_support_adjacency = convex_hull::BAKE_SUPPORT_ADJACENCY_ALWAYS});
    CHECK(ctx, h->convex_hull.data->shrunk_point_positions.length > 0 && h->convex_hull.data->point_neighbors.length > 0);
    drop(h);
    util::heap_stats(&after);
    CHECK(ctx, before.bytes_in_use == after.bytes_in_use && before.allocations == after.allocations);
}

void test_convex_hull_interior_points(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    // cube vertices plus interior points, which the builder drops
    float positions[30];
    for (uint32_t i = 0; i < 24; i++) positions[i] = CUBE_POINTS[i];
    const float interior[6] = {0, 0, 0, 0.5f, 0.5f, 0.5f};
    for (uint32_t i = 0; i < 6; i++) positions[24 + i] = interior[i];
    Shape *h = make({.positions = positions, .num_positions = 10, .convex_radius = 0.1f, .max_error_convex_radius = 1.0f});
    CHECK(ctx, h->convex_hull.data->num_points == 8);
    // interior points filtered, radius kept with a high error tolerance
    CHECK_NEAR(ctx, h->convex_hull.data->convex_radius, 0.1f, 5e-4f);
    drop(h);
}

/* convex radius reduction (crashcat's convex-hull.test.ts) */

void test_convex_hull_radius_thickness(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    // very thin disk, 0.02 thick: radius reduced to fit twice in the thickness
    const float thin[24] = {-10, -10, 0.01f, 10, -10, 0.01f, 10, 10, 0.01f, -10, 10, 0.01f,
                            -10, -10, -0.01f, 10, -10, -0.01f, 10, 10, -0.01f, -10, 10, -0.01f};
    Shape *h = hull(thin, 8, 5.0f);
    CHECK(ctx, h->convex_hull.data->convex_radius < 0.02f && h->convex_hull.data->convex_radius > 0.0f);
    drop(h);

    // a cube's sharp corners: the 3-plane shift is |(1, 1, 1)| = sqrt 3, so the radius is 0.05 / (sqrt 3 - 1)
    h = hull(CUBE_POINTS, 8, 0.1f);
    CHECK(ctx, h->convex_hull.data->convex_radius < 0.1f && h->convex_hull.data->convex_radius > 0.05f);
    CHECK_NEAR(ctx, h->convex_hull.data->convex_radius, 0.05f / (scalar::sqrt(3.0f) - 1.0f), 1e-5f);
    drop(h);

    // with a larger error tolerance the radius survives
    h = make({.positions = CUBE_POINTS, .num_positions = 8, .convex_radius = 0.1f, .max_error_convex_radius = 1.0f});
    CHECK_NEAR(ctx, h->convex_hull.data->convex_radius, 0.1f, 5e-4f);
    drop(h);

    // zero stays zero
    h = hull(CUBE_POINTS, 8, 0.0f);
    CHECK(ctx, h->convex_hull.data->convex_radius == 0.0f);
    drop(h);
}

static const float PYRAMID[15] = {0, 10, 0, -5, 0, -5, -5, 0, 5, 5, 0, -5, 5, 0, 5};

void test_convex_hull_radius_sharp_edges(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    // a sharp apex reduces the radius
    Shape *pyramid = make({.positions = PYRAMID, .num_positions = 5, .convex_radius = 2.0f, .max_error_convex_radius = 0.05f});
    CHECK(ctx, pyramid->convex_hull.data->convex_radius < 2.0f);
    drop(pyramid);

    // stricter tolerance, smaller radius
    Shape *strict = make({.positions = PYRAMID, .num_positions = 5, .convex_radius = 2.0f, .max_error_convex_radius = 0.01f});
    Shape *lenient = make({.positions = PYRAMID, .num_positions = 5, .convex_radius = 2.0f, .max_error_convex_radius = 0.1f});
    CHECK(ctx, strict->convex_hull.data->convex_radius < lenient->convex_hull.data->convex_radius);
    drop(strict);
    drop(lenient);

    // a very sharp needle
    const float needle[15] = {0, 20, 0, -0.5f, 0, -0.5f, -0.5f, 0, 0.5f, 0.5f, 0, -0.5f, 0.5f, 0, 0.5f};
    Shape *h = make({.positions = needle, .num_positions = 5, .convex_radius = 1.0f, .max_error_convex_radius = 0.05f});
    CHECK(ctx, h->convex_hull.data->convex_radius < 0.3f && h->convex_hull.data->convex_radius >= 0.0f);
    drop(h);

    // the default max error still reduces
    h = hull(PYRAMID, 5, 2.0f);
    CHECK(ctx, h->convex_hull.data->convex_radius <= 2.0f);
    drop(h);

    // a very flat hull: the radius goes to nearly nothing
    const float flat[24] = {-10, -10, 0.001f, 10, -10, 0.001f, 10, 10, 0.001f, -10, 10, 0.001f,
                            -10, -10, -0.001f, 10, -10, -0.001f, 10, 10, -0.001f, -10, 10, -0.001f};
    h = make({.positions = flat, .num_positions = 8, .convex_radius = 5.0f, .max_error_convex_radius = 0.05f});
    CHECK(ctx, h->convex_hull.data->convex_radius < 0.01f);
    drop(h);

    // the other properties stay valid after a reduction
    h = make({.positions = PYRAMID, .num_positions = 5, .convex_radius = 2.0f}, nullptr, 1000.0f);
    const convex_hull::ConvexHullData &data = *h->convex_hull.data;
    CHECK(ctx, data.num_points == 5 && data.faces.length == 5 && data.planes.length == data.faces.length);
    // a square pyramid of base 10 x 10 and height 10
    CHECK_NEAR(ctx, h->volume, 1000.0f / 3.0f, 1e-2f);
    CHECK_NEAR(ctx, h->center_of_mass.y, 2.5f, 1e-4f);
    CHECK(ctx, h->convex_hull.density == 1000.0f);
    drop(h);
}

/** the tetrahedron's outward unit plane of the face leaving out vertex `excluded` */
static Plane tetrahedron_plane(const Float3 *vertices, uint32_t excluded) {
    Vec3f face[3];
    uint32_t k = 0;
    for (uint32_t i = 0; i < 4; i++) {
        if (i != excluded) face[k++] = vec3f::load(vertices[i]);
    }
    Vec3f normal = vec3f::normalize(vec3f::cross(vec3f::subtract(face[1], face[0]), vec3f::subtract(face[2], face[0])));
    if (vec3f::dot(normal, vec3f::subtract(vec3f::load(vertices[excluded]), face[0])) > 0.0f) normal = vec3f::negate(normal);
    return {normal, -vec3f::dot(normal, face[0])};
}

void test_convex_hull_radius_non_orthogonal_planes(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    const Float3 vertices[4] = {{0, 0, 0}, {3, 0, 0}, {0.5f, 2, 0}, {0.8f, 0.7f, 1.9f}};
    float positions[12];
    for (uint32_t i = 0; i < 4; i++) {
        positions[i * 3] = vertices[i].x;
        positions[i * 3 + 1] = vertices[i].y;
        positions[i * 3 + 2] = vertices[i].z;
    }
    const float convex_radius = 0.5f;
    const float max_error_convex_radius = 0.05f;
    Shape *h = make(
        {.positions = positions, .num_positions = 4, .convex_radius = convex_radius, .max_error_convex_radius = max_error_convex_radius});
    const convex_hull::ConvexHullData &data = *h->convex_hull.data;

    // every vertex is shifted along its 3 planes
    CHECK(ctx, data.num_points == 4);
    for (uint32_t p = 0; p < 4; p++) CHECK(ctx, data.point_num_faces[p] == 3);

    Plane planes[4];
    for (uint32_t i = 0; i < 4; i++) planes[i] = tetrahedron_plane(vertices, i);

    // pass 1: half the thinnest extent across any plane
    float expected = convex_radius;
    for (uint32_t f = 0; f < 4; f++) {
        float thickness = 0.0f;
        for (uint32_t v = 0; v < 4; v++)
            thickness = scalar::max(thickness, -(vec3f::dot(planes[f].normal, vec3f::load(vertices[v])) + planes[f].constant));
        expected = scalar::min(expected, 0.5f * thickness);
    }

    // pass 2: n has the vertex's plane normals as rows, so n^-1 has columns (n2 x n3, n3 x n1, n1 x n2) / det(n)
    bool saw_non_unit_determinant = false;
    for (uint32_t v = 0; v < 4; v++) {
        Vec3f normals[3];
        uint32_t k = 0;
        for (uint32_t f = 0; f < 4; f++) {
            if (f != v) normals[k++] = planes[f].normal;
        }
        const Vec3f n2_cross_n3 = vec3f::cross(normals[1], normals[2]);
        const Vec3f n3_cross_n1 = vec3f::cross(normals[2], normals[0]);
        const Vec3f n1_cross_n2 = vec3f::cross(normals[0], normals[1]);
        const float det = vec3f::dot(normals[0], n2_cross_n3);
        if (scalar::abs(scalar::abs(det) - 1.0f) > 0.1f) saw_non_unit_determinant = true;
        const Vec3f sum = vec3f::add(vec3f::add(n2_cross_n3, n3_cross_n1), n1_cross_n2);
        const float offset = vec3f::length(sum) / scalar::abs(det);
        expected = scalar::min(expected, max_error_convex_radius / (offset - 1.0f));
    }
    CHECK(ctx, saw_non_unit_determinant);

    CHECK_NEAR(ctx, data.convex_radius, expected, 1e-5f);
    drop(h);
}

/* hull max error */

/** a wide square with one point this high above its centre */
static void square_with_apex(float *out, float half_extent, float apex_height) {
    const float corners[8] = {-1, -1, 1, -1, 1, 1, -1, 1};
    for (uint32_t i = 0; i < 4; i++) {
        out[i * 3] = corners[i * 2] * half_extent;
        out[i * 3 + 1] = corners[i * 2 + 1] * half_extent;
        out[i * 3 + 2] = 0.0f;
    }
    out[12] = 0.0f;
    out[13] = 0.0f;
    out[14] = apex_height;
}

void test_convex_hull_max_error(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    convex_hull::BuildResult result = convex_hull::BUILD_SUCCESS;

    // the builder treats points within 6 coplanar distances of the initial triangle's plane as flat and builds a
    // two-sided hull, but the hull is only allowed an error of 4 * max(coplanar distance, hull tolerance)
    const float half_extent = 10000.0f;
    const float coplanar_distance = 3.0f * 1.1920929e-7f * (2.0f * half_extent);
    float positions[15];
    square_with_apex(positions, half_extent, 5.0f * coplanar_distance);

    Vec3f points[5];
    for (uint32_t i = 0; i < 5; i++) points[i] = vec3f::load(positions, i * 3);
    convex_hull_builder::ConvexHullBuilder builder = convex_hull_builder::create(points, 5);
    CHECK(ctx, convex_hull_builder::initialize(builder, convex_hull::MAX_POINTS_IN_HULL, 1e-3f) == convex_hull_builder::SUCCESS);
    CHECK(ctx, builder.faces.length == 2);
    convex_hull_builder::MaxError max_error;
    convex_hull_builder::determine_max_error(max_error, builder);
    CHECK(ctx, max_error.max_error_position_idx == 4);
    CHECK(ctx, max_error.max_error > 4.0f * scalar::max(max_error.coplanar_distance, 1e-3f));
    convex_hull_builder::release(builder);

    CHECK(ctx, make({.positions = positions, .num_positions = 5}, &result) == nullptr);
    CHECK(ctx, result == convex_hull::BUILD_HULL_ERROR_TOO_LARGE);

    // a rejection leaks nothing
    util::HeapStats before, after;
    util::heap_stats(&before);
    make({.positions = positions, .num_positions = 5}, &result);
    util::heap_stats(&after);
    CHECK(ctx, before.bytes_in_use == after.bytes_in_use && before.allocations == after.allocations);

    // the same square with the point within the bound
    square_with_apex(positions, half_extent, 2.0f * coplanar_distance);
    Shape *h = make({.positions = positions, .num_positions = 5}, &result);
    CHECK(ctx, h != nullptr && result == convex_hull::BUILD_SUCCESS && h->convex_hull.data->faces.length == 2);
    drop(h);

    // a clean box
    float corners[24];
    box_corners(corners, {1, 2, 3});
    h = make({.positions = corners, .num_positions = 8}, &result);
    CHECK(ctx, h != nullptr && result == convex_hull::BUILD_SUCCESS && h->convex_hull.data->faces.length == 6);
    CHECK_NEAR(ctx, h->volume, 48.0f, 1e-4f);
    drop(h);
}

/** every shrunk vertex sits on its neighbouring planes offset inward by the radius, or is pushed straight back along its one normal */
static void check_shrunk_on_planes(TestCtx &ctx, const Shape *h) {
    const convex_hull::ConvexHullData &data = *h->convex_hull.data;
    const float r = data.convex_radius;
    CHECK(ctx, r > 0.0f);
    CHECK(ctx, data.shrunk_point_positions.length == data.num_points * 3);
    const float *shrunk = data.shrunk_point_positions.data;
    for (uint32_t p = 0; p < data.num_points; p++) {
        const uint32_t b = p * 3;
        const Vec3f sp = vec3f::load(shrunk, b);
        const uint32_t num_faces = data.point_num_faces[p];
        if (num_faces == 1) {
            const Float3 &n = data.planes[data.point_faces[b]].normal;
            const float *pp = data.point_positions.data + b;
            CHECK(ctx, near3(sp, pp[0] - n.x * r, pp[1] - n.y * r, pp[2] - n.z * r, 1e-6f));
        } else {
            for (uint32_t f = 0; f < num_faces; f++) {
                const Plane3f &plane = data.planes[data.point_faces[b + f]];
                CHECK_NEAR(ctx, vec3f::dot(vec3f::load(plane.normal), sp) + plane.constant + r, 0.0f, 2e-6f);
            }
        }
    }
}

void test_convex_hull_shrunk_points(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    float sphere_points[100 * 3];
    fibonacci_sphere(sphere_points, 100);
    Shape *h = hull(sphere_points, 100, 0.05f);
    check_shrunk_on_planes(ctx, h);
    drop(h);

    h = hull(CUBE_POINTS, 8, 0.05f);
    check_shrunk_on_planes(ctx, h);
    // a cube corner shrinks diagonally inward by the radius on every axis
    const convex_hull::ConvexHullData &data = *h->convex_hull.data;
    for (uint32_t i = 0; i < 24; i++) {
        const float p = data.point_positions.data[i];
        CHECK_NEAR(ctx, data.shrunk_point_positions.data[i], p - scalar::sign(p) * 0.05f, 1e-6f);
    }
    drop(h);

    // radius 0: the shrink is the identity and nothing is stored
    h = hull(CUBE_POINTS, 8, 0.0f);
    CHECK(ctx, h->convex_hull.data->convex_radius == 0.0f && h->convex_hull.data->shrunk_point_positions.length == 0);
    drop(h);
}

/* support adjacency (crashcat's convex-hull-adjacency.test.ts) */

static uint32_t neighbor_count(const convex_hull::ConvexHullData &data, uint32_t p) {
    return data.point_neighbors_start[p + 1] - data.point_neighbors_start[p];
}

static bool has_neighbor(const convex_hull::ConvexHullData &data, uint32_t p, uint32_t n) {
    for (uint32_t i = data.point_neighbors_start[p]; i < data.point_neighbors_start[p + 1]; i++) {
        if (data.point_neighbors[i] == n) return true;
    }
    return false;
}

static bool is_face_edge(const convex_hull::ConvexHullData &data, uint32_t a, uint32_t b) {
    for (uint32_t f = 0; f < data.faces.length; f++) {
        const uint32_t first = data.faces[f].first_vertex;
        const uint32_t count = data.faces[f].num_vertices;
        for (uint32_t v = 0; v < count; v++) {
            const uint32_t x = data.vertex_indices[first + v];
            const uint32_t y = data.vertex_indices[first + (v + 1) % count];
            if ((x == a && y == b) || (x == b && y == a)) return true;
        }
    }
    return false;
}

static void check_adjacency(TestCtx &ctx, const Shape *h) {
    const convex_hull::ConvexHullData &data = *h->convex_hull.data;
    // csr layout: numPoints + 1 starts from 0, monotone, ending at the neighbour count
    CHECK(ctx, data.point_neighbors_start.length == data.num_points + 1);
    CHECK(ctx, data.point_neighbors_start[0] == 0);
    for (uint32_t p = 0; p < data.num_points; p++) CHECK(ctx, data.point_neighbors_start[p + 1] >= data.point_neighbors_start[p]);
    CHECK(ctx, data.point_neighbors_start[data.num_points] == data.point_neighbors.length);

    bool symmetric = true, no_self_or_duplicate = true, pairs_are_edges = true, spans = true;
    for (uint32_t p = 0; p < data.num_points; p++) {
        if (neighbor_count(data, p) == 0) spans = false;
        for (uint32_t i = data.point_neighbors_start[p]; i < data.point_neighbors_start[p + 1]; i++) {
            const uint32_t n = data.point_neighbors[i];
            if (!has_neighbor(data, n, p)) symmetric = false;
            if (n == p) no_self_or_duplicate = false;
            for (uint32_t j = i + 1; j < data.point_neighbors_start[p + 1]; j++) {
                if (data.point_neighbors[j] == n) no_self_or_duplicate = false;
            }
            if (!is_face_edge(data, p, n)) pairs_are_edges = false;
        }
    }
    CHECK(ctx, symmetric);
    CHECK(ctx, no_self_or_duplicate);
    CHECK(ctx, pairs_are_edges);
    CHECK(ctx, spans);

    // every face edge is an adjacency pair
    bool edges_are_pairs = true;
    uint32_t face_vertex_count = 0;
    for (uint32_t f = 0; f < data.faces.length; f++) {
        const uint32_t first = data.faces[f].first_vertex;
        const uint32_t count = data.faces[f].num_vertices;
        face_vertex_count += count;
        for (uint32_t v = 0; v < count; v++) {
            if (!has_neighbor(data, data.vertex_indices[first + v], data.vertex_indices[first + (v + 1) % count])) edges_are_pairs = false;
        }
    }
    CHECK(ctx, edges_are_pairs);
    // handshaking: the valencies sum to twice the edge count, which is the face vertex count
    CHECK(ctx, data.point_neighbors.length == face_vertex_count);
}

void test_convex_hull_adjacency_flag(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    // a box (8 points) is not baked by default
    Shape *b = hull(CUBE_POINTS, 8, 0.05f);
    CHECK(ctx, b->convex_hull.data->num_points == 8 && b->convex_hull.data->num_points <= convex_hull::SUPPORT_HILL_CLIMB_MIN_POINTS);
    CHECK(ctx, b->convex_hull.data->point_neighbors_start.length == 0 && b->convex_hull.data->point_neighbors.length == 0);
    drop(b);

    // forced
    b = hull(CUBE_POINTS, 8, 0.05f, convex_hull::BAKE_SUPPORT_ADJACENCY_ALWAYS);
    CHECK(ctx, b->convex_hull.data->point_neighbors_start.length == 9 && b->convex_hull.data->point_neighbors.length > 0);
    drop(b);

    // never, even above the threshold
    float points[256 * 3];
    fibonacci_sphere(points, 100);
    Shape *h = hull(points, 100, 0.05f, convex_hull::BAKE_SUPPORT_ADJACENCY_NEVER);
    CHECK(ctx, h->convex_hull.data->num_points > convex_hull::SUPPORT_HILL_CLIMB_MIN_POINTS);
    CHECK(ctx, h->convex_hull.data->point_neighbors_start.length == 0 && h->convex_hull.data->point_neighbors.length == 0);
    drop(h);

    // auto above the threshold
    const uint32_t counts[3] = {60, 100, 256};
    for (uint32_t n : counts) {
        fibonacci_sphere(points, n);
        h = hull(points, n, 0.05f);
        CHECK(ctx, h->convex_hull.data->num_points > convex_hull::SUPPORT_HILL_CLIMB_MIN_POINTS);
        CHECK(ctx, h->convex_hull.data->point_neighbors_start.length == h->convex_hull.data->num_points + 1);
        drop(h);
    }
}

void test_convex_hull_adjacency_csr(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    float points[256 * 3];

    Shape *h = hull(CUBE_POINTS, 8, 0.05f, convex_hull::BAKE_SUPPORT_ADJACENCY_ALWAYS);
    check_adjacency(ctx, h);
    // a cube vertex has three neighbours
    for (uint32_t p = 0; p < 8; p++) CHECK(ctx, neighbor_count(*h->convex_hull.data, p) == 3);
    drop(h);

    const uint32_t counts[3] = {60, 100, 256};
    for (uint32_t n : counts) {
        fibonacci_sphere(points, n);
        h = hull(points, n, 0.05f);
        check_adjacency(ctx, h);
        drop(h);
    }

    slab_grid(points);
    h = hull(points, 50, 0.02f, convex_hull::BAKE_SUPPORT_ADJACENCY_ALWAYS);
    check_adjacency(ctx, h);
    drop(h);
}

/* a hull of a box's corners behaves as the box */

static const Float3 BOX_HALF_EXTENTS = {1, 2, 3};

void test_convex_hull_as_box_mass_and_radius(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    float corners[24];
    box_corners(corners, BOX_HALF_EXTENTS);
    Shape *h = make({.positions = corners, .num_positions = 8}, nullptr, 10.0f);
    Shape *b = box::create({.half_extents = BOX_HALF_EXTENTS, .density = 10.0f});

    mass_properties::MassProperties hull_mass = mass_properties::create();
    mass_properties::MassProperties box_mass = mass_properties::create();
    convex_hull::compute_mass_properties(hull_mass, h);
    box::compute_mass_properties(box_mass, b);
    CHECK_NEAR(ctx, hull_mass.mass, box_mass.mass, 1e-2f);
    CHECK_NEAR(ctx, hull_mass.mass, 480.0f, 1e-2f);
    for (uint32_t col = 0; col < 3; col++) {
        for (uint32_t row = 0; row < 3; row++) {
            CHECK_NEAR(ctx, hull_mass.inertia.m[col * 4 + row], box_mass.inertia.m[col * 4 + row], 1e-2f);
        }
    }
    // m / 12 (b^2 + c^2) and so on, for full extents 2, 4, 6
    CHECK_NEAR(ctx, hull_mass.inertia.m[0], 480.0f / 12.0f * (16.0f + 36.0f), 1e-2f);
    CHECK_NEAR(ctx, hull_mass.inertia.m[5], 480.0f / 12.0f * (4.0f + 36.0f), 1e-2f);
    CHECK_NEAR(ctx, hull_mass.inertia.m[10], 480.0f / 12.0f * (4.0f + 16.0f), 1e-2f);

    // bounds and inner radius
    CHECK(ctx, near3(h->aabb.min, b->aabb.min, 0.0f) && near3(h->aabb.max, b->aabb.max, 0.0f));
    CHECK_NEAR(ctx, convex_hull::get_inner_radius(h), box::get_inner_radius(b), 1e-6f);
    CHECK_NEAR(ctx, convex_hull::get_inner_radius(h), 1.0f, 1e-6f);

    drop(h);
    box::destroy(b);

    // crashcat's inner radius test: a 2x2x2 box hull
    h = make({.positions = CUBE_POINTS, .num_positions = 8});
    CHECK_NEAR(ctx, convex_hull::get_inner_radius(h), 1.0f, 1e-6f);
    drop(h);

    // the inner radius is measured from the centre of mass, wherever the hull sits
    float shifted[24];
    for (uint32_t i = 0; i < 24; i++) shifted[i] = CUBE_POINTS[i] + (i % 3 == 0 ? 10.0f : 0.0f);
    h = make({.positions = shifted, .num_positions = 8});
    CHECK(ctx, near3(h->center_of_mass, 10, 0, 0, 1e-5f));
    CHECK_NEAR(ctx, convex_hull::get_inner_radius(h), 1.0f, 1e-5f);
    drop(h);

    // a skewed tetrahedron: the shortest centre of mass to plane distance
    const Float3 vertices[4] = {{3, 1, 2}, {7, 1, 2}, {4, 6, 2}, {5, 3, 8}};
    float positions[12];
    Vec3f center_of_mass = vec3f::zero();
    for (uint32_t i = 0; i < 4; i++) {
        positions[i * 3] = vertices[i].x;
        positions[i * 3 + 1] = vertices[i].y;
        positions[i * 3 + 2] = vertices[i].z;
        // a tetrahedron's centre of mass is the mean of its vertices
        center_of_mass = vec3f::scale_and_add(center_of_mass, vec3f::load(vertices[i]), 0.25f);
    }
    h = make({.positions = positions, .num_positions = 4});
    CHECK(ctx, near3(h->center_of_mass, center_of_mass, 1e-5f));
    float expected = __FLT_MAX__;
    for (uint32_t i = 0; i < 4; i++) {
        const Plane plane = tetrahedron_plane(vertices, i);
        expected = scalar::min(expected, -(vec3f::dot(plane.normal, center_of_mass) + plane.constant));
    }
    CHECK(ctx, expected > 0.5f);
    CHECK_NEAR(ctx, convex_hull::get_inner_radius(h), expected, 1e-5f);
    drop(h);
}

void test_convex_hull_as_box_surface_normal(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    float corners[24];
    box_corners(corners, BOX_HALF_EXTENTS);
    Shape *h = hull(corners, 8, 0.0f);
    Shape *b = box::create({.half_extents = BOX_HALF_EXTENTS});

    const Float3 positions[] = {{1.1f, 0.0f, 0.0f}, {0.3f, 2.0f, -0.2f}, {-0.5f, 0.5f, -3.0f + 1e-3f}, {0.0f, -2.1f, 0.0f},
                               {-1.0f, 0.2f, 0.4f}, {0.2f, 0.3f, 2.95f}, {0.9f, 1.5f, 0.5f}};
    for (const Float3 &position : positions) {
        shapes::SurfaceNormalResult hull_io, box_io;
        hull_io.position = vec3f::load(position);
        box_io.position = vec3f::load(position);
        convex_hull::get_surface_normal(hull_io, h, sub_shape::EMPTY_SUB_SHAPE_ID);
        box::get_surface_normal(box_io, b, sub_shape::EMPTY_SUB_SHAPE_ID);
        CHECK(ctx, near3(hull_io.normal, box_io.normal, 1e-6f));
    }

    // crashcat's surface normal tests: always a unit axis normal, even at the centre
    const Float3 queries[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 0}};
    for (const Float3 &query : queries) {
        const Vec3f normal = shapes::get_shape_surface_normal(h, vec3f::load(query), sub_shape::EMPTY_SUB_SHAPE_ID);
        CHECK_NEAR(ctx, vec3f::length(normal), 1.0f, 1e-6f);
        const uint32_t axes =
            (scalar::abs(vec3f::x(normal)) > 0.9f) + (scalar::abs(vec3f::y(normal)) > 0.9f) + (scalar::abs(vec3f::z(normal)) > 0.9f);
        CHECK(ctx, axes == 1);
    }

    drop(h);
    box::destroy(b);
}

static void hull_face(shapes::SupportingFaceResult &io, const Shape *shape, Vec3f direction, const Mat4f &transform, Vec3f scale) {
    io.transform = transform;
    io.scale = scale;
    convex_hull::get_supporting_face(io, direction, shape, sub_shape::EMPTY_SUB_SHAPE_ID);
}

void test_convex_hull_as_box_supporting_face(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    float corners[24];
    box_corners(corners, BOX_HALF_EXTENTS);
    Shape *h = hull(corners, 8, 0.0f);
    Shape *b = box::create({.half_extents = BOX_HALF_EXTENTS});

    const Mat4f placed = mat4f::from_rotation_translation(axis_angle(vec3f::make(1, 2, -3), 0.7f), vec3f::make(5, -6, 7));
    const Mat4f identity = mat4f::identity();

    const Float3 directions[] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}, {10, 1, 1}, {0.2f, -3, 0.5f}};
    const Float3 scales[] = {{1, 1, 1}, {2, 0.5f, 3}, {-1, 1, 1}, {-2, -1, 0.5f}, {-1.5f, -1.5f, -1.5f}};
    const Mat4f *transforms[] = {&identity, &placed};

    for (const Mat4f *transform : transforms) {
        for (const Float3 &scale : scales) {
            for (const Float3 &direction : directions) {
                shapes::SupportingFaceResult hull_io, box_io;
                hull_face(hull_io, h, vec3f::load(direction), *transform, vec3f::load(scale));
                box_io.transform = *transform;
                box_io.scale = vec3f::load(scale);
                // the mirrored box is the same box, so both give the world face opposing the direction
                box::get_supporting_face(box_io, vec3f::load(direction), b, sub_shape::EMPTY_SUB_SHAPE_ID);
                CHECK(ctx, hull_io.face.num_vertices == 4);
                CHECK(ctx, same_vertex_set(hull_io.face, box_io.face, 1e-4f));
                // both wound outward: the winding normal agrees, whatever the mirror
                CHECK(ctx, vec3f::dot(face_winding(hull_io.face), face_winding(box_io.face)) > 0.0f);
            }
        }
    }

    drop(h);
    box::destroy(b);
}

void test_convex_hull_as_box_support(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    float corners[24];
    box_corners(corners, BOX_HALF_EXTENTS);
    Shape *h = hull(corners, 8, 0.05f);
    Shape *b = box::create({.half_extents = BOX_HALF_EXTENTS, .convex_radius = 0.05f});
    CHECK(ctx, h->convex_hull.data->convex_radius == 0.05f);

    Vec3f directions[40];
    random_directions(directions, 40, 0x5eed);
    // scales where the box's radius cap (the default convex radius) does not bite
    const Float3 scales[] = {{1, 1, 1}, {0.5f, 0.5f, 0.5f}, {1, 2, 0.5f}, {-1, 1, 1}, {-1.5f, 0.5f, -1}};
    const support::SupportFunctionMode modes[] = {support::INCLUDE_CONVEX_RADIUS, support::EXCLUDE_CONVEX_RADIUS};

    support::Support hull_support = support::create();
    support::Support box_support = support::create();
    for (const Float3 &scale : scales) {
        for (support::SupportFunctionMode mode : modes) {
            convex_hull::set_support(hull_support, h, mode, vec3f::load(scale));
            box::set_support(box_support, b, mode, vec3f::load(scale));
            CHECK(ctx, hull_support.get_support == support::hull_support);
            CHECK_NEAR(ctx, hull_support.convex_radius, box_support.convex_radius, 1e-6f);
            for (Vec3f direction : directions) {
                CHECK(ctx, near3(support_point(hull_support, direction), support_point(box_support, direction), 1e-5f));
            }
        }
    }
    drop(h);
    box::destroy(b);
}

/* supporting face (crashcat's supporting-face.test.ts and convex-hull-supporting-face.test.ts) */

void test_convex_hull_supporting_face_cases(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *h = hull(CUBE_POINTS, 8, 0.0f);
    const Mat4f identity = mat4f::identity();
    face::Face face;

    // +y gives the face opposing it, the bottom
    shapes::get_shape_supporting_face(face, h, sub_shape::EMPTY_SUB_SHAPE_ID, vec3f::make(0, 1, 0), identity, vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < face.num_vertices; i++) CHECK_NEAR(ctx, vec3f::y(face.vertices[i]), -1.0f, 1e-6f);
    shapes::get_shape_supporting_face(face, h, sub_shape::EMPTY_SUB_SHAPE_ID, vec3f::make(0, -1, 0), identity, vec3f::make(1, 1, 1));
    for (uint32_t i = 0; i < face.num_vertices; i++) CHECK_NEAR(ctx, vec3f::y(face.vertices[i]), 1.0f, 1e-6f);
    shapes::get_shape_supporting_face(face, h, sub_shape::EMPTY_SUB_SHAPE_ID, vec3f::make(1, 0, 0), identity, vec3f::make(1, 1, 1));
    for (uint32_t i = 0; i < face.num_vertices; i++) CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), -1.0f, 1e-6f);

    // scaled: the bottom at -1 * 3
    shapes::get_shape_supporting_face(face, h, sub_shape::EMPTY_SUB_SHAPE_ID, vec3f::make(0, 1, 0), identity, vec3f::make(2, 3, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < face.num_vertices; i++) CHECK_NEAR(ctx, vec3f::y(face.vertices[i]), -3.0f, 1e-6f);

    // translated: the bottom at 10 - 1
    const Mat4f translated = mat4f::from_translation(vec3f::make(5, 10, -3));
    shapes::get_shape_supporting_face(face, h, sub_shape::EMPTY_SUB_SHAPE_ID, vec3f::make(0, 1, 0), translated, vec3f::make(1, 1, 1));
    for (uint32_t i = 0; i < face.num_vertices; i++) CHECK_NEAR(ctx, vec3f::y(face.vertices[i]), 9.0f, 1e-6f);

    // inside out: the mirrored -x face (now at +x) faces -x, its winding reversed so it stays outward
    shapes::get_shape_supporting_face(face, h, sub_shape::EMPTY_SUB_SHAPE_ID, vec3f::make(1, 0, 0), identity, vec3f::make(-1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < face.num_vertices; i++) CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), -1.0f, 1e-6f);
    CHECK(ctx, vec3f::x(face_winding(face)) < 0.0f);
    drop(h);

    // a pyramid: -y returns one of the triangular sides
    const float pyramid[15] = {-1, 0, -1, 1, 0, -1, -1, 0, 1, 1, 0, 1, 0, 2, 0};
    h = hull(pyramid, 5, 0.0f);
    shapes::get_shape_supporting_face(face, h, sub_shape::EMPTY_SUB_SHAPE_ID, vec3f::make(0, -1, 0), identity, vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 3);
    drop(h);
}

/** the sqrt-per-plane selection the uniform fast path replaced */
static void reference_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape) {
    const convex_hull::ConvexHullData &data = *shape->convex_hull.data;
    const Vec3f inv_scale = vec3f::make(1.0f / vec3f::x(io.scale), 1.0f / vec3f::y(io.scale), 1.0f / vec3f::z(io.scale));
    Vec3f plane_normal = vec3f::multiply(inv_scale, vec3f::load(data.planes[0].normal));
    float best_dot = vec3f::dot(plane_normal, direction) / vec3f::length(plane_normal);
    uint32_t best_face_idx = 0;
    for (uint32_t i = 1; i < data.planes.length; i++) {
        plane_normal = vec3f::multiply(inv_scale, vec3f::load(data.planes[i].normal));
        const float dot = vec3f::dot(plane_normal, direction) / vec3f::length(plane_normal);
        if (dot < best_dot) {
            best_dot = dot;
            best_face_idx = i;
        }
    }

    const convex_hull::ConvexHullFace &best_face = data.faces[best_face_idx];
    const int32_t num_vertices = (int32_t)best_face.num_vertices;
    const int32_t max_vertices_to_return = (int32_t)(convex_hull::MAX_FACE_VERTICES / 2);
    const int32_t delta_vtx = (num_vertices + max_vertices_to_return - 1) / max_vertices_to_return;
    face::Face &face = io.face;
    face.num_vertices = 0;
    const bool inside_out = face::is_scale_inside_out(io.scale);
    for (int32_t k = 0; k < num_vertices; k += delta_vtx) {
        const int32_t i = inside_out ? num_vertices - 1 - k : k;
        const uint32_t pbase = data.vertex_indices[best_face.first_vertex + (uint32_t)i] * 3;
        face.vertices[face.num_vertices++] = vec3f::load(data.point_positions.data, pbase);
    }
    face::transform_with_mat4_scale(face, io.transform, io.scale);
}

void test_convex_hull_supporting_face_uniform_fast_path(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    float sphere_points[100 * 3];
    fibonacci_sphere(sphere_points, 100);
    float slab[50 * 3];
    slab_grid(slab);
    Shape *hulls[3] = {hull(CUBE_POINTS, 8, 0.05f), hull(sphere_points, 100, 0.05f),
                       hull(slab, 50, 0.02f, convex_hull::BAKE_SUPPORT_ADJACENCY_ALWAYS)};

    Vec3f directions[83];
    random_directions(directions, 80, 0xabcdef);
    directions[80] = vec3f::axis_x();
    directions[81] = vec3f::make(0, -1, 0);
    directions[82] = vec3f::axis_z();

    const Mat4f transform = mat4f::from_rotation_translation(axis_angle(vec3f::make(1, 2, -3), 0.7f), vec3f::make(5, -6, 7));

    const Float3 scales[] = {{0.5f, 0.5f, 0.5f}, {1, 1, 1}, {2.37f, 2.37f, 2.37f}, {-1.5f, -1.5f, -1.5f}, {2, 0.5f, 3}, {-2, 1, 0.5f}};
    for (Shape *h : hulls) {
        for (const Float3 &scale : scales) {
            bool same = true;
            for (Vec3f direction : directions) {
                shapes::SupportingFaceResult actual, reference;
                hull_face(actual, h, direction, transform, vec3f::load(scale));
                reference.transform = transform;
                reference.scale = vec3f::load(scale);
                reference_supporting_face(reference, direction, h);
                if (actual.face.num_vertices != reference.face.num_vertices) {
                    same = false;
                    continue;
                }
                for (uint32_t i = 0; i < reference.face.num_vertices; i++) {
                    if (!near3(actual.face.vertices[i], reference.face.vertices[i], 1e-5f)) same = false;
                }
            }
            CHECK(ctx, same);
        }
    }
    for (Shape *h : hulls) drop(h);
}

/* support (crashcat's support.test.ts and support-hill-climb.test.ts) */

void test_convex_hull_support_include(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    // unit cube corners plus the centre, which must not survive the hull
    float positions[27];
    for (uint32_t i = 0; i < 24; i++) positions[i] = CUBE_POINTS[i];
    positions[24] = positions[25] = positions[26] = 0.0f;
    Shape *h = hull(positions, 9, 0.0f);
    const convex_hull::ConvexHullData &data = *h->convex_hull.data;
    CHECK(ctx, data.num_points == 8);

    const Float3 axes[] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}, {1, 1, 1}, {-1, 2, -3}};
    support::Support s = support::create();
    convex_hull::set_support(s, h, support::INCLUDE_CONVEX_RADIUS, vec3f::make(1, 1, 1));
    CHECK(ctx, s.get_support == support::hull_support && s.convex_radius == 0.0f && s.hull.output_scale == 1.0f);
    for (const Float3 &axis : axes) {
        CHECK(ctx, near3(support_point(s, vec3f::load(axis)), brute_force_max_dot(data.point_positions.data, 8, vec3f::load(axis)), 0.0f));
    }

    // scaled include scans the scaled vertices
    const Float3 scale = {2, 0.5f, 3};
    float scaled[24];
    for (uint32_t i = 0; i < 24; i += 3) {
        scaled[i] = data.point_positions.data[i] * 2.0f;
        scaled[i + 1] = data.point_positions.data[i + 1] * 0.5f;
        scaled[i + 2] = data.point_positions.data[i + 2] * 3.0f;
    }
    convex_hull::set_support(s, h, support::INCLUDE_CONVEX_RADIUS, vec3f::load(scale));
    CHECK(ctx, s.hull.output_scale == 1.0f && s.hull.vertices == s.hull.scratch);
    for (const Float3 &axis : axes) CHECK(ctx, near3(support_point(s, vec3f::load(axis)), brute_force_max_dot(scaled, 8, vec3f::load(axis)), 0.0f));

    // a sub-threshold hull brute scans: no adjacency borrowed
    CHECK(ctx, s.hull.neighbors_start == nullptr);
    convex_hull::set_support(s, h, support::INCLUDE_CONVEX_RADIUS, vec3f::make(1, 1, 1));
    CHECK(ctx, near3(support_point(s, vec3f::make(1, 1, 1)), 1, 1, 1, 0.0f));

    // through the def: the dispatch installs the hull evaluator
    dispatch::init();
    const float tetrahedron[12] = {-1, -1, -1, 1, -1, -1, 0, 1, -1, 0, 0, 1};
    Shape *t = hull(tetrahedron, 4, support::DEFAULT_CONVEX_RADIUS);
    CHECK(ctx, shapes::is_convex(t));
    shapes::set_shape_support(s, t, support::INCLUDE_CONVEX_RADIUS, vec3f::make(1, 1, 1));
    CHECK(ctx, s.get_support == support::hull_support);

    drop(h);
    drop(t);
}

void test_convex_hull_support_scaling(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    float sphere_points[100 * 3];
    fibonacci_sphere(sphere_points, 100);
    Shape *h = hull(sphere_points, 100, 0.05f);
    const convex_hull::ConvexHullData &data = *h->convex_hull.data;
    Vec3f directions[50];
    random_directions(directions, 50, 0x9e3779b9u);

    static float scaled[256 * 3];
    support::Support s = support::create();
    const float uniform_scales[3] = {0.5f, 1.0f, 2.37f};
    for (float k : uniform_scales) {
        // include: the raw vertices scaled by k, through the output scale
        convex_hull::set_support(s, h, support::INCLUDE_CONVEX_RADIUS, vec3f::make(k, k, k));
        CHECK(ctx, s.convex_radius == 0.0f && s.hull.output_scale == k && s.hull.vertices == data.point_positions.data);
        for (uint32_t i = 0; i < data.num_points * 3; i++) scaled[i] = data.point_positions.data[i] * k;
        for (Vec3f direction : directions) {
            CHECK(ctx, near3(support_point(s, direction), brute_force_max_dot(scaled, data.num_points, direction), 1e-6f));
        }

        // exclude: the shrunk vertices scaled by k, the radius reported as r k
        convex_hull::set_support(s, h, support::EXCLUDE_CONVEX_RADIUS, vec3f::make(k, k, k));
        CHECK_NEAR(ctx, s.convex_radius, data.convex_radius * k, 1e-7f);
        CHECK(ctx, s.hull.vertices == data.shrunk_point_positions.data);
        for (uint32_t i = 0; i < data.num_points * 3; i++) scaled[i] = data.shrunk_point_positions.data[i] * k;
        for (Vec3f direction : directions) {
            CHECK(ctx, near3(support_point(s, direction), brute_force_max_dot(scaled, data.num_points, direction), 1e-6f));
        }
    }
    drop(h);

    // non-uniform exclude bakes the scaled shrunk vertices: each scaled corner pushed in by r minScale along its three normals
    Shape *cube = hull(CUBE_POINTS, 8, 0.05f);
    const Float3 scale = {1, 2, 0.5f};
    const float scaled_radius = cube->convex_hull.data->convex_radius * 0.5f;
    convex_hull::set_support(s, cube, support::EXCLUDE_CONVEX_RADIUS, vec3f::load(scale));
    CHECK_NEAR(ctx, s.convex_radius, scaled_radius, 1e-7f);
    CHECK(ctx, s.hull.output_scale == 1.0f);
    float reference[24];
    for (uint32_t i = 0; i < 24; i += 3) {
        const float *c = cube->convex_hull.data->point_positions.data + i;
        reference[i] = c[0] * scale.x - scalar::sign(c[0]) * scaled_radius;
        reference[i + 1] = c[1] * scale.y - scalar::sign(c[1]) * scaled_radius;
        reference[i + 2] = c[2] * scale.z - scalar::sign(c[2]) * scaled_radius;
    }
    for (Vec3f direction : directions) CHECK(ctx, near3(support_point(s, direction), brute_force_max_dot(reference, 8, direction), 1e-6f));
    drop(cube);
}

static float max_abs_vertex(const support::Support &s) {
    float m = 0.0f;
    for (uint32_t i = 0; i < s.hull.vertex_count * 3; i++) m = scalar::max(m, scalar::abs(s.hull.vertices[i]));
    return m * s.hull.output_scale;
}

void test_convex_hull_support_hill_climb(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    static float points[256 * 3];
    Shape *hulls[5];
    const bool strictly_convex[5] = {true, true, true, false, false};
    fibonacci_sphere(points, 60);
    hulls[0] = hull(points, 60, 0.05f);
    fibonacci_sphere(points, 100);
    hulls[1] = hull(points, 100, 0.05f);
    fibonacci_sphere(points, 256);
    hulls[2] = hull(points, 256, 0.05f);
    hulls[3] = hull(CUBE_POINTS, 8, 0.05f, convex_hull::BAKE_SUPPORT_ADJACENCY_ALWAYS);
    slab_grid(points);
    hulls[4] = hull(points, 50, 0.02f, convex_hull::BAKE_SUPPORT_ADJACENCY_ALWAYS);

    const Float3 scales[] = {{1, 1, 1}, {2.37f, 2.37f, 2.37f}, {2, 0.5f, 3}, {-1.5f, -1.5f, -1.5f}, {-2, 1, 0.5f}};
    const support::SupportFunctionMode modes[] = {support::INCLUDE_CONVEX_RADIUS, support::EXCLUDE_CONVEX_RADIUS};
    Vec3f random[60];
    Vec3f rotating[120];
    rotating_directions(rotating, 120);

    support::Support accelerated = support::create();
    support::Support reference = support::create();
    for (uint32_t hi = 0; hi < 5; hi++) {
        const Shape *h = hulls[hi];
        // the adjacency must be baked, else the climb is never exercised
        CHECK(ctx, h->convex_hull.data->point_neighbors.length > 0);
        random_directions(random, 60, 0x1234 + h->convex_hull.data->num_points);

        for (const Float3 &scale : scales) {
            for (support::SupportFunctionMode mode : modes) {
                // the reference: the same resolved vertices with the adjacency stripped, a brute scan
                convex_hull::set_support(reference, h, mode, vec3f::load(scale));
                reference.hull.neighbors_start = nullptr;
                const float tolerance = 1e-5f * max_abs_vertex(reference);
                bool climb_reaches_max = true, same_vertex = true;

                // cold: each direction from a fresh fill, the first evaluation brute seeding the hint
                for (Vec3f direction : random) {
                    convex_hull::set_support(accelerated, h, mode, vec3f::load(scale));
                    CHECK(ctx, accelerated.hull.neighbors_start != nullptr && accelerated.hull.last_vertex == -1);
                    const Vec3f a = support_point(accelerated, direction);
                    const Vec3f b = support_point(reference, direction);
                    if (vec3f::dot(a, direction) < vec3f::dot(b, direction) - tolerance) climb_reaches_max = false;
                    if (strictly_convex[hi] && !near3(a, b, 1e-5f)) same_vertex = false;
                }

                // warm: one fill keeps its hint across a rotating sequence
                convex_hull::set_support(accelerated, h, mode, vec3f::load(scale));
                for (Vec3f direction : rotating) {
                    const Vec3f a = support_point(accelerated, direction);
                    const Vec3f b = support_point(reference, direction);
                    if (vec3f::dot(a, direction) < vec3f::dot(b, direction) - tolerance) climb_reaches_max = false;
                    if (strictly_convex[hi] && !near3(a, b, 1e-5f)) same_vertex = false;
                }
                CHECK(ctx, climb_reaches_max);
                CHECK(ctx, same_vertex);
            }
        }
    }
    for (Shape *h : hulls) drop(h);
}

/* through the dispatch tables */

static void collide(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings, const Shape *a,
                    const Transformf &transform_a, const Shape *b, const Transformf &transform_b) {
    shapes::collide_fn(a->type, b->type)(collector, settings, a, NO_SUB_SHAPE, transform_a, b, NO_SUB_SHAPE, transform_b);
}

void test_convex_hull_dispatch_wiring(TestCtx &ctx) {
    dispatch::init();
    const shapes::ShapeType convex_types[] = {shapes::SHAPE_SPHERE, shapes::SHAPE_BOX, shapes::SHAPE_CAPSULE, shapes::SHAPE_CONVEX_HULL,
                                              shapes::SHAPE_CYLINDER};
    for (shapes::ShapeType t : convex_types) {
        CHECK(ctx, shapes::collide_fn(shapes::SHAPE_CONVEX_HULL, t) == convex::collide_convex_vs_convex);
        CHECK(ctx, shapes::collide_fn(t, shapes::SHAPE_CONVEX_HULL) == convex::collide_convex_vs_convex);
        CHECK(ctx, shapes::cast_fn(shapes::SHAPE_CONVEX_HULL, t) == convex::cast_convex_vs_convex);
        CHECK(ctx, shapes::cast_fn(t, shapes::SHAPE_CONVEX_HULL) == convex::cast_convex_vs_convex);
    }
    // registering the hull leaves the specialised pairs alone
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_SPHERE, shapes::SHAPE_BOX) == box::collide_sphere_vs_box);
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_SPHERE, shapes::SHAPE_SPHERE) == sphere::collide_sphere_vs_sphere);
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_CONVEX_HULL, shapes::SHAPE_EMPTY) == nullptr);

    const shapes::ShapeDef &hull_def = shapes::shape_defs[shapes::shape_type_index(shapes::SHAPE_CONVEX_HULL)];
    CHECK(ctx, hull_def.type == shapes::SHAPE_CONVEX_HULL && hull_def.set_support == convex_hull::set_support);
    CHECK(ctx, hull_def.cast_ray == convex::cast_ray_vs_convex && hull_def.collide_point == convex::collide_point_vs_convex);
    CHECK(ctx, hull_def.get_leaf_shape == shapes::default_get_leaf_shape);
}

void test_convex_hull_collide_sphere(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *h = make({.positions = CUBE_POINTS, .num_positions = 8}, nullptr, shapes::DEFAULT_SHAPE_DENSITY, 5);
    Shape *s = sphere::create({.radius = 0.5f, .material_id = 2});
    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.collect_faces = true;
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // a sphere 0.1 into the +y face: the axis moving the sphere out is +y, a to b
    collide(collector, settings, h, pose({0, 0, 0}), s, pose({0.3f, 1.4f, -0.2f}));
    CHECK(ctx, collector.hits.length == 1);
    const collide_shape::CollideShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.penetration, 0.1f, 1e-4f);
    CHECK(ctx, vec3f::dot(unit(hit.penetration_axis), vec3f::axis_y()) > 0.9999f);
    CHECK(ctx, near3(hit.point_a, 0.3f, 1.0f, -0.2f, 1e-4f));
    CHECK(ctx, near3(hit.point_b, 0.3f, 0.9f, -0.2f, 1e-4f));
    CHECK(ctx, hit.material_id_a == 5 && hit.material_id_b == 2);
    // the hull's face is its +y face
    CHECK(ctx, hit.face_a.num_vertices == 4);
    for (uint32_t i = 0; i < 4; i++) CHECK_NEAR(ctx, vec3f::y(hit.face_a.vertices[i]), 1.0f, 1e-6f);

    // crashcat's hull vs sphere: radius 1.51 at 2.5 just reaches the x = 1 face, 0.01 deep
    sphere::destroy(s);
    s = sphere::create({.radius = 1.51f});
    Shape *sharp = hull(CUBE_POINTS, 8, 0.0f);
    collide_shape::reset(collector);
    collide(collector, settings, sharp, pose({0, 0, 0}), s, pose({2.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.01f, 1e-4f);

    // deep: radius 1 at 1.5 is 0.5 in
    sphere::destroy(s);
    s = sphere::create({.radius = 1.0f});
    collide_shape::reset(collector);
    collide(collector, settings, sharp, pose({0, 0, 0}), s, pose({1.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-4f);
    CHECK(ctx, vec3f::dot(unit(collector.hits[0].penetration_axis), vec3f::axis_x()) > 0.9999f);

    // the hull turned a quarter about z and moved: its faces still flat, the sphere 0.1 into what is now the -x face
    collide_shape::reset(collector);
    Float4 quarter_z;
    quatf::store(quarter_z, quatf::set_axis_angle(vec3f::axis_z(), scalar::PI * 0.5f));
    sphere::destroy(s);
    s = sphere::create({.radius = 0.5f});
    collide(collector, settings, s, pose({10 - 1.4f, 0, 0}), h, pose({10, 0, 0}, quarter_z));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-4f);
    CHECK(ctx, vec3f::dot(unit(collector.hits[0].penetration_axis), vec3f::axis_x()) > 0.9999f);
    CHECK(ctx, near3(collector.hits[0].point_b, 9, 0, 0, 1e-4f));

    // scaled by 2 on y: the top face at y = 2
    collide_shape::reset(collector);
    collide(collector, settings, h, pose({0, 0, 0}, {0, 0, 0, 1}, {1, 2, 1}), s, pose({0, 2.4f, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-4f);

    // clear: nothing
    collide_shape::reset(collector);
    collide(collector, settings, h, pose({0, 0, 0}), s, pose({0, 1.6f, 0}));
    CHECK(ctx, collector.hits.length == 0);

    collide_shape::release(collector);
    drop(h);
    drop(sharp);
    sphere::destroy(s);
}

void test_convex_hull_collide_box_and_hull(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *h = hull(CUBE_POINTS, 8, 0.0f);
    Shape *rounded = hull(CUBE_POINTS, 8, 0.05f);
    Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // a box 0.05 into the hull's +y face, the rounded cores still apart
    collide(collector, collide_shape::default_settings(), rounded, pose({0, 0, 0}), b, pose({0.2f, 1.45f, 0.1f}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.05f, 1e-4f);
    CHECK(ctx, vec3f::dot(unit(collector.hits[0].penetration_axis), vec3f::axis_y()) > 0.9999f);
    CHECK_NEAR(ctx, vec3f::y(collector.hits[0].point_a), 1.0f, 1e-4f);
    CHECK_NEAR(ctx, vec3f::y(collector.hits[0].point_b), 0.95f, 1e-4f);

    // crashcat's hull vs hull: two 2-cubes 1.5 apart overlap by 0.5
    Shape *other = hull(CUBE_POINTS, 8, 0.0f);
    collide_shape::reset(collector);
    collide(collector, collide_shape::default_settings(), h, pose({0, 0, 0}), other, pose({1.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-4f);

    // with a convex radius on both, 1.8 apart: 0.2 overlap
    Shape *rounded_a = hull(CUBE_POINTS, 8, 0.1f);
    Shape *rounded_b = hull(CUBE_POINTS, 8, 0.1f);
    collide_shape::reset(collector);
    collide(collector, collide_shape::default_settings(), rounded_a, pose({0, 0, 0}), rounded_b, pose({1.8f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.2f, 1e-4f);

    // a tetrahedron hull overlapping the cube
    const float tetrahedron[12] = {-1, 0, -1, 0, 0, 1, 1, 0, -1, 0, -1, 0};
    Shape *t = hull(tetrahedron, 4, 0.0f);
    collide_shape::reset(collector);
    collide(collector, collide_shape::default_settings(), h, pose({0, 0, 0}), t, pose({0.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1 && collector.hits[0].penetration > 0.0f);

    collide_shape::release(collector);
    drop(h);
    drop(rounded);
    drop(other);
    drop(rounded_a);
    drop(rounded_b);
    drop(t);
    box::destroy(b);
}

void test_convex_hull_cast(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *h = make({.positions = CUBE_POINTS, .num_positions = 8});
    Shape *s = sphere::create({.radius = 0.5f});
    Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    // a sphere down from y = 5 by 10 meets the +y face with its centre at 1.5: fraction 3.5 / 10
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    shapes::cast_fn(shapes::SHAPE_SPHERE, shapes::SHAPE_CONVEX_HULL)(collector, cast_shape::default_settings(), s, NO_SUB_SHAPE,
                                                                     pose({0.2f, 5, -0.3f}), vec3f::make(0, -10, 0), h, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit);
    CHECK(ctx, collector.hits[0].status == cast_shape::COLLIDING);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.35f, 1e-4f);
    CHECK(ctx, near3(collector.hits[0].point_b, 0.2f, 1, -0.3f, 1e-3f));

    // the hull swept sideways into a box: its +x face at 1 meets the box's -x face at 4.5, fraction 3.5 / 7
    cast_shape::reset(collector);
    shapes::cast_fn(shapes::SHAPE_CONVEX_HULL, shapes::SHAPE_BOX)(collector, cast_shape::default_settings(), h, NO_SUB_SHAPE, pose({0, 0, 0}),
                                                                  vec3f::make(7, 0, 0), b, NO_SUB_SHAPE, pose({5, 0.2f, 0}));
    CHECK(ctx, collector.has_hit);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.5f, 1e-4f);
    CHECK_NEAR(ctx, vec3f::x(collector.hits[0].point_b), 4.5f, 1e-3f);

    // a miss: the box passes above
    cast_shape::reset(collector);
    shapes::cast_fn(shapes::SHAPE_CONVEX_HULL, shapes::SHAPE_BOX)(collector, cast_shape::default_settings(), h, NO_SUB_SHAPE, pose({0, 0, 0}),
                                                                  vec3f::make(7, 0, 0), b, NO_SUB_SHAPE, pose({5, 3, 0}));
    CHECK(ctx, !collector.has_hit);
    cast_shape::release(collector);

    // a ray down onto the top face from y = 5, length 10: fraction 4 / 10
    cast_ray::CastRayCollector rays = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    const cast_ray::Rayf ray = {vec3f::make(0.3f, 5, 0.1f), vec3f::make(0, -1, 0), 10.0f};
    shapes::def_of(h).cast_ray(rays, cast_ray::default_settings(), ray, h, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, rays.hits.length == 1);
    CHECK_NEAR(ctx, rays.hits[0].fraction, 0.4f, 1e-4f);
    cast_ray::release(rays);

    // point inside and outside
    collide_point::CollidePointCollector points = collide_point::create_collector(collector::COLLECT_ALL);
    shapes::def_of(h).collide_point(points, collide_point::default_settings(), vec3f::make(0.5f, -0.5f, 0.9f), h, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, points.hits.length == 1);
    shapes::def_of(h).collide_point(points, collide_point::default_settings(), vec3f::make(0.5f, -1.1f, 0.0f), h, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, points.hits.length == 1);
    collide_point::release(points);

    drop(h);
    sphere::destroy(s);
    box::destroy(b);
}

void test_convex_hull_shared_data(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();

    // one built hull under two shapes, as box3d: the shape settings are per shape, the data is not
    util::HeapStats before, after;
    util::heap_stats(&before);
    convex_hull::ConvexHullData *data = convex_hull::build({.positions = CUBE_POINTS, .num_positions = 8});
    CHECK(ctx, data != nullptr && data->num_points == 8);
    Shape *light = convex_hull::create({.data = data, .density = 1.0f, .material_id = 1});
    Shape *heavy = convex_hull::create({.data = data, .density = 2.0f, .material_id = 2});
    CHECK(ctx, light->convex_hull.data == data && heavy->convex_hull.data == data);
    CHECK(ctx, light->material_id == 1 && heavy->material_id == 2);
    mass_properties::MassProperties a = mass_properties::create();
    mass_properties::MassProperties b = mass_properties::create();
    shapes::compute_mass_properties(a, light);
    shapes::compute_mass_properties(b, heavy);
    CHECK_NEAR(ctx, a.mass, 8.0f, 1e-4f);
    CHECK_NEAR(ctx, b.mass, 16.0f, 1e-4f);

    // destroying one shape leaves the data and the other shape whole; releasing the data last frees everything
    convex_hull::destroy(light);
    CHECK(ctx, data->num_points == 8 && heavy->convex_hull.data->faces.length == 6);
    CHECK(ctx, convex_hull::get_inner_radius(heavy) == 1.0f);
    convex_hull::destroy(heavy);
    convex_hull::release(data);
    util::heap_stats(&after);
    CHECK(ctx, before.bytes_in_use == after.bytes_in_use && before.allocations == after.allocations);
}

/* material by sub shape id */

void test_convex_hull_material(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // a hull is one leaf: the empty sub shape id names its own material
    Shape *painted = make({.positions = CUBE_POINTS, .num_positions = 8}, nullptr, shapes::DEFAULT_SHAPE_DENSITY, 6);
    Shape *plain = make({.positions = CUBE_POINTS, .num_positions = 8});
    CHECK(ctx, painted != nullptr && plain != nullptr);
    CHECK(ctx, shapes::get_shape_material(painted, sub_shape::EMPTY_SUB_SHAPE_ID) == 6);
    CHECK(ctx, shapes::get_shape_material(plain, sub_shape::EMPTY_SUB_SHAPE_ID) == -1);
    CHECK(ctx, shapes::def_of(painted).get_material == convex::get_convex_material);
    drop(painted);
    drop(plain);
}
