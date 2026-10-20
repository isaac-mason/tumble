#include "math/vec3f.h"
#include "physics/collision/triangle.h"
#include "test.h"
#include "test_math.h"

/** the closest point on triangle abc to p, as the ts calls it: vertices relative to the query point. */
static triangle::ClosestPointOnTriangleResult closest_from(const Float3 &a, const Float3 &b, const Float3 &c, const Float3 &p) {
    const Vec3f ra = vec3f::subtract(vec3f::load(a), vec3f::load(p));
    const Vec3f rb = vec3f::subtract(vec3f::load(b), vec3f::load(p));
    const Vec3f rc = vec3f::subtract(vec3f::load(c), vec3f::load(p));
    triangle::ClosestPointOnTriangleResult r = triangle::create_closest_point_on_triangle_result();
    triangle::get_closest_point_on_triangle(r, ra, rb, rc);
    return r;
}


void test_triangle_closest_point_regions(TestCtx &ctx) {
    // right triangle in the xz plane, legs of 2 along +x and +z
    const Float3 a = {0.0f, 0.0f, 0.0f};
    const Float3 b = {2.0f, 0.0f, 0.0f};
    const Float3 c = {0.0f, 0.0f, 2.0f};

    // above the interior: the foot of the perpendicular, one unit down
    triangle::ClosestPointOnTriangleResult r = closest_from(a, b, c, {0.5f, 1.0f, 0.5f});
    CHECK(ctx, r.feature == triangle::FACE);
    CHECK(ctx, near3(r.point, 0.0f, -1.0f, 0.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 1.0f, 1e-5f);

    // below the interior: the face too, the point now above
    r = closest_from(a, b, c, {0.25f, -3.0f, 1.0f});
    CHECK(ctx, r.feature == triangle::FACE);
    CHECK(ctx, near3(r.point, 0.0f, 3.0f, 0.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 9.0f, 1e-4f);

    // beyond edge ab (z < 0): the projection onto the edge
    r = closest_from(a, b, c, {1.0f, 1.0f, -1.0f});
    CHECK(ctx, r.feature == triangle::EDGE_AB);
    CHECK(ctx, near3(r.point, 0.0f, -1.0f, 1.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 2.0f, 1e-5f);

    // beyond edge ac (x < 0)
    r = closest_from(a, b, c, {-1.0f, 0.0f, 1.5f});
    CHECK(ctx, r.feature == triangle::EDGE_AC);
    CHECK(ctx, near3(r.point, 1.0f, 0.0f, 0.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 1.0f, 1e-5f);

    // beyond the hypotenuse bc (x + z > 2): lands at its midpoint (1, 0, 1)
    r = closest_from(a, b, c, {2.0f, 0.5f, 2.0f});
    CHECK(ctx, r.feature == triangle::EDGE_BC);
    CHECK(ctx, near3(r.point, -1.0f, -0.5f, -1.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 2.25f, 1e-5f);

    // beyond each vertex: the vertex itself
    r = closest_from(a, b, c, {-1.0f, 1.0f, -1.0f});
    CHECK(ctx, r.feature == triangle::VERTEX_A);
    CHECK(ctx, near3(r.point, 1.0f, -1.0f, 1.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 3.0f, 1e-5f);

    r = closest_from(a, b, c, {3.0f, 0.0f, -1.0f});
    CHECK(ctx, r.feature == triangle::VERTEX_B);
    CHECK(ctx, near3(r.point, -1.0f, 0.0f, 1.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 2.0f, 1e-5f);

    r = closest_from(a, b, c, {-1.0f, 2.0f, 3.0f});
    CHECK(ctx, r.feature == triangle::VERTEX_C);
    CHECK(ctx, near3(r.point, 1.0f, -2.0f, -1.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 6.0f, 1e-5f);

    // winding does not matter: acb gives the same point, with the edge bits following the vertex names
    r = closest_from(a, c, b, {1.0f, 1.0f, -1.0f});
    CHECK(ctx, r.feature == triangle::EDGE_AC);
    CHECK(ctx, near3(r.point, 0.0f, -1.0f, 1.0f, 1e-5f));
}

void test_triangle_closest_point_degenerate(TestCtx &ctx) {
    // collinear along x: no area, so the vertices and edges are scanned
    const Float3 a = {0.0f, 0.0f, 0.0f};
    const Float3 b = {1.0f, 0.0f, 0.0f};
    const Float3 c = {2.0f, 0.0f, 0.0f};

    // beside the segment: ac (scanned before bc and ab) holds the foot and wins the tie
    triangle::ClosestPointOnTriangleResult r = closest_from(a, b, c, {1.5f, 1.0f, 0.0f});
    CHECK(ctx, r.feature == triangle::EDGE_AC);
    CHECK(ctx, near3(r.point, 0.0f, -1.0f, 0.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 1.0f, 1e-5f);

    // past c: the vertex
    r = closest_from(a, b, c, {3.0f, 0.0f, 0.0f});
    CHECK(ctx, r.feature == triangle::VERTEX_C);
    CHECK(ctx, near3(r.point, -1.0f, 0.0f, 0.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 1.0f, 1e-5f);

    // all three at one point: that point, as vertex c
    const Float3 p = {1.0f, 2.0f, 3.0f};
    r = closest_from(p, p, p, {1.0f, 2.0f, 5.0f});
    CHECK(ctx, r.feature == triangle::VERTEX_C);
    CHECK(ctx, near3(r.point, 0.0f, 0.0f, -2.0f, 1e-5f));
    CHECK_NEAR(ctx, r.distance_sq, 4.0f, 1e-5f);
}
