#include "math/storage/box3f.h"
#include "math/storage/plane3f.h"
#include "math/storage/quatf.h"
#include "math/spheref.h"
#include "math/storage/triangle3f.h"
#include "test.h"
#include "test_math.h"

static bool near_box(const Box3f &b, float x0, float y0, float z0, float x1, float y1, float z1, float eps = 1e-5f) {
    return near3(b.min, x0, y0, z0, eps) && near3(b.max, x1, y1, z1, eps);
}

void test_shapes(TestCtx &ctx) {
    Box3f box;
    Box3f out;
    Float3 v;

    // an empty box is inverted infinities; expanding by a point makes it that point; union and margin
    CHECK(ctx, box3f::is_empty(box3f::create()));
    CHECK(ctx, box3f::is_empty(box3f::empty(box)));
    box3f::expand_by_point(box, box, vec3f::from_values(1.0f, 2.0f, 3.0f));
    CHECK(ctx, near_box(box, 1, 2, 3, 1, 2, 3));
    CHECK(ctx, !box3f::is_empty(box));
    box3f::expand_by_point(box, box, vec3f::from_values(-1.0f, 5.0f, 0.0f));
    CHECK(ctx, near_box(box, -1, 2, 0, 1, 5, 3));
    box3f::union_(out, box, box3f::set(out, 0, 0, 0, 2, 2, 10));
    CHECK(ctx, near_box(out, -1, 0, 0, 2, 5, 10));
    CHECK(ctx, near_box(box3f::expand_by_margin(out, box, 1.0f), -2, 1, -1, 2, 6, 4));
    CHECK(ctx, near_box(box3f::expand_by_extents(out, box, vec3f::from_values(1, 0, 2)), -2, 2, -2, 2, 5, 5));
    box3f::center(v, box);
    CHECK(ctx, near3(v, 0, 3.5f, 1.5f, 1e-5f));
    box3f::extents(v, box);
    CHECK(ctx, near3(v, 1, 1.5f, 1.5f, 1e-5f));
    box3f::size(v, box);
    CHECK(ctx, near3(v, 2, 3, 3, 1e-5f));
    CHECK(ctx, box3f::surface_area(box) == 2.0f * (2 * 3 + 2 * 3 + 3 * 3));
    box3f::set_from_center_and_size(out, vec3f::from_values(1, 1, 1), vec3f::from_values(2, 4, 6));
    CHECK(ctx, near_box(out, 0, -1, -2, 2, 3, 4));
    box3f::min(v, box);
    CHECK(ctx, near3(v, -1, 2, 0, 1e-5f));
    box3f::max(v, box);
    CHECK(ctx, near3(v, 1, 5, 3, 1e-5f));

    // a negative scale keeps min below max
    CHECK(ctx, near_box(box3f::scale(out, box, vec3f::from_values(-2, 1, 0.5f)), -2, 2, 0, 2, 5, 1.5f));

    // transforming the unit cube by a quarter turn about z plus a translation
    Box3f unit;
    box3f::set(unit, -1, -1, -1, 1, 1, 1);
    Float4x4 m;
    Float4 q;
    quatf::set_axis_angle(q, vec3f::from_values(0, 0, 1), scalar::PI * 0.25f);
    mat4f::from_rotation_translation_scale(m, q, vec3f::from_values(10, 0, 0), vec3f::from_values(1, 1, 2));
    box3f::transform_mat4(out, unit, m);
    const float r = 1.41421356f;
    CHECK(ctx, near_box(out, 10 - r, -r, -2, 10 + r, r, 2, 1e-5f));
    // an empty box stays empty rather than becoming a point
    CHECK(ctx, box3f::is_empty(box3f::transform_mat4(out, box3f::create(), m)));

    CHECK(ctx, box3f::contains_point(unit, vec3f::from_values(1, 0, -1)));
    CHECK(ctx, !box3f::contains_point(unit, vec3f::from_values(1.1f, 0, 0)));
    CHECK(ctx, box3f::contains_box3(unit, box3f::set(out, -0.5f, -1, 0, 1, 1, 1)));
    CHECK(ctx, !box3f::contains_box3(unit, box3f::set(out, -0.5f, -1, 0, 1, 1, 1.5f)));
    // closed overlap: touching faces intersect; a gap does not
    CHECK(ctx, box3f::intersects_box3(unit, box3f::set(out, 1, -1, -1, 2, 1, 1)));
    CHECK(ctx, !box3f::intersects_box3(unit, box3f::set(out, 1.001f, -1, -1, 2, 1, 1)));
    CHECK(ctx, !box3f::intersects_box3(unit, box3f::set(out, -1, 1.5f, -1, 1, 2, 1)));
    CHECK(ctx, box3f::intersects_box3(unit, box3f::set(out, -0.1f, -0.1f, -0.1f, 0.1f, 0.1f, 0.1f)));

    // triangle: one through the cube, one far outside, one whose bounds overlap but whose plane misses
    CHECK(ctx, box3f::intersects_triangle3(unit, vec3f::from_values(-2, 0, 0), vec3f::from_values(2, 0, 0), vec3f::from_values(0, 2, 0)));
    CHECK(ctx, !box3f::intersects_triangle3(unit, vec3f::from_values(5, 0, 0), vec3f::from_values(6, 0, 0), vec3f::from_values(5, 1, 0)));
    // bounds overlap the cube but the plane x + y + z = 3.5 clears its (1, 1, 1) corner
    CHECK(ctx, !box3f::intersects_triangle3(unit, vec3f::from_values(3.5f, 0, 0), vec3f::from_values(0, 3.5f, 0), vec3f::from_values(0, 0, 3.5f)));
    // and the same triangle pulled in to x + y + z = 2.5 cuts the corner off
    CHECK(ctx, box3f::intersects_triangle3(unit, vec3f::from_values(2.5f, 0, 0), vec3f::from_values(0, 2.5f, 0), vec3f::from_values(0, 0, 2.5f)));
    CHECK(ctx, !box3f::intersects_triangle3(box3f::create(), vec3f::create(), vec3f::create(), vec3f::create()));

    const Spheref s = {{3.0f, 0.0f, 0.0f}, 2.0f};
    CHECK(ctx, box3f::intersects_sphere(unit, s));
    CHECK(ctx, !box3f::intersects_sphere(unit, Spheref{{3.0f, 0.0f, 0.0f}, 1.9f}));
    CHECK(ctx, box3f::intersects_sphere(unit, Spheref{{0.0f, 0.0f, 0.0f}, 0.1f}));

    Plane3f p;
    plane3f::from_normal_and_constant(p, vec3f::from_values(1, 0, 0), -0.5f);  // x = 0.5
    CHECK(ctx, box3f::intersects_plane3(unit, p));
    plane3f::from_normal_and_constant(p, vec3f::from_values(1, 0, 0), -1.5f);  // x = 1.5
    CHECK(ctx, !box3f::intersects_plane3(unit, p));

    CHECK(ctx, box3f::exact_equals(unit, box3f::clone(unit)));
    Box3f nearly = unit;
    nearly.max.z += 0.5f * scalar::EPSILON;
    CHECK(ctx, box3f::equals(unit, nearly));
    CHECK(ctx, !box3f::equals(unit, box));

    // planes
    plane3f::from_normal_and_point(p, vec3f::from_values(0, 1, 0), vec3f::from_values(0, 3, 0));
    CHECK(ctx, p.constant == -3.0f);
    CHECK_NEAR(ctx, plane3f::distance_to_point(p, vec3f::from_values(5, 5, 5)), 2.0f, 1e-6f);
    plane3f::project_point(v, p, vec3f::from_values(5, 5, 5));
    CHECK(ctx, near3(v, 5, 3, 5, 1e-5f));
    // three points in the z = 2 plane, counter-clockwise seen from +z, give normal +z
    plane3f::from_coplanar_points(p, vec3f::from_values(0, 0, 2), vec3f::from_values(1, 0, 2), vec3f::from_values(0, 1, 2));
    CHECK(ctx, near3(p.normal, 0, 0, 1, 1e-5f));
    CHECK_NEAR(ctx, p.constant, -2.0f, 1e-6f);
    Plane3f scaled = {{0, 0, 2}, -4};
    plane3f::normalize(scaled, scaled);
    CHECK(ctx, plane3f::equals(scaled, p));
    plane3f::negate(out.min.x == 0 ? scaled : scaled, p);
    CHECK(ctx, near3(scaled.normal, 0, 0, -1, 1e-5f) && scaled.constant == 2.0f);
    plane3f::offset(scaled, p, 1.0f);
    CHECK_NEAR(ctx, plane3f::distance_to_point(scaled, vec3f::from_values(0, 0, 3)), 0.0f, 1e-6f);
    // transform: translating the z = 2 plane up by 5 gives z = 7; rotating it a quarter turn about x gives y = -2
    Float4x4 t;
    mat4f::from_translation(t, vec3f::from_values(0, 0, 5));
    plane3f::transform(scaled, p, t);
    CHECK(ctx, near3(scaled.normal, 0, 0, 1, 1e-5f) && scalar::abs(scaled.constant + 7.0f) < 1e-5f);
    mat4f::from_x_rotation(t, scalar::PI * 0.5f);
    plane3f::transform(scaled, p, t);
    CHECK(ctx, near3(scaled.normal, 0, -1, 0, 1e-6f));
    CHECK_NEAR(ctx, plane3f::distance_to_point(scaled, vec3f::from_values(0, -2, 0)), 0.0f, 1e-5f);
    CHECK(ctx, plane3f::intersects_sphere(p, Spheref{{0, 0, 3}, 1.0f}));
    CHECK(ctx, !plane3f::intersects_sphere(p, Spheref{{0, 0, 3.5f}, 1.0f}));
    // three axis planes meet at (1, 2, 3); two parallel planes have no point
    Plane3f px, py, pz;
    plane3f::from_normal_and_constant(px, vec3f::from_values(1, 0, 0), -1);
    plane3f::from_normal_and_constant(py, vec3f::from_values(0, 1, 0), -2);
    plane3f::from_normal_and_constant(pz, vec3f::from_values(0, 0, 1), -3);
    CHECK(ctx, plane3f::intersect(v, px, py, pz));
    CHECK(ctx, near3(v, 1, 2, 3, 1e-5f));
    CHECK(ctx, !plane3f::intersect(v, px, px, pz));
    CHECK(ctx, plane3f::exact_equals(px, plane3f::clone(px)));
    CHECK(ctx, !plane3f::exact_equals(px, py));

    // triangles
    Box3f tb;
    triangle3f::bounds(tb, vec3f::from_values(1, 5, -1), vec3f::from_values(-2, 0, 4), vec3f::from_values(0, 1, 2));
    CHECK(ctx, near_box(tb, -2, 0, -1, 1, 5, 4));
    triangle3f::normal(v, vec3f::from_values(0, 0, 0), vec3f::from_values(1, 0, 0), vec3f::from_values(0, 1, 0));
    CHECK(ctx, near3(v, 0, 0, 1, 1e-5f));
    triangle3f::normal(v, vec3f::from_values(0, 0, 0), vec3f::from_values(1, 0, 0), vec3f::from_values(2, 0, 0));
    CHECK(ctx, near3(v, 0, 0, 0, 1e-5f));
    triangle3f::centroid(v, vec3f::from_values(0, 0, 0), vec3f::from_values(3, 0, 0), vec3f::from_values(0, 3, 3));
    CHECK(ctx, near3(v, 1, 1, 1, 1e-5f));

    // spheres
    const float points[9] = {0, 0, 0, 4, 0, 0, 2, 2, 0};
    Spheref sp;
    spheref::set_from_points(sp, points, 3);
    CHECK(ctx, near3(sp.center, 2, 1, 0, 1e-5f));
    CHECK(ctx, sp.radius >= scalar::sqrt(5.0f));
    for (int i = 0; i < 3; i++) {
        CHECK(ctx, spheref::contains_point(sp, vec3f::from_values(points[i * 3], points[i * 3 + 1], points[i * 3 + 2])));
    }
    CHECK(ctx, !spheref::contains_point(sp, vec3f::from_values(2, 4, 0)));
    spheref::set_from_points(sp, points, 0);
    CHECK(ctx, sp.radius == 0.0f && near3(sp.center, 0, 0, 0, 1e-5f));
    CHECK(ctx, spheref::create().radius == 1.0f);
}
