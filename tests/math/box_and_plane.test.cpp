#include "math/storage/box3f.h"
#include "math/storage/plane3f.h"
#include "math/storage/quatf.h"
#include "math/storage/triangle3f.h"
#include "test.h"
#include "test_math.h"

static bool near_box(const Box3f &b, float x0, float y0, float z0, float x1, float y1, float z1, float eps = 1e-5f) {
    return near3(b.min, x0, y0, z0, eps) && near3(b.max, x1, y1, z1, eps);
}

void test_box3f(TestCtx &ctx) {
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

    // a negative scale keeps min below max
    CHECK(ctx, near_box(box3f::scale(out, box, vec3f::from_values(-2, 1, 0.5f)), -2, 2, 0, 2, 5, 1.5f));

    // transforming the unit cube by an eighth turn about z, a translation and a scale
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

    Box3f tb;
    triangle3f::bounds(tb, vec3f::from_values(1, 5, -1), vec3f::from_values(-2, 0, 4), vec3f::from_values(0, 1, 2));
    CHECK(ctx, near_box(tb, -2, 0, -1, 1, 5, 4));
}

void test_plane3f(TestCtx &ctx) {
    // the y = 3 plane
    const Plane3f p = {{0, 1, 0}, -3.0f};
    CHECK_NEAR(ctx, plane3f::distance_to_point(p, vec3f::from_values(5, 5, 5)), 2.0f, 1e-6f);
    Float3 v;
    plane3f::project_point(v, p, vec3f::from_values(5, 5, 5));
    CHECK(ctx, near3(v, 5, 3, 5, 1e-5f));
}
