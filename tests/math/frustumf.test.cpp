#include "math/storage/box3f.h"
#include "math/storage/frustumf.h"
#include "test.h"
#include "test_math.h"

// the projections the renderer builds, written out here since mat4f leaves them to it
static Float4x4 perspective_no(float fovy, float aspect, float near, float far) {
    const float f = 1.0f / scalar::tan(fovy / 2.0f);
    const float nf = 1.0f / (near - far);
    return mat4f::from_values(f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2.0f * far * near * nf, 0);
}

static Float4x4 perspective_zo(float fovy, float aspect, float near, float far) {
    const float f = 1.0f / scalar::tan(fovy / 2.0f);
    const float nf = 1.0f / (near - far);
    return mat4f::from_values(f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, far * nf, -1, 0, 0, far * near * nf, 0);
}

void test_frustumf(TestCtx &ctx) {
    // a 90 degree square frustum from the origin looking down -z, near 1 far 10
    const Float4x4 proj = perspective_no(scalar::PI * 0.5f, 1.0f, 1.0f, 10.0f);
    const Float4x4 view = mat4f::create();
    Frustumf f;
    frustumf::set_from_view_projection_matrix_no(f, proj, view);

    // every plane normal is unit and faces inward: the point on the axis halfway in is in front of all six
    const Float3 inside = vec3f::from_values(0, 0, -5);
    for (int i = 0; i < 6; i++) {
        CHECK_NEAR(ctx, vec3f::length(f.planes[i].normal), 1.0f, 1e-5f);
        CHECK(ctx, plane3f::distance_to_point(f.planes[i], inside) > 0.0f);
    }
    // near plane at z = -1, far at z = -10, sides at 45 degrees
    CHECK_NEAR(ctx, plane3f::distance_to_point(f.planes[4], vec3f::from_values(0, 0, -1)), 0.0f, 1e-5f);
    CHECK_NEAR(ctx, plane3f::distance_to_point(f.planes[5], vec3f::from_values(0, 0, -10)), 0.0f, 1e-4f);
    CHECK_NEAR(ctx, plane3f::distance_to_point(f.planes[0], vec3f::from_values(-5, 0, -5)), 0.0f, 1e-4f);
    CHECK_NEAR(ctx, plane3f::distance_to_point(f.planes[1], vec3f::from_values(5, 0, -5)), 0.0f, 1e-4f);
    CHECK_NEAR(ctx, plane3f::distance_to_point(f.planes[2], vec3f::from_values(0, -5, -5)), 0.0f, 1e-4f);
    CHECK_NEAR(ctx, plane3f::distance_to_point(f.planes[3], vec3f::from_values(0, 5, -5)), 0.0f, 1e-4f);

    CHECK(ctx, frustumf::contains_point(f, inside));
    CHECK(ctx, !frustumf::contains_point(f, vec3f::from_values(0, 0, 5)));     // behind
    CHECK(ctx, !frustumf::contains_point(f, vec3f::from_values(0, 0, -0.5f)));  // before near
    CHECK(ctx, !frustumf::contains_point(f, vec3f::from_values(0, 0, -11)));    // past far
    CHECK(ctx, !frustumf::contains_point(f, vec3f::from_values(6, 0, -5)));     // right of the side
    CHECK(ctx, frustumf::sides_contains_point(f, vec3f::from_values(0, 0, -11)));
    CHECK(ctx, !frustumf::sides_contains_point(f, vec3f::from_values(6, 0, -5)));

    CHECK(ctx, frustumf::intersects_sphere(f, Spheref{{0, 0, -5}, 1.0f}));
    CHECK(ctx, frustumf::intersects_sphere(f, Spheref{{0, 0, -11}, 2.0f}));   // straddles far
    CHECK(ctx, !frustumf::intersects_sphere(f, Spheref{{0, 0, -13}, 2.0f}));
    CHECK(ctx, frustumf::sides_intersects_sphere(f, Spheref{{0, 0, -13}, 2.0f}));
    CHECK(ctx, !frustumf::intersects_sphere(f, Spheref{{8, 0, -5}, 1.0f}));

    Box3f box;
    CHECK(ctx, frustumf::intersects_box3(f, box3f::set(box, -1, -1, -6, 1, 1, -4)));
    CHECK(ctx, frustumf::intersects_box3(f, box3f::set(box, 4, 4, -6, 8, 8, -4)));    // corner pokes in
    CHECK(ctx, !frustumf::intersects_box3(f, box3f::set(box, 7, -1, -6, 9, 1, -4)));  // beside
    CHECK(ctx, !frustumf::intersects_box3(f, box3f::set(box, -1, -1, 1, 1, 1, 3)));   // behind the eye
    CHECK(ctx, !frustumf::intersects_box3(f, box3f::set(box, -1, -1, -20, 1, 1, -12)));
    CHECK(ctx, frustumf::sides_intersects_box3(f, box3f::set(box, -1, -1, -20, 1, 1, -12)));

    // rays: down the axis from the eye enters; away from it never does; from inside, any direction
    CHECK(ctx, frustumf::intersects_ray(f, vec3f::create(), vec3f::from_values(0, 0, -1)));
    CHECK(ctx, !frustumf::intersects_ray(f, vec3f::create(), vec3f::from_values(0, 0, 1)));
    CHECK(ctx, frustumf::intersects_ray(f, inside, vec3f::from_values(1, 0, 0)));
    CHECK(ctx, !frustumf::intersects_ray(f, vec3f::from_values(20, 0, -5), vec3f::from_values(0, 0, -1)));  // parallel, outside
    CHECK(ctx, frustumf::intersects_ray(f, vec3f::from_values(20, 0, -5), vec3f::from_values(-1, 0, 0)));   // comes in from the side
    // the sides meet at the eye, so behind it is outside them too; a ray through the apex gets in, one away never does
    CHECK(ctx, !frustumf::sides_intersects_ray(f, vec3f::from_values(0, 0, 5), vec3f::from_values(0, 0, 1)));
    CHECK(ctx, frustumf::sides_intersects_ray(f, vec3f::from_values(0, 0, 5), vec3f::from_values(0, 0, -1)));
    CHECK(ctx, !frustumf::intersects_ray(f, vec3f::from_values(0, 0, 5), vec3f::from_values(0, 0, 1)));

    // corners: near at z = -1 with half-size 1, far at z = -10 with half-size 10
    Float3 c[8];
    frustumf::corners(c, f);
    CHECK(ctx, near3(c[0], -1, -1, -1, 1e-4f));
    CHECK(ctx, near3(c[1], -1, 1, -1, 1e-4f));
    CHECK(ctx, near3(c[2], 1, 1, -1, 1e-4f));
    CHECK(ctx, near3(c[3], 1, -1, -1, 1e-4f));
    CHECK(ctx, near3(c[4], -10, -10, -10, 1e-3f));
    CHECK(ctx, near3(c[6], 10, 10, -10, 1e-3f));

    // the zo projection gives the same frustum
    Frustumf fz;
    frustumf::set_from_view_projection_matrix_zo(fz, perspective_zo(scalar::PI * 0.5f, 1.0f, 1.0f, 10.0f), view);
    for (int i = 0; i < 6; i++) CHECK(ctx, plane3f::equals(fz.planes[i], f.planes[i]) || near3(fz.planes[i].normal, f.planes[i].normal.x, f.planes[i].normal.y, f.planes[i].normal.z, 1e-5f));
    CHECK_NEAR(ctx, plane3f::distance_to_point(fz.planes[4], vec3f::from_values(0, 0, -1)), 0.0f, 1e-5f);
    CHECK_NEAR(ctx, plane3f::distance_to_point(fz.planes[5], vec3f::from_values(0, 0, -10)), 0.0f, 1e-4f);

    // sides only leaves near and far untouched
    Frustumf fs = frustumf::create();
    frustumf::set_from_view_projection_matrix_sides(fs, proj, view);
    CHECK(ctx, plane3f::equals(fs.planes[0], f.planes[0]) && plane3f::equals(fs.planes[3], f.planes[3]));
    CHECK(ctx, fs.planes[4].normal.y == 1.0f && fs.planes[4].constant == 0.0f);

    // a moved camera: looking from (0, 0, 5) at the origin puts the near plane at z = 4
    Float4x4 v2;
    mat4f::look_at(v2, vec3f::from_values(0, 0, 5), vec3f::create(), vec3f::from_values(0, 1, 0));
    frustumf::set_from_view_projection_matrix_no(f, proj, v2);
    CHECK_NEAR(ctx, plane3f::distance_to_point(f.planes[4], vec3f::from_values(0, 0, 4)), 0.0f, 1e-5f);
    CHECK(ctx, frustumf::contains_point(f, vec3f::create()));
    CHECK(ctx, !frustumf::contains_point(f, vec3f::from_values(0, 0, 6)));
}
