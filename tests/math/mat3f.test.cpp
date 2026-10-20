#include "tumble/math/mat3f.h"
#include "math/storage/quatf.h"
#include "math/storage/vec3f.h"
#include "test.h"

static bool near9(const Mat3f &m, const float *e, float eps = 1e-6f) {
    for (int i = 0; i < 9; i++) {
        if (scalar::abs(m.m[i] - e[i]) > eps) return false;
    }
    return true;
}

void test_mat3f(TestCtx &ctx) {
    const Mat3f I = mat3f::create();
    Mat3f out;

    const float ident[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    CHECK(ctx, near9(I, ident));
    CHECK(ctx, near9(mat3f::identity(out), ident));
    const float zeros[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    CHECK(ctx, near9(mat3f::zero(out), zeros));

    const Mat3f a = mat3f::from_values(1, 2, 3, 4, 5, 6, 7, 8, 10);
    const float at[9] = {1, 4, 7, 2, 5, 8, 3, 6, 10};
    CHECK(ctx, near9(mat3f::transpose(out, a), at));
    // in place
    mat3f::copy(out, a);
    mat3f::transpose(out, out);
    CHECK(ctx, near9(out, at));

    CHECK_NEAR(ctx, mat3f::determinant(a), -3.0f, 1e-5f);
    // inverse times the original is the identity; the singular matrix is refused and out untouched
    Mat3f inv;
    CHECK(ctx, mat3f::invert(inv, a));
    mat3f::multiply(out, a, inv);
    CHECK(ctx, near9(out, ident, 1e-5f));
    mat3f::copy(inv, a);
    CHECK(ctx, !mat3f::invert(inv, mat3f::from_values(1, 2, 3, 4, 5, 6, 7, 8, 9)));
    CHECK(ctx, mat3f::exact_equals(inv, a));
    // adjoint is det times inverse
    Mat3f adj;
    mat3f::adjoint(adj, a);
    mat3f::invert(inv, a);
    mat3f::multiply_scalar(inv, inv, mat3f::determinant(a));
    CHECK(ctx, mat3f::equals(adj, inv) || near9(adj, inv.m, 1e-4f));

    // multiply by the identity is itself; a rotation composed with its inverse rotation is the identity
    CHECK(ctx, near9(mat3f::multiply(out, a, I), a.m));
    Mat3f r;
    Mat3f rinv;
    mat3f::from_rotation(r, 0.7f);
    mat3f::from_rotation(rinv, -0.7f);
    CHECK(ctx, near9(mat3f::multiply(out, r, rinv), ident, 1e-6f));
    // rotate(a, rad) is a * from_rotation(rad)
    Mat3f expect;
    mat3f::multiply(expect, a, r);
    CHECK(ctx, near9(mat3f::rotate(out, a, 0.7f), expect.m, 1e-5f));

    const Vec2f v = {2.0f, 3.0f};
    Mat3f t;
    mat3f::from_translation(t, v);
    mat3f::multiply(expect, a, t);
    CHECK(ctx, near9(mat3f::translate(out, a, v), expect.m, 1e-5f));
    Mat3f s;
    mat3f::from_scaling(s, v);
    mat3f::multiply(expect, a, s);
    CHECK(ctx, near9(mat3f::scale(out, a, v), expect.m, 1e-5f));

    // from_quat of a quarter turn about z is the rotation matrix (column-major)
    Float4 q;
    quatf::set_axis_angle(q, vec3f::from_values(0.0f, 0.0f, 1.0f), scalar::PI * 0.5f);
    const float rz[9] = {0, 1, 0, -1, 0, 0, 0, 0, 1};
    CHECK(ctx, near9(mat3f::from_quat(out, q), rz, 1e-6f));
    // and quatf::from_mat3 inverts it
    Float4 back;
    quatf::from_mat3(back, out);
    CHECK(ctx, quatf::equals(back, q));

    const Float4x4 m4 = {{1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 10, 0, 11, 12, 13, 1}};
    CHECK(ctx, near9(mat3f::from_mat4(out, m4), a.m));
    // normal matrix of a pure rotation is the rotation itself; a scaled matrix gets the inverse scale
    Float4x4 r4 = {{0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 5, 6, 7, 1}};
    CHECK(ctx, mat3f::normal_from_mat4(out, r4));
    CHECK(ctx, near9(out, rz, 1e-6f));
    const Float4x4 s4 = {{2, 0, 0, 0, 0, 4, 0, 0, 0, 0, 8, 0, 0, 0, 0, 1}};
    CHECK(ctx, mat3f::normal_from_mat4(out, s4));
    const float sinv[9] = {0.5f, 0, 0, 0, 0.25f, 0, 0, 0, 0.125f};
    CHECK(ctx, near9(out, sinv, 1e-6f));
    const Float4x4 singular = {{1, 2, 3, 0, 2, 4, 6, 0, 0, 0, 0, 0, 0, 0, 0, 1}};
    CHECK(ctx, !mat3f::normal_from_mat4(out, singular));

    const float proj[9] = {2.0f / 640.0f, 0, 0, 0, -2.0f / 480.0f, 0, -1, 1, 1};
    CHECK(ctx, near9(mat3f::projection(out, 640.0f, 480.0f), proj));

    CHECK_NEAR(ctx, mat3f::frob(I), scalar::sqrt(3.0f), 1e-6f);
    const float sum[9] = {2, 2, 3, 4, 6, 6, 7, 8, 11};
    CHECK(ctx, near9(mat3f::add(out, a, I), sum));
    const float diff[9] = {0, 2, 3, 4, 4, 6, 7, 8, 9};
    CHECK(ctx, near9(mat3f::subtract(out, a, I), diff));
    const float twice[9] = {2, 4, 6, 8, 10, 12, 14, 16, 20};
    CHECK(ctx, near9(mat3f::multiply_scalar(out, a, 2.0f), twice));
    const float plus_half[9] = {1.5f, 2, 3, 4, 5.5f, 6, 7, 8, 10.5f};
    CHECK(ctx, near9(mat3f::multiply_scalar_and_add(out, a, I, 0.5f), plus_half));

    CHECK(ctx, mat3f::exact_equals(a, mat3f::clone(a)));
    CHECK(ctx, !mat3f::exact_equals(a, I));
    Mat3f nearly = a;
    nearly.m[8] += 0.5f * scalar::EPSILON;
    CHECK(ctx, mat3f::equals(a, nearly));
    CHECK(ctx, !mat3f::equals(a, I));
}
