#include "math/mat2f.h"
#include "math/vec2f.h"
#include "test.h"

static bool near2(const Vec2f &a, float x, float y, float eps = 1e-6f) {
    return scalar::abs(a.x - x) <= eps && scalar::abs(a.y - y) <= eps;
}

static bool near4(const Mat2f &m, float a, float b, float c, float d, float eps = 1e-6f) {
    return scalar::abs(m.m[0] - a) <= eps && scalar::abs(m.m[1] - b) <= eps && scalar::abs(m.m[2] - c) <= eps &&
           scalar::abs(m.m[3] - d) <= eps;
}

void test_vec2f(TestCtx &ctx) {
    const Vec2f a = vec2f::from_values(3.0f, 4.0f);
    const Vec2f b = vec2f::from_values(-1.0f, 2.0f);
    Vec2f out;

    CHECK(ctx, near2(vec2f::add(out, a, b), 2.0f, 6.0f));
    CHECK(ctx, near2(vec2f::subtract(out, a, b), 4.0f, 2.0f));
    CHECK(ctx, near2(vec2f::multiply(out, a, b), -3.0f, 8.0f));
    CHECK(ctx, near2(vec2f::scale(out, a, 2.0f), 6.0f, 8.0f));
    CHECK(ctx, near2(vec2f::scale_and_add(out, a, b, 2.0f), 1.0f, 8.0f));
    CHECK(ctx, near2(vec2f::negate(out, a), -3.0f, -4.0f));
    CHECK(ctx, vec2f::dot(a, b) == 5.0f);
    CHECK(ctx, vec2f::length(a) == 5.0f);
    CHECK(ctx, vec2f::squared_length(a) == 25.0f);
    CHECK_NEAR(ctx, vec2f::distance(a, vec2f::from_values(0.0f, 0.0f)), 5.0f, 1e-6f);
    CHECK(ctx, near2(vec2f::normalize(out, a), 0.6f, 0.8f));
    CHECK(ctx, near2(vec2f::normalize(out, vec2f::create()), 0.0f, 0.0f));
    Float3 z;
    vec2f::cross(z, a, b);
    CHECK(ctx, z.x == 0.0f && z.y == 0.0f && z.z == 10.0f);
    CHECK(ctx, near2(vec2f::lerp(out, a, b, 0.5f), 1.0f, 3.0f));
    CHECK(ctx, near2(vec2f::rotate(out, vec2f::from_values(1.0f, 0.0f), vec2f::create(), scalar::PI * 0.5f), 0.0f, 1.0f));
    CHECK(ctx, near2(vec2f::rotate(out, vec2f::from_values(2.0f, 1.0f), vec2f::from_values(1.0f, 1.0f), scalar::PI), 0.0f, 1.0f));
    CHECK_NEAR(ctx, vec2f::angle(vec2f::from_values(1.0f, 0.0f), vec2f::from_values(0.0f, 1.0f)), scalar::PI * 0.5f, 1e-6f);
    CHECK_NEAR(ctx, vec2f::angle(a, vec2f::create()), scalar::PI * 0.5f, 1e-6f);
    CHECK(ctx, vec2f::exact_equals(a, vec2f::clone(a)) && !vec2f::exact_equals(a, b));
    CHECK(ctx, vec2f::equals(a, vec2f::from_values(3.0f, 4.0f + 0.5f * scalar::EPSILON)));
    CHECK(ctx, near2(vec2f::zero(out), 0.0f, 0.0f));

    // a quarter turn as a mat2 takes x to y
    Mat2f r;
    mat2f::from_rotation(r, scalar::PI * 0.5f);
    CHECK(ctx, near2(vec2f::transform_mat2(out, vec2f::from_values(1.0f, 0.0f), r), 0.0f, 1.0f));

    const Mat2f m = mat2f::from_values(1, 2, 3, 4);
    Mat2f mo;
    CHECK(ctx, near4(mat2f::create(), 1, 0, 0, 1));
    CHECK(ctx, near4(mat2f::transpose(mo, m), 1, 3, 2, 4));
    mat2f::copy(mo, m);
    mat2f::transpose(mo, mo);
    CHECK(ctx, near4(mo, 1, 3, 2, 4));
    CHECK(ctx, mat2f::determinant(m) == -2.0f);
    Mat2f inv;
    CHECK(ctx, mat2f::invert(inv, m));
    mat2f::multiply(mo, m, inv);
    CHECK(ctx, near4(mo, 1, 0, 0, 1));
    CHECK(ctx, !mat2f::invert(inv, mat2f::from_values(1, 2, 2, 4)));
    mat2f::adjoint(mo, m);
    CHECK(ctx, near4(mo, 4, -2, -3, 1));
    mat2f::multiply(mo, m, mat2f::create());
    CHECK(ctx, mat2f::exact_equals(mo, m));
    Mat2f expect;
    mat2f::multiply(expect, m, r);
    CHECK(ctx, near4(mat2f::rotate(mo, m, scalar::PI * 0.5f), expect.m[0], expect.m[1], expect.m[2], expect.m[3]));
    Mat2f s;
    mat2f::from_scaling(s, vec2f::from_values(2.0f, 3.0f));
    mat2f::multiply(expect, m, s);
    CHECK(ctx, near4(mat2f::scale(mo, m, vec2f::from_values(2.0f, 3.0f)), expect.m[0], expect.m[1], expect.m[2], expect.m[3]));
    CHECK_NEAR(ctx, mat2f::frob(mat2f::create()), scalar::sqrt(2.0f), 1e-6f);
    // ldu factors the entries read row-major (as gl-matrix does): l = a[2] / a[0]
    Mat2f L, D, U;
    mat2f::ldu(L, D, U, m);
    CHECK(ctx, near4(L, 1, 0, 3, 1));
    CHECK(ctx, near4(D, 1, 0, 0, 1));
    CHECK(ctx, near4(U, 1, 2, 0, -2));
    CHECK(ctx, near4(mat2f::add(mo, m, mat2f::create()), 2, 2, 3, 5));
    CHECK(ctx, near4(mat2f::subtract(mo, m, mat2f::create()), 0, 2, 3, 3));
    CHECK(ctx, near4(mat2f::multiply_scalar(mo, m, 2.0f), 2, 4, 6, 8));
    CHECK(ctx, near4(mat2f::multiply_scalar_and_add(mo, m, mat2f::create(), 0.5f), 1.5f, 2, 3, 4.5f));
    CHECK(ctx, mat2f::exact_equals(m, mat2f::clone(m)) && !mat2f::exact_equals(m, r));
    Mat2f nearly = m;
    nearly.m[3] += 0.5f * scalar::EPSILON;
    CHECK(ctx, mat2f::equals(m, nearly));
}
