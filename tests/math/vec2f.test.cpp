#include "tumble/math/mat2f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec2f.h"
#include "test.h"

static bool near2(const Vec2f &a, float x, float y, float eps = 1e-6f) {
    return scalar::abs(a.x - x) <= eps && scalar::abs(a.y - y) <= eps;
}

static bool near4(const Mat2f &m, float a, float b, float c, float d, float eps = 1e-6f) {
    return scalar::abs(m.m[0] - a) <= eps && scalar::abs(m.m[1] - b) <= eps && scalar::abs(m.m[2] - c) <= eps &&
           scalar::abs(m.m[3] - d) <= eps;
}

void test_vec2f(TestCtx &ctx) {
    Vec2f a;
    vec2f::set(a, 3.0f, 4.0f);
    Vec2f out;
    CHECK(ctx, near2(vec2f::scale(out, a, 2.0f), 6.0f, 8.0f));
    CHECK(ctx, near2(vec2f::copy(out, a), 3.0f, 4.0f));
    CHECK(ctx, near2(vec2f::zero(out), 0.0f, 0.0f));

    // a quarter turn takes x to y
    Mat2f quarter;
    mat2f::set(quarter, 0.0f, 1.0f, -1.0f, 0.0f);
    Vec2f x_axis;
    vec2f::set(x_axis, 1.0f, 0.0f);
    CHECK(ctx, near2(vec2f::transform_mat2(out, x_axis, quarter), 0.0f, 1.0f));
}

void test_mat2f_invert(TestCtx &ctx) {
    Mat2f m;
    mat2f::set(m, 1, 2, 3, 4);
    Mat2f inv;
    CHECK(ctx, mat2f::invert(inv, m));
    CHECK(ctx, near4(inv, -2.0f, 1.0f, 1.5f, -0.5f));

    // a singular matrix leaves out untouched
    Mat2f singular;
    mat2f::set(singular, 1, 2, 2, 4);
    CHECK(ctx, !mat2f::invert(inv, singular));
    CHECK(ctx, near4(inv, -2.0f, 1.0f, 1.5f, -0.5f));
}
