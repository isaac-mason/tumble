#include "math/simd.h"
#include "test.h"

void test_simd(TestCtx &ctx) {
    // load3 reads exactly three floats: the fourth stays whatever the loader left, and the guard
    // after the struct must not be touched by store3.
    float src[4] = {1.0f, 2.0f, 3.0f, 99.0f};
    float dst[4] = {0.0f, 0.0f, 0.0f, -1.0f};
    const v128_t v = v4f::load3(src);
    CHECK(ctx, v4f::lane<0>(v) == 1.0f);
    CHECK(ctx, v4f::lane<1>(v) == 2.0f);
    CHECK(ctx, v4f::lane<2>(v) == 3.0f);
    CHECK(ctx, v4f::lane<3>(v) == 0.0f);
    v4f::store3(dst, v);
    CHECK(ctx, dst[0] == 1.0f && dst[1] == 2.0f && dst[2] == 3.0f);
    CHECK(ctx, dst[3] == -1.0f);

    CHECK(ctx, v4f::dot3(v, v) == 14.0f);
    CHECK(ctx, v4f::hsum4(v4f::make(1.0f, 2.0f, 3.0f, 4.0f)) == 10.0f);
    CHECK(ctx, v4f::dot4(v4f::make(1.0f, 2.0f, 3.0f, 4.0f), v4f::make(1.0f, 1.0f, 1.0f, 1.0f)) == 10.0f);

    // x cross y = z
    const v128_t c = v4f::cross(v4f::make(1.0f, 0.0f, 0.0f, 0.0f), v4f::make(0.0f, 1.0f, 0.0f, 0.0f));
    CHECK(ctx, v4f::lane<0>(c) == 0.0f && v4f::lane<1>(c) == 0.0f && v4f::lane<2>(c) == 1.0f);

    const v128_t yzx = v4f::yzx(v);
    CHECK(ctx, v4f::lane<0>(yzx) == 2.0f && v4f::lane<1>(yzx) == 3.0f && v4f::lane<2>(yzx) == 1.0f);

    const v128_t mask = v4f::lt(v4f::make(1.0f, 5.0f, 3.0f, 7.0f), v4f::splat(4.0f));
    CHECK(ctx, v4f::bitmask(mask) == 0b0101u);
    const v128_t sel = v4f::select(v4f::splat(1.0f), v4f::splat(0.0f), mask);
    CHECK(ctx, v4f::lane<0>(sel) == 1.0f && v4f::lane<1>(sel) == 0.0f);

    CHECK(ctx, v4f::lane<1>(v4f::min(v4f::make(1.0f, 5.0f, 3.0f, 7.0f), v4f::splat(4.0f))) == 4.0f);
    CHECK(ctx, v4f::lane<0>(v4f::sqrt(v4f::splat(16.0f))) == 4.0f);
}
