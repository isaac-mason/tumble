#include "tumble/math/scalar.h"

#include "export.h"
#include "test.h"

// run.mjs sweeps these against node's Math
WASM_EXPORT float probe_sin(float x) { return scalar::sin(x); }
WASM_EXPORT float probe_cos(float x) { return scalar::cos(x); }
WASM_EXPORT float probe_atan(float x) { return scalar::atan(x); }
WASM_EXPORT float probe_atan2(float y, float x) { return scalar::atan2(y, x); }
WASM_EXPORT float probe_acos(float x) { return scalar::acos(x); }

void test_scalar_clamp(TestCtx &ctx) {
    CHECK(ctx, scalar::clamp(5.0f, 0.0f, 1.0f) == 1.0f);
    CHECK(ctx, scalar::clamp(-5.0f, 0.0f, 1.0f) == 0.0f);
    CHECK(ctx, scalar::clamp(0.25f, 0.0f, 1.0f) == 0.25f);
}

void test_scalar_min_max_propagate_nan_and_order_negative_zero_below_positive(TestCtx &ctx) {
    const float nan = __builtin_nanf("");
    CHECK(ctx, scalar::min(nan, 1.0f) != scalar::min(nan, 1.0f));
    CHECK(ctx, scalar::max(1.0f, nan) != scalar::max(1.0f, nan));
    CHECK(ctx, __builtin_signbit(scalar::min(0.0f, -0.0f)) && !__builtin_signbit(scalar::max(-0.0f, 0.0f)));
}

void test_scalar_sign(TestCtx &ctx) {
    CHECK(ctx, scalar::sign(3.0f) == 1.0f && scalar::sign(-3.0f) == -1.0f && scalar::sign(0.0f) == 0.0f);
}

void test_scalar_transcendentals_at_known_points(TestCtx &ctx) {
    CHECK(ctx, scalar::sin(0.0f) == 0.0f);
    CHECK_NEAR(ctx, scalar::sin(scalar::PI * 0.5f), 1.0f, 1e-7f);
    CHECK_NEAR(ctx, scalar::cos(scalar::PI), -1.0f, 1e-7f);
    CHECK_NEAR(ctx, scalar::atan(1.0f), scalar::PI * 0.25f, 1e-7f);
    CHECK_NEAR(ctx, scalar::atan2(1.0f, 1.0f), scalar::PI * 0.25f, 1e-7f);
    CHECK_NEAR(ctx, scalar::atan2(0.0f, -1.0f), scalar::PI, 1e-7f);
    // musl's acosf(0) is pio2_hi, one ulp under the nearest f32 to pi/2
    CHECK_NEAR(ctx, scalar::acos(0.0f), scalar::PI * 0.5f, 2e-7f);
}
