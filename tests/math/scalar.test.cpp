#include "tumble/math/scalar.h"

#include "export.h"
#include "test.h"

// the transcendentals are swept against node's Math by run.mjs through these probes.
WASM_EXPORT float probe_sin(float x) { return scalar::sin(x); }
WASM_EXPORT float probe_cos(float x) { return scalar::cos(x); }
WASM_EXPORT float probe_tan(float x) { return scalar::tan(x); }
WASM_EXPORT float probe_atan(float x) { return scalar::atan(x); }
WASM_EXPORT float probe_atan2(float y, float x) { return scalar::atan2(y, x); }
WASM_EXPORT float probe_acos(float x) { return scalar::acos(x); }
WASM_EXPORT float probe_asin(float x) { return scalar::asin(x); }
WASM_EXPORT float probe_exp(float x) { return scalar::exp(x); }
WASM_EXPORT float probe_pow(float x, float y) { return scalar::pow(x, y); }
WASM_EXPORT float probe_log(float x) { return scalar::log(x); }

void test_scalar(TestCtx &ctx) {
    // symmetric round: halves away from zero, unlike js Math.round on negatives
    CHECK(ctx, scalar::round(2.5f) == 3.0f);
    CHECK(ctx, scalar::round(-2.5f) == -3.0f);
    CHECK(ctx, scalar::round(-2.4f) == -2.0f);

    CHECK(ctx, scalar::equals(1.0f, 1.0f + 0.5f * scalar::EPSILON));
    CHECK(ctx, !scalar::equals(1.0f, 1.0f + 2.0f * scalar::EPSILON));
    // relative above 1: 1000 vs 1000.0005 is within 1e-6 * 1000
    CHECK(ctx, scalar::equals(1000.0f, 1000.0005f));

    CHECK(ctx, scalar::fade(0.0f) == 0.0f);
    CHECK(ctx, scalar::fade(1.0f) == 1.0f);
    CHECK(ctx, scalar::fade(0.5f) == 0.5f);

    CHECK(ctx, scalar::lerp(2.0f, 4.0f, 0.25f) == 2.5f);
    CHECK_NEAR(ctx, scalar::lagrange(1.0f, 2.0f, 3.0f, 0.0f), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, scalar::lagrange(1.0f, 2.0f, 3.0f, 0.5f), 2.0f, 1e-6f);
    CHECK_NEAR(ctx, scalar::lagrange(1.0f, 2.0f, 3.0f, 1.0f), 3.0f, 1e-6f);

    CHECK(ctx, scalar::binomial(5, 2) == 10.0f);
    CHECK(ctx, scalar::binomial(10, 7) == 120.0f);
    CHECK(ctx, scalar::binomial(4, 5) == 0.0f);
    CHECK(ctx, scalar::binomial(4, -1) == 0.0f);

    CHECK(ctx, scalar::clamp(5.0f, 0.0f, 1.0f) == 1.0f);
    CHECK(ctx, scalar::clamp(-5.0f, 0.0f, 1.0f) == 0.0f);

    // floored modulo
    CHECK(ctx, scalar::repeat(-1.0f, 3.0f) == 2.0f);
    CHECK(ctx, scalar::repeat(4.0f, 3.0f) == 1.0f);
    CHECK(ctx, scalar::fmod(-1.0f, 3.0f) == -1.0f);

    CHECK(ctx, scalar::remap(5.0f, 0.0f, 10.0f, 0.0f, 1.0f) == 0.5f);
    CHECK(ctx, scalar::remap_clamp(15.0f, 0.0f, 10.0f, 0.0f, 1.0f) == 1.0f);

    CHECK(ctx, scalar::smoothstep(0.0f, 1.0f, 0.5f) == 0.5f);
    CHECK(ctx, scalar::smoothstep(0.0f, 1.0f, 2.0f) == 1.0f);
    CHECK(ctx, scalar::smoothstep(1.0f, 1.0f, 1.0f) == 0.0f);
    CHECK(ctx, scalar::smoothstep(1.0f, 1.0f, 1.5f) == 1.0f);
    CHECK(ctx, scalar::smootherstep(0.0f, 1.0f, 0.5f) == 0.5f);

    CHECK_NEAR(ctx, scalar::degrees_to_radians(180.0f), scalar::PI, 1e-6f);
    CHECK_NEAR(ctx, scalar::radians_to_degrees(scalar::PI), 180.0f, 1e-4f);

    // wrap into (-pi, pi]; 3pi lands on the boundary and floors to -pi, exactly as the js does
    CHECK_NEAR(ctx, scalar::wrap_angle(scalar::PI * 3.0f), -scalar::PI, 1e-5f);
    CHECK_NEAR(ctx, scalar::wrap_angle(scalar::PI * 2.5f), scalar::PI * 0.5f, 1e-5f);
    CHECK_NEAR(ctx, scalar::wrap_angle(-scalar::PI * 1.5f), scalar::PI * 0.5f, 1e-5f);
    CHECK_NEAR(ctx, scalar::delta_angle(0.1f, scalar::TAU - 0.1f), -0.2f, 1e-5f);
    CHECK_NEAR(ctx, scalar::delta_angle(-0.1f, 0.1f), 0.2f, 1e-6f);

    // the musl routines at points whose values are exact or well known
    CHECK(ctx, scalar::sin(0.0f) == 0.0f);
    CHECK_NEAR(ctx, scalar::sin(scalar::PI * 0.5f), 1.0f, 1e-7f);
    CHECK_NEAR(ctx, scalar::cos(scalar::PI), -1.0f, 1e-7f);
    CHECK_NEAR(ctx, scalar::atan2(1.0f, 1.0f), scalar::PI * 0.25f, 1e-7f);
    CHECK_NEAR(ctx, scalar::atan2(0.0f, -1.0f), scalar::PI, 1e-7f);
    // musl's acosf(0) is pio2_hi, one ulp under the nearest f32 to pi/2
    CHECK_NEAR(ctx, scalar::acos(0.0f), scalar::PI * 0.5f, 2e-7f);
    CHECK_NEAR(ctx, scalar::asin(1.0f), scalar::PI * 0.5f, 1e-7f);
    CHECK_NEAR(ctx, scalar::exp(1.0f), 2.7182817f, 1e-6f);
    CHECK_NEAR(ctx, scalar::log(2.7182817f), 1.0f, 1e-6f);
    CHECK(ctx, scalar::pow(2.0f, 10.0f) == 1024.0f);
    CHECK_NEAR(ctx, scalar::pow(9.0f, 0.5f), 3.0f, 1e-6f);
    CHECK(ctx, scalar::is_nan(scalar::sqrt(-1.0f)));
    CHECK(ctx, !scalar::is_finite(scalar::log(0.0f)));
}
