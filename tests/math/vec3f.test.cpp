#include "math/storage/vec3f.h"
#include "test.h"
#include "test_math.h"

void test_vec3f(TestCtx &ctx) {
    const Float3 a = vec3f::from_values(1.0f, 2.0f, 3.0f);
    const Float3 b = vec3f::from_values(4.0f, -5.0f, 6.0f);
    Float3 out;

    CHECK(ctx, near3(vec3f::add(out, a, b), 5.0f, -3.0f, 9.0f, 1e-6f));
    CHECK(ctx, near3(vec3f::subtract(out, a, b), -3.0f, 7.0f, -3.0f, 1e-6f));
    CHECK(ctx, near3(vec3f::multiply(out, a, b), 4.0f, -10.0f, 18.0f, 1e-6f));
    CHECK(ctx, near3(vec3f::divide(out, a, b), 0.25f, -0.4f, 0.5f, 1e-6f));
    CHECK(ctx, near3(vec3f::scale(out, a, 2.0f), 2.0f, 4.0f, 6.0f, 1e-6f));
    CHECK(ctx, near3(vec3f::scale_and_add(out, a, b, 0.5f), 3.0f, -0.5f, 6.0f, 1e-6f));
    CHECK(ctx, near3(vec3f::negate(out, a), -1.0f, -2.0f, -3.0f, 1e-6f));
    CHECK(ctx, near3(vec3f::min(out, a, b), 1.0f, -5.0f, 3.0f, 1e-6f));
    CHECK(ctx, near3(vec3f::max(out, a, b), 4.0f, 2.0f, 6.0f, 1e-6f));
    CHECK(ctx, near3(vec3f::inverse(out, a), 1.0f, 0.5f, 1.0f / 3.0f, 1e-6f));

    CHECK(ctx, vec3f::dot(a, b) == 12.0f);
    CHECK(ctx, vec3f::squared_length(a) == 14.0f);
    CHECK_NEAR(ctx, vec3f::length(vec3f::from_values(3.0f, 4.0f, 0.0f)), 5.0f, 1e-7f);
    CHECK(ctx, vec3f::squared_distance(a, b) == 67.0f);
    CHECK_NEAR(ctx, vec3f::distance(vec3f::from_values(0.0f, 0.0f, 0.0f), vec3f::from_values(0.0f, 3.0f, 4.0f)), 5.0f, 1e-7f);

    CHECK(ctx, near3(vec3f::cross(out, vec3f::from_values(1.0f, 0.0f, 0.0f), vec3f::from_values(0.0f, 1.0f, 0.0f)), 0.0f, 0.0f, 1.0f, 1e-6f));
    CHECK(ctx, near3(vec3f::cross(out, a, b), 27.0f, 6.0f, -13.0f, 1e-6f));

    CHECK(ctx, near3(vec3f::normalize(out, vec3f::from_values(0.0f, 3.0f, 4.0f)), 0.0f, 0.6f, 0.8f, 1e-6f));
    CHECK(ctx, near3(vec3f::normalize(out, vec3f::from_values(0.0f, 0.0f, 0.0f)), 0.0f, 0.0f, 0.0f, 1e-6f));

    // perpendicular is unit and orthogonal; the zero vector gets +x
    vec3f::perpendicular(out, a);
    CHECK_NEAR(ctx, vec3f::dot(out, a), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::length(out), 1.0f, 1e-6f);
    vec3f::perpendicular(out, vec3f::from_values(0.0f, 5.0f, 1.0f));
    CHECK_NEAR(ctx, vec3f::dot(out, vec3f::from_values(0.0f, 5.0f, 1.0f)), 0.0f, 1e-6f);
    CHECK(ctx, near3(vec3f::perpendicular(out, vec3f::from_values(0.0f, 0.0f, 0.0f)), 1.0f, 0.0f, 0.0f, 1e-6f));

    CHECK(ctx, near3(vec3f::lerp(out, a, b, 0.5f), 2.5f, -1.5f, 4.5f, 1e-6f));

    // a translation matrix moves the point; the perspective divide splits by w, and a zero w divides by one
    Float4x4 m = {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 10, 20, 30, 1}};
    CHECK(ctx, near3(vec3f::transform_mat4(out, a, m), 11.0f, 22.0f, 33.0f, 1e-6f));
    m.m[15] = 2.0f;
    CHECK(ctx, near3(vec3f::transform_mat4(out, a, m), 5.5f, 11.0f, 16.5f, 1e-6f));
    m.m[15] = 0.0f;
    CHECK(ctx, near3(vec3f::transform_mat4(out, a, m), 11.0f, 22.0f, 33.0f, 1e-6f));

    // 90 degrees about z as a quaternion takes x to y
    const float h = 0.70710678f;
    const Float4 qz = {0.0f, 0.0f, h, h};
    CHECK(ctx, near3(vec3f::transform_quat(out, vec3f::from_values(1.0f, 0.0f, 0.0f), qz), 0.0f, 1.0f, 0.0f, 1e-6f));
    CHECK(ctx, near3(vec3f::transform_quat(out, a, Float4{0.0f, 0.0f, 0.0f, 1.0f}), 1.0f, 2.0f, 3.0f, 1e-6f));

    CHECK(ctx, vec3f::exact_equals(a, vec3f::from_values(1.0f, 2.0f, 3.0f)));
    CHECK(ctx, !vec3f::exact_equals(a, b));

    CHECK(ctx, !vec3f::is_scale_inside_out(vec3f::from_values(1.0f, 1.0f, 1.0f)));
    CHECK(ctx, vec3f::is_scale_inside_out(vec3f::from_values(-1.0f, 1.0f, 1.0f)));
    CHECK(ctx, !vec3f::is_scale_inside_out(vec3f::from_values(-1.0f, -1.0f, 1.0f)));
    CHECK(ctx, vec3f::is_scale_inside_out(vec3f::from_values(-1.0f, -1.0f, -1.0f)));
}
