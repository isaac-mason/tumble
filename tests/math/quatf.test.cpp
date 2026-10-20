#include "tumble/math/eulerf.h"
#include "math/storage/quatf.h"
#include "test.h"
#include "test_math.h"

static bool near4(const Float4 &q, float x, float y, float z, float w, float eps = 1e-6f) {
    return scalar::abs(q.x - x) <= eps && scalar::abs(q.y - y) <= eps && scalar::abs(q.z - z) <= eps && scalar::abs(q.w - w) <= eps;
}

void test_quatf(TestCtx &ctx) {
    const float h = 0.70710678f;
    const Float3 X = vec3f::from_values(1.0f, 0.0f, 0.0f);
    const Float3 Y = vec3f::from_values(0.0f, 1.0f, 0.0f);
    const Float3 Z = vec3f::from_values(0.0f, 0.0f, 1.0f);
    Float4 out;
    Float4 q;
    Float3 v;

    CHECK(ctx, near4(quatf::create(), 0.0f, 0.0f, 0.0f, 1.0f));
    CHECK(ctx, near4(quatf::identity(out), 0.0f, 0.0f, 0.0f, 1.0f));

    // 90 degrees about z
    quatf::set_axis_angle(q, Z, scalar::PI * 0.5f);
    CHECK(ctx, near4(q, 0.0f, 0.0f, h, h));
    CHECK_NEAR(ctx, quatf::get_axis_angle(v, q), scalar::PI * 0.5f, 1e-6f);
    CHECK(ctx, near3(v, 0.0f, 0.0f, 1.0f, 1e-5f));
    // the identity has no axis: x and angle zero
    CHECK_NEAR(ctx, quatf::get_axis_angle(v, quatf::create()), 0.0f, 1e-6f);
    CHECK(ctx, near3(v, 1.0f, 0.0f, 0.0f, 1e-6f));

    // multiply: two quarter turns about z make a half turn, and it is not commutative
    quatf::multiply(out, q, q);
    CHECK(ctx, near4(out, 0.0f, 0.0f, 1.0f, 0.0f, 1e-6f));
    Float4 qx;
    quatf::set_axis_angle(qx, X, scalar::PI * 0.5f);
    Float4 ab;
    Float4 ba;
    quatf::multiply(ab, q, qx);
    quatf::multiply(ba, qx, q);
    CHECK(ctx, !quatf::exact_equals(ab, ba));
    // ab against the scalar formula
    CHECK(ctx, near4(ab, q.x * qx.w + q.w * qx.x + q.y * qx.z - q.z * qx.y, q.y * qx.w + q.w * qx.y + q.z * qx.x - q.x * qx.z,
                     q.z * qx.w + q.w * qx.z + q.x * qx.y - q.y * qx.x, q.w * qx.w - q.x * qx.x - q.y * qx.y - q.z * qx.z));
    // identity is the unit of the product
    quatf::multiply(out, q, quatf::create());
    CHECK(ctx, near4(out, q.x, q.y, q.z, q.w));
    // and the product rotates vectors as the composition does: x -> y (by q) then y -> z (by qx)
    vec3f::transform_quat(v, X, ba);
    CHECK(ctx, near3(v, 0.0f, 0.0f, 1.0f, 1e-6f));

    // rotate_x/y/z agree with multiplying by the axis rotation
    quatf::rotate_x(out, q, 0.3f);
    Float4 rx;
    quatf::set_axis_angle(rx, X, 0.3f);
    quatf::multiply(ab, q, rx);
    CHECK(ctx, near4(out, ab.x, ab.y, ab.z, ab.w));
    quatf::rotate_y(out, q, 0.3f);
    quatf::set_axis_angle(rx, Y, 0.3f);
    quatf::multiply(ab, q, rx);
    CHECK(ctx, near4(out, ab.x, ab.y, ab.z, ab.w));
    quatf::rotate_z(out, q, 0.3f);
    quatf::set_axis_angle(rx, Z, 0.3f);
    quatf::multiply(ab, q, rx);
    CHECK(ctx, near4(out, ab.x, ab.y, ab.z, ab.w));

    CHECK(ctx, near4(quatf::calculate_w(out, quatf::from_values(0.0f, 0.0f, h, 0.0f)), 0.0f, 0.0f, h, h, 1e-6f));

    // exp and ln are inverses; pow(q, 2) is q * q
    quatf::ln(out, q);
    quatf::exp(out, out);
    CHECK(ctx, near4(out, q.x, q.y, q.z, q.w, 1e-6f));
    quatf::pow(out, q, 2.0f);
    CHECK(ctx, near4(out, 0.0f, 0.0f, 1.0f, 0.0f, 1e-6f));

    // slerp: halfway from identity to a quarter turn is an eighth turn; the far side flips to the short arc
    quatf::slerp(out, quatf::create(), q, 0.5f);
    quatf::set_axis_angle(ab, Z, scalar::PI * 0.25f);
    CHECK(ctx, near4(out, ab.x, ab.y, ab.z, ab.w, 1e-6f));
    quatf::slerp(out, quatf::create(), quatf::from_values(-q.x, -q.y, -q.z, -q.w), 0.5f);
    CHECK(ctx, near4(out, -ab.x, -ab.y, -ab.z, -ab.w, 1e-6f) || near4(out, ab.x, ab.y, ab.z, ab.w, 1e-6f));
    // near-identical quaternions take the linear branch
    quatf::slerp(out, q, q, 0.3f);
    CHECK(ctx, near4(out, q.x, q.y, q.z, q.w));

    quatf::invert(out, q);
    quatf::multiply(ab, q, out);
    CHECK(ctx, near4(ab, 0.0f, 0.0f, 0.0f, 1.0f, 1e-6f));
    CHECK(ctx, near4(quatf::invert(out, quatf::from_values(0.0f, 0.0f, 0.0f, 0.0f)), 0.0f, 0.0f, 0.0f, 0.0f));
    CHECK(ctx, near4(quatf::conjugate(out, q), -q.x, -q.y, -q.z, q.w));

    // from_mat3 on a 90 degree z rotation (column-major) gives q; the trace-negative branch too
    const Mat3f rz = {{0, 1, 0, -1, 0, 0, 0, 0, 1}};
    CHECK(ctx, near4(quatf::from_mat3(out, rz), 0.0f, 0.0f, h, h, 1e-6f));
    const Mat3f rz180 = {{-1, 0, 0, 0, -1, 0, 0, 0, 1}};
    quatf::from_mat3(out, rz180);
    CHECK(ctx, near4(out, 0.0f, 0.0f, 1.0f, 0.0f, 1e-6f) || near4(out, 0.0f, 0.0f, -1.0f, 0.0f, 1e-6f));
    const Float4x4 rz4 = {{0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 5, 6, 7, 1}};
    CHECK(ctx, near4(quatf::from_mat4(out, rz4), 0.0f, 0.0f, h, h, 1e-6f));

    // euler: a single-axis angle is the axis rotation in every order
    for (int order = 0; order < 6; order++) {
        quatf::from_euler(out, eulerf::from_values(0.0f, 0.0f, scalar::PI * 0.5f, (EulerOrder)order));
        CHECK(ctx, near4(out, 0.0f, 0.0f, h, h, 1e-6f));
    }
    // xyz applies x then y then z (intrinsic): matches multiplying z * y * x
    Float4 ex;
    Float4 ey;
    Float4 ez;
    quatf::set_axis_angle(ex, X, 0.3f);
    quatf::set_axis_angle(ey, Y, 0.5f);
    quatf::set_axis_angle(ez, Z, 0.7f);
    quatf::multiply(ab, ex, ey);
    quatf::multiply(ab, ab, ez);
    quatf::from_euler(out, eulerf::from_values(0.3f, 0.5f, 0.7f, EULER_XYZ));
    CHECK(ctx, near4(out, ab.x, ab.y, ab.z, ab.w, 1e-6f));
    quatf::from_degrees(out, 0.0f, 0.0f, 90.0f, EULER_XYZ);
    CHECK(ctx, near4(out, 0.0f, 0.0f, h, h, 1e-6f));

    CHECK(ctx, quatf::equals(q, quatf::from_values(-q.x, -q.y, -q.z, -q.w)));
    CHECK(ctx, !quatf::equals(q, quatf::create()));
    CHECK_NEAR(ctx, quatf::get_angle(q, quatf::create()), scalar::PI * 0.5f, 1e-6f);

    // rotation_to: x onto y is the quarter turn about z; antiparallel is a half turn; parallel is identity
    quatf::rotation_to(out, X, Y);
    CHECK(ctx, near4(out, 0.0f, 0.0f, h, h, 1e-6f));
    quatf::rotation_to(out, X, vec3f::from_values(-1.0f, 0.0f, 0.0f));
    vec3f::transform_quat(v, X, out);
    CHECK(ctx, near3(v, -1.0f, 0.0f, 0.0f, 1e-6f));
    CHECK(ctx, near4(quatf::rotation_to(out, X, X), 0.0f, 0.0f, 0.0f, 1.0f));

    // sqlerp at the ends is the end points
    CHECK(ctx, near4(quatf::sqlerp(out, quatf::create(), q, q, ab, 0.0f), 0.0f, 0.0f, 0.0f, 1.0f, 1e-6f));
    CHECK(ctx, near4(quatf::sqlerp(out, quatf::create(), q, q, ab, 1.0f), ab.x, ab.y, ab.z, ab.w, 1e-6f));

    // set_axes with the standard frame (view along -z) is the identity
    CHECK(ctx, near4(quatf::set_axes(out, vec3f::from_values(0.0f, 0.0f, -1.0f), X, Y), 0.0f, 0.0f, 0.0f, 1.0f, 1e-6f));

    // target_to: looking along -z from the origin is the identity; looking along -x turns by -90 about y
    quatf::target_to(out, vec3f::create(), vec3f::from_values(0.0f, 0.0f, -1.0f), Y);
    CHECK(ctx, near4(out, 0.0f, 0.0f, 0.0f, 1.0f, 1e-6f));
    quatf::target_to(out, vec3f::create(), vec3f::from_values(-1.0f, 0.0f, 0.0f), Y);
    vec3f::transform_quat(v, vec3f::from_values(0.0f, 0.0f, -1.0f), out);
    CHECK(ctx, near3(v, -1.0f, 0.0f, 0.0f, 1e-6f));
    // up along the view direction falls back to another axis rather than nan
    quatf::target_to(out, vec3f::create(), vec3f::from_values(0.0f, 0.0f, -1.0f), Z);
    CHECK(ctx, scalar::is_finite(out.x) && scalar::is_finite(out.w));

    // the vec4 arithmetic
    CHECK(ctx, near4(quatf::add(out, q, q), 0.0f, 0.0f, 2.0f * h, 2.0f * h));
    CHECK(ctx, near4(quatf::scale(out, q, 2.0f), 0.0f, 0.0f, 2.0f * h, 2.0f * h));
    CHECK_NEAR(ctx, quatf::dot(q, q), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, quatf::length(quatf::from_values(0.0f, 3.0f, 0.0f, 4.0f)), 5.0f, 1e-6f);
    CHECK(ctx, near4(quatf::normalize(out, quatf::from_values(0.0f, 0.0f, 0.0f, 2.0f)), 0.0f, 0.0f, 0.0f, 1.0f));
    CHECK(ctx, near4(quatf::lerp(out, quatf::create(), quatf::from_values(0.0f, 0.0f, 1.0f, 0.0f), 0.5f), 0.0f, 0.0f, 0.5f, 0.5f));
    float buffer[5] = {0, 0, 0, 0, 0};
    quatf::to_buffer(buffer, q, 1);
    CHECK(ctx, buffer[0] == 0.0f && buffer[3] == h && buffer[4] == h);
    quatf::from_buffer(out, buffer, 1);
    CHECK(ctx, quatf::exact_equals(out, q));
}
