#include "math/storage/mat4f.h"
#include "math/storage/quatf.h"
#include "math/storage/vec3f.h"
#include "test.h"
#include "test_math.h"

static bool near16(const Float4x4 &m, const Float4x4 &e, float eps = 1e-5f) {
    for (int i = 0; i < 16; i++) {
        if (scalar::abs(m.m[i] - e.m[i]) > eps) return false;
    }
    return true;
}

void test_mat4f(TestCtx &ctx) {
    const Float4x4 I = mat4f::create();
    Float4x4 out;
    Float4x4 expect;
    const Float3 Z = vec3f::from_values(0.0f, 0.0f, 1.0f);

    // a general invertible matrix (column-major): rotation-ish upper block, a translation, a w row
    const Float4x4 a = mat4f::from_values(1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 10, 0, 11, 12, 13, 1);

    CHECK(ctx, mat4f::exact_equals(mat4f::multiply(out, a, I), a));
    CHECK(ctx, mat4f::exact_equals(mat4f::multiply(out, I, a), a));

    // transpose twice is itself, in place too
    mat4f::transpose(out, a);
    mat4f::transpose(out, out);
    CHECK(ctx, mat4f::exact_equals(out, a));
    mat4f::transpose(out, a);
    CHECK(ctx, out.m[1] == 4.0f && out.m[4] == 2.0f && out.m[12] == 0.0f && out.m[3] == 11.0f);

    CHECK_NEAR(ctx, mat4f::determinant(a), -3.0f, 1e-4f);
    const Float4x4 diag = mat4f::from_values(2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4, 0, 0, 0, 0, 1);
    CHECK(ctx, mat4f::determinant(diag) == 24.0f);

    // inverse: a * inv(a) = I; the singular matrix is refused and out untouched
    Float4x4 inv;
    CHECK(ctx, mat4f::invert(inv, a));
    mat4f::multiply(out, a, inv);
    CHECK(ctx, near16(out, I, 1e-4f));
    mat4f::copy(inv, a);
    CHECK(ctx, !mat4f::invert(inv, mat4f::from_values(1, 2, 3, 0, 2, 4, 6, 0, 0, 0, 0, 0, 0, 0, 0, 1)));
    CHECK(ctx, mat4f::exact_equals(inv, a));
    // invert3x3 inverts only the block and clears the translation
    CHECK(ctx, mat4f::invert3x3(inv, a));
    mat4f::multiply3x3(out, a, inv);
    CHECK(ctx, near16(out, I, 1e-4f));
    CHECK(ctx, inv.m[12] == 0.0f && inv.m[15] == 1.0f);
    // adjoint is det times inverse
    Float4x4 adj;
    mat4f::adjoint(adj, a);
    mat4f::invert(inv, a);
    mat4f::multiply_scalar(inv, inv, mat4f::determinant(a));
    CHECK(ctx, near16(adj, inv, 1e-3f));

    // multiply against the scalar definition, and aliasing both ways
    const Float4x4 b = mat4f::from_values(2, 0, 1, 0, 1, 3, 0, 0, 0, 1, 4, 0, 5, 6, 7, 1);
    mat4f::multiply(expect, a, b);
    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 4; i++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += a.m[k * 4 + i] * b.m[j * 4 + k];
            CHECK_NEAR(ctx, expect.m[j * 4 + i], s, 1e-5f);
        }
    }
    mat4f::copy(out, a);
    mat4f::multiply(out, out, b);
    CHECK(ctx, mat4f::exact_equals(out, expect));
    mat4f::copy(out, b);
    mat4f::multiply(out, a, out);
    CHECK(ctx, mat4f::exact_equals(out, expect));

    // multiply3x3 is the block product with identity elsewhere; right-transposed uses b's transpose
    mat4f::multiply3x3(out, a, b);
    CHECK(ctx, out.m[3] == 0.0f && out.m[12] == 0.0f && out.m[15] == 1.0f);
    CHECK_NEAR(ctx, out.m[0], b.m[0] * a.m[0] + b.m[1] * a.m[4] + b.m[2] * a.m[8], 1e-5f);
    Float4x4 bt;
    mat4f::transpose(bt, b);
    mat4f::multiply3x3(expect, a, bt);
    mat4f::multiply3x3_right_transposed(out, a, b);
    CHECK(ctx, near16(out, expect));
    Float3 v;
    const Float3 p = vec3f::from_values(1.0f, 2.0f, 3.0f);
    mat4f::multiply3x3_vec(v, a, p);
    CHECK(ctx, near3(v, 1 * 1 + 4 * 2 + 7 * 3, 2 * 1 + 5 * 2 + 8 * 3, 3 * 1 + 6 * 2 + 10 * 3, 1e-5f));
    mat4f::multiply3x3_transposed_vec(v, a, p);
    CHECK(ctx, near3(v, 1 * 1 + 2 * 2 + 3 * 3, 4 * 1 + 5 * 2 + 6 * 3, 7 * 1 + 8 * 2 + 10 * 3, 1e-5f));

    // the cross product matrix of w times u is w x u
    const Float3 w = vec3f::from_values(0.5f, -1.0f, 2.0f);
    Float4x4 cpm;
    mat4f::cross_product_matrix(cpm, w);
    mat4f::multiply3x3_vec(v, cpm, p);
    Float3 c;
    vec3f::cross(c, w, p);
    CHECK(ctx, near3(v, c.x, c.y, c.z, 1e-5f));

    // translate and scale are a * T and a * S; aliasing too
    Float4x4 t;
    mat4f::translate(t, I, p);
    CHECK(ctx, t.m[12] == 1.0f && t.m[13] == 2.0f && t.m[14] == 3.0f && t.m[0] == 1.0f);
    mat4f::multiply(expect, a, t);
    CHECK(ctx, near16(mat4f::translate(out, a, p), expect));
    mat4f::copy(out, a);
    mat4f::translate(out, out, p);
    CHECK(ctx, near16(out, expect));
    Float4x4 s;
    mat4f::from_scaling(s, p);
    mat4f::multiply(expect, a, s);
    CHECK(ctx, near16(mat4f::scale(out, a, p), expect));
    mat4f::copy(out, a);
    mat4f::scale(out, out, p);
    CHECK(ctx, near16(out, expect));

    // from_quat of a quarter turn about z takes x to y
    Float4 qz;
    quatf::set_axis_angle(qz, Z, scalar::PI * 0.5f);
    Float4x4 r;
    mat4f::from_quat(r, qz);
    mat4f::multiply3x3_vec(v, r, vec3f::from_values(1.0f, 0.0f, 0.0f));
    CHECK(ctx, near3(v, 0.0f, 1.0f, 0.0f, 1e-6f));

    // from_rotation_translation_scale is T * R * S, from_rotation_translation T * R
    Float4 q;
    quatf::from_euler(q, Eulerf{0.3f, -0.5f, 0.8f, EulerOrder::XYZ});
    const Float3 tr = vec3f::from_values(1.0f, -2.0f, 3.0f);
    const Float3 sc = vec3f::from_values(2.0f, 3.0f, 0.5f);
    mat4f::translate(t, I, tr);
    mat4f::from_quat(r, q);
    mat4f::from_scaling(s, sc);
    mat4f::multiply(expect, t, r);
    mat4f::multiply(expect, expect, s);
    mat4f::from_rotation_translation_scale(out, q, tr, sc);
    CHECK(ctx, near16(out, expect));
    mat4f::get_translation(v, out);
    CHECK(ctx, near3(v, tr.x, tr.y, tr.z, 1e-5f));
    mat4f::from_rotation_translation(out, q, tr);
    mat4f::multiply(expect, t, r);
    CHECK(ctx, near16(out, expect));

    mat4f::add(out, a, I);
    CHECK(ctx, out.m[0] == 2.0f && out.m[1] == 2.0f && out.m[15] == 2.0f);
    mat4f::subtract(out, a, I);
    CHECK(ctx, out.m[0] == 0.0f && out.m[5] == 4.0f);
    mat4f::multiply_scalar(out, a, 2.0f);
    CHECK(ctx, out.m[2] == 6.0f && out.m[14] == 26.0f);
}
