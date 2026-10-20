#include "math/eulerf.h"
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
    const Float3 X = vec3f::from_values(1.0f, 0.0f, 0.0f);
    const Float3 Y = vec3f::from_values(0.0f, 1.0f, 0.0f);
    const Float3 Z = vec3f::from_values(0.0f, 0.0f, 1.0f);

    // a general invertible matrix (column-major): rotation-ish upper block, a translation, a w row
    const Float4x4 a = mat4f::from_values(1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 10, 0, 11, 12, 13, 1);

    CHECK(ctx, mat4f::exact_equals(mat4f::identity(out), I));
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

    // translate / scale / rotate agree with multiplying by from_*; aliasing too
    Float4x4 t;
    mat4f::from_translation(t, p);
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
    Float4x4 r;
    CHECK(ctx, mat4f::from_rotation(r, 0.6f, w));
    mat4f::multiply(expect, a, r);
    CHECK(ctx, mat4f::rotate(out, a, 0.6f, w));
    CHECK(ctx, near16(out, expect));
    mat4f::copy(out, a);
    CHECK(ctx, mat4f::rotate(out, out, 0.6f, w));
    CHECK(ctx, near16(out, expect));
    CHECK(ctx, !mat4f::rotate(out, a, 0.6f, vec3f::create()));
    CHECK(ctx, !mat4f::from_rotation(r, 0.6f, vec3f::create()));
    // axis rotations
    mat4f::from_x_rotation(r, 0.4f);
    mat4f::multiply(expect, a, r);
    CHECK(ctx, near16(mat4f::rotate_x(out, a, 0.4f), expect));
    mat4f::copy(out, a);
    CHECK(ctx, near16(mat4f::rotate_x(out, out, 0.4f), expect));
    mat4f::from_y_rotation(r, 0.4f);
    mat4f::multiply(expect, a, r);
    CHECK(ctx, near16(mat4f::rotate_y(out, a, 0.4f), expect));
    mat4f::copy(out, a);
    CHECK(ctx, near16(mat4f::rotate_y(out, out, 0.4f), expect));
    mat4f::from_z_rotation(r, 0.4f);
    mat4f::multiply(expect, a, r);
    CHECK(ctx, near16(mat4f::rotate_z(out, a, 0.4f), expect));
    mat4f::copy(out, a);
    CHECK(ctx, near16(mat4f::rotate_z(out, out, 0.4f), expect));
    // from_x_rotation by a quarter turn takes y to z
    mat4f::from_x_rotation(r, scalar::PI * 0.5f);
    mat4f::multiply3x3_vec(v, r, Y);
    CHECK(ctx, near3(v, 0.0f, 0.0f, 1.0f, 1e-6f));
    // from_rotation about z matches from_z_rotation and from_quat
    mat4f::from_rotation(r, 0.9f, Z);
    mat4f::from_z_rotation(expect, 0.9f);
    CHECK(ctx, near16(r, expect));
    Float4 qz;
    quatf::set_axis_angle(qz, Z, 0.9f);
    CHECK(ctx, near16(mat4f::from_quat(out, qz), expect));

    // trs: from_rotation_translation_scale is T * R * S; decompose gets the parts back
    Float4 q;
    quatf::from_euler(q, eulerf::from_values(0.3f, -0.5f, 0.8f, EULER_XYZ));
    const Float3 tr = vec3f::from_values(1.0f, -2.0f, 3.0f);
    const Float3 sc = vec3f::from_values(2.0f, 3.0f, 0.5f);
    mat4f::from_translation(t, tr);
    mat4f::from_quat(r, q);
    mat4f::from_scaling(s, sc);
    mat4f::multiply(expect, t, r);
    mat4f::multiply(expect, expect, s);
    mat4f::from_rotation_translation_scale(out, q, tr, sc);
    CHECK(ctx, near16(out, expect));
    mat4f::from_rotation_translation(out, q, tr);
    mat4f::multiply(expect, t, r);
    CHECK(ctx, near16(out, expect));
    Float4 dq;
    Float3 dt;
    Float3 ds;
    mat4f::from_rotation_translation_scale(out, q, tr, sc);
    mat4f::decompose(dq, dt, ds, out);
    CHECK(ctx, near3(dt, tr.x, tr.y, tr.z, 1e-5f));
    CHECK(ctx, near3(ds, sc.x, sc.y, sc.z, 1e-5f));
    CHECK(ctx, quatf::equals(dq, q));
    mat4f::get_translation(v, out);
    CHECK(ctx, near3(v, tr.x, tr.y, tr.z, 1e-5f));
    mat4f::get_scaling(v, out);
    CHECK(ctx, near3(v, sc.x, sc.y, sc.z, 1e-5f));
    mat4f::get_rotation(dq, out);
    CHECK(ctx, quatf::equals(dq, q));
    // every branch of the rotation extraction: half turns about each axis have a non-positive trace
    const Float3 axes[3] = {X, Y, Z};
    for (int i = 0; i < 3; i++) {
        Float4 half;
        quatf::set_axis_angle(half, axes[i], scalar::PI);
        mat4f::from_quat(out, half);
        mat4f::get_rotation(dq, out);
        CHECK(ctx, quatf::equals(dq, half));
    }
    // with an origin: the origin point stays fixed by rotation and scale, then translates
    const Float3 o = vec3f::from_values(1.0f, 1.0f, 1.0f);
    mat4f::from_rotation_translation_scale_origin(out, q, tr, sc, o);
    vec3f::transform_mat4(v, o, out);
    CHECK(ctx, near3(v, o.x + tr.x, o.y + tr.y, o.z + tr.z, 1e-5f));

    // look_at from +z toward the origin maps the origin to -5 on z; coincident eye and center is identity
    const Float3 eye = vec3f::from_values(0.0f, 0.0f, 5.0f);
    mat4f::look_at(out, eye, vec3f::create(), Y);
    vec3f::transform_mat4(v, vec3f::create(), out);
    CHECK(ctx, near3(v, 0.0f, 0.0f, -5.0f, 1e-5f));
    vec3f::transform_mat4(v, eye, out);
    CHECK(ctx, near3(v, 0.0f, 0.0f, 0.0f, 1e-5f));
    CHECK(ctx, mat4f::exact_equals(mat4f::look_at(out, eye, eye, Y), I));
    // target_to is the inverse of look_at
    mat4f::target_to(out, eye, vec3f::create(), Y);
    mat4f::look_at(expect, eye, vec3f::create(), Y);
    mat4f::multiply(out, out, expect);
    CHECK(ctx, near16(out, I));

    CHECK_NEAR(ctx, mat4f::frob(I), 2.0f, 1e-6f);
    mat4f::add(out, a, I);
    CHECK(ctx, out.m[0] == 2.0f && out.m[1] == 2.0f && out.m[15] == 2.0f);
    mat4f::subtract(out, a, I);
    CHECK(ctx, out.m[0] == 0.0f && out.m[5] == 4.0f);
    mat4f::multiply_scalar(out, a, 2.0f);
    CHECK(ctx, out.m[2] == 6.0f && out.m[14] == 26.0f);
    mat4f::multiply_scalar_and_add(out, a, I, 0.5f);
    CHECK(ctx, out.m[0] == 1.5f && out.m[1] == 2.0f && out.m[15] == 1.5f);

    CHECK(ctx, mat4f::exact_equals(a, mat4f::clone(a)));
    CHECK(ctx, !mat4f::exact_equals(a, I));
    Float4x4 nearly = a;
    nearly.m[10] += 0.5f * scalar::EPSILON;
    CHECK(ctx, mat4f::equals(a, nearly));
    CHECK(ctx, !mat4f::equals(a, I));
}
