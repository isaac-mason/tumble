#include "tumble/math/eulerf.h"
#include "math/storage/mat4f.h"
#include "test.h"

void test_eulerf(TestCtx &ctx) {
    Eulerf e;
    Eulerf back;
    Float4 q;

    CHECK(ctx, eulerf::create().order == EULER_XYZ);
    eulerf::from_degrees(e, 90.0f, 0.0f, 0.0f, EULER_ZYX);
    CHECK_NEAR(ctx, e.x, scalar::PI * 0.5f, 1e-6f);
    CHECK(ctx, e.order == EULER_ZYX);

    // from_quat inverts quatf::from_euler in every order, away from the gimbal lock
    for (int order = 0; order < 6; order++) {
        eulerf::set(e, 0.3f, -0.4f, 0.7f, (EulerOrder)order);
        quatf::from_euler(q, e);
        eulerf::from_quat(back, q, (EulerOrder)order);
        CHECK(ctx, eulerf::equals(e, back) || (scalar::abs(e.x - back.x) < 1e-5f && scalar::abs(e.y - back.y) < 1e-5f && scalar::abs(e.z - back.z) < 1e-5f));
    }
    // and from the matrix of the same rotation
    Float4x4 m;
    eulerf::set(e, 0.3f, -0.4f, 0.7f, EULER_XYZ);
    quatf::from_euler(q, e);
    mat4f::from_quat(m, q);
    eulerf::from_rotation_mat4(back, m, EULER_XYZ);
    CHECK(ctx, scalar::abs(e.x - back.x) < 1e-5f && scalar::abs(e.y - back.y) < 1e-5f && scalar::abs(e.z - back.z) < 1e-5f);

    // at the lock (pitch 90 in xyz) the z angle folds into x and comes back as zero
    eulerf::set(e, 0.3f, scalar::PI * 0.5f, 0.7f, EULER_XYZ);
    quatf::from_euler(q, e);
    eulerf::from_quat(back, q, EULER_XYZ);
    CHECK(ctx, back.z == 0.0f);
    CHECK_NEAR(ctx, back.y, scalar::PI * 0.5f, 1e-3f);
    Float4 q2;
    quatf::from_euler(q2, back);
    CHECK(ctx, quatf::equals(q, q2));

    // reorder keeps the rotation
    eulerf::set(e, 0.3f, -0.4f, 0.7f, EULER_XYZ);
    eulerf::reorder(back, e, EULER_ZYX);
    CHECK(ctx, back.order == EULER_ZYX);
    quatf::from_euler(q, e);
    quatf::from_euler(q2, back);
    CHECK(ctx, quatf::equals(q, q2));

    CHECK(ctx, eulerf::exact_equals(e, eulerf::clone(e)));
    CHECK(ctx, !eulerf::exact_equals(e, back));
    Eulerf nearly = e;
    nearly.x += 0.5f * scalar::EPSILON;
    CHECK(ctx, eulerf::equals(e, nearly));
    nearly.order = EULER_YXZ;
    CHECK(ctx, !eulerf::equals(e, nearly));
}
