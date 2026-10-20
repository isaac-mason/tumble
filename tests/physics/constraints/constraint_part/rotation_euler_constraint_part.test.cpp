#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/constraint_part/rotation_euler_constraint_part.h"
#include "physics/constraints/constraint_part/part_body.h"
#include "test.h"
#include "test_math.h"

using rigid_body::RigidBody;
using rotation_euler_constraint_part::RotationEulerConstraintPart;

static void calculate(RotationEulerConstraintPart &part, RigidBody &a, RigidBody &b) {
    Float4x4 rotation_a, rotation_b;
    mat4f::store(rotation_a, mat4f::from_quat(quatf::load(a.quaternion)));
    mat4f::store(rotation_b, mat4f::from_quat(quatf::load(b.quaternion)));
    rotation_euler_constraint_part::calculate_constraint_properties(part, a, mat4f::load(rotation_a), b, mat4f::load(rotation_b),
                                                                    motion_properties::STEP_STAMP_NONE);
}

/** the angle of q2 * r0^-1 * q1^-1, the drift from the initial relative rotation */
static float drift(const RigidBody &a, const RigidBody &b, Quatf inv_initial_orientation) {
    const Quatf diff =
        quatf::multiply(quatf::multiply(quatf::load(b.quaternion), inv_initial_orientation), quatf::conjugate(quatf::load(a.quaternion)));
    const float x = quatf::x(diff), y = quatf::y(diff), z = quatf::z(diff);
    const float sin_half = scalar::sqrt(x * x + y * y + z * z);
    return 2.0f * scalar::atan2(sin_half, scalar::abs(quatf::w(diff)));
}

void test_rotation_euler_constraint_part_create_and_deactivate(TestCtx &ctx) {
    RotationEulerConstraintPart part = rotation_euler_constraint_part::create();
    CHECK(ctx, !rotation_euler_constraint_part::is_active(part));
    part.effective_mass = mat4f::identity();
    part.total_lambda = vec3f::make(1.0f, 2.0f, 3.0f);
    CHECK(ctx, rotation_euler_constraint_part::is_active(part));
    rotation_euler_constraint_part::deactivate(part);
    CHECK(ctx, !rotation_euler_constraint_part::is_active(part));
    CHECK(ctx, vec3f::exact_equals(part.total_lambda, vec3f::zero()));
}

void test_rotation_euler_constraint_part_inv_initial_orientation(TestCtx &ctx) {
    // r0^-1 = q2^-1 q1, so q2 r0^-1 q1^-1 is the identity at the initial pose
    const Float4 q1 = stored_axis_angle(vec3f::make(1.0f, 2.0f, 3.0f), 0.7f);
    const Float4 q2 = stored_axis_angle(vec3f::make(-2.0f, 0.5f, 1.0f), 1.9f);
    const Quatf inv_initial = rotation_euler_constraint_part::get_inv_initial_orientation(quatf::load(q1), quatf::load(q2));
    Quatf check = quatf::multiply(quatf::load(q2), inv_initial);
    const Quatf q1_conj = quatf::conjugate(quatf::load(q1));
    check = quatf::multiply(check, q1_conj);
    CHECK_NEAR(ctx, scalar::abs(quatf::w(check)), 1.0f, 1e-6f);

    // identical axes give the identity
    Quatf from_axes = rotation_euler_constraint_part::get_inv_initial_orientation_xy(vec3f::axis_x(), vec3f::axis_y(), vec3f::axis_x(),
                                                                                         vec3f::axis_y());
    CHECK(ctx, quatf::exact_equals(from_axes, quatf::identity()));

    // body 2's axes are body 1's turned by r: c2 c1^-1 = r
    const Quatf r = axis_angle(vec3f::make(0.3f, -1.0f, 0.5f), 1.2f);
    const Quatf c1 = axis_angle(vec3f::make(1.0f, 1.0f, 0.0f), 0.4f);
    const Vec3f x1 = vec3f::transform_quat(vec3f::axis_x(), c1);
    const Vec3f y1 = vec3f::transform_quat(vec3f::axis_y(), c1);
    const Vec3f x2 = vec3f::transform_quat(x1, r);
    const Vec3f y2 = vec3f::transform_quat(y1, r);
    from_axes = rotation_euler_constraint_part::get_inv_initial_orientation_xy(x1, y1, x2, y2);
    CHECK_NEAR(ctx, scalar::abs(quatf::dot(from_axes, r)), 1.0f, 1e-5f);
}

void test_rotation_euler_constraint_part_effective_mass(TestCtx &ctx) {
    // K = I1^-1 + I2^-1 and the effective mass is its inverse, with turned bodies and unequal inertias
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {0.5f, 4.0f, 1.0f});
    a.quaternion = stored_axis_angle(vec3f::make(1.0f, 2.0f, 3.0f), 0.7f);
    b.quaternion = stored_axis_angle(vec3f::make(0.0f, 1.0f, -1.0f), -1.1f);
    RotationEulerConstraintPart part = rotation_euler_constraint_part::create();
    calculate(part, a, b);
    CHECK(ctx, rotation_euler_constraint_part::is_active(part));

    Float4x4 product;
    mat4f::store(product, mat4f::multiply3x3(part.effective_mass, mat4f::add(part.inv_i1, part.inv_i2)));
    for (int col = 0; col < 3; col++)
        for (int row = 0; row < 3; row++) CHECK_NEAR(ctx, product.m[col * 4 + row], row == col ? 1.0f : 0.0f, 1e-5f);

    // a static body contributes a zero inverse inertia; nothing dynamic deactivates
    b.motion_type = motion_type::MOTION_STATIC;
    calculate(part, a, b);
    Float4x4 inv_i2;
    mat4f::store(inv_i2, part.inv_i2);
    for (int i = 0; i < 16; i++) CHECK(ctx, inv_i2.m[i] == 0.0f);
    a.motion_type = motion_type::MOTION_KINEMATIC;
    part.total_lambda = vec3f::make(1.0f, 1.0f, 1.0f);
    calculate(part, a, b);
    CHECK(ctx, !rotation_euler_constraint_part::is_active(part));
    CHECK(ctx, vec3f::exact_equals(part.total_lambda, vec3f::zero()));
}

void test_rotation_euler_constraint_part_velocity_locks_rotation(TestCtx &ctx) {
    // one solve makes the angular velocities equal and keeps the total angular momentum
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {0.5f, 4.0f, 1.0f});
    a.motion_properties.angular_velocity = {1.0f, -2.0f, 0.5f};
    b.motion_properties.angular_velocity = {-0.5f, 0.25f, 3.0f};
    // identity rotations: L = I w with I = diag(1 / inv)
    const Float3 momentum = {1.0f / 1.0f + -0.5f / 0.5f, -2.0f / 2.0f + 0.25f / 4.0f, 0.5f / 3.0f + 3.0f / 1.0f};

    RotationEulerConstraintPart part = rotation_euler_constraint_part::create();
    calculate(part, a, b);
    CHECK(ctx, rotation_euler_constraint_part::solve_velocity_constraint(part, a, b));

    const Float3 &wa = a.motion_properties.angular_velocity;
    const Float3 &wb = b.motion_properties.angular_velocity;
    CHECK(ctx, vec3f::distance(vec3f::load(wa), vec3f::load(wb)) < 1e-5f);
    const Float3 after = {wa.x / 1.0f + wb.x / 0.5f, wa.y / 2.0f + wb.y / 4.0f, wa.z / 3.0f + wb.z / 1.0f};
    CHECK(ctx, vec3f::distance(vec3f::load(after), vec3f::load(momentum)) < 1e-5f);

    // warm starting the stored impulse on the original velocities lands on the same result
    RigidBody a2 = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    RigidBody b2 = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {0.5f, 4.0f, 1.0f});
    a2.motion_properties.angular_velocity = {1.0f, -2.0f, 0.5f};
    b2.motion_properties.angular_velocity = {-0.5f, 0.25f, 3.0f};
    rotation_euler_constraint_part::warm_start(part, a2, b2, 1.0f);
    CHECK(ctx, vec3f::distance(vec3f::load(a2.motion_properties.angular_velocity), vec3f::load(wa)) < 1e-5f);
    CHECK(ctx, vec3f::distance(vec3f::load(b2.motion_properties.angular_velocity), vec3f::load(wb)) < 1e-5f);

    // against a kinematic body the dynamic one takes on its spin
    RigidBody kinematic = make_body(motion_type::MOTION_KINEMATIC, 1.0f, {0.0f, 0.0f, 0.0f});
    kinematic.motion_properties.angular_velocity = {0.0f, 1.0f, 0.0f};
    RigidBody dynamic = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    calculate(part, kinematic, dynamic);
    rotation_euler_constraint_part::solve_velocity_constraint(part, kinematic, dynamic);
    CHECK(ctx, vec3f::distance(vec3f::load(dynamic.motion_properties.angular_velocity), vec3f::make(0.0f, 1.0f, 0.0f)) < 1e-6f);
    CHECK(ctx, vec3f::distance(vec3f::load(kinematic.motion_properties.angular_velocity), vec3f::make(0.0f, 1.0f, 0.0f)) == 0.0f);
}

void test_rotation_euler_constraint_part_position_removes_drift(TestCtx &ctx) {
    // the bodies started at a known relative rotation, then b drifted 0.2 rad off it; position steps remove the drift
    RigidBody a = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {0.5f, 4.0f, 1.0f});
    a.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.5f);
    b.quaternion = stored_axis_angle(vec3f::make(1.0f, 0.0f, 0.0f), 0.8f);
    const Quatf inv_initial = rotation_euler_constraint_part::get_inv_initial_orientation(quatf::load(a.quaternion), quatf::load(b.quaternion));
    CHECK(ctx, drift(a, b, inv_initial) < 1e-5f);

    quatf::store(b.quaternion, quatf::multiply(axis_angle(vec3f::make(1.0f, -1.0f, 2.0f), 0.2f), quatf::load(b.quaternion)));
    CHECK_NEAR(ctx, drift(a, b, inv_initial), 0.2f, 1e-5f);

    RotationEulerConstraintPart part = rotation_euler_constraint_part::create();
    float remaining[6];
    for (int i = 0; i < 6; i++) {
        calculate(part, a, b);
        rotation_euler_constraint_part::solve_position_constraint(part, a, b, inv_initial, 1.0f);
        remaining[i] = drift(a, b, inv_initial);
    }
    CHECK(ctx, remaining[0] < 0.2f * 0.05f);
    CHECK(ctx, remaining[5] < 1e-3f);

    // no drift: nothing applied
    calculate(part, a, b);
    const Quatf at_rest = rotation_euler_constraint_part::get_inv_initial_orientation(quatf::load(a.quaternion), quatf::load(b.quaternion));
    CHECK(ctx, !rotation_euler_constraint_part::solve_position_constraint(part, a, b, at_rest, 1.0f));
}
