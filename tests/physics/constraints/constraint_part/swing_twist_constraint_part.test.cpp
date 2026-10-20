#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/constraint_part/angle_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/swing_twist_constraint_part.h"
#include "physics/constraints/constraint_part/part_body.h"
#include "test.h"
#include "test_math.h"

using rigid_body::RigidBody;
using swing_twist_constraint_part::SwingTwistConstraintPart;

static bool quat_near(Quatf a, Quatf b, float eps) {
    return scalar::abs(quatf::x(a) - quatf::x(b)) <= eps && scalar::abs(quatf::y(a) - quatf::y(b)) <= eps &&
           scalar::abs(quatf::z(a) - quatf::z(b)) <= eps && scalar::abs(quatf::w(a) - quatf::w(b)) <= eps;
}

static const float PI = scalar::PI;
static const uint32_t ALL_SWING = swing_twist_constraint_part::SWING_Y_MIN | swing_twist_constraint_part::SWING_Y_MAX |
                                  swing_twist_constraint_part::SWING_Z_MIN | swing_twist_constraint_part::SWING_Z_MAX;

/** a cone of total half-angle `cone` (both swing axes), twist free */
static SwingTwistConstraintPart make_cone(float cone) {
    SwingTwistConstraintPart part = swing_twist_constraint_part::create();
    swing_twist_constraint_part::set_limits(part, -PI, PI, -cone, cone, -cone, cone);
    return part;
}

void test_swing_twist_constraint_part_create(TestCtx &ctx) {
    SwingTwistConstraintPart part = swing_twist_constraint_part::create();
    CHECK(ctx, part.swing_type == swing_twist_constraint_part::CONE && part.rotation_flags == 0);
    CHECK(ctx, part.cos_twist_half_min_angle == 1.0f && part.cos_swing_y_half_max_angle == 1.0f &&
                   part.cos_swing_z_half_min_angle == 1.0f);
    CHECK(ctx, !swing_twist_constraint_part::is_active(part));
    part.twist_limit_constraint_part.effective_mass = 1.0f;
    CHECK(ctx, swing_twist_constraint_part::is_active(part));
    swing_twist_constraint_part::deactivate(part);
    CHECK(ctx, !swing_twist_constraint_part::is_active(part));
}

void test_swing_twist_constraint_part_set_limits(TestCtx &ctx) {
    SwingTwistConstraintPart part = swing_twist_constraint_part::create();

    // twist limited, swing y locked (under half a degree), swing z free (past 179.5 degrees)
    swing_twist_constraint_part::set_limits(part, -0.5f, 0.75f, -0.004f, 0.004f, -PI, PI);
    CHECK(ctx, part.rotation_flags == (swing_twist_constraint_part::SWING_Y_LOCKED | swing_twist_constraint_part::SWING_Z_FREE));
    CHECK(ctx, part.sin_twist_half_min_angle == scalar::sin(-0.25f) && part.cos_twist_half_max_angle == scalar::cos(0.375f));
    CHECK(ctx, part.sin_swing_y_half_min_angle == 0.0f && part.cos_swing_y_half_max_angle == 1.0f);
    CHECK(ctx, part.sin_swing_z_half_min_angle == -1.0f && part.sin_swing_z_half_max_angle == 1.0f &&
                   part.cos_swing_z_half_min_angle == 0.0f);
    // the half angles are kept for the pyramid whatever the flags
    CHECK(ctx, part.swing_y_half_max_angle == 0.002f && part.swing_z_half_min_angle == -PI * 0.5f);

    // twist locked, swing both free, flags reset from the last call
    swing_twist_constraint_part::set_limits(part, 0.0f, 0.0f, -PI, PI, -PI, PI);
    CHECK(ctx, part.rotation_flags == (swing_twist_constraint_part::TWIST_X_LOCKED | swing_twist_constraint_part::SWING_YZ_FREE));
}

void test_swing_twist_constraint_part_get_swing_twist(TestCtx &ctx) {
    // q = swing * twist with the swing about an axis in the yz plane and the twist about x
    const Quatf swing = axis_angle(vec3f::make(0.0f, 0.6f, 0.8f), 0.7f);
    const Quatf twist = axis_angle(vec3f::axis_x(), 0.5f);
    const Quatf q = quatf::multiply(swing, twist);

    Quatf out_swing, out_twist;
    swing_twist_constraint_part::get_swing_twist(out_swing, out_twist, q);
    CHECK(ctx, quatf::x(out_swing) == 0.0f && quatf::y(out_twist) == 0.0f && quatf::z(out_twist) == 0.0f);
    CHECK(ctx, quat_near(out_swing, swing, 1e-6f));
    CHECK(ctx, quat_near(out_twist, twist, 1e-6f));
    CHECK(ctx, quat_near(quatf::multiply(out_swing, out_twist), q, 1e-6f));

    // a half turn about y has no x or w: all swing
    swing_twist_constraint_part::get_swing_twist(out_swing, out_twist, quatf::make(0.0f, 1.0f, 0.0f, 0.0f));
    CHECK(ctx, quatf::exact_equals(out_twist, quatf::identity()));
    CHECK(ctx, quatf::exact_equals(out_swing, quatf::make(0.0f, 1.0f, 0.0f, 0.0f)));
}

void test_swing_twist_constraint_part_clamp_twist(TestCtx &ctx) {
    SwingTwistConstraintPart part = swing_twist_constraint_part::create();
    swing_twist_constraint_part::set_limits(part, -0.5f, 0.5f, -PI, PI, -PI, PI);
    const Quatf twist_max = quatf::make(scalar::sin(0.25f), 0.0f, 0.0f, scalar::cos(0.25f));
    const Quatf twist_min = quatf::make(scalar::sin(-0.25f), 0.0f, 0.0f, scalar::cos(-0.25f));

    Quatf swing = quatf::identity();
    Quatf twist = axis_angle(vec3f::axis_x(), 0.8f);
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(part, swing, twist) == swing_twist_constraint_part::TWIST_MAX);
    CHECK(ctx, quatf::exact_equals(twist, twist_max));

    twist = axis_angle(vec3f::axis_x(), -0.8f);
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(part, swing, twist) == swing_twist_constraint_part::TWIST_MIN);
    CHECK(ctx, quatf::exact_equals(twist, twist_min));

    // within: untouched
    twist = axis_angle(vec3f::axis_x(), 0.3f);
    const Quatf inside = twist;
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(part, swing, twist) == 0);
    CHECK(ctx, quatf::exact_equals(twist, inside));

    // the same rotation with negative w clamps the same and keeps its sign
    twist = quatf::scale(axis_angle(vec3f::axis_x(), 0.8f), -1.0f);
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(part, swing, twist) == swing_twist_constraint_part::TWIST_MAX);
    CHECK(ctx, quatf::x(twist) == -quatf::x(twist_max) && quatf::w(twist) == -quatf::w(twist_max));

    // locked twist clamps any twist to identity
    swing_twist_constraint_part::set_limits(part, 0.0f, 0.0f, -PI, PI, -PI, PI);
    twist = axis_angle(vec3f::axis_x(), 0.1f);
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(part, swing, twist) ==
                   (swing_twist_constraint_part::TWIST_MIN | swing_twist_constraint_part::TWIST_MAX));
    CHECK(ctx, quatf::exact_equals(twist, quatf::identity()));
}

void test_swing_twist_constraint_part_clamp_swing(TestCtx &ctx) {
    Quatf twist = quatf::identity();

    // cone of 0.4: a 0.6 swing about y lands on the cone's edge in the same direction
    SwingTwistConstraintPart cone = make_cone(0.4f);
    Quatf swing = axis_angle(vec3f::axis_y(), 0.6f);
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(cone, swing, twist) == ALL_SWING);
    CHECK(ctx, quat_near(swing, axis_angle(vec3f::axis_y(), 0.4f), 1e-5f));

    // inside the cone: untouched
    swing = axis_angle(vec3f::make(0.0f, 1.0f, 1.0f), 0.3f);
    const Quatf inside = swing;
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(cone, swing, twist) == 0);
    CHECK(ctx, quatf::exact_equals(swing, inside));

    // an elliptic cone (0.8 about y, 0.4 about z): a diagonal swing is projected onto the ellipse, unit length
    SwingTwistConstraintPart ellipse = swing_twist_constraint_part::create();
    swing_twist_constraint_part::set_limits(ellipse, -PI, PI, -0.8f, 0.8f, -0.4f, 0.4f);
    swing = axis_angle(vec3f::make(0.0f, 1.0f, 1.0f), 1.0f);
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(ellipse, swing, twist) == ALL_SWING);
    const float ey = quatf::y(swing) / scalar::sin(0.4f);
    const float ez = quatf::z(swing) / scalar::sin(0.2f);
    CHECK_NEAR(ctx, ey * ey + ez * ez, 1.0f, 1e-5f);
    CHECK_NEAR(ctx, quatf::length(swing), 1.0f, 1e-6f);
    CHECK(ctx, quatf::x(swing) == 0.0f && quatf::y(swing) > 0.0f && quatf::z(swing) > 0.0f);

    // pyramid: y in [-0.2, 0.6], z in [-0.4, 0.4]; y = -0.5 is clamped to -0.2, z within is kept
    SwingTwistConstraintPart pyramid = swing_twist_constraint_part::create();
    pyramid.swing_type = swing_twist_constraint_part::PYRAMID;
    swing_twist_constraint_part::set_limits(pyramid, -PI, PI, -0.2f, 0.6f, -0.4f, 0.4f);
    swing = axis_angle(vec3f::axis_y(), -0.5f);
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(pyramid, swing, twist) == ALL_SWING);
    CHECK_NEAR(ctx, 2.0f * scalar::atan2(quatf::y(swing), quatf::w(swing)), -0.2f, 1e-6f);
    CHECK_NEAR(ctx, quatf::z(swing), 0.0f, 1e-7f);
    swing = axis_angle(vec3f::axis_y(), 0.5f);
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(pyramid, swing, twist) == 0);

    // swing z locked, y limited to 0.4: a y swing past it clamps to the max side only
    SwingTwistConstraintPart z_locked = swing_twist_constraint_part::create();
    swing_twist_constraint_part::set_limits(z_locked, -PI, PI, -0.4f, 0.4f, 0.0f, 0.0f);
    swing = axis_angle(vec3f::axis_y(), 0.6f);
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(z_locked, swing, twist) == swing_twist_constraint_part::SWING_Y_MAX);
    CHECK(ctx, quatf::exact_equals(swing, quatf::make(0.0f, scalar::sin(0.2f), 0.0f, scalar::cos(0.2f))));
}

void test_swing_twist_constraint_part_cone_velocity_limit(TestCtx &ctx) {
    // a static a and a dynamic b turned 0.6 about z, past a 0.4 cone. constraint space is world space and a sits at
    // identity, so the constraint rotation is b's rotation
    RigidBody a = make_body(motion_type::MOTION_STATIC, 1.0f, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    b.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.6f);
    SwingTwistConstraintPart part = make_cone(0.4f);
    swing_twist_constraint_part::calculate_constraint_properties(part, a, b, quatf::load(b.quaternion), quatf::identity());

    // one limit part, about the axis from the clamped swing to the actual one (+z), and no twist
    CHECK(ctx, angle_constraint_part::is_active(part.swing_limit_y_constraint_part));
    CHECK(ctx, !angle_constraint_part::is_active(part.swing_limit_z_constraint_part));
    CHECK(ctx, !angle_constraint_part::is_active(part.twist_limit_constraint_part));
    CHECK(ctx, vec3f::distance(part.world_space_swing_limit_y_rotation_axis, vec3f::make(0.0f, 0.0f, 1.0f)) < 1e-6f);

    // spinning further out of the cone: that spin is removed, the rest kept
    b.motion_properties.angular_velocity = {0.3f, -0.2f, 1.0f};
    CHECK(ctx, swing_twist_constraint_part::solve_velocity_constraint(part, a, b));
    CHECK(ctx, vec3f::distance(vec3f::load(b.motion_properties.angular_velocity), vec3f::make(0.3f, -0.2f, 0.0f)) < 1e-6f);
    CHECK_NEAR(ctx, swing_twist_constraint_part::get_total_swing_y_lambda(part), -1.0f, 1e-6f);
    CHECK(ctx, swing_twist_constraint_part::get_total_swing_z_lambda(part) == 0.0f &&
                   swing_twist_constraint_part::get_total_twist_lambda(part) == 0.0f);

    // spinning back into the cone, a fresh step (no accumulated impulse to give back): the limit only pushes, so
    // nothing is applied
    part = make_cone(0.4f);
    swing_twist_constraint_part::calculate_constraint_properties(part, a, b, quatf::load(b.quaternion), quatf::identity());
    b.motion_properties.angular_velocity = {0.3f, -0.2f, -1.0f};
    CHECK(ctx, !swing_twist_constraint_part::solve_velocity_constraint(part, a, b));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(b.motion_properties.angular_velocity), vec3f::make(0.3f, -0.2f, -1.0f)));

    // within the cone the limit is inactive and any spin is untouched
    b.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.2f);
    swing_twist_constraint_part::calculate_constraint_properties(part, a, b, quatf::load(b.quaternion), quatf::identity());
    CHECK(ctx, !swing_twist_constraint_part::is_active(part));
    b.motion_properties.angular_velocity = {0.0f, 0.0f, 1.0f};
    CHECK(ctx, !swing_twist_constraint_part::solve_velocity_constraint(part, a, b));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(b.motion_properties.angular_velocity), vec3f::make(0.0f, 0.0f, 1.0f)));
}

void test_swing_twist_constraint_part_locked_twist_velocity(TestCtx &ctx) {
    // twist locked: always active, removes spin about the twist axis either way
    RigidBody a = make_body(motion_type::MOTION_STATIC, 1.0f, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    SwingTwistConstraintPart part = swing_twist_constraint_part::create();
    swing_twist_constraint_part::set_limits(part, 0.0f, 0.0f, -PI, PI, -PI, PI);

    for (int sign = -1; sign <= 1; sign += 2) {
        swing_twist_constraint_part::calculate_constraint_properties(part, a, b, quatf::load(b.quaternion), quatf::identity());
        CHECK(ctx, angle_constraint_part::is_active(part.twist_limit_constraint_part));
        CHECK(ctx, vec3f::distance(part.world_space_twist_limit_rotation_axis, vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-6f);
        b.motion_properties.angular_velocity = {(float)sign, 0.5f, 0.0f};
        CHECK(ctx, swing_twist_constraint_part::solve_velocity_constraint(part, a, b));
        CHECK(ctx, vec3f::distance(vec3f::load(b.motion_properties.angular_velocity), vec3f::make(0.0f, 0.5f, 0.0f)) < 1e-6f);
    }

    // warm start carries the scaled impulse
    part.twist_limit_constraint_part.total_lambda = 1.0f;
    b.motion_properties.angular_velocity = {0.0f, 0.0f, 0.0f};
    swing_twist_constraint_part::warm_start(part, a, b, 0.5f);
    CHECK(ctx, part.twist_limit_constraint_part.total_lambda == 0.5f);
    CHECK(ctx, vec3f::distance(vec3f::load(b.motion_properties.angular_velocity), vec3f::make(0.5f, 0.0f, 0.0f)) < 1e-6f);
}

void test_swing_twist_constraint_part_position_returns_to_cone(TestCtx &ctx) {
    // b past the 0.4 cone at 0.6 about z: one baumgarte 1 step brings it back to the edge (static a takes nothing)
    RigidBody a = make_body(motion_type::MOTION_STATIC, 1.0f, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 2.0f, 3.0f});
    b.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.6f);
    SwingTwistConstraintPart part = make_cone(0.4f);
    CHECK(ctx, swing_twist_constraint_part::solve_position_constraint(part, a, b, quatf::load(b.quaternion), quatf::identity(), quatf::identity(), 1.0f));
    // the error is linearised (2 sin(0.1) for a 0.2 overshoot), so the step lands within a few 1e-4 of the edge
    CHECK_NEAR(ctx, 2.0f * scalar::atan2(b.quaternion.z, b.quaternion.w), 0.4f, 1e-3f);
    CHECK_NEAR(ctx, b.quaternion.x, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, b.quaternion.y, 0.0f, 1e-6f);
    CHECK(ctx, quatf::exact_equals(quatf::load(a.quaternion), quatf::identity()));

    // within the cone nothing is corrected
    b.quaternion = stored_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), 0.3f);
    const Quatf inside = quatf::load(b.quaternion);
    CHECK(ctx, !swing_twist_constraint_part::solve_position_constraint(part, a, b, inside, quatf::identity(), quatf::identity(), 1.0f));
    CHECK(ctx, quatf::exact_equals(quatf::load(b.quaternion), inside));
}

void test_swing_twist_constraint_part_tiny_overshoot_activates_limit(TestCtx &ctx) {
    // a pyramid swung 4e-7 rad past its 0.5 rad limit about y: the clamp puts the half angle back at 0.25, so the
    // axis from the clamped swing to the swing has length about 4e-7, and it is still a limit
    RigidBody a = make_body(motion_type::MOTION_STATIC, 1.0f, {0.0f, 0.0f, 0.0f});
    RigidBody b = make_body(motion_type::MOTION_DYNAMIC, 1.0f, {1.0f, 1.0f, 1.0f});
    SwingTwistConstraintPart part = swing_twist_constraint_part::create();
    part.swing_type = swing_twist_constraint_part::PYRAMID;
    swing_twist_constraint_part::set_limits(part, -0.5f, 0.5f, -0.5f, 0.5f, -0.5f, 0.5f);
    b.quaternion = stored_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), 0.5f + 4e-7f);
    swing_twist_constraint_part::calculate_constraint_properties(part, a, b, quatf::load(b.quaternion), quatf::identity());
    CHECK(ctx, angle_constraint_part::is_active(part.swing_limit_y_constraint_part));
    CHECK(ctx, !angle_constraint_part::is_active(part.swing_limit_z_constraint_part));
    // both x axes lie in the xz plane, so desired x current is exactly along +y
    CHECK(ctx, vec3f::distance(part.world_space_swing_limit_y_rotation_axis, vec3f::make(0.0f, 1.0f, 0.0f)) < 1e-6f);
}

void test_swing_twist_constraint_part_eccentric_cone_clamp_converges(TestCtx &ctx) {
    // y just wider than the locked angle, z nearly free: the narrowest ellipse set_limits allows. a swing far
    // outside it is clamped on to it; newton exits once g(t) = (y/a)^2 + (z/b)^2 - 1 is under 1e-6
    SwingTwistConstraintPart part = swing_twist_constraint_part::create();
    const float deg = PI / 180.0f;
    swing_twist_constraint_part::set_limits(part, -0.5f, 0.5f, -0.6f * deg, 0.6f * deg, -170.0f * deg, 170.0f * deg);
    Quatf swing = quatf::make(0.0f, 0.6f, 0.3f, scalar::sqrt(1.0f - 0.36f - 0.09f));
    Quatf twist = quatf::identity();
    CHECK(ctx, swing_twist_constraint_part::clamp_swing_twist(part, swing, twist) == ALL_SWING);
    const float y = quatf::y(swing) / part.sin_swing_y_half_max_angle;
    const float z = quatf::z(swing) / part.sin_swing_z_half_max_angle;
    CHECK(ctx, scalar::abs(y * y + z * z - 1.0f) < 2e-6f);
    CHECK_NEAR(ctx, quatf::length(swing), 1.0f, 1e-6f);
}
