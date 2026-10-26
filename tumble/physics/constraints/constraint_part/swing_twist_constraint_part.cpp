#include "tumble/physics/constraints/constraint_part/swing_twist_constraint_part.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec2f.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/constraint_part/rotation_euler_constraint_part.h"

TUMBLE_NAMESPACE_BEGIN

namespace swing_twist_constraint_part {

void set_limits(SwingTwistConstraintPart &part, float twist_min_angle, float twist_max_angle, float swing_y_min_angle,
                float swing_y_max_angle, float swing_z_min_angle, float swing_z_max_angle) {
    const float locked_angle = 0.5f * scalar::DEGREES_TO_RADIANS;
    const float free_angle = 179.5f * scalar::DEGREES_TO_RADIANS;

    const float half_twist_min = 0.5f * twist_min_angle;
    const float half_twist_max = 0.5f * twist_max_angle;

    const float half_swing_y_min = 0.5f * swing_y_min_angle;
    const float half_swing_y_max = 0.5f * swing_y_max_angle;
    const float half_swing_z_min = 0.5f * swing_z_min_angle;
    const float half_swing_z_max = 0.5f * swing_z_max_angle;

    // half angles for the pyramid limit
    part.swing_y_half_min_angle = half_swing_y_min;
    part.swing_y_half_max_angle = half_swing_y_max;
    part.swing_z_half_min_angle = half_swing_z_min;
    part.swing_z_half_max_angle = half_swing_z_max;

    part.rotation_flags = 0;

    // twist
    if (twist_min_angle > -locked_angle && twist_max_angle < locked_angle) {
        part.rotation_flags |= SwingTwistConstraintPart::RotationFlags::TWIST_X_LOCKED;
        part.sin_twist_half_min_angle = 0.0f;
        part.sin_twist_half_max_angle = 0.0f;
        part.cos_twist_half_min_angle = 1.0f;
        part.cos_twist_half_max_angle = 1.0f;
    } else if (twist_min_angle < -free_angle && twist_max_angle > free_angle) {
        part.rotation_flags |= SwingTwistConstraintPart::RotationFlags::TWIST_X_FREE;
        part.sin_twist_half_min_angle = -1.0f;
        part.sin_twist_half_max_angle = 1.0f;
        part.cos_twist_half_min_angle = 0.0f;
        part.cos_twist_half_max_angle = 0.0f;
    } else {
        part.sin_twist_half_min_angle = scalar::sin(half_twist_min);
        part.sin_twist_half_max_angle = scalar::sin(half_twist_max);
        part.cos_twist_half_min_angle = scalar::cos(half_twist_min);
        part.cos_twist_half_max_angle = scalar::cos(half_twist_max);
    }

    // swing y
    if (swing_y_min_angle > -locked_angle && swing_y_max_angle < locked_angle) {
        part.rotation_flags |= SwingTwistConstraintPart::RotationFlags::SWING_Y_LOCKED;
        part.sin_swing_y_half_min_angle = 0.0f;
        part.sin_swing_y_half_max_angle = 0.0f;
        part.cos_swing_y_half_min_angle = 1.0f;
        part.cos_swing_y_half_max_angle = 1.0f;
    } else if (swing_y_min_angle < -free_angle && swing_y_max_angle > free_angle) {
        part.rotation_flags |= SwingTwistConstraintPart::RotationFlags::SWING_Y_FREE;
        part.sin_swing_y_half_min_angle = -1.0f;
        part.sin_swing_y_half_max_angle = 1.0f;
        part.cos_swing_y_half_min_angle = 0.0f;
        part.cos_swing_y_half_max_angle = 0.0f;
    } else {
        part.sin_swing_y_half_min_angle = scalar::sin(half_swing_y_min);
        part.sin_swing_y_half_max_angle = scalar::sin(half_swing_y_max);
        part.cos_swing_y_half_min_angle = scalar::cos(half_swing_y_min);
        part.cos_swing_y_half_max_angle = scalar::cos(half_swing_y_max);
    }

    // swing z
    if (swing_z_min_angle > -locked_angle && swing_z_max_angle < locked_angle) {
        part.rotation_flags |= SwingTwistConstraintPart::RotationFlags::SWING_Z_LOCKED;
        part.sin_swing_z_half_min_angle = 0.0f;
        part.sin_swing_z_half_max_angle = 0.0f;
        part.cos_swing_z_half_min_angle = 1.0f;
        part.cos_swing_z_half_max_angle = 1.0f;
    } else if (swing_z_min_angle < -free_angle && swing_z_max_angle > free_angle) {
        part.rotation_flags |= SwingTwistConstraintPart::RotationFlags::SWING_Z_FREE;
        part.sin_swing_z_half_min_angle = -1.0f;
        part.sin_swing_z_half_max_angle = 1.0f;
        part.cos_swing_z_half_min_angle = 0.0f;
        part.cos_swing_z_half_max_angle = 0.0f;
    } else {
        part.sin_swing_z_half_min_angle = scalar::sin(half_swing_z_min);
        part.sin_swing_z_half_max_angle = scalar::sin(half_swing_z_max);
        part.cos_swing_z_half_min_angle = scalar::cos(half_swing_z_min);
        part.cos_swing_z_half_max_angle = scalar::cos(half_swing_z_max);
    }
}

/** whether we're closer to the min or the max limit */
static bool distance_to_min_shorter(float delta_min, float delta_max) {
    // a swing/twist of -1 and 1 are the same angle: past 1 the short way is the other way round
    float abs_min = scalar::abs(delta_min);
    if (abs_min > 1.0f) abs_min = 2.0f - abs_min;
    float abs_max = scalar::abs(delta_max);
    if (abs_max > 1.0f) abs_max = 2.0f - abs_max;
    return abs_min < abs_max;
}

void get_swing_twist(Quatf &out_swing, Quatf &out_twist, Quatf q) {
    const float x = quatf::x(q);
    const float y = quatf::y(q);
    const float z = quatf::z(q);
    const float w = quatf::w(q);

    const float s = scalar::sqrt(w * w + x * x);
    if (s != 0.0f) {
        out_twist = quatf::make(x / s, 0.0f, 0.0f, w / s);
        out_swing = quatf::make(0.0f, (w * y - x * z) / s, (w * z + x * y) / s, s);
    } else {
        // both x and w zero: a 180 degree rotation around y or z
        out_twist = quatf::identity();
        out_swing = q;
    }
}

/**
 * the closest point on an ellipse to a point outside it, by newton-raphson on the lagrange multiplier.
 *
 * @see rotation joint limits in quaternion space by gino van den bergen, section 10.1 in game engine gems 3.
 */
static void get_closest_point_on_ellipse(Vec2f &out, float px, float py, float a, float b) {
    // degenerate cases
    if (a <= 0.0f) {
        out.x = 0.0f;
        out.y = py >= 0.0f ? b : -b;
        return;
    }
    if (b <= 0.0f) {
        out.x = px >= 0.0f ? a : -a;
        out.y = 0.0f;
        return;
    }

    const float a_sq = a * a;
    const float b_sq = b * b;

    // equation of ellipse: f(x, y) = (x/a)^2 + (y/b)^2 - 1 = 0
    // normal on surface: (df/dx, df/dy) = (2 x / a^2, 2 y / b^2)
    // closest point (x', y') on ellipse to point (x, y): (x', y') + t (x / a^2, y / b^2) = (x, y)
    // <=> (x', y') = (a^2 x / (t + a^2), b^2 y / (t + b^2))
    // requiring the point to be on the ellipse: g(t) = (a x / (t + a^2))^2 + (b y / (t + b^2))^2 - 1 = 0

    // newton raphson iteration, starting at t = 0
    float t = 0.0f;
    for (;;) {
        const float t_plus_a_sq = t + a_sq;
        const float t_plus_b_sq = t + b_sq;
        const float gx = (a * px) / t_plus_a_sq;
        const float gy = (b * py) / t_plus_b_sq;
        const float gt = gx * gx + gy * gy - 1.0f;

        // g(t) close enough to zero
        if (scalar::abs(gt) < 1e-6f) {
            out.x = (a_sq * px) / t_plus_a_sq;
            out.y = (b_sq * py) / t_plus_b_sq;
            return;
        }

        // dg/dt = g'(t) = -2 (a^2 x^2 / (t + a^2)^3 + b^2 y^2 / (t + b^2)^3)
        const float gt_accent = -2.0f * ((a_sq * px * px) / (t_plus_a_sq * t_plus_a_sq * t_plus_a_sq) +
                                         (b_sq * py * py) / (t_plus_b_sq * t_plus_b_sq * t_plus_b_sq));

        // t_n+1 = t_n - g(t) / g'(t)
        t = t - gt / gt_accent;
    }
}

uint32_t clamp_swing_twist(const SwingTwistConstraintPart &part, Quatf &io_swing, Quatf &io_twist) {
    uint32_t clamped_axis = 0;

    // w > 0 for consistent clamping
    const bool negate_swing = quatf::w(io_swing) < 0.0f;
    if (negate_swing) io_swing = quatf::scale(io_swing, -1.0f);
    const bool negate_twist = quatf::w(io_twist) < 0.0f;
    if (negate_twist) io_twist = quatf::scale(io_twist, -1.0f);

    // clamp twist
    if (part.rotation_flags & SwingTwistConstraintPart::RotationFlags::TWIST_X_LOCKED) {
        // twist axis is locked
        if (quatf::x(io_twist) != 0.0f) clamped_axis |= SwingTwistConstraintPart::ClampedAxis::TWIST_MIN | SwingTwistConstraintPart::ClampedAxis::TWIST_MAX;
        io_twist = quatf::identity();
    } else if ((part.rotation_flags & SwingTwistConstraintPart::RotationFlags::TWIST_X_FREE) == 0) {
        // twist has limits
        const float delta_min = part.sin_twist_half_min_angle - quatf::x(io_twist);
        const float delta_max = quatf::x(io_twist) - part.sin_twist_half_max_angle;
        if (delta_min > 0.0f || delta_max > 0.0f) {
            if (distance_to_min_shorter(delta_min, delta_max)) {
                io_twist = quatf::make(part.sin_twist_half_min_angle, 0.0f, 0.0f, part.cos_twist_half_min_angle);
                clamped_axis |= SwingTwistConstraintPart::ClampedAxis::TWIST_MIN;
            } else {
                io_twist = quatf::make(part.sin_twist_half_max_angle, 0.0f, 0.0f, part.cos_twist_half_max_angle);
                clamped_axis |= SwingTwistConstraintPart::ClampedAxis::TWIST_MAX;
            }
        }
    }

    // clamp swing
    if (part.rotation_flags & SwingTwistConstraintPart::RotationFlags::SWING_Y_LOCKED) {
        if (part.rotation_flags & SwingTwistConstraintPart::RotationFlags::SWING_Z_LOCKED) {
            // both swing y and z are locked
            if (quatf::y(io_swing) != 0.0f) clamped_axis |= SwingTwistConstraintPart::ClampedAxis::SWING_Y_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Y_MAX;
            if (quatf::z(io_swing) != 0.0f) clamped_axis |= SwingTwistConstraintPart::ClampedAxis::SWING_Z_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Z_MAX;
            io_swing = quatf::identity();
        } else {
            // only swing y is locked
            if (quatf::y(io_swing) != 0.0f) clamped_axis |= SwingTwistConstraintPart::ClampedAxis::SWING_Y_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Y_MAX;
            const float delta_min = part.sin_swing_z_half_min_angle - quatf::z(io_swing);
            const float delta_max = quatf::z(io_swing) - part.sin_swing_z_half_max_angle;
            if (delta_min > 0.0f || delta_max > 0.0f) {
                if (distance_to_min_shorter(delta_min, delta_max)) {
                    io_swing = quatf::make(0.0f, 0.0f, part.sin_swing_z_half_min_angle, part.cos_swing_z_half_min_angle);
                    clamped_axis |= SwingTwistConstraintPart::ClampedAxis::SWING_Z_MIN;
                } else {
                    io_swing = quatf::make(0.0f, 0.0f, part.sin_swing_z_half_max_angle, part.cos_swing_z_half_max_angle);
                    clamped_axis |= SwingTwistConstraintPart::ClampedAxis::SWING_Z_MAX;
                }
            } else if (clamped_axis & SwingTwistConstraintPart::ClampedAxis::SWING_Y_MIN) {
                const float z = quatf::z(io_swing);
                io_swing = quatf::make(0.0f, 0.0f, z, scalar::sqrt(1.0f - z * z));
            }
        }
    } else if (part.rotation_flags & SwingTwistConstraintPart::RotationFlags::SWING_Z_LOCKED) {
        // only swing z is locked
        if (quatf::z(io_swing) != 0.0f) clamped_axis |= SwingTwistConstraintPart::ClampedAxis::SWING_Z_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Z_MAX;
        const float delta_min = part.sin_swing_y_half_min_angle - quatf::y(io_swing);
        const float delta_max = quatf::y(io_swing) - part.sin_swing_y_half_max_angle;
        if (delta_min > 0.0f || delta_max > 0.0f) {
            if (distance_to_min_shorter(delta_min, delta_max)) {
                io_swing = quatf::make(0.0f, part.sin_swing_y_half_min_angle, 0.0f, part.cos_swing_y_half_min_angle);
                clamped_axis |= SwingTwistConstraintPart::ClampedAxis::SWING_Y_MIN;
            } else {
                io_swing = quatf::make(0.0f, part.sin_swing_y_half_max_angle, 0.0f, part.cos_swing_y_half_max_angle);
                clamped_axis |= SwingTwistConstraintPart::ClampedAxis::SWING_Y_MAX;
            }
        } else if (clamped_axis & SwingTwistConstraintPart::ClampedAxis::SWING_Z_MIN) {
            const float y = quatf::y(io_swing);
            io_swing = quatf::make(0.0f, y, 0.0f, scalar::sqrt(1.0f - y * y));
        }
    } else {
        // two degrees of freedom in swing
        if (part.swing_type == SwingType::CONE) {
            // ellipse limit: inside when (y/a)^2 + (z/b)^2 <= 1
            const float y = quatf::y(io_swing);
            const float z = quatf::z(io_swing);
            const float a = part.sin_swing_y_half_max_angle;
            const float b = part.sin_swing_z_half_max_angle;
            if (a > 0.0f && b > 0.0f) {
                const float ellipse_value = (y * y) / (a * a) + (z * z) / (b * b);
                if (ellipse_value > 1.0f) {
                    // project to the ellipse boundary
                    Vec2f closest;
                    get_closest_point_on_ellipse(closest, y, z, a, b);
                    const float new_w = scalar::sqrt(scalar::max(0.0f, 1.0f - closest.x * closest.x - closest.y * closest.y));
                    io_swing = quatf::make(0.0f, closest.x, closest.y, new_w);
                    clamped_axis |= SwingTwistConstraintPart::ClampedAxis::SWING_Y_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Y_MAX | SwingTwistConstraintPart::ClampedAxis::SWING_Z_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Z_MAX;
                }
            }
        } else {
            // pyramid limit: clamp y and z independently, y/2 = atan2(q.y, q.w) and z/2 = atan2(q.z, q.w)
            const float half_angle_y = scalar::atan2(quatf::y(io_swing), quatf::w(io_swing));
            const float half_angle_z = scalar::atan2(quatf::z(io_swing), quatf::w(io_swing));

            const float clamped_half_y =
                scalar::max(part.swing_y_half_min_angle, scalar::min(part.swing_y_half_max_angle, half_angle_y));
            const float clamped_half_z =
                scalar::max(part.swing_z_half_min_angle, scalar::min(part.swing_z_half_max_angle, half_angle_z));

            if (half_angle_y != clamped_half_y || half_angle_z != clamped_half_z) {
                // rebuild the quaternion without introducing twist: [0, sin_y cos_z, cos_y sin_z, cos_y cos_z] (normalized)
                const float sin_y = scalar::sin(clamped_half_y);
                const float cos_y = scalar::cos(clamped_half_y);
                const float sin_z = scalar::sin(clamped_half_z);
                const float cos_z = scalar::cos(clamped_half_z);
                const float new_y = sin_y * cos_z;
                const float new_z = cos_y * sin_z;
                const float new_w = cos_y * cos_z;
                const float len = scalar::sqrt(new_y * new_y + new_z * new_z + new_w * new_w);
                io_swing = quatf::make(0.0f, new_y / len, new_z / len, new_w / len);
                clamped_axis |= SwingTwistConstraintPart::ClampedAxis::SWING_Y_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Y_MAX | SwingTwistConstraintPart::ClampedAxis::SWING_Z_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Z_MAX;
            }
        }
    }

    // flip sign back
    if (negate_swing) io_swing = quatf::scale(io_swing, -1.0f);
    if (negate_twist) io_twist = quatf::scale(io_twist, -1.0f);

    return clamped_axis;
}

void calculate_constraint_properties(SwingTwistConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                                     Quatf constraint_rotation, Quatf constraint_to_world) {
    const Vec3f axis_x = vec3f::axis_x();
    const Vec3f axis_y = vec3f::axis_y();
    const Vec3f axis_z = vec3f::axis_z();

    // decompose into swing and twist
    Quatf q_swing, q_twist;
    get_swing_twist(q_swing, q_twist, constraint_rotation);

    // clamp against limits
    Quatf q_clamped_swing = q_swing;
    Quatf q_clamped_twist = q_twist;
    const uint32_t clamped_axis = clamp_swing_twist(part, q_clamped_swing, q_clamped_twist);

    Quatf twist_to_world;

    // swing
    if (part.rotation_flags & SwingTwistConstraintPart::RotationFlags::SWING_Y_LOCKED) {
        twist_to_world = quatf::multiply(constraint_to_world, q_swing);
        part.world_space_swing_limit_y_rotation_axis = vec3f::transform_quat(axis_y, twist_to_world);
        part.world_space_swing_limit_z_rotation_axis = vec3f::transform_quat(axis_z, twist_to_world);

        if (part.rotation_flags & SwingTwistConstraintPart::RotationFlags::SWING_Z_LOCKED) {
            // both swing axes locked
            angle_constraint_part::calculate_constraint_properties(part.swing_limit_y_constraint_part, body_a, body_b,
                                                                   part.world_space_swing_limit_y_rotation_axis);
            angle_constraint_part::calculate_constraint_properties(part.swing_limit_z_constraint_part, body_a, body_b,
                                                                   part.world_space_swing_limit_z_rotation_axis);
        } else {
            // only y locked
            angle_constraint_part::calculate_constraint_properties(part.swing_limit_y_constraint_part, body_a, body_b,
                                                                   part.world_space_swing_limit_y_rotation_axis);
            if (clamped_axis & (SwingTwistConstraintPart::ClampedAxis::SWING_Z_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Z_MAX)) {
                // flip the axis at the min limit: the impulse limit is [-max, 0]
                if (clamped_axis & SwingTwistConstraintPart::ClampedAxis::SWING_Z_MIN) {
                    part.world_space_swing_limit_z_rotation_axis = vec3f::negate(part.world_space_swing_limit_z_rotation_axis);
                }
                angle_constraint_part::calculate_constraint_properties(part.swing_limit_z_constraint_part, body_a, body_b,
                                                                       part.world_space_swing_limit_z_rotation_axis);
            } else {
                angle_constraint_part::deactivate(part.swing_limit_z_constraint_part);
            }
        }
    } else if (part.rotation_flags & SwingTwistConstraintPart::RotationFlags::SWING_Z_LOCKED) {
        // only z locked
        twist_to_world = quatf::multiply(constraint_to_world, q_swing);
        part.world_space_swing_limit_y_rotation_axis = vec3f::transform_quat(axis_y, twist_to_world);
        part.world_space_swing_limit_z_rotation_axis = vec3f::transform_quat(axis_z, twist_to_world);

        if (clamped_axis & (SwingTwistConstraintPart::ClampedAxis::SWING_Y_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Y_MAX)) {
            if (clamped_axis & SwingTwistConstraintPart::ClampedAxis::SWING_Y_MIN) {
                part.world_space_swing_limit_y_rotation_axis = vec3f::negate(part.world_space_swing_limit_y_rotation_axis);
            }
            angle_constraint_part::calculate_constraint_properties(part.swing_limit_y_constraint_part, body_a, body_b,
                                                                   part.world_space_swing_limit_y_rotation_axis);
        } else {
            angle_constraint_part::deactivate(part.swing_limit_y_constraint_part);
        }
        angle_constraint_part::calculate_constraint_properties(part.swing_limit_z_constraint_part, body_a, body_b,
                                                               part.world_space_swing_limit_z_rotation_axis);
    } else if ((part.rotation_flags & SwingTwistConstraintPart::RotationFlags::SWING_YZ_FREE) != SwingTwistConstraintPart::RotationFlags::SWING_YZ_FREE) {
        // swing has limits around y and z
        if (clamped_axis & (SwingTwistConstraintPart::ClampedAxis::SWING_Y_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Y_MAX | SwingTwistConstraintPart::ClampedAxis::SWING_Z_MIN | SwingTwistConstraintPart::ClampedAxis::SWING_Z_MAX)) {
            // axis of rotation from clamped swing to swing: desired x current
            twist_to_world = quatf::multiply(constraint_to_world, q_swing);
            const Vec3f current = vec3f::transform_quat(axis_x, twist_to_world);

            twist_to_world = quatf::multiply(constraint_to_world, q_clamped_swing);
            const Vec3f desired = vec3f::transform_quat(axis_x, twist_to_world);

            const Vec3f cross = vec3f::cross(desired, current);
            const float len = vec3f::length(cross);
            if (len != 0.0f) {
                part.world_space_swing_limit_y_rotation_axis = vec3f::scale(cross, 1.0f / len);
                angle_constraint_part::calculate_constraint_properties(part.swing_limit_y_constraint_part, body_a, body_b,
                                                                       part.world_space_swing_limit_y_rotation_axis);
            } else {
                angle_constraint_part::deactivate(part.swing_limit_y_constraint_part);
            }
        } else {
            angle_constraint_part::deactivate(part.swing_limit_y_constraint_part);
        }
        angle_constraint_part::deactivate(part.swing_limit_z_constraint_part);
    } else {
        // no swing limits
        angle_constraint_part::deactivate(part.swing_limit_y_constraint_part);
        angle_constraint_part::deactivate(part.swing_limit_z_constraint_part);
    }

    // twist
    if (part.rotation_flags & SwingTwistConstraintPart::RotationFlags::TWIST_X_LOCKED) {
        // twist locked, always active
        twist_to_world = quatf::multiply(constraint_to_world, q_swing);
        part.world_space_twist_limit_rotation_axis = vec3f::transform_quat(axis_x, twist_to_world);
        angle_constraint_part::calculate_constraint_properties(part.twist_limit_constraint_part, body_a, body_b,
                                                               part.world_space_twist_limit_rotation_axis);
    } else if ((part.rotation_flags & SwingTwistConstraintPart::RotationFlags::TWIST_X_FREE) == 0) {
        // twist has limits
        if (clamped_axis & (SwingTwistConstraintPart::ClampedAxis::TWIST_MIN | SwingTwistConstraintPart::ClampedAxis::TWIST_MAX)) {
            twist_to_world = quatf::multiply(constraint_to_world, q_swing);
            part.world_space_twist_limit_rotation_axis = vec3f::transform_quat(axis_x, twist_to_world);
            if (clamped_axis & SwingTwistConstraintPart::ClampedAxis::TWIST_MIN) {
                part.world_space_twist_limit_rotation_axis = vec3f::negate(part.world_space_twist_limit_rotation_axis);
            }
            angle_constraint_part::calculate_constraint_properties(part.twist_limit_constraint_part, body_a, body_b,
                                                                   part.world_space_twist_limit_rotation_axis);
        } else {
            angle_constraint_part::deactivate(part.twist_limit_constraint_part);
        }
    } else {
        // no twist limits
        angle_constraint_part::deactivate(part.twist_limit_constraint_part);
    }
}

void warm_start(SwingTwistConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                float warm_start_impulse_ratio) {
    angle_constraint_part::warm_start(part.swing_limit_y_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    angle_constraint_part::warm_start(part.swing_limit_z_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    angle_constraint_part::warm_start(part.twist_limit_constraint_part, body_a, body_b, warm_start_impulse_ratio);
}

bool solve_velocity_constraint(SwingTwistConstraintPart &part, RigidBody &body_a, RigidBody &body_b) {
    const float float_max = scalar::FLOAT_MAX;
    bool impulse = false;

    // swing
    if (angle_constraint_part::is_active(part.swing_limit_y_constraint_part)) {
        const float max_lambda = part.sin_swing_y_half_min_angle == part.sin_swing_y_half_max_angle ? float_max : 0.0f;
        impulse = angle_constraint_part::solve_velocity_constraint(part.swing_limit_y_constraint_part, body_a, body_b,
                                                                   part.world_space_swing_limit_y_rotation_axis, -float_max,
                                                                   max_lambda) ||
                  impulse;
    }

    if (angle_constraint_part::is_active(part.swing_limit_z_constraint_part)) {
        const float max_lambda = part.sin_swing_z_half_min_angle == part.sin_swing_z_half_max_angle ? float_max : 0.0f;
        impulse = angle_constraint_part::solve_velocity_constraint(part.swing_limit_z_constraint_part, body_a, body_b,
                                                                   part.world_space_swing_limit_z_rotation_axis, -float_max,
                                                                   max_lambda) ||
                  impulse;
    }

    // twist
    if (angle_constraint_part::is_active(part.twist_limit_constraint_part)) {
        const float max_lambda = part.sin_twist_half_min_angle == part.sin_twist_half_max_angle ? float_max : 0.0f;
        impulse = angle_constraint_part::solve_velocity_constraint(part.twist_limit_constraint_part, body_a, body_b,
                                                                   part.world_space_twist_limit_rotation_axis, -float_max,
                                                                   max_lambda) ||
                  impulse;
    }

    return impulse;
}

bool solve_position_constraint(SwingTwistConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                               Quatf constraint_rotation, Quatf constraint_to_body1, Quatf constraint_to_body2,
                               float baumgarte) {
    Quatf q_swing, q_twist;
    get_swing_twist(q_swing, q_twist, constraint_rotation);

    const uint32_t clamped_axis = clamp_swing_twist(part, q_swing, q_twist);

    // solve rotation violations
    if (clamped_axis != 0) {
        // inv_initial_orientation = constraint_to_body2 * (constraint_to_body1 * q_swing * q_twist)^-1
        const Quatf inv_initial_orientation = quatf::multiply(
            constraint_to_body2, quatf::conjugate(quatf::multiply(quatf::multiply(constraint_to_body1, q_swing), q_twist)));

        // a fresh rotation euler part
        RotationEulerConstraintPart rotation_part = rotation_euler_constraint_part::create();
        rotation_euler_constraint_part::calculate_constraint_properties(rotation_part, body_a, body_b, motion_properties::STEP_STAMP_NONE);

        return rotation_euler_constraint_part::solve_position_constraint(rotation_part, body_a, body_b, inv_initial_orientation,
                                                                         baumgarte);
    }

    return false;
}

float get_total_swing_y_lambda(const SwingTwistConstraintPart &part) { return part.swing_limit_y_constraint_part.total_lambda; }

float get_total_swing_z_lambda(const SwingTwistConstraintPart &part) { return part.swing_limit_z_constraint_part.total_lambda; }

float get_total_twist_lambda(const SwingTwistConstraintPart &part) { return part.twist_limit_constraint_part.total_lambda; }

}  // namespace swing_twist_constraint_part

TUMBLE_NAMESPACE_END
