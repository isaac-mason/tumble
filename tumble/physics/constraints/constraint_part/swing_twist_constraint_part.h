#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/constraints/constraint_part/angle_constraint_part.h"

TUMBLE_NAMESPACE_BEGIN

struct RigidBody;

// a rotation split into swing and twist, each kept within its limits

/** how the swing limit behaves */
enum class SwingType : uint32_t {
    /** swing is limited by a cone shape (symmetric around 0) */
    CONE = 0,
    /** swing is limited by a pyramid shape (asymmetric limits supported) */
    PYRAMID = 1,
};

/**
 * decomposes rotation into swing and twist components and constrains them within limits.
 *
 * quaternion decomposition: q = q_swing * q_twist, where q_swing.x = 0 and q_twist.y = q_twist.z = 0
 *
 * - twist (rotation around x) is within [twist_min_angle, twist_max_angle]
 * - swing (rotation around y and z) is limited by an ellipsoid (cone) or pyramid
 */
struct SwingTwistConstraintPart {
    /** flags for the axes clamp_swing_twist clamped */
    enum ClampedAxis : uint32_t {
        TWIST_MIN = 1u << 0,
        TWIST_MAX = 1u << 1,
        SWING_Y_MIN = 1u << 2,
        SWING_Y_MAX = 1u << 3,
        SWING_Z_MIN = 1u << 4,
        SWING_Z_MAX = 1u << 5,
    };

    /** flags indicating rotation state */
    enum RotationFlags : uint32_t {
        TWIST_X_LOCKED = 1u << 0,
        SWING_Y_LOCKED = 1u << 1,
        SWING_Z_LOCKED = 1u << 2,
        TWIST_X_FREE = 1u << 3,
        SWING_Y_FREE = 1u << 4,
        SWING_Z_FREE = 1u << 5,
        SWING_YZ_FREE = (1u << 4) | (1u << 5),
    };

    SwingType swing_type;
    uint32_t rotation_flags;

    // twist limits (sin/cos of half angles)
    float sin_twist_half_min_angle;
    float sin_twist_half_max_angle;
    float cos_twist_half_min_angle;
    float cos_twist_half_max_angle;

    // swing y limits (half angles and their sin/cos)
    float swing_y_half_min_angle;
    float swing_y_half_max_angle;
    float sin_swing_y_half_min_angle;
    float sin_swing_y_half_max_angle;
    float cos_swing_y_half_min_angle;
    float cos_swing_y_half_max_angle;

    // swing z limits (half angles and their sin/cos)
    float swing_z_half_min_angle;
    float swing_z_half_max_angle;
    float sin_swing_z_half_min_angle;
    float sin_swing_z_half_max_angle;
    float cos_swing_z_half_min_angle;
    float cos_swing_z_half_max_angle;

    // world space rotation axes
    Vec3f world_space_swing_limit_y_rotation_axis;
    Vec3f world_space_swing_limit_z_rotation_axis;
    Vec3f world_space_twist_limit_rotation_axis;

    // a part for each dof
    AngleConstraintPart swing_limit_y_constraint_part;
    AngleConstraintPart swing_limit_z_constraint_part;
    AngleConstraintPart twist_limit_constraint_part;
};

namespace swing_twist_constraint_part {

inline SwingTwistConstraintPart create() {
    SwingTwistConstraintPart part = {};
    part.swing_type = SwingType::CONE;
    part.cos_twist_half_min_angle = 1.0f;
    part.cos_twist_half_max_angle = 1.0f;
    part.cos_swing_y_half_min_angle = 1.0f;
    part.cos_swing_y_half_max_angle = 1.0f;
    part.cos_swing_z_half_min_angle = 1.0f;
    part.cos_swing_z_half_max_angle = 1.0f;
    return part;
}

inline void deactivate(SwingTwistConstraintPart &part) {
    angle_constraint_part::deactivate(part.swing_limit_y_constraint_part);
    angle_constraint_part::deactivate(part.swing_limit_z_constraint_part);
    angle_constraint_part::deactivate(part.twist_limit_constraint_part);
}

/** true if any of the three parts is active */
inline bool is_active(const SwingTwistConstraintPart &part) {
    return angle_constraint_part::is_active(part.swing_limit_y_constraint_part) ||
           angle_constraint_part::is_active(part.swing_limit_z_constraint_part) ||
           angle_constraint_part::is_active(part.twist_limit_constraint_part);
}

/** set the limits, every angle in radians in [-pi, pi] */
void set_limits(SwingTwistConstraintPart &part, float twist_min_angle, float twist_max_angle, float swing_y_min_angle,
                float swing_y_max_angle, float swing_z_min_angle, float swing_z_max_angle);

/** decompose q = swing * twist where swing.x = 0 and twist.y = twist.z = 0 */
void get_swing_twist(Quatf &out_swing, Quatf &out_twist, Quatf q);

/** clamp swing and twist (constraint space) against the limits, in place. returns the ClampedAxis flags */
uint32_t clamp_swing_twist(const SwingTwistConstraintPart &part, Quatf &io_swing, Quatf &io_twist);

/**
 * constraint_rotation is the current rotation of the constraint in constraint space, constraint_to_world
 * rotates from constraint space to world space
 */
void calculate_constraint_properties(SwingTwistConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                                     Quatf constraint_rotation, Quatf constraint_to_world);

void warm_start(SwingTwistConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                float warm_start_impulse_ratio);

/** true if any impulse was applied */
bool solve_velocity_constraint(SwingTwistConstraintPart &part, RigidBody &body_a, RigidBody &body_b);

/**
 * constraint_rotation is the current rotation in constraint space, constraint_to_body1 / constraint_to_body2
 * rotate from constraint space to each body's space. true if any correction was applied
 */
bool solve_position_constraint(SwingTwistConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                               Quatf constraint_rotation, Quatf constraint_to_body1, Quatf constraint_to_body2,
                               float baumgarte);

float get_total_swing_y_lambda(const SwingTwistConstraintPart &part);
float get_total_swing_z_lambda(const SwingTwistConstraintPart &part);
float get_total_twist_lambda(const SwingTwistConstraintPart &part);

}  // namespace swing_twist_constraint_part

TUMBLE_NAMESPACE_END
