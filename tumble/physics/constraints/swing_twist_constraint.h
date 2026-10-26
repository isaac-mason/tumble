#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/constraints/constraint_part/angle_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/motor_settings.h"
#include "tumble/physics/constraints/constraint_part/point_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/swing_twist_constraint_part.h"
#include "tumble/physics/constraints/constraints.h"

TUMBLE_NAMESPACE_BEGIN

struct RigidBody;

// a point on each body held together, the rotation limited apart in swing (about y and z, a cone or a pyramid) and
// twist (about x), with optional friction and swing and twist motors: a ragdoll joint

struct SwingTwistConstraint {
    ConstraintBase base;
    /** attachment point on body 1 relative to its center of mass, in body space */
    Vec3f local_space_position1;
    /** attachment point on body 2 relative to its center of mass, in body space */
    Vec3f local_space_position2;

    /** rotation from constraint space to body 1 space */
    Quatf constraint_to_body1;
    /** rotation from constraint space to body 2 space */
    Quatf constraint_to_body2;

    /** swing limit about the z axis (radians) */
    float normal_half_cone_angle;
    /** swing limit about the y axis (radians) */
    float plane_half_cone_angle;
    /** twist limits (radians), in [-pi, pi] */
    float twist_min_angle;
    float twist_max_angle;

    /** maximum friction torque (N m) while not driven by a motor */
    float max_friction_torque;

    MotorState swing_motor_state;
    MotorState twist_motor_state;
    /** target angular velocity in constraint space of body 2 */
    Vec3f target_angular_velocity;
    /** target orientation in constraint space */
    Quatf target_orientation;
    MotorSettings swing_motor_settings;
    MotorSettings twist_motor_settings;

    /** world space motor axes (x, y, z) */
    Vec3f world_space_motor_axis[3];

    /** point constraint for translation (3 DOF) */
    PointConstraintPart point_constraint_part;
    /** the rotation limits */
    SwingTwistConstraintPart swing_twist_constraint_part;
    /** motors: twist = 0, swing y = 1, swing z = 2 */
    AngleConstraintPart motor_constraint_parts[3];
};

struct SwingTwistConstraintSettings {
    /** pivot point on body 1 */
    Float3 point_a = {0.0f, 0.0f, 0.0f};
    /** pivot point on body 2 */
    Float3 point_b = {0.0f, 0.0f, 0.0f};
    /** twist axis on body 1, the x axis of constraint space (normalized on create) */
    Float3 twist_axis_a = {1.0f, 0.0f, 0.0f};
    /** plane axis on body 1, the z axis of constraint space, perpendicular to the twist axis (normalized on create) */
    Float3 plane_axis_a = {0.0f, 0.0f, 1.0f};
    /** twist axis on body 2 (normalized on create) */
    Float3 twist_axis_b = {1.0f, 0.0f, 0.0f};
    /** plane axis on body 2 (normalized on create) */
    Float3 plane_axis_b = {0.0f, 0.0f, 1.0f};
    ConstraintSpace space = ConstraintSpace::WORLD;
    SwingType swing_type = SwingType::CONE;
    /** swing limit about the z axis (radians) */
    float normal_half_cone_angle = 0.0f;
    /** swing limit about the y axis (radians) */
    float plane_half_cone_angle = 0.0f;
    float twist_min_angle = 0.0f;
    float twist_max_angle = 0.0f;
    float max_friction_torque = 0.0f;
    MotorSettings swing_motor_settings = {};
    MotorSettings twist_motor_settings = {};
    /** higher numbers are more likely to be solved correctly (solved later) */
    uint32_t constraint_priority = 0;
    /** 0 takes the world's iteration counts */
    uint32_t num_velocity_steps_override = 0;
    uint32_t num_position_steps_override = 0;
};

namespace swing_twist_constraint {

/** a swing twist constraint between two live bodies, linked into both bodies' constraint_ids; INVALID_CONSTRAINT_ID (Error::QUOTA) when the pool is at its most */
ConstraintId create(World &world, RigidBody *body_a,
                                   RigidBody *body_b, const SwingTwistConstraintSettings &settings);
/** null when the id is stale or not a swing twist constraint */
SwingTwistConstraint *get(World &world, ConstraintId id);

void set_normal_half_cone_angle(SwingTwistConstraint &constraint, float angle);
void set_plane_half_cone_angle(SwingTwistConstraint &constraint, float angle);
void set_twist_min_angle(SwingTwistConstraint &constraint, float angle);
void set_twist_max_angle(SwingTwistConstraint &constraint, float angle);
void set_max_friction_torque(SwingTwistConstraint &constraint, float torque);
/** a change of state clears the motor parts' warm start */
void set_swing_motor_state(SwingTwistConstraint &constraint, MotorState state);
void set_twist_motor_state(SwingTwistConstraint &constraint, MotorState state);
/** target angular velocity in constraint space of body 2 */
void set_target_angular_velocity_cs(SwingTwistConstraint &constraint, Vec3f velocity);
/** target orientation in constraint space, clamped to the limits */
void set_target_orientation_cs(SwingTwistConstraint &constraint, Quatf orientation);
/** target orientation in body space: R2 = R1 * orientation */
void set_target_orientation_bs(SwingTwistConstraint &constraint, Quatf orientation);
/** the current rotation of the constraint in constraint space, (q1 c1)^-1 (q2 c2) */
Quatf get_rotation_in_constraint_space(const SwingTwistConstraint &constraint, const Bodies &bodies);

void setup_velocity(SwingTwistConstraint &constraint, Bodies &bodies, float delta_time);
void warm_start_velocity(SwingTwistConstraint &constraint, Bodies &bodies, float warm_start_impulse_ratio);
bool solve_velocity(SwingTwistConstraint &constraint, Bodies &bodies, float delta_time);
bool solve_position(SwingTwistConstraint &constraint, Bodies &bodies, float delta_time, float baumgarte);
/** the body's centre of mass moved by delta_com in its local space: the point on it follows, so the joint holds where it was */
void notify_shape_changed(SwingTwistConstraint &constraint, BodyId body_id, Vec3f delta_com);
void reset_warm_start(SwingTwistConstraint &constraint);
void get_iteration_overrides(Constraints::IterationOverrides &out, const SwingTwistConstraint &constraint);
void get_sort_fields(Constraints::SortFields &out, const SwingTwistConstraint &constraint);

Vec3f get_total_lambda_position(const SwingTwistConstraint &constraint);
float get_total_lambda_twist(const SwingTwistConstraint &constraint);
float get_total_lambda_swing_y(const SwingTwistConstraint &constraint);
float get_total_lambda_swing_z(const SwingTwistConstraint &constraint);
Vec3f get_total_lambda_motor(const SwingTwistConstraint &constraint);

}  // namespace swing_twist_constraint

TUMBLE_NAMESPACE_END
