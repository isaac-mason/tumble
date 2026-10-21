#pragma once

#include <stdint.h>

#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/constraints/constraint_part/angle_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/hinge_rotation_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/motor_settings.h"
#include "tumble/physics/constraints/constraint_part/point_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/spring_settings.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

// rotation only about the hinge axis (3 translational and 2 rotational degrees of freedom removed), with optional angle
// limits, friction and a velocity or position motor
namespace hinge_constraint {

struct HingeConstraint {
    constraints::ConstraintBase base;
    /** attachment point on body 1 relative to its center of mass, in body space */
    Vec3f local_space_position1;
    /** attachment point on body 2 relative to its center of mass, in body space */
    Vec3f local_space_position2;
    /** hinge axis on body 1 in body space (normalized) */
    Vec3f local_space_hinge_axis1;
    /** hinge axis on body 2 in body space (normalized) */
    Vec3f local_space_hinge_axis2;
    /** normal axis perpendicular to the hinge on body 1 (angle reference) */
    Vec3f local_space_normal_axis1;
    /** normal axis perpendicular to the hinge on body 2 (angle reference) */
    Vec3f local_space_normal_axis2;

    /** inverse of the initial relative orientation, for the current angle */
    Quatf inv_initial_orientation;
    /** current hinge angle (radians), cached during setup */
    float theta;
    /** world space hinge axis, cached during setup */
    Vec3f world_space_hinge_axis1;

    bool has_limits;
    /** minimum angle limit (radians) */
    float limits_min;
    /** maximum angle limit (radians) */
    float limits_max;
    /** soft limits when the spring has a frequency or stiffness */
    spring_settings::SpringSettings limits_spring_settings;

    motor_settings::MotorState motor_state;
    /** target angular velocity (rad/s) of the velocity motor */
    float target_angular_velocity;
    /** target angle (radians) of the position motor */
    float target_angle;
    /** maximum friction torque while the motor is off */
    float max_friction_torque;
    motor_settings::MotorSettings motor_settings;

    /** point constraint for translation (3 DOF) */
    point_constraint_part::PointConstraintPart point_constraint_part;
    /** keeps the hinge axes aligned (2 DOF) */
    hinge_rotation_constraint_part::HingeRotationConstraintPart rotation_constraint_part;
    /** angle limits (1 DOF) */
    angle_constraint_part::AngleConstraintPart rotation_limits_constraint_part;
    /** motor (1 DOF) */
    angle_constraint_part::AngleConstraintPart motor_constraint_part;
};

struct HingeConstraintSettings {
    /** pivot point on body a */
    Float3 point_a = {0.0f, 0.0f, 0.0f};
    /** pivot point on body b */
    Float3 point_b = {0.0f, 0.0f, 0.0f};
    /** hinge axis on body a (normalized on create) */
    Float3 hinge_axis_a = {0.0f, 1.0f, 0.0f};
    /** hinge axis on body b (normalized on create) */
    Float3 hinge_axis_b = {0.0f, 1.0f, 0.0f};
    /** normal axis perpendicular to the hinge on body a, the angle reference (normalized on create) */
    Float3 normal_axis_a = {1.0f, 0.0f, 0.0f};
    /** normal axis perpendicular to the hinge on body b (normalized on create) */
    Float3 normal_axis_b = {1.0f, 0.0f, 0.0f};
    constraints::ConstraintSpace space = constraints::SPACE_WORLD;
    /** minimum angle limit in radians, in [-pi, 0]; [-pi, pi] together means no limits */
    float limits_min = -scalar::PI;
    /** maximum angle limit in radians, in [0, pi] */
    float limits_max = scalar::PI;
    spring_settings::SpringSettings limits_spring_settings = {};
    /** maximum friction torque while the motor is off */
    float max_friction_torque = 0.0f;
    motor_settings::MotorSettings motor_settings = {};
    /** higher numbers are more likely to be solved correctly (solved later) */
    uint32_t constraint_priority = 0;
    /** 0 takes the world's iteration counts */
    uint32_t num_velocity_steps_override = 0;
    uint32_t num_position_steps_override = 0;
};

/** a hinge constraint between two live bodies, linked into both bodies' constraint_ids; INVALID_CONSTRAINT_ID (error set) when the pool cannot grow */
constraint_id::ConstraintId create(world::World &world, rigid_body::RigidBody *body_a,
                                   rigid_body::RigidBody *body_b, const HingeConstraintSettings &settings);
/** false for a stale id or one of another type */
bool remove(world::World &world, constraint_id::ConstraintId id);
/** null when the id is stale or not a hinge constraint */
HingeConstraint *get(world::World &world, constraint_id::ConstraintId id);

/** angle limits in radians, min in [-pi, 0] and max in [0, pi]; [-pi, pi] is no limits */
void set_limits(HingeConstraint &constraint, float min, float max);
void set_motor_state(HingeConstraint &constraint, motor_settings::MotorState state);
/** target angular velocity of the velocity motor, rad/s */
void set_target_angular_velocity(HingeConstraint &constraint, float velocity);
/** target angle of the position motor in radians, clamped to the limits */
void set_target_angle(HingeConstraint &constraint, float angle);
/** the hinge angle in radians as of the last setup */
float get_current_angle(const HingeConstraint &constraint);

void setup_velocity(HingeConstraint &constraint, bodies::Bodies &bodies, float delta_time);
void warm_start_velocity(HingeConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio);
bool solve_velocity(HingeConstraint &constraint, bodies::Bodies &bodies, float delta_time);
bool solve_position(HingeConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte);
/** the body's centre of mass moved by delta_com in its local space: the point on it follows, so the joint holds where it was */
void notify_shape_changed(HingeConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com);
void reset_warm_start(HingeConstraint &constraint);
void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const HingeConstraint &constraint);
void get_sort_fields(constraints::ConstraintSortFields &out, const HingeConstraint &constraint);

}  // namespace hinge_constraint

TUMBLE_NAMESPACE_END
