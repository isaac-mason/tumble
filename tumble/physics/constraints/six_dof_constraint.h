#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/constraints/constraint_part/angle_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/axis_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/motor_settings.h"
#include "tumble/physics/constraints/constraint_part/point_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/rotation_euler_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/spring_settings.h"
#include "tumble/physics/constraints/constraint_part/swing_twist_constraint_part.h"
#include "tumble/physics/constraints/constraints.h"

TUMBLE_NAMESPACE_BEGIN

struct RigidBody;

// each of the six axes free, fixed or limited, with optional friction and a velocity or position motor

struct SixDOFConstraint {
    enum Axis : uint32_t {
        TRANSLATION_X = 0,
        TRANSLATION_Y = 1,
        TRANSLATION_Z = 2,
        ROTATION_X = 3,
        ROTATION_Y = 4,
        ROTATION_Z = 5,
        NUM = 6,
        NUM_TRANSLATION = 3,
    };

    ConstraintBase base;
    /** attachment point on body 1 relative to its center of mass, in body space */
    Vec3f local_space_position1;
    /** attachment point on body 2 relative to its center of mass, in body space */
    Vec3f local_space_position2;
    /** rotation from constraint space to body 1 space */
    Quatf constraint_to_body1;
    /** rotation from constraint space to body 2 space */
    Quatf constraint_to_body2;

    /** per axis limits: min >= max is fixed (at min), -FLT_MAX / FLT_MAX (translation) or -pi / pi (rotation) is free */
    float limit_min[Axis::NUM];
    float limit_max[Axis::NUM];
    /** soft translation limits */
    SpringSettings limits_spring_settings[Axis::NUM_TRANSLATION];
    /** per axis maximum friction force or torque, 0 for none */
    float max_friction[Axis::NUM];

    MotorState motor_state[Axis::NUM];
    /** per axis motor settings: translation motors use the force limits, rotation motors the torque limits */
    MotorSettings motor_settings[Axis::NUM];

    /** motor targets, in constraint space */
    Vec3f target_velocity;
    Vec3f target_angular_velocity;
    Vec3f target_position;
    Quatf target_orientation;

    SwingType swing_type;

    /** world space axes of body 1's constraint space, cached during setup */
    Vec3f translation_axis[3];
    /** world space axes of body 2's constraint space, the rotation motor axes */
    Vec3f rotation_axis[3];
    /** displacement along each translation axis, cached during setup */
    float displacement[3];
    /** bitmasks over SixDOFConstraint::Axis */
    uint32_t free_axis;
    uint32_t fixed_axis;
    bool translation_motor_active;
    bool rotation_motor_active;
    /** bitmask over the three rotation axes */
    uint32_t rotation_position_motor_active;
    bool has_spring_limits;

    AxisConstraintPart translation_constraint_part[3];
    PointConstraintPart point_constraint_part;
    SwingTwistConstraintPart swing_twist_constraint_part;
    RotationEulerConstraintPart rotation_constraint_part;
    AxisConstraintPart motor_translation_constraint_part[3];
    AngleConstraintPart motor_rotation_constraint_part[3];
};

struct SixDOFConstraintSettings {
    Float3 point_a = {0.0f, 0.0f, 0.0f};
    Float3 point_b = {0.0f, 0.0f, 0.0f};
    /** x and y axes of constraint space on body 1 (normalized on create) */
    Float3 axis_x_a = {1.0f, 0.0f, 0.0f};
    Float3 axis_y_a = {0.0f, 1.0f, 0.0f};
    /** x and y axes of constraint space on body 2 (normalized on create) */
    Float3 axis_x_b = {1.0f, 0.0f, 0.0f};
    Float3 axis_y_b = {0.0f, 1.0f, 0.0f};
    ConstraintSpace space = ConstraintSpace::WORLD;
    SwingType swing_type = SwingType::CONE;
    /** per axis limits; every axis free by default */
    float limit_min[SixDOFConstraint::Axis::NUM] = {-scalar::FLOAT_MAX, -scalar::FLOAT_MAX, -scalar::FLOAT_MAX, -scalar::PI, -scalar::PI, -scalar::PI};
    float limit_max[SixDOFConstraint::Axis::NUM] = {scalar::FLOAT_MAX, scalar::FLOAT_MAX, scalar::FLOAT_MAX, scalar::PI, scalar::PI, scalar::PI};
    float max_friction[SixDOFConstraint::Axis::NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    SpringSettings limits_spring_settings[SixDOFConstraint::Axis::NUM_TRANSLATION] = {};
    /** per axis motor settings; 2 hz critically damped, no force or torque limits by default */
    MotorSettings motor_settings[SixDOFConstraint::Axis::NUM] = {};
    /** higher numbers are more likely to be solved correctly (solved later) */
    uint32_t constraint_priority = 0;
    /** 0 takes the world's iteration counts */
    uint32_t num_velocity_steps_override = 0;
    uint32_t num_position_steps_override = 0;
};

namespace six_dof_constraint {

/** a six dof constraint between two live bodies, linked into both bodies' constraint_ids; INVALID_CONSTRAINT_ID (Error::QUOTA) when the pool is at its most */
ConstraintId create(World &world, RigidBody *body_a,
                                   RigidBody *body_b, const SixDOFConstraintSettings &settings);
/** null when the id is stale or not a six dof constraint */
SixDOFConstraint *get(World &world, ConstraintId id);

void set_translation_limits(SixDOFConstraint &constraint, Vec3f limit_min, Vec3f limit_max);
void set_rotation_limits(SixDOFConstraint &constraint, Vec3f limit_min, Vec3f limit_max);
/** a change of state clears the axis' motor warm start */
void set_motor_state(SixDOFConstraint &constraint, SixDOFConstraint::Axis axis, MotorState state);
void set_target_velocity_cs(SixDOFConstraint &constraint, Vec3f velocity);
void set_target_angular_velocity_cs(SixDOFConstraint &constraint, Vec3f angular_velocity);
void set_target_position_cs(SixDOFConstraint &constraint, Vec3f position);
/** target orientation in constraint space, clamped to the rotation limits */
void set_target_orientation_cs(SixDOFConstraint &constraint, Quatf orientation);
void set_max_friction(SixDOFConstraint &constraint, uint32_t axis, float friction);
void make_free_axis(SixDOFConstraint &constraint, uint32_t axis);
/** locked at 0 */
void make_fixed_axis(SixDOFConstraint &constraint, uint32_t axis);
/** if max < min the axis becomes fixed at 0 */
void set_limited_axis(SixDOFConstraint &constraint, uint32_t axis, float min, float max);

void setup_velocity(SixDOFConstraint &constraint, Bodies &bodies, float delta_time);
void warm_start_velocity(SixDOFConstraint &constraint, Bodies &bodies, float warm_start_impulse_ratio);
bool solve_velocity(SixDOFConstraint &constraint, Bodies &bodies, float delta_time);
bool solve_position(SixDOFConstraint &constraint, Bodies &bodies, float delta_time, float baumgarte);
/** the body's centre of mass moved by delta_com in its local space: the point on it follows, so the joint holds where it was */
void notify_shape_changed(SixDOFConstraint &constraint, BodyId body_id, Vec3f delta_com);
void reset_warm_start(SixDOFConstraint &constraint);
void get_iteration_overrides(Constraints::IterationOverrides &out, const SixDOFConstraint &constraint);
void get_sort_fields(Constraints::SortFields &out, const SixDOFConstraint &constraint);

}  // namespace six_dof_constraint

TUMBLE_NAMESPACE_END
