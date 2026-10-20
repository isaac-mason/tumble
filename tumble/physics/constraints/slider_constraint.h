#pragma once

// crashcat's slider-constraint.ts: a prismatic joint. removes 5 DOF (2 translation perpendicular to the slider
// axis + 3 rotation), allowing movement only along one axis, like a piston or a rail. optional position
// limits, friction and a velocity or position motor.

#include <stdint.h>

#include "math/scalar.h"
#include "math/types.h"
#include "physics/body/bodies.h"
#include "physics/body/body_id.h"
#include "physics/constraints/constraint_part/axis_constraint_part.h"
#include "physics/constraints/constraint_part/dual_axis_constraint_part.h"
#include "physics/constraints/constraint_part/motor_settings.h"
#include "physics/constraints/constraint_part/rotation_euler_constraint_part.h"
#include "physics/constraints/constraint_part/spring_settings.h"
#include "physics/constraints/constraints.h"

namespace rigid_body {
struct RigidBody;
}

namespace slider_constraint {

struct SliderConstraint {
    constraints::ConstraintBase base;
    /** attachment point on body a relative to its center of mass, in body space */
    Vec3f local_space_position_a;
    /** attachment point on body b relative to its center of mass, in body space */
    Vec3f local_space_position_b;
    /** slider axis on body a in body space (normalized), the direction of allowed movement */
    Vec3f local_space_slider_axis_a;
    /** normal axis perpendicular to the slider on body a */
    Vec3f local_space_normal_a;
    /** second normal axis, slider x normal */
    Vec3f local_space_normal_b;

    /** inverse of the initial relative orientation, for the rotation constraint */
    Quatf inv_initial_orientation;

    /** r1 = rotation1 x local_space_position_a, the world space offset from the center of mass to the attachment */
    Vec3f r1;
    /** r2 = rotation2 x local_space_position_b */
    Vec3f r2;
    /** u = x2 + r2 - x1 - r1 = p2 - p1, the separation in world space */
    Vec3f u;
    Vec3f world_space_slider_axis;
    /** world space normals of the position constraint */
    Vec3f n1;
    Vec3f n2;
    /** current distance along the slider axis, u . slider axis */
    float d;

    bool has_limits;
    /** minimum position limit (<= 0) */
    float limits_min;
    /** maximum position limit (>= 0) */
    float limits_max;
    /** soft limits when the spring has a frequency or stiffness */
    spring_settings::SpringSettings limits_spring_settings;

    motor_settings::MotorState motor_state;
    /** target velocity (m/s) of the velocity motor */
    float target_velocity;
    /** target position of the position motor */
    float target_position;
    /** maximum friction force while the motor is off */
    float max_friction_force;
    motor_settings::MotorSettings motor_settings;

    /** position constraint for the 2 DOF perpendicular to the slider */
    dual_axis_constraint_part::DualAxisConstraintPart position_constraint_part;
    /** locks all rotation (3 DOF) */
    rotation_euler_constraint_part::RotationEulerConstraintPart rotation_constraint_part;
    /** position limits along the slider axis */
    axis_constraint_part::AxisConstraintPart position_limits_constraint_part;
    /** motor along the slider axis */
    axis_constraint_part::AxisConstraintPart motor_constraint_part;
};

struct SliderConstraintSettings {
    /** pivot point on body a */
    Float3 point_a = {0.0f, 0.0f, 0.0f};
    /** pivot point on body b */
    Float3 point_b = {0.0f, 0.0f, 0.0f};
    /** slider axis on body a, the direction of allowed movement (normalized on create) */
    Float3 slider_axis_a = {1.0f, 0.0f, 0.0f};
    /** slider axis on body b (normalized on create) */
    Float3 slider_axis_b = {1.0f, 0.0f, 0.0f};
    /** normal axis perpendicular to the slider on body a (normalized on create) */
    Float3 normal_axis_a = {0.0f, 1.0f, 0.0f};
    /** normal axis perpendicular to the slider on body b (normalized on create) */
    Float3 normal_axis_b = {0.0f, 1.0f, 0.0f};
    constraints::ConstraintSpace space = constraints::SPACE_WORLD;
    /** world space only: place the attachment point on the other body's center of mass when one body is static, else
     *  between the centers of mass, weighted towards the lighter body */
    bool auto_detect_point = false;
    /** minimum position limit, at most 0; -FLT_MAX and FLT_MAX together mean no limits */
    float limits_min = -scalar::FLOAT_MAX;
    /** maximum position limit, at least 0 */
    float limits_max = scalar::FLOAT_MAX;
    spring_settings::SpringSettings limits_spring_settings = {};
    /** maximum friction force while the motor is off */
    float max_friction_force = 0.0f;
    motor_settings::MotorSettings motor_settings = {};
    /** higher numbers are more likely to be solved correctly (solved later) */
    int32_t constraint_priority = 0;
    /** 0 takes the world's iteration counts */
    uint32_t num_velocity_steps_override = 0;
    uint32_t num_position_steps_override = 0;
};

/** a slider constraint between two live bodies, linked into both bodies' constraint_ids; INVALID_CONSTRAINT_ID (error set) when the pool cannot grow */
constraint_id::ConstraintId create(constraints::Constraints &constraints, bodies::Bodies &bodies, rigid_body::RigidBody *body_a,
                                   rigid_body::RigidBody *body_b, const SliderConstraintSettings &settings);
void remove(constraints::Constraints &constraints, bodies::Bodies &bodies, SliderConstraint &constraint);
/** null when the id is stale or not a slider constraint */
SliderConstraint *get(constraints::Constraints &constraints, constraint_id::ConstraintId id);

/** the distance along the slider axis from the rest position, from the bodies' current transforms */
float get_current_position(const SliderConstraint &constraint, const bodies::Bodies &bodies);
/** position limits, min <= 0 and max >= 0 */
void set_limits(SliderConstraint &constraint, float min, float max);
void set_motor_state(SliderConstraint &constraint, motor_settings::MotorState state);
/** target velocity of the velocity motor, m/s */
void set_target_velocity(SliderConstraint &constraint, float velocity);
/** target position of the position motor, clamped to the limits */
void set_target_position(SliderConstraint &constraint, float position);
void set_max_friction_force(SliderConstraint &constraint, float force);
float get_max_friction_force(const SliderConstraint &constraint);

void setup_velocity(SliderConstraint &constraint, bodies::Bodies &bodies, float delta_time);
void warm_start_velocity(SliderConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio);
bool solve_velocity(SliderConstraint &constraint, bodies::Bodies &bodies, float delta_time);
bool solve_position(SliderConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte);
/** the body's centre of mass moved by delta_com in its local space: the point on it follows, so the joint holds where it was */
void notify_shape_changed(SliderConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com);
void reset_warm_start(SliderConstraint &constraint);
void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const SliderConstraint &constraint);
void get_sort_fields(constraints::ConstraintSortFields &out, const SliderConstraint &constraint);

Vec2f &get_total_lambda_position(Vec2f &out, const SliderConstraint &constraint);
float get_total_lambda_position_limits(const SliderConstraint &constraint);
Vec3f get_total_lambda_rotation(const SliderConstraint &constraint);
float get_total_lambda_motor(const SliderConstraint &constraint);

}  // namespace slider_constraint
