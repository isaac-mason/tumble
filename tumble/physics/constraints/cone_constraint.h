#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/constraints/constraint_part/angle_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/point_constraint_part.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

// a point on each body held together, and the angle between the two twist axes kept within a cone (t1 . t2 >= cos(half
// cone angle)); twist about the axis stays free
namespace cone_constraint {

/** removes 3 translation DOF (point constraint) and limits rotation to a cone (1 DOF limit) */
struct ConeConstraint {
    constraints::ConstraintBase base;
    /** attachment point on body 1 relative to its center of mass, in body space */
    Vec3f local_space_position1;
    /** attachment point on body 2 relative to its center of mass, in body space */
    Vec3f local_space_position2;
    /** twist axis on body 1 in body space (normalized) */
    Vec3f local_space_twist_axis1;
    /** twist axis on body 2 in body space (normalized) */
    Vec3f local_space_twist_axis2;
    /** cos of the half cone angle; the limit acts while t1 . t2 is below it */
    float cos_half_cone_angle;
    /** world space rotation axis (t2 x t1, normalized) of the angular limit; kept from the last step when the axes align */
    Vec3f world_space_rotation_axis;
    /** cos of the angle between the twist axes (t1 . t2) */
    float cos_theta;
    /** point constraint for translation (3 DOF) */
    point_constraint_part::PointConstraintPart point_constraint_part;
    /** angle constraint for the cone limit (1 DOF) */
    angle_constraint_part::AngleConstraintPart angle_constraint_part;
};

struct ConeConstraintSettings {
    /** pivot point on body a */
    Float3 point_a = {0.0f, 0.0f, 0.0f};
    /** twist axis on body a (normalized on create) */
    Float3 twist_axis_a = {1.0f, 0.0f, 0.0f};
    /** pivot point on body b */
    Float3 point_b = {0.0f, 0.0f, 0.0f};
    /** twist axis on body b (normalized on create) */
    Float3 twist_axis_b = {1.0f, 0.0f, 0.0f};
    /** half cone angle in radians (0 = no swing, pi/2 = a 90 degree cone) */
    float half_cone_angle = 0.0f;
    constraints::ConstraintSpace space = constraints::SPACE_WORLD;
    /** higher numbers are more likely to be solved correctly (solved later) */
    uint32_t constraint_priority = 0;
    /** 0 takes the world's iteration counts */
    uint32_t num_velocity_steps_override = 0;
    uint32_t num_position_steps_override = 0;
};

/** a cone constraint between two live bodies, linked into both bodies' constraint_ids; INVALID_CONSTRAINT_ID (error set) when the pool cannot grow */
constraint_id::ConstraintId create(world::World &world, rigid_body::RigidBody *body_a,
                                   rigid_body::RigidBody *body_b, const ConeConstraintSettings &settings);
/** null when the id is stale or not a cone constraint */
ConeConstraint *get(world::World &world, constraint_id::ConstraintId id);

/** half cone angle in radians, in [0, pi] */
void set_half_cone_angle(ConeConstraint &constraint, float half_cone_angle);
float get_cos_half_cone_angle(const ConeConstraint &constraint);
/** half cone angle in radians */
float get_half_cone_angle(const ConeConstraint &constraint);

void setup_velocity(ConeConstraint &constraint, bodies::Bodies &bodies, float delta_time);
void warm_start_velocity(ConeConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio);
bool solve_velocity(ConeConstraint &constraint, bodies::Bodies &bodies, float delta_time);
bool solve_position(ConeConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte);
/** the body's centre of mass moved by delta_com in its local space: the point on it follows, so the joint holds where it was */
void notify_shape_changed(ConeConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com);
void reset_warm_start(ConeConstraint &constraint);
void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const ConeConstraint &constraint);
void get_sort_fields(constraints::ConstraintSortFields &out, const ConeConstraint &constraint);

/** total lambda of the position constraint */
Vec3f get_total_lambda_position(const ConeConstraint &constraint);
/** total lambda of the rotation constraint */
float get_total_lambda_rotation(const ConeConstraint &constraint);

}  // namespace cone_constraint

TUMBLE_NAMESPACE_END
