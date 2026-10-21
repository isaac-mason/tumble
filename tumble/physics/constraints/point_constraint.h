#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/constraints/constraint_part/point_constraint_part.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

// a point on each body held together: the three translational degrees of freedom
namespace point_constraint {

/** point constraint removes 3 translational DOF */
struct PointConstraint {
    constraints::ConstraintBase base;
    Vec3f local_space_position1;
    Vec3f local_space_position2;
    point_constraint_part::PointConstraintPart point_constraint_part;
};

struct PointConstraintSettings {
    /** point on body a in world space or local space depending on `space` */
    Float3 point_a = {0.0f, 0.0f, 0.0f};
    /** point on body b in world space or local space depending on `space` */
    Float3 point_b = {0.0f, 0.0f, 0.0f};
    constraints::ConstraintSpace space = constraints::SPACE_WORLD;
    /** higher numbers are more likely to be solved correctly (solved later) */
    uint32_t constraint_priority = 0;
    /** 0 takes the world's iteration counts */
    uint32_t num_velocity_steps_override = 0;
    uint32_t num_position_steps_override = 0;
};

/** a point constraint between two live bodies, linked into both bodies' constraint_ids; INVALID_CONSTRAINT_ID (error set) when the pool cannot grow */
constraint_id::ConstraintId create(world::World &world, rigid_body::RigidBody *body_a,
                                   rigid_body::RigidBody *body_b, const PointConstraintSettings &settings);
/** false for a stale id or one of another type */
bool remove(world::World &world, constraint_id::ConstraintId id);
/** null when the id is stale or not a point constraint */
PointConstraint *get(world::World &world, constraint_id::ConstraintId id);

void setup_velocity(PointConstraint &constraint, bodies::Bodies &bodies, float delta_time);
void warm_start_velocity(PointConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio);
bool solve_velocity(PointConstraint &constraint, bodies::Bodies &bodies, float delta_time);
bool solve_position(PointConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte);
/** the body's centre of mass moved by delta_com in its local space: the point on it follows, so the joint holds where it was */
void notify_shape_changed(PointConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com);
void reset_warm_start(PointConstraint &constraint);
void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const PointConstraint &constraint);
void get_sort_fields(constraints::ConstraintSortFields &out, const PointConstraint &constraint);

/** total accumulated impulse, for debugging, telemetry, or breaking constraints by force */
Vec3f get_total_lambda(const PointConstraint &constraint);

}  // namespace point_constraint

TUMBLE_NAMESPACE_END
