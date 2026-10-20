#pragma once

// crashcat's distance-constraint.ts: keeps the distance between a point on each body within [min, max],
// removing one translational degree of freedom.

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/constraints/constraint_part/axis_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/spring_settings.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

namespace distance_constraint {

/** distance constraint removes 1 translational DOF (distance between two points) */
struct DistanceConstraint {
    constraints::ConstraintBase base;
    Vec3f local_space_position1;
    Vec3f local_space_position2;
    float min_distance;
    float max_distance;
    spring_settings::SpringSettings limits_spring_settings;
    // runtime state
    Vec3f world_space_position1;
    Vec3f world_space_position2;
    /** from point 1 to point 2; kept from the last step when the points meet */
    Vec3f world_space_normal;
    float min_lambda;
    float max_lambda;
    axis_constraint_part::AxisConstraintPart axis_constraint;
};

struct DistanceConstraintSettings {
    /**
     * the point on body a (space by `space`). the constraint keeps point_a and point_b at the same distance;
     * point_a = point_b makes a cheap point constraint, though it removes only 1 degree of freedom instead of 3
     */
    Float3 point_a = {0.0f, 0.0f, 0.0f};
    /** the point on body b (space by `space`) */
    Float3 point_b = {0.0f, 0.0f, 0.0f};
    /** minimum distance, -1 for the initial distance */
    float min_distance = -1.0f;
    /** maximum distance, -1 for the initial distance */
    float max_distance = -1.0f;
    /** spring settings for soft limits */
    spring_settings::SpringSettings spring_settings = {};
    constraints::ConstraintSpace space = constraints::SPACE_WORLD;
    /** higher numbers are more likely to be solved correctly (solved later) */
    int32_t constraint_priority = 0;
    /** 0 takes the world's iteration counts */
    uint32_t num_velocity_steps_override = 0;
    uint32_t num_position_steps_override = 0;
};

/** a distance constraint between two live bodies, linked into both bodies' constraint_ids; INVALID_CONSTRAINT_ID (error set) when the pool cannot grow */
constraint_id::ConstraintId create(constraints::Constraints &constraints, bodies::Bodies &bodies, rigid_body::RigidBody *body_a,
                                   rigid_body::RigidBody *body_b, const DistanceConstraintSettings &settings);
void remove(constraints::Constraints &constraints, bodies::Bodies &bodies, DistanceConstraint &constraint);
/** null when the id is stale or not a distance constraint */
DistanceConstraint *get(constraints::Constraints &constraints, constraint_id::ConstraintId id);

void setup_velocity(DistanceConstraint &constraint, bodies::Bodies &bodies, float delta_time);
void warm_start_velocity(DistanceConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio);
bool solve_velocity(DistanceConstraint &constraint, bodies::Bodies &bodies, float delta_time);
bool solve_position(DistanceConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte);
/** the body's centre of mass moved by delta_com in its local space: the point on it follows, so the joint holds where it was */
void notify_shape_changed(DistanceConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com);
void reset_warm_start(DistanceConstraint &constraint);
void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const DistanceConstraint &constraint);
void get_sort_fields(constraints::ConstraintSortFields &out, const DistanceConstraint &constraint);

/** total accumulated impulse, for debugging, telemetry, or breaking constraints by force */
float get_total_lambda(const DistanceConstraint &constraint);

/** sets both limits; min must not be above max */
void set_distance(DistanceConstraint &constraint, float min_distance, float max_distance);

}  // namespace distance_constraint

TUMBLE_NAMESPACE_END
