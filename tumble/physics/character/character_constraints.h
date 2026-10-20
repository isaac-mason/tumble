#pragma once

// the pure parts of jolt's CharacterVirtual solver, used by the kcc (CharacterConstraint) and written over any
// constraint type with the same fields: a constraint is a plane with a velocity, and each has plane_normal,
// plane_distance, linear_velocity, projected_velocity, toi and contact->motion_type.

#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace character_constraints {

/**
 * when the constraint is hit: FLOAT_MAX if it is not pushing (projected velocity under a threshold) or the penetration
 * at the end of the time would be acceptable, otherwise 0 or the positive time to reach the plane. stores the
 * constraint's projected velocity.
 */
template <typename Constraint>
inline float calculate_constraint_toi(Constraint &constraint, Vec3f velocity, Vec3f displacement, float time_remaining) {
    // signed distance to the plane from the displacement so far
    const float dist_to_plane = vec3f::dot(constraint.plane_normal, displacement) + constraint.plane_distance;

    // how fast the constraint pushes relative to character motion
    const float projected_velocity =
        vec3f::dot(constraint.linear_velocity, constraint.plane_normal) - vec3f::dot(velocity, constraint.plane_normal);
    constraint.projected_velocity = projected_velocity;

    if (projected_velocity < 1e-6f) return scalar::FLOAT_MAX;

    // too little penetration at the end of the time step, accept the movement
    if (dist_to_plane - projected_velocity * time_remaining > -1e-4f) return scalar::FLOAT_MAX;

    return scalar::max(0.0f, dist_to_plane / projected_velocity);
}

/**
 * whether a sorts before b: both at toi <= 0 by projected velocity, highest first (the hardest pushing, which also
 * resolves the deepest penetrations); then by toi; then dynamic before static
 */
template <typename Constraint>
inline bool constraint_before(const Constraint &a, const Constraint &b) {
    if (a.toi <= 0.0f && b.toi <= 0.0f && a.projected_velocity != b.projected_velocity) return a.projected_velocity > b.projected_velocity;
    if (a.toi != b.toi) return a.toi < b.toi;
    return a.contact->motion_type > b.contact->motion_type;
}

/** the velocity with the part moving into the plane, relative to the constraint, removed */
template <typename Constraint>
inline Vec3f slide_velocity_along_plane(Vec3f velocity, const Constraint &constraint) {
    const float rel_vel_dot_normal = vec3f::dot(velocity, constraint.plane_normal) - vec3f::dot(constraint.linear_velocity, constraint.plane_normal);
    return vec3f::scale_and_add(velocity, constraint.plane_normal, -rel_vel_dot_normal);
}

/** the velocity slid along the intersection of two constraint planes; parallel planes slide along the first */
template <typename Constraint>
inline Vec3f slide_along_edge(Vec3f velocity, const Constraint &constraint1, const Constraint &constraint2) {
    Vec3f edge_direction = vec3f::cross(constraint1.plane_normal, constraint2.plane_normal);
    if (vec3f::squared_length(edge_direction) < 1e-12f) return slide_velocity_along_plane(velocity, constraint1);
    edge_direction = vec3f::normalize(edge_direction);

    // the velocity along the edge, plus each constraint's velocity perpendicular to it
    const Vec3f perpendicular_velocity1 =
        vec3f::scale_and_add(constraint1.linear_velocity, edge_direction, -vec3f::dot(constraint1.linear_velocity, edge_direction));
    const Vec3f perpendicular_velocity2 =
        vec3f::scale_and_add(constraint2.linear_velocity, edge_direction, -vec3f::dot(constraint2.linear_velocity, edge_direction));
    return vec3f::add(vec3f::add(vec3f::scale(edge_direction, vec3f::dot(velocity, edge_direction)), perpendicular_velocity1),
                      perpendicular_velocity2);
}

}  // namespace character_constraints

TUMBLE_NAMESPACE_END
