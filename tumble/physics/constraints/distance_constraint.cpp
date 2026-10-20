#include "tumble/physics/constraints/distance_constraint.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/world.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace distance_constraint {

using constraint_id::ConstraintId;
using rigid_body::RigidBody;

static bool is_body_live(const bodies::Bodies &bodies, uint32_t index) {
    return index < bodies.pool.length && bodies.pool[index] != nullptr && bodies.sequences[index] != bodies::SLOT_FREE;
}

/** creates default distance constraint */
static DistanceConstraint make_distance_constraint() {
    DistanceConstraint constraint;
    constraint.base = constraints::make_constraint_base();
    constraint.local_space_position1 = vec3f::zero();
    constraint.local_space_position2 = vec3f::zero();
    constraint.min_distance = 0.0f;
    constraint.max_distance = 0.0f;
    constraint.limits_spring_settings = spring_settings::create();
    constraint.world_space_position1 = vec3f::zero();
    constraint.world_space_position2 = vec3f::zero();
    // most likely gravity pulls the points apart; only used while the points meet
    constraint.world_space_normal = vec3f::axis_y();
    constraint.min_lambda = 0.0f;
    constraint.max_lambda = 0.0f;
    constraint.axis_constraint = axis_constraint_part::create();
    return constraint;
}

/** reset constraint state for pooling reuse */
static void reset_constraint(DistanceConstraint &constraint) {
    constraint.base.enabled = true;
    constraint.base.sleeping = false;
    constraint.base.pooled = false;
    axis_constraint_part::deactivate(constraint.axis_constraint);
}

ConstraintId create(world::World &world, RigidBody *body_a, RigidBody *body_b,
                    const DistanceConstraintSettings &settings) {
    constraints::Constraints &constraints = world.constraints;
    bodies::Bodies &bodies = world.bodies;
    TUMBLE_ASSERT(is_body_live(bodies, body_a->index) && bodies.pool[body_a->index] == body_a);
    TUMBLE_ASSERT(is_body_live(bodies, body_b->index) && bodies.pool[body_b->index] == body_b);
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::DISTANCE);

    const uint32_t index = constraints::allocate_index(pool);
    if (index == ~0u) return constraint_id::INVALID_CONSTRAINT_ID;
    const uint32_t sequence = constraints::next_sequence(pool, index);
    DistanceConstraint &constraint = *reinterpret_cast<DistanceConstraint *>(constraints::base_at(pool, index));
    constraint = make_distance_constraint();
    reset_constraint(constraint);

    constraint.base.id = constraint_id::ser(index, constraint_id::DISTANCE, sequence);
    constraint.base.index = index;
    constraint.base.sequence = sequence;

    constraint.base.constraint_priority = settings.constraint_priority;
    constraint.base.num_velocity_steps_override = settings.num_velocity_steps_override;
    constraint.base.num_position_steps_override = settings.num_position_steps_override;

    constraint.base.body_index_a = body_a->index;
    constraint.base.body_index_b = body_b->index;

    if (settings.space == constraints::SPACE_WORLD) {
        // local = quat^-1 * (world point - center of mass position)
        const Quatf inv_quat_a = quatf::conjugate(quatf::load(body_a->quaternion));
        const Quatf inv_quat_b = quatf::conjugate(quatf::load(body_b->quaternion));

        constraint.local_space_position1 = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point_a), vec3f::load(body_a->center_of_mass_position)), inv_quat_a);

        constraint.local_space_position2 = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point_b), vec3f::load(body_b->center_of_mass_position)), inv_quat_b);

        // initial world positions for the distance
        constraint.world_space_position1 = vec3f::load(settings.point_a);
        constraint.world_space_position2 = vec3f::load(settings.point_b);
    } else {
        constraint.local_space_position1 = vec3f::load(settings.point_a);
        constraint.local_space_position2 = vec3f::load(settings.point_b);

        // world = center of mass position + rotation * local point
        constraint.world_space_position1 = vec3f::add(vec3f::transform_quat(constraint.local_space_position1, quatf::load(body_a->quaternion)),
                                                      vec3f::load(body_a->center_of_mass_position));
        constraint.world_space_position2 = vec3f::add(vec3f::transform_quat(constraint.local_space_position2, quatf::load(body_b->quaternion)),
                                                      vec3f::load(body_b->center_of_mass_position));
    }

    // the initial distance stands in for a limit given as negative
    const float distance = vec3f::distance(constraint.world_space_position2, constraint.world_space_position1);
    const float min_distance = settings.min_distance;
    const float max_distance = settings.max_distance;
    if (min_distance < 0.0f && max_distance < 0.0f) {
        set_distance(constraint, distance, distance);
    } else {
        set_distance(constraint, min_distance < 0.0f ? scalar::min(distance, max_distance) : min_distance,
                     max_distance < 0.0f ? scalar::max(distance, min_distance) : max_distance);
    }

    constraint.limits_spring_settings = settings.spring_settings;

    // track constraint on both bodies
    const ConstraintId id = constraint.base.id;
    bool linked = body_a->constraint_ids.push(id);
    if (linked && body_a != body_b) linked = body_b->constraint_ids.push(id);
    if (!linked) {
        constraints::remove_constraint_by_id(constraints, bodies, id);
        return constraint_id::INVALID_CONSTRAINT_ID;
    }

    return id;
}

bool remove(world::World &world, ConstraintId id) {
    DistanceConstraint *found = get(world, id);
    if (found == nullptr) return false;
    DistanceConstraint &constraint = *found;
    constraints::Constraints &constraints = world.constraints;
    bodies::Bodies &bodies = world.bodies;
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::DISTANCE);

    // remove from bodies' constraint_ids
    const uint32_t a = constraint.base.body_index_a;
    const uint32_t b = constraint.base.body_index_b;
    if (is_body_live(bodies, a)) constraints::remove_constraint_id_from_body(*bodies.pool[a], constraint.base.id);
    if (a != b && is_body_live(bodies, b)) constraints::remove_constraint_id_from_body(*bodies.pool[b], constraint.base.id);

    reset_constraint(constraint);

    constraint.base.pooled = true;
    constraint.base.id = constraint_id::INVALID_CONSTRAINT_ID;
    pool.free_indices.push(constraint.base.index);
    return true;
}

DistanceConstraint *get(world::World &world, ConstraintId id) {
    if (constraint_id::type(id) != constraint_id::DISTANCE) return nullptr;
    return reinterpret_cast<DistanceConstraint *>(constraints::get(world.constraints, id));
}

/** updates the world positions and normal, then sets the axis part up by the limit the distance is at */
static void calculate_distance_constraint_properties(DistanceConstraint &constraint, RigidBody &body_a, RigidBody &body_b,
                                                     float delta_time, uint32_t step_stamp) {
    // world space positions (the bodies may have moved): center of mass position + rotation * local position
    const Vec3f center_of_mass_a = vec3f::load(body_a.center_of_mass_position);
    const Vec3f center_of_mass_b = vec3f::load(body_b.center_of_mass_position);
    constraint.world_space_position1 = vec3f::add(vec3f::transform_quat(constraint.local_space_position1, quatf::load(body_a.quaternion)), center_of_mass_a);
    constraint.world_space_position2 = vec3f::add(vec3f::transform_quat(constraint.local_space_position2, quatf::load(body_b.quaternion)), center_of_mass_b);

    // world space normal, from point 1 to point 2; kept from the last step when the points meet
    const Vec3f delta = vec3f::subtract(constraint.world_space_position2, constraint.world_space_position1);
    const float delta_len = vec3f::length(delta);
    if (delta_len > 0.0f) constraint.world_space_normal = vec3f::scale(delta, 1.0f / delta_len);

    // points relative to the bodies: r1 + u = (p1 - x1) + (p2 - p1) = p2 - x1
    const Vec3f r1_plus_u = vec3f::subtract(constraint.world_space_position2, center_of_mass_a);
    const Vec3f r2 = vec3f::subtract(constraint.world_space_position2, center_of_mass_b);

    const bool dynamic_a = body_a.motion_type == motion_type::MOTION_DYNAMIC;
    const bool dynamic_b = body_b.motion_type == motion_type::MOTION_DYNAMIC;
    const float inv_mass_a = dynamic_a ? body_a.motion_properties.inv_mass : 0.0f;
    const float inv_mass_b = dynamic_b ? body_b.motion_properties.inv_mass : 0.0f;

    // the axis part reads an inverse inertia only for a dynamic body
    const Mat4f inv_inertia_a =
        dynamic_a ? motion_properties::get_world_inverse_inertia(body_a.motion_properties, quatf::load(body_a.quaternion), step_stamp) : mat4f::zero();
    const Mat4f inv_inertia_b =
        dynamic_b ? motion_properties::get_world_inverse_inertia(body_b.motion_properties, quatf::load(body_b.quaternion), step_stamp) : mat4f::zero();

    if (constraint.min_distance == constraint.max_distance) {
        // a single distance: impulses in both directions
        axis_constraint_part::calculate_constraint_properties_with_settings(
            constraint.axis_constraint, delta_time, body_a, body_b, inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b,
            r1_plus_u, r2, constraint.world_space_normal, 0.0f, delta_len - constraint.min_distance,
            constraint.limits_spring_settings);
        constraint.min_lambda = -scalar::FLOAT_MAX;
        constraint.max_lambda = scalar::FLOAT_MAX;
    } else if (delta_len <= constraint.min_distance) {
        // too close: only impulses that push apart
        axis_constraint_part::calculate_constraint_properties_with_settings(
            constraint.axis_constraint, delta_time, body_a, body_b, inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b,
            r1_plus_u, r2, constraint.world_space_normal, 0.0f, delta_len - constraint.min_distance,
            constraint.limits_spring_settings);
        constraint.min_lambda = 0.0f;
        constraint.max_lambda = scalar::FLOAT_MAX;
    } else if (delta_len >= constraint.max_distance) {
        // too far: only impulses that pull together
        axis_constraint_part::calculate_constraint_properties_with_settings(
            constraint.axis_constraint, delta_time, body_a, body_b, inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b,
            r1_plus_u, r2, constraint.world_space_normal, 0.0f, delta_len - constraint.max_distance,
            constraint.limits_spring_settings);
        constraint.min_lambda = -scalar::FLOAT_MAX;
        constraint.max_lambda = 0.0f;
    } else {
        // within limits: nothing to do
        axis_constraint_part::deactivate(constraint.axis_constraint);
    }
}

void setup_velocity(DistanceConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    calculate_distance_constraint_properties(constraint, body_a, body_b, delta_time, bodies.step_stamp);
}

void warm_start_velocity(DistanceConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const float inv_mass_a = body_a.motion_type == motion_type::MOTION_DYNAMIC ? body_a.motion_properties.inv_mass : 0.0f;
    const float inv_mass_b = body_b.motion_type == motion_type::MOTION_DYNAMIC ? body_b.motion_properties.inv_mass : 0.0f;

    axis_constraint_part::warm_start(constraint.axis_constraint, body_a, body_b, inv_mass_a, inv_mass_b,
                                     constraint.world_space_normal, warm_start_impulse_ratio);
}

bool solve_velocity(DistanceConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    (void)delta_time;
    if (!axis_constraint_part::is_active(constraint.axis_constraint)) return false;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    return axis_constraint_part::solve_velocity_constraint(constraint.axis_constraint, body_a, body_b, constraint.world_space_normal,
                                                           constraint.min_lambda, constraint.max_lambda);
}

bool solve_position(DistanceConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte) {
    // a soft limit is the velocity bias's to correct
    if (constraint.limits_spring_settings.frequency_or_stiffness > 0.0f) return false;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    // current distance along the constraint normal
    const float distance =
        vec3f::dot(vec3f::subtract(constraint.world_space_position2, constraint.world_space_position1), constraint.world_space_normal);

    float position_error = 0.0f;
    if (distance < constraint.min_distance) {
        position_error = distance - constraint.min_distance;
    } else if (distance > constraint.max_distance) {
        position_error = distance - constraint.max_distance;
    }

    if (position_error == 0.0f) return false;

    // recalculate constraint properties (the bodies may have moved during the position solve)
    calculate_distance_constraint_properties(constraint, body_a, body_b, delta_time, motion_properties::STEP_STAMP_NONE);

    return axis_constraint_part::solve_position_constraint(constraint.axis_constraint, body_a, body_b, constraint.world_space_normal,
                                                           position_error, baumgarte);
}

void notify_shape_changed(DistanceConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com) {
    // separate tests, not an else-if: a constraint between a body and itself moves both points
    const uint32_t body_index = body_id::index(body_id);
    if (constraint.base.body_index_a == body_index)
        constraint.local_space_position1 = vec3f::subtract(constraint.local_space_position1, delta_com);
    if (constraint.base.body_index_b == body_index)
        constraint.local_space_position2 = vec3f::subtract(constraint.local_space_position2, delta_com);
}

void reset_warm_start(DistanceConstraint &constraint) { axis_constraint_part::deactivate(constraint.axis_constraint); }

void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const DistanceConstraint &constraint) {
    out.velocity = constraint.base.num_velocity_steps_override;
    out.position = constraint.base.num_position_steps_override;
}

void get_sort_fields(constraints::ConstraintSortFields &out, const DistanceConstraint &constraint) {
    out.priority = constraint.base.constraint_priority;
    out.index = constraint.base.index;
}

float get_total_lambda(const DistanceConstraint &constraint) { return constraint.axis_constraint.total_lambda; }

void set_distance(DistanceConstraint &constraint, float min_distance, float max_distance) {
    TUMBLE_ASSERT(min_distance <= max_distance);
    constraint.min_distance = min_distance;
    constraint.max_distance = max_distance;
}

}  // namespace distance_constraint

TUMBLE_NAMESPACE_END
