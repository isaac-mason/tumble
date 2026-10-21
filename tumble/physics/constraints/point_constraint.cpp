#include "tumble/physics/constraints/point_constraint.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/world.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace point_constraint {

using constraint_id::ConstraintId;
using rigid_body::RigidBody;

static bool is_body_live(const bodies::Bodies &bodies, uint32_t index) {
    return index < bodies.pool.length && bodies.pool[index] != nullptr && bodies.sequences[index] != bodies::SLOT_FREE;
}

/** a record slot in the pool: the last freed one, else a new one at the end; ~0u (error set) when the pool cannot grow */
/** creates default point constraint */
static PointConstraint make_point_constraint() {
    PointConstraint constraint;
    constraint.base = constraints::make_constraint_base();
    constraint.local_space_position1 = vec3f::zero();
    constraint.local_space_position2 = vec3f::zero();
    constraint.point_constraint_part = point_constraint_part::create();
    return constraint;
}

/** reset constraint state for pooling reuse */
static void reset_constraint(PointConstraint &constraint) {
    constraint.base.enabled = true;
    constraint.base.sleeping = false;
    constraint.base.pooled = false;
    point_constraint_part::deactivate(constraint.point_constraint_part);
}

ConstraintId create(world::World &world, RigidBody *body_a, RigidBody *body_b,
                    const PointConstraintSettings &settings) {
    constraints::Constraints &constraints = world.constraints;
    bodies::Bodies &bodies = world.bodies;
    TUMBLE_ASSERT(is_body_live(bodies, body_a->index) && bodies.pool[body_a->index] == body_a);
    TUMBLE_ASSERT(is_body_live(bodies, body_b->index) && bodies.pool[body_b->index] == body_b);
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::POINT);

    const uint32_t index = constraints::allocate_index(pool);
    if (index == ~0u) return constraint_id::INVALID_CONSTRAINT_ID;
    const uint32_t sequence = constraints::next_sequence(pool, index);
    PointConstraint &constraint = *reinterpret_cast<PointConstraint *>(constraints::base_at(pool, index));
    constraint = make_point_constraint();
    reset_constraint(constraint);

    constraint.base.id = constraint_id::ser(index, constraint_id::POINT, sequence);
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
    } else {
        constraint.local_space_position1 = vec3f::load(settings.point_a);
        constraint.local_space_position2 = vec3f::load(settings.point_b);
    }

    // track constraint on both bodies
    const ConstraintId id = constraint.base.id;
    bool linked = body_a->constraint_ids.push(id);
    if (linked && body_a != body_b) linked = body_b->constraint_ids.push(id);
    if (linked) linked = constraints::add_to_list(constraints, constraint.base);
    if (!linked) {
        constraints::remove_constraint_by_id(constraints, bodies, id);
        return constraint_id::INVALID_CONSTRAINT_ID;
    }

    return id;
}

bool remove(world::World &world, ConstraintId id) {
    PointConstraint *found = get(world, id);
    if (found == nullptr) return false;
    PointConstraint &constraint = *found;
    constraints::Constraints &constraints = world.constraints;
    bodies::Bodies &bodies = world.bodies;
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::POINT);

    // remove from bodies' constraint_ids
    const uint32_t a = constraint.base.body_index_a;
    const uint32_t b = constraint.base.body_index_b;
    if (is_body_live(bodies, a)) constraints::remove_constraint_id_from_body(*bodies.pool[a], constraint.base.id);
    if (a != b && is_body_live(bodies, b)) constraints::remove_constraint_id_from_body(*bodies.pool[b], constraint.base.id);

    reset_constraint(constraint);

    constraint.base.pooled = true;
    constraint.base.id = constraint_id::INVALID_CONSTRAINT_ID;
    constraints::remove_from_list(constraints, constraint.base);
    pool.free_indices.push(constraint.base.index);
    return true;
}

PointConstraint *get(world::World &world, ConstraintId id) {
    if (constraint_id::type(id) != constraint_id::POINT) return nullptr;
    return reinterpret_cast<PointConstraint *>(constraints::get(world.constraints, id));
}

void setup_velocity(PointConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    (void)delta_time;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                           constraint.local_space_position1, body_b, rotation_b,
                                                           constraint.local_space_position2, bodies.step_stamp);
}

void warm_start_velocity(PointConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    point_constraint_part::warm_start(constraint.point_constraint_part, body_a, body_b, warm_start_impulse_ratio);
}

bool solve_velocity(PointConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    (void)delta_time;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    return point_constraint_part::solve_velocity_constraint(constraint.point_constraint_part, body_a, body_b);
}

bool solve_position(PointConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte) {
    (void)delta_time;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    // recalculate constraint properties (bodies may have moved during position solve)
    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                           constraint.local_space_position1, body_b, rotation_b,
                                                           constraint.local_space_position2, motion_properties::STEP_STAMP_NONE);

    return point_constraint_part::solve_position_constraint(constraint.point_constraint_part, body_a, body_b, baumgarte);
}

void notify_shape_changed(PointConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com) {
    // separate tests, not an else-if: a constraint between a body and itself moves both points
    const uint32_t body_index = body_id::index(body_id);
    if (constraint.base.body_index_a == body_index)
        constraint.local_space_position1 = vec3f::subtract(constraint.local_space_position1, delta_com);
    if (constraint.base.body_index_b == body_index)
        constraint.local_space_position2 = vec3f::subtract(constraint.local_space_position2, delta_com);
}

void reset_warm_start(PointConstraint &constraint) { point_constraint_part::deactivate(constraint.point_constraint_part); }

void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const PointConstraint &constraint) {
    out.velocity = constraint.base.num_velocity_steps_override;
    out.position = constraint.base.num_position_steps_override;
}

void get_sort_fields(constraints::ConstraintSortFields &out, const PointConstraint &constraint) {
    out.priority = constraint.base.constraint_priority;
    out.index = constraint.base.list_index;
}

Vec3f get_total_lambda(const PointConstraint &constraint) { return constraint.point_constraint_part.total_lambda; }

}  // namespace point_constraint

TUMBLE_NAMESPACE_END
