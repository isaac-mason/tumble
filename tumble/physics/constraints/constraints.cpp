#include "tumble/physics/constraints/constraints.h"

#include "tumble/instance.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/cone_constraint.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/constraints/fixed_constraint.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/constraints/six_dof_constraint.h"
#include "tumble/physics/constraints/slider_constraint.h"
#include "tumble/physics/constraints/swing_twist_constraint.h"
#include "tumble/util/assert.h"
#include "tumble/util/mem.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

namespace constraints {

template <typename T>
static constexpr uint32_t record_stride() {
    return (uint32_t)((sizeof(T) + alignof(T) - 1) & ~(alignof(T) - 1));
}

template <typename T>
static T &record_at(ConstraintPool &pool, uint32_t index) {
    return *reinterpret_cast<T *>(base_at(pool, index));
}

template <typename T>
static const T &record_at(const ConstraintPool &pool, uint32_t index) {
    return *reinterpret_cast<const T *>(base_at(pool, index));
}

static bool is_body_live(const bodies::Bodies &bodies, uint32_t index) {
    return index < bodies.pool.length && bodies.pool[index] != nullptr && bodies.sequences[index] != bodies::SLOT_FREE;
}

Constraints create() {
    Constraints constraints;
    util::zero(&constraints, sizeof(Constraints));
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        constraints.pools[type].type = (ConstraintType)type;
    }
    return constraints;
}

void release(Constraints &constraints) {
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        ConstraintPool &pool = constraints.pools[type];
        pool.storage.release();
        pool.free_indices.release();
        pool.stride = 0;
        pool.count = 0;
    }
}

ConstraintPool &ensure_pool(Constraints &constraints, ConstraintType type) {
    TUMBLE_ASSERT(type < constraint_id::CONSTRAINT_TYPE_COUNT);
    ConstraintPool &pool = constraints.pools[type];
    if (pool.stride != 0) return pool;
    pool.type = type;
    switch (type) {
        case constraint_id::POINT:
            pool.stride = record_stride<point_constraint::PointConstraint>();
            break;
        case constraint_id::DISTANCE:
            pool.stride = record_stride<distance_constraint::DistanceConstraint>();
            break;
        case constraint_id::FIXED:
            pool.stride = record_stride<fixed_constraint::FixedConstraint>();
            break;
        case constraint_id::CONE:
            pool.stride = record_stride<cone_constraint::ConeConstraint>();
            break;
        case constraint_id::HINGE:
            pool.stride = record_stride<hinge_constraint::HingeConstraint>();
            break;
        case constraint_id::SLIDER:
            pool.stride = record_stride<slider_constraint::SliderConstraint>();
            break;
        case constraint_id::SWING_TWIST:
            pool.stride = record_stride<swing_twist_constraint::SwingTwistConstraint>();
            break;
        case constraint_id::SIX_DOF:
            pool.stride = record_stride<six_dof_constraint::SixDOFConstraint>();
            break;
        // there are no user constraint types
        default:
            TUMBLE_ASSERT(false);
            break;
    }
    return pool;
}

ConstraintBase *get(Constraints &constraints, ConstraintId id) {
    const ConstraintType type = constraint_id::type(id);
    if (type >= constraint_id::CONSTRAINT_TYPE_COUNT) return nullptr;
    ConstraintPool &pool = constraints.pools[type];
    const uint32_t index = constraint_id::index(id);
    if (pool.stride == 0 || index >= pool.count) return nullptr;
    ConstraintBase *constraint = base_at(pool, index);
    if (constraint->pooled || constraint->sequence != constraint_id::sequence(id)) return nullptr;
    return constraint;
}

void remove_constraint_by_id(Constraints &constraints, bodies::Bodies &bodies, ConstraintId id) {
    const ConstraintType type = constraint_id::type(id);
    if (type >= constraint_id::CONSTRAINT_TYPE_COUNT) return;
    ConstraintPool &pool = constraints.pools[type];
    const uint32_t index = constraint_id::index(id);
    if (pool.stride == 0 || index >= pool.count) return;

    ConstraintBase *constraint = base_at(pool, index);
    if (!constraint->pooled && constraint->id == id) {
        // remove from bodies' constraint_ids
        const uint32_t a = constraint->body_index_a;
        const uint32_t b = constraint->body_index_b;
        if (is_body_live(bodies, a)) remove_constraint_id_from_body(*bodies.pool[a], id);
        if (a != b && is_body_live(bodies, b)) remove_constraint_id_from_body(*bodies.pool[b], id);

        // return to pool
        pool.free_indices.push(index);
        constraint->pooled = true;
    }
}

void destroy_body_constraints(Constraints &constraints, bodies::Bodies &bodies, rigid_body::RigidBody &body) {
    // backwards, since removing swap-removes from this body's list
    for (uint32_t i = body.constraint_ids.length; i-- > 0;) {
        if (i >= body.constraint_ids.length) continue;
        remove_constraint_by_id(constraints, bodies, body.constraint_ids[i]);
    }
    body.constraint_ids.length = 0;
}

void notify_shape_changed(Constraints &constraints, const rigid_body::RigidBody &body, Vec3f delta_com) {
    for (uint32_t i = 0; i < body.constraint_ids.length; i++) {
        const ConstraintId id = body.constraint_ids[i];
        ConstraintPool &pool = constraints.pools[constraint_id::type(id)];
        const uint32_t index = constraint_id::index(id);
        switch (constraint_id::type(id)) {
            case constraint_id::POINT:
                point_constraint::notify_shape_changed(record_at<point_constraint::PointConstraint>(pool, index), body.id, delta_com);
                break;
            case constraint_id::DISTANCE:
                distance_constraint::notify_shape_changed(record_at<distance_constraint::DistanceConstraint>(pool, index), body.id, delta_com);
                break;
            case constraint_id::FIXED:
                fixed_constraint::notify_shape_changed(record_at<fixed_constraint::FixedConstraint>(pool, index), body.id, delta_com);
                break;
            case constraint_id::CONE:
                cone_constraint::notify_shape_changed(record_at<cone_constraint::ConeConstraint>(pool, index), body.id, delta_com);
                break;
            case constraint_id::HINGE:
                hinge_constraint::notify_shape_changed(record_at<hinge_constraint::HingeConstraint>(pool, index), body.id, delta_com);
                break;
            case constraint_id::SLIDER:
                slider_constraint::notify_shape_changed(record_at<slider_constraint::SliderConstraint>(pool, index), body.id, delta_com);
                break;
            case constraint_id::SWING_TWIST:
                swing_twist_constraint::notify_shape_changed(record_at<swing_twist_constraint::SwingTwistConstraint>(pool, index), body.id,
                                                             delta_com);
                break;
            case constraint_id::SIX_DOF:
                six_dof_constraint::notify_shape_changed(record_at<six_dof_constraint::SixDOFConstraint>(pool, index), body.id, delta_com);
                break;
            default:
                TUMBLE_ASSERT(false);
                break;
        }
    }
}

void get_constraint_iteration_overrides(ConstraintIterationOverrides &out, const Constraints &constraints, ConstraintId id) {
    out.velocity = 0;
    out.position = 0;
    const ConstraintType type = constraint_id::type(id);
    if (type >= constraint_id::CONSTRAINT_TYPE_COUNT) return;
    const ConstraintPool &pool = constraints.pools[type];
    if (pool.stride == 0) return;
    const uint32_t index = constraint_id::index(id);
    TUMBLE_ASSERT(index < pool.count);
    switch (type) {
        case constraint_id::POINT:
            point_constraint::get_iteration_overrides(out, record_at<point_constraint::PointConstraint>(pool, index));
            break;
        case constraint_id::DISTANCE:
            distance_constraint::get_iteration_overrides(out, record_at<distance_constraint::DistanceConstraint>(pool, index));
            break;
        case constraint_id::FIXED:
            fixed_constraint::get_iteration_overrides(out, record_at<fixed_constraint::FixedConstraint>(pool, index));
            break;
        case constraint_id::CONE:
            cone_constraint::get_iteration_overrides(out, record_at<cone_constraint::ConeConstraint>(pool, index));
            break;
        case constraint_id::HINGE:
            hinge_constraint::get_iteration_overrides(out, record_at<hinge_constraint::HingeConstraint>(pool, index));
            break;
        case constraint_id::SLIDER:
            slider_constraint::get_iteration_overrides(out, record_at<slider_constraint::SliderConstraint>(pool, index));
            break;
        case constraint_id::SWING_TWIST:
            swing_twist_constraint::get_iteration_overrides(out, record_at<swing_twist_constraint::SwingTwistConstraint>(pool, index));
            break;
        case constraint_id::SIX_DOF:
            six_dof_constraint::get_iteration_overrides(out, record_at<six_dof_constraint::SixDOFConstraint>(pool, index));
            break;
        default:
            break;
    }
}

void get_constraint_sort_fields(ConstraintSortFields &out, const Constraints &constraints, ConstraintId id) {
    out.priority = 0;
    out.index = 0;
    const ConstraintType type = constraint_id::type(id);
    if (type >= constraint_id::CONSTRAINT_TYPE_COUNT) return;
    const ConstraintPool &pool = constraints.pools[type];
    if (pool.stride == 0) return;
    const uint32_t index = constraint_id::index(id);
    TUMBLE_ASSERT(index < pool.count);
    switch (type) {
        case constraint_id::POINT:
            point_constraint::get_sort_fields(out, record_at<point_constraint::PointConstraint>(pool, index));
            break;
        case constraint_id::DISTANCE:
            distance_constraint::get_sort_fields(out, record_at<distance_constraint::DistanceConstraint>(pool, index));
            break;
        case constraint_id::FIXED:
            fixed_constraint::get_sort_fields(out, record_at<fixed_constraint::FixedConstraint>(pool, index));
            break;
        case constraint_id::CONE:
            cone_constraint::get_sort_fields(out, record_at<cone_constraint::ConeConstraint>(pool, index));
            break;
        case constraint_id::HINGE:
            hinge_constraint::get_sort_fields(out, record_at<hinge_constraint::HingeConstraint>(pool, index));
            break;
        case constraint_id::SLIDER:
            slider_constraint::get_sort_fields(out, record_at<slider_constraint::SliderConstraint>(pool, index));
            break;
        case constraint_id::SWING_TWIST:
            swing_twist_constraint::get_sort_fields(out, record_at<swing_twist_constraint::SwingTwistConstraint>(pool, index));
            break;
        case constraint_id::SIX_DOF:
            six_dof_constraint::get_sort_fields(out, record_at<six_dof_constraint::SixDOFConstraint>(pool, index));
            break;
        default:
            break;
    }
}

/** solve order: lower priority first, so the highest priority is solved last and most likely satisfied; then by index */
static bool sorts_before(const Constraints &constraints, ConstraintId a, ConstraintId b) {
    ConstraintSortFields fields_a;
    ConstraintSortFields fields_b;
    get_constraint_sort_fields(fields_a, constraints, a);
    get_constraint_sort_fields(fields_b, constraints, b);
    if (fields_a.priority != fields_b.priority) return fields_a.priority < fields_b.priority;
    return fields_a.index < fields_b.index;
}

void sort_constraint_ids(const Constraints &constraints, collections::Vec<ConstraintId> &ids) {
    // an island's constraint list is short
    sort::insertion_sort(ids.data, ids.data + ids.length, [&](ConstraintId a, ConstraintId b) { return sorts_before(constraints, a, b); });
}

void update_sleeping(Constraints &constraints, const bodies::Bodies &bodies) {
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        ConstraintPool &pool = constraints.pools[type];
        for (uint32_t i = 0; i < pool.count; i++) {
            ConstraintBase *constraint = base_at(pool, i);
            if (constraint->pooled) continue;
            const rigid_body::RigidBody *body_a = bodies.pool[constraint->body_index_a];
            const rigid_body::RigidBody *body_b = bodies.pool[constraint->body_index_b];
            // asleep unless a body is active; a static body never is
            constraint->sleeping = (body_a->motion_type == motion_type::MOTION_STATIC || body_a->sleeping) &&
                                   (body_b->motion_type == motion_type::MOTION_STATIC || body_b->sleeping);
        }
    }
}

static bool is_solvable(const ConstraintBase *constraint) {
    return !constraint->pooled && constraint->enabled && !constraint->sleeping;
}

void setup_velocity_constraints(Constraints &constraints, bodies::Bodies &bodies, float delta_time) {
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        ConstraintPool &pool = constraints.pools[type];
        for (uint32_t i = 0; i < pool.count; i++) {
            if (!is_solvable(base_at(pool, i))) continue;
            switch (pool.type) {
                case constraint_id::POINT:
                    point_constraint::setup_velocity(record_at<point_constraint::PointConstraint>(pool, i), bodies, delta_time);
                    break;
                case constraint_id::DISTANCE:
                    distance_constraint::setup_velocity(record_at<distance_constraint::DistanceConstraint>(pool, i), bodies,
                                                        delta_time);
                    break;
                case constraint_id::FIXED:
                    fixed_constraint::setup_velocity(record_at<fixed_constraint::FixedConstraint>(pool, i), bodies, delta_time);
                    break;
                case constraint_id::CONE:
                    cone_constraint::setup_velocity(record_at<cone_constraint::ConeConstraint>(pool, i), bodies, delta_time);
                    break;
                case constraint_id::HINGE:
                    hinge_constraint::setup_velocity(record_at<hinge_constraint::HingeConstraint>(pool, i), bodies, delta_time);
                    break;
                case constraint_id::SLIDER:
                    slider_constraint::setup_velocity(record_at<slider_constraint::SliderConstraint>(pool, i), bodies, delta_time);
                    break;
                case constraint_id::SWING_TWIST:
                    swing_twist_constraint::setup_velocity(record_at<swing_twist_constraint::SwingTwistConstraint>(pool, i), bodies,
                                                           delta_time);
                    break;
                case constraint_id::SIX_DOF:
                    six_dof_constraint::setup_velocity(record_at<six_dof_constraint::SixDOFConstraint>(pool, i), bodies, delta_time);
                    break;
                default:
                    break;
            }
        }
    }
}

void warm_start_velocity_constraints(Constraints &constraints, bodies::Bodies &bodies, const collections::Vec<ConstraintId> &ids, float warm_start_impulse_ratio) {
    for (uint32_t i = 0; i < ids.length; i++) {
        const ConstraintId id = ids[i];
        const ConstraintType type = constraint_id::type(id);
        if (type >= constraint_id::CONSTRAINT_TYPE_COUNT) continue;
        ConstraintPool &pool = constraints.pools[type];
        if (pool.stride == 0) continue;
        const uint32_t index = constraint_id::index(id);
        TUMBLE_ASSERT(index < pool.count);

        switch (type) {
            case constraint_id::POINT:
                point_constraint::warm_start_velocity(record_at<point_constraint::PointConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case constraint_id::DISTANCE:
                distance_constraint::warm_start_velocity(record_at<distance_constraint::DistanceConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case constraint_id::FIXED:
                fixed_constraint::warm_start_velocity(record_at<fixed_constraint::FixedConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case constraint_id::CONE:
                cone_constraint::warm_start_velocity(record_at<cone_constraint::ConeConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case constraint_id::HINGE:
                hinge_constraint::warm_start_velocity(record_at<hinge_constraint::HingeConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case constraint_id::SLIDER:
                slider_constraint::warm_start_velocity(record_at<slider_constraint::SliderConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case constraint_id::SWING_TWIST:
                swing_twist_constraint::warm_start_velocity(record_at<swing_twist_constraint::SwingTwistConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case constraint_id::SIX_DOF:
                six_dof_constraint::warm_start_velocity(record_at<six_dof_constraint::SixDOFConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            default:
                break;
        }
    }
}

bool solve_velocity_constraints_for_island(Constraints &constraints, bodies::Bodies &bodies, const collections::Vec<ConstraintId> &ids,
                                           float delta_time) {
    bool any_impulse_applied = false;

    for (uint32_t i = 0; i < ids.length; i++) {
        const ConstraintId id = ids[i];
        const ConstraintType type = constraint_id::type(id);
        if (type >= constraint_id::CONSTRAINT_TYPE_COUNT) continue;
        ConstraintPool &pool = constraints.pools[type];
        if (pool.stride == 0) continue;
        const uint32_t index = constraint_id::index(id);
        TUMBLE_ASSERT(index < pool.count);

        bool applied = false;
        switch (type) {
            case constraint_id::POINT:
                applied = point_constraint::solve_velocity(record_at<point_constraint::PointConstraint>(pool, index), bodies,
                                                           delta_time);
                break;
            case constraint_id::DISTANCE:
                applied = distance_constraint::solve_velocity(record_at<distance_constraint::DistanceConstraint>(pool, index),
                                                              bodies, delta_time);
                break;
            case constraint_id::FIXED:
                applied = fixed_constraint::solve_velocity(record_at<fixed_constraint::FixedConstraint>(pool, index), bodies,
                                                           delta_time);
                break;
            case constraint_id::CONE:
                applied = cone_constraint::solve_velocity(record_at<cone_constraint::ConeConstraint>(pool, index), bodies,
                                                          delta_time);
                break;
            case constraint_id::HINGE:
                applied = hinge_constraint::solve_velocity(record_at<hinge_constraint::HingeConstraint>(pool, index), bodies,
                                                           delta_time);
                break;
            case constraint_id::SLIDER:
                applied = slider_constraint::solve_velocity(record_at<slider_constraint::SliderConstraint>(pool, index), bodies,
                                                            delta_time);
                break;
            case constraint_id::SWING_TWIST:
                applied = swing_twist_constraint::solve_velocity(
                    record_at<swing_twist_constraint::SwingTwistConstraint>(pool, index), bodies, delta_time);
                break;
            case constraint_id::SIX_DOF:
                applied = six_dof_constraint::solve_velocity(record_at<six_dof_constraint::SixDOFConstraint>(pool, index), bodies,
                                                             delta_time);
                break;
            default:
                break;
        }
        any_impulse_applied = any_impulse_applied || applied;
    }

    return any_impulse_applied;
}

bool solve_position_constraints_for_island(Constraints &constraints, bodies::Bodies &bodies, const collections::Vec<ConstraintId> &ids,
                                           float baumgarte_factor, float delta_time) {
    bool applied_impulse = false;

    for (uint32_t i = 0; i < ids.length; i++) {
        const ConstraintId id = ids[i];
        const ConstraintType type = constraint_id::type(id);
        if (type >= constraint_id::CONSTRAINT_TYPE_COUNT) continue;
        ConstraintPool &pool = constraints.pools[type];
        if (pool.stride == 0) continue;
        const uint32_t index = constraint_id::index(id);
        TUMBLE_ASSERT(index < pool.count);

        bool applied = false;
        switch (type) {
            case constraint_id::POINT:
                applied = point_constraint::solve_position(record_at<point_constraint::PointConstraint>(pool, index), bodies,
                                                           delta_time, baumgarte_factor);
                break;
            case constraint_id::DISTANCE:
                applied = distance_constraint::solve_position(record_at<distance_constraint::DistanceConstraint>(pool, index),
                                                              bodies, delta_time, baumgarte_factor);
                break;
            case constraint_id::FIXED:
                applied = fixed_constraint::solve_position(record_at<fixed_constraint::FixedConstraint>(pool, index), bodies,
                                                           delta_time, baumgarte_factor);
                break;
            case constraint_id::CONE:
                applied = cone_constraint::solve_position(record_at<cone_constraint::ConeConstraint>(pool, index), bodies,
                                                          delta_time, baumgarte_factor);
                break;
            case constraint_id::HINGE:
                applied = hinge_constraint::solve_position(record_at<hinge_constraint::HingeConstraint>(pool, index), bodies,
                                                           delta_time, baumgarte_factor);
                break;
            case constraint_id::SLIDER:
                applied = slider_constraint::solve_position(record_at<slider_constraint::SliderConstraint>(pool, index), bodies,
                                                            delta_time, baumgarte_factor);
                break;
            case constraint_id::SWING_TWIST:
                applied = swing_twist_constraint::solve_position(
                    record_at<swing_twist_constraint::SwingTwistConstraint>(pool, index), bodies, delta_time, baumgarte_factor);
                break;
            case constraint_id::SIX_DOF:
                applied = six_dof_constraint::solve_position(record_at<six_dof_constraint::SixDOFConstraint>(pool, index), bodies,
                                                             delta_time, baumgarte_factor);
                break;
            default:
                break;
        }
        applied_impulse = applied_impulse || applied;
    }

    return applied_impulse;
}

void remove_constraint_id_from_body(rigid_body::RigidBody &body, ConstraintId id) {
    for (uint32_t i = 0; i < body.constraint_ids.length; i++) {
        if (body.constraint_ids[i] == id) {
            body.constraint_ids.swap_remove(i);
            return;
        }
    }
}

uint32_t allocate_index(ConstraintPool &pool) {
    if (pool.free_indices.length > 0) return pool.free_indices.data[--pool.free_indices.length];
    // grow only as far as the id has index bits for
    if (pool.count > constraint_id::MAX_CONSTRAINT_INDEX) {
        util::set_error(util::ERROR_QUOTA);
        return ~0u;
    }
    const uint32_t bytes = (pool.count + 1) * pool.stride;
    if (bytes > pool.storage.capacity) {
        const uint32_t doubled = pool.storage.capacity * 2;
        if (!pool.storage.reserve(doubled > bytes ? doubled : bytes)) return ~0u;
    }
    pool.storage.length = bytes;
    base_at(pool, pool.count)->sequence = 0;
    return pool.count++;
}

}  // namespace constraints

TUMBLE_NAMESPACE_END
