#include "tumble/physics/update.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body_step.h"
#include "tumble/physics/body/sleep.h"
#include "tumble/physics/collision/cast_utils.h"
#include "tumble/physics/collision/narrowphase.h"
#include "tumble/physics/constraints/constraint_part/contact_constraint_part.h"
#include "tumble/physics/manifold/manifold.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

namespace update {

using rigid_body::RigidBody;
using world::World;

/** a collide collector's bound that never stops it: below any -penetration */
static constexpr float COLLIDE_EARLY_OUT = -scalar::FLOAT_MAX;
/** the most manifolds the reduction keeps per body pair */
static constexpr uint32_t MAX_REDUCTION_MANIFOLDS = 32;

/** the body's pose as the narrowphase takes it: the shape origin, unscaled */
static Transformf body_transform(const RigidBody &body) { return {body.position, body.quaternion, {1.0f, 1.0f, 1.0f}}; }

/** integrates forces into velocities (a = F/m + g, v += a dt), applies damping and the velocity clamps and the dof masks */
static void acceleration_integration_update(World &world, float delta_time) {
    const float gravity_enabled = world.settings.gravity_enabled ? 1.0f : 0.0f;
    const Vec3f gravity = vec3f::load(world.settings.gravity);
    for (uint32_t i = 0; i < world.bodies.active_body_count; i++) {
        RigidBody *body = world.bodies.pool[world.bodies.active_body_indices[i]];
        if (body->motion_type != motion_type::MOTION_DYNAMIC || body->sleeping) continue;

        motion_properties::MotionProperties &mp = body->motion_properties;

        // linear acceleration: a = F/m + g, then integrate: v += a * dt
        const float gravity_scale = gravity_enabled * mp.gravity_factor;
        const Vec3f acceleration = vec3f::scale_and_add(vec3f::scale(vec3f::load(mp.force), mp.inv_mass), gravity, gravity_scale);
        Vec3f linear_velocity = vec3f::scale_and_add(vec3f::load(mp.linear_velocity), acceleration, delta_time);

        // linear damping: v *= max(0, 1 - damping * dt)
        linear_velocity = vec3f::scale(linear_velocity, scalar::max(0.0f, 1.0f - mp.linear_damping * delta_time));

        vec3f::store(mp.linear_velocity, motion_properties::apply_translation_dof_constraint(
                                             motion_properties::clamp_length(linear_velocity, mp.max_linear_velocity), mp.allowed_degrees_of_freedom));

        // angular acceleration: w += I^-1 t dt. gravity applies no torque, so most bodies have none and skip this; the
        // vector form rotates t into inertia space and back without building the world inverse inertia
        Vec3f angular_velocity = vec3f::load(mp.angular_velocity);
        if (mp.torque.x != 0.0f || mp.torque.y != 0.0f || mp.torque.z != 0.0f) {
            const Vec3f angular_delta =
                motion_properties::multiply_world_space_inverse_inertia_by_vector(mp, quatf::load(body->quaternion), vec3f::load(mp.torque));
            angular_velocity = vec3f::scale_and_add(angular_velocity, angular_delta, delta_time);
        }

        // angular damping: w *= max(0, 1 - damping * dt)
        angular_velocity = vec3f::scale(angular_velocity, scalar::max(0.0f, 1.0f - mp.angular_damping * delta_time));

        vec3f::store(mp.angular_velocity, motion_properties::apply_rotation_dof_constraint(
                                              motion_properties::clamp_length(angular_velocity, mp.max_angular_velocity), mp.allowed_degrees_of_freedom));
    }
}

void step_begin(World &world, float delta_time) {
    world.delta_time = delta_time;

    // every event buffer and answer starts the step empty
    world.events.pairs.length = 0;
    world.events.contacts_added.length = 0;
    world.events.contacts_persisted.length = 0;
    world.events.contacts_removed.length = 0;
    world.events.moved.length = 0;
    world.events.woken = 0;
    world.answers.pair_reject.length = 0;
    world.answers.contact_override.length = 0;

    // a step of no time only brings the broadphase up to date, as jolt's: no collision, no events, the contacts and the
    // queued removals kept for the next step and the forces added for it left to it
    if (delta_time <= 0.0f) {
        broadphase::optimize(world.broadphase, world.bodies);
        return;
    }

    // the contact removals queued by body removal since the last step
    contacts::flush_pending_contact_removed(world.contacts, world.events);

    ccd::clear(world.ccd, world.bodies);

    // the step before's contact constraints
    world.contact_constraints.count = 0;

    // this step's first find_pairs sweeps
    world.pairs.swept = false;

    // a contact is fresh this step iff its last_processed_frame matches
    world.contacts.frame_stamp++;

    // keys the per-step world inverse inertia memo, and never equals STEP_STAMP_NONE
    world.bodies.step_stamp++;
    if (world.bodies.step_stamp == motion_properties::STEP_STAMP_NONE) world.bodies.step_stamp = 0;

    acceleration_integration_update(world, delta_time);

    // dirty-gated balanced rebuild, one tree per step (round-robin)
    broadphase::optimize(world.broadphase, world.bodies);
}

void find_pairs(World &world) {
    if (world.delta_time <= 0.0f) return;
    const float speculative_contact_distance = world.settings.narrowphase.speculative_contact_distance;
    const uint32_t first_pair = world.events.pairs.length;
    if (!world.pairs.swept) {
        pairs::find_colliding_pairs(world.pairs, world.contacts, world.broadphase, world.settings.layers, world.bodies,
                                    speculative_contact_distance, world.events);
    } else {
        // a later call this step: the bodies the narrowphase's contacts woke bring their own pairs
        pairs::find_woken_body_pairs(world.pairs, world.broadphase, world.settings.layers, world.bodies, speculative_contact_distance,
                                     world.events);
    }

    // a reject bit per pair it added, for the caller to set
    events::reset_pair_answers(world.answers, world.events, first_pair);
}

/** the narrowphase collectors' shared state: the body pair and where its constraints go */
struct NarrowphaseContext {
    World *world;
    RigidBody *body_a;
    RigidBody *body_b;
    int32_t pair_record;
    float delta_time;
    /** the reduction collector's manifolds in use */
    uint32_t num_manifolds;
    /** the non-reduction collector: whether any constraint was created */
    bool constraints_created;
};

// the reduction collector's manifolds, with the unit normal of the hit that started each one: hits merge against it and
// pruning projects along it
static manifold::ContactManifold narrowphase_manifolds[MAX_REDUCTION_MANIFOLDS];
static Vec3f narrowphase_first_world_space_normals[MAX_REDUCTION_MANIFOLDS];
static manifold::ContactManifold narrowphase_temp_manifold;

static float max_contact_distance(const World &world) {
    return world.settings.narrowphase.speculative_contact_distance + world.settings.narrowphase.manifold_tolerance;
}

static bool narrowphase_should_early_out(const collide_shape::CollideShapeCollector &collector) {
    return collector.early_out_fraction <= COLLIDE_EARLY_OUT;
}

static void start_manifold(manifold::ContactManifold &m, Vec3f base_offset, Vec3f world_space_normal,
                           const collide_shape::CollideShapeHit &hit) {
    manifold::reset(m);
    m.base_offset = base_offset;
    m.world_space_normal = world_space_normal;
    m.penetration_depth = hit.penetration;
    m.sub_shape_id_a = hit.sub_shape_id_a;
    m.sub_shape_id_b = hit.sub_shape_id_b;
}

/** with manifold reduction: the hit's points join the manifold whose first normal it is within the max delta rotation of */
static void reduction_add_hit(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeHit &hit) {
    NarrowphaseContext &context = *static_cast<NarrowphaseContext *>(collector.context);
    const World &world = *context.world;
    const Vec3f base_offset = vec3f::load(context.body_a->position);
    const float normal_threshold = world.settings.narrowphase.normal_cos_max_delta_rotation;

    Vec3f world_space_normal = vec3f::normalize(hit.penetration_axis);

    // an existing manifold with a similar normal
    int32_t found_index = -1;
    for (uint32_t i = 0; i < context.num_manifolds; i++) {
        if (vec3f::dot(world_space_normal, narrowphase_first_world_space_normals[i]) >= normal_threshold) {
            found_index = (int32_t)i;
            break;
        }
    }

    if (found_index < 0) {
        if (context.num_manifolds >= MAX_REDUCTION_MANIFOLDS) {
            // full: replace the shallowest manifold if this hit is deeper
            uint32_t shallowest = 0;
            for (uint32_t i = 1; i < context.num_manifolds; i++) {
                if (narrowphase_manifolds[i].penetration_depth < narrowphase_manifolds[shallowest].penetration_depth) shallowest = i;
            }
            if (hit.penetration < narrowphase_manifolds[shallowest].penetration_depth) return;
            found_index = (int32_t)shallowest;
        } else {
            found_index = (int32_t)context.num_manifolds;
            context.num_manifolds++;
        }
        start_manifold(narrowphase_manifolds[found_index], base_offset, world_space_normal, hit);
        narrowphase_first_world_space_normals[found_index] = world_space_normal;
    } else {
        // accumulate the normal, normalised before the constraints are made
        manifold::ContactManifold &found = narrowphase_manifolds[found_index];
        found.world_space_normal = vec3f::add(found.world_space_normal, world_space_normal);
    }
    manifold::ContactManifold &found = narrowphase_manifolds[found_index];

    // the hit's contact points, appended to the manifold
    manifold::ContactManifold &temp = narrowphase_temp_manifold;
    manifold::reset(temp);
    temp.base_offset = base_offset;
    manifold::manifold_between_two_faces(temp, hit.point_a, hit.point_b, hit.penetration_axis, max_contact_distance(world), hit.face_a,
                                         hit.face_b);
    for (uint32_t i = 0; i < temp.num_contact_points; i++) {
        if (found.num_contact_points >= manifold::MAX_CLIPPING_CONTACT_POINTS) break;
        found.relative_contact_points_on_a[found.num_contact_points] = temp.relative_contact_points_on_a[i];
        found.relative_contact_points_on_b[found.num_contact_points] = temp.relative_contact_points_on_b[i];
        found.num_contact_points++;
    }

    found.penetration_depth = scalar::max(found.penetration_depth, hit.penetration);

    // prune past 32 points so the next hit has room (the accumulated normal is not unit yet, so along the first)
    if (found.num_contact_points > 32) manifold::prune_contact_points(found, narrowphase_first_world_space_normals[found_index]);
}

/** the reduced manifolds made into contact constraints; whether any was created */
static bool reduction_finalize_and_create_constraints(NarrowphaseContext &context) {
    World &world = *context.world;
    bool constraints_created = false;
    for (uint32_t i = 0; i < context.num_manifolds; i++) {
        manifold::ContactManifold &m = narrowphase_manifolds[i];

        // the sum of every merged hit's normal
        m.world_space_normal = vec3f::normalize(m.world_space_normal);

        if (m.num_contact_points > manifold::MAX_CONTACT_POINTS) manifold::prune_contact_points(m, m.world_space_normal);

        if (m.num_contact_points > 0) {
            const bool created = contact_constraints::add_contact_constraint(
                world.contact_constraints, world.contacts, world.pairs, context.pair_record, *context.body_a, *context.body_b, m,
                world.settings, world.events, context.delta_time, world.bodies.step_stamp);
            constraints_created = constraints_created || created;
        }
    }
    return constraints_created;
}

/** without manifold reduction: a manifold and a constraint per hit */
static void non_reduction_add_hit(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeHit &hit) {
    NarrowphaseContext &context = *static_cast<NarrowphaseContext *>(collector.context);
    World &world = *context.world;

    manifold::ContactManifold &m = narrowphase_temp_manifold;
    Vec3f world_space_normal = vec3f::normalize(hit.penetration_axis);
    start_manifold(m, vec3f::load(context.body_a->position), world_space_normal, hit);
    manifold::manifold_between_two_faces(m, hit.point_a, hit.point_b, hit.penetration_axis, max_contact_distance(world), hit.face_a,
                                         hit.face_b);

    if (m.num_contact_points > manifold::MAX_CONTACT_POINTS) manifold::prune_contact_points(m, m.world_space_normal);

    if (m.num_contact_points > 0) {
        const bool created =
            contact_constraints::add_contact_constraint(world.contact_constraints, world.contacts, world.pairs, context.pair_record,
                                                        *context.body_a, *context.body_b, m, world.settings, world.events,
                                                        context.delta_time, world.bodies.step_stamp);
        context.constraints_created = context.constraints_created || created;
    }
}

/**
 * the narrowphase of one body pair: collides the shapes and makes contact constraints from the manifolds; whether any
 * constraint was created.
 *
 * the collision runs in world space and each shape pair handler moves into a shape's local space, so gjk and epa keep
 * their precision; the constraints take centre of mass local points.
 */
static bool narrowphase_pair(World &world, RigidBody &body_a, RigidBody &body_b, float delta_time, int32_t pair_record) {
    const world_settings::NarrowphaseSettings &narrowphase_settings = world.settings.narrowphase;

    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.collect_faces = true;
    settings.collide_with_backfaces = narrowphase_settings.collide_with_backfaces;
    settings.collision_tolerance = narrowphase_settings.collision_tolerance;
    settings.penetration_tolerance = narrowphase_settings.penetration_tolerance;
    settings.collide_only_with_active_edges = narrowphase_settings.collide_only_with_active_edges;
    settings.internal_edge_removal_vertex_tolerance_sq = narrowphase_settings.internal_edge_removal_vertex_tolerance_sq;
    // the active edges direction from the relative linear velocity
    settings.active_edge_movement_direction = vec3f::subtract(vec3f::load(body_a.motion_properties.linear_velocity), vec3f::load(body_b.motion_properties.linear_velocity));
    settings.max_separation_distance = body_a.sensor || body_b.sensor ? 0.0f : narrowphase_settings.speculative_contact_distance;

    const bool use_manifold_reduction =
        narrowphase_settings.use_manifold_reduction && body_a.use_manifold_reduction && body_b.use_manifold_reduction;

    NarrowphaseContext context = {&world, &body_a, &body_b, pair_record, delta_time, 0, false};
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_CUSTOM);
    collector.body_id_b = body_b.id;
    collector.custom_add_hit = use_manifold_reduction ? reduction_add_hit : non_reduction_add_hit;
    collector.custom_should_early_out = narrowphase_should_early_out;
    collector.context = &context;

    const Transformf transform_a = body_transform(body_a);
    const Transformf transform_b = body_transform(body_b);
    if (body_a.enhanced_internal_edge_removal || body_b.enhanced_internal_edge_removal) {
        narrowphase::collide_shape_vs_shape_with_internal_edge_removal(collector, settings, body_a.shape, sub_shape::builder(), transform_a,
                                                                       body_b.shape, sub_shape::builder(), transform_b);
    } else {
        narrowphase::collide_shape_vs_shape(collector, settings, body_a.shape, sub_shape::builder(), transform_a, body_b.shape,
                                            sub_shape::builder(), transform_b);
    }

    if (use_manifold_reduction) return reduction_finalize_and_create_constraints(context);
    return context.constraints_created;
}

/** body b's centre of mass relative to a in a's frame, and inv(ra) * rb */
static void compute_body_pair_delta(Vec3f &out_delta_position, Quatf &out_delta_rotation, const RigidBody &body_a, const RigidBody &body_b) {
    const Quatf inv_rotation_a = quatf::invert(quatf::load(body_a.quaternion));

    Vec3f difference = vec3f::subtract(vec3f::load(body_b.center_of_mass_position), vec3f::load(body_a.center_of_mass_position));
    out_delta_position = vec3f::transform_quat(difference, inv_rotation_a);

    out_delta_rotation = quatf::multiply(inv_rotation_a, quatf::load(body_b.quaternion));
}

/**
 * the pair from last step's cached manifolds instead of the narrowphase: true when neither body moved enough relative to
 * the other since the pair's last narrowphase and every contact of its chain was rebuilt into a constraint
 */
static bool get_contacts_from_cache(World &world, float delta_time, Vec3f current_delta_position, Quatf current_delta_rotation,
                                    int32_t pair_record) {
    const pairs::PoseCache &cache = world.pairs.pose_cache[(uint32_t)pair_record];

    // no last-narrowphase pose cached for this record: run the narrowphase
    if (!cache.valid) return false;

    // a pair whose last contact died must not cache-hit into reconstructing zero contacts: on wake the record survives
    // with its pose cache valid but its chain emptied, so the narrowphase must run clean
    if (world.pairs.first_contact[(uint32_t)pair_record] == contacts::INVALID_CONTACT_KEY) return false;

    const Vec3f cached_delta_position = vec3f::load(cache.delta_position);
    const Quatf cached_delta_rotation = quatf::load(cache.delta_rotation);

    // |current - cached|^2 within the threshold
    Vec3f difference = vec3f::subtract(current_delta_position, cached_delta_position);
    if (vec3f::squared_length(difference) > world.settings.narrowphase.body_pair_cache_max_delta_position_sq) return false;

    // |current . cached| >= cos(max angle / 2)
    if (scalar::abs(quatf::dot(current_delta_rotation, cached_delta_rotation)) <
        world.settings.narrowphase.body_pair_cache_cos_max_delta_rotation_div_2) {
        return false;
    }

    // every contact of the chain rebuilt from its read side cache
    int32_t contact_index = world.pairs.first_contact[(uint32_t)pair_record];
    while (contact_index != contacts::INVALID_CONTACT_KEY) {
        contacts::Contact &contact = world.contacts.contacts[(uint32_t)contact_index];
        const int32_t next_index = contact.next_in_pair;

        if (contacts::get_read_manifold(contact, world.contacts).num_contact_points > 0) {
            // the contact stores body a's id at most b's; passing the bodies that way round makes the build's swap a no-op
            RigidBody &ordered_a = *world.bodies.pool[contact.body_index_a];
            RigidBody &ordered_b = *world.bodies.pool[contact.body_index_b];
            contact_constraints::add_contact_constraint_from_cache(world.contact_constraints, world.contacts, ordered_a, ordered_b, contact,
                                                                   world.settings, world.events, delta_time, world.bodies.step_stamp);
        }

        contact_index = next_index;
    }

    return true;
}

void narrowphase(World &world) {
    const float delta_time = world.delta_time;
    if (delta_time <= 0.0f) return;
    const bool use_body_pair_cache = world.settings.narrowphase.use_body_pair_contact_cache;
    const collections::Vec<pairs::CollidingPair> &colliding_pairs = world.pairs.colliding_pairs;
    const uint32_t colliding_pair_count = colliding_pairs.length;
    const uint32_t first_override = world.events.contacts_added.length + world.events.contacts_persisted.length;

    // a later call this step picks up where the last stopped, at the pairs find_pairs added since
    for (uint32_t i = world.pairs.narrowphase_index; i < colliding_pair_count; i++) {
        const int32_t pair_record = colliding_pairs[i].pair_record;

        // the caller rejected the pair: no contacts, and the ones it had are gone
        if (events::pair_rejected(world.answers, i)) {
            if (world.pairs.first_contact[(uint32_t)pair_record] != contacts::INVALID_CONTACT_KEY) {
                contacts::destroy_pair_chain(world.contacts, world.pairs, pair_record, world.events, false);
            }
            continue;
        }

        RigidBody *body_a = world.bodies.pool[colliding_pairs[i].body_index_a];
        RigidBody *body_b = world.bodies.pool[colliding_pairs[i].body_index_b];

        // body a has the higher motion type (dynamic trumps kinematic), so the collision runs in the space of a moving body,
        // which avoids accuracy problems testing a very large static object against a small dynamic one; the lower id
        // first when the motion types are the same
        if (body_a->motion_type < body_b->motion_type || (body_a->motion_type == body_b->motion_type && body_b->id < body_a->id)) {
            RigidBody *temp = body_a;
            body_a = body_b;
            body_b = temp;
        }

        // the body pair contact cache: a pair that barely moved relative to itself since the last narrowphase reuses its
        // manifolds, skipping gjk and epa (and the per step jitter of the penetration axis that perturbs friction in stacks)
        bool pair_handled = false;
        Vec3f current_delta_position;
        Quatf current_delta_rotation;
        const bool pair_cacheable = use_body_pair_cache && shapes::caches_pairs(body_a->shape) && shapes::caches_pairs(body_b->shape);
        if (pair_cacheable) {
            compute_body_pair_delta(current_delta_position, current_delta_rotation, *body_a, *body_b);
            pair_handled = get_contacts_from_cache(world, delta_time, current_delta_position, current_delta_rotation, pair_record);
        }

        const bool any_constraints_created = pair_handled ? true : narrowphase_pair(world, *body_a, *body_b, delta_time, pair_record);

        // the cache keeps the relative pose of the last fresh narrowphase, refreshed only when the narrowphase ran: a hit
        // carries the old pose forward so the threshold measures the total drift since then, not since the last step
        if (pair_cacheable && !pair_handled && any_constraints_created) {
            pairs::set_cache(world.pairs, pair_record, current_delta_position, current_delta_rotation);
        }

        // a contact constraint wakes the sleeping dynamic bodies
        if (any_constraints_created) {
            if (body_a->motion_type == motion_type::MOTION_DYNAMIC && body_a->sleeping) rigid_body::wake(world, body_a);
            if (body_b->motion_type == motion_type::MOTION_DYNAMIC && body_b->sleeping) rigid_body::wake(world, body_b);
        }

        // contacts not stamped this step (sub shapes no longer colliding); a cache hit stamped every one
        if (!pair_handled) contacts::destroy_stale_contacts_in_pair(world.contacts, world.pairs, pair_record, world.events);
    }

    world.pairs.narrowphase_index = colliding_pair_count;

    // the bodies its contacts woke: the next find_pairs brings their pairs
    const uint32_t active_body_count = world.bodies.active_body_count;
    world.events.woken = world.pairs.swept && active_body_count > world.pairs.active_read_index ? active_body_count - world.pairs.active_read_index : 0;

    // an override per contact event it added, for the caller to write
    events::reset_contact_answers(world.answers, world.events, first_override);
}

/**
 * wakes the sleeping bodies of active user constraints: enabled, with a body awake and a body dynamic. bodies joined by
 * constraints sleep together, and a wake spreads through a chain one step at a time
 */
static void wake_bodies_in_user_constraints(World &world) {
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        constraints::ConstraintPool &pool = world.constraints.pools[type];
        if (pool.stride == 0) continue;
        for (uint32_t i = 0; i < pool.count; i++) {
            const constraints::ConstraintBase &constraint = *constraints::base_at(pool, i);
            if (constraint.pooled || !constraint.enabled) continue;

            RigidBody *body_a = world.bodies.pool[constraint.body_index_a];
            RigidBody *body_b = world.bodies.pool[constraint.body_index_b];

            // a static body is never active: a sleeping body jointed to one stays asleep
            const bool has_active_body = (body_a->motion_type != motion_type::MOTION_STATIC && !body_a->sleeping) ||
                                         (body_b->motion_type != motion_type::MOTION_STATIC && !body_b->sleeping);
            const bool has_dynamic_body = body_a->motion_type == motion_type::MOTION_DYNAMIC || body_b->motion_type == motion_type::MOTION_DYNAMIC;
            if (has_active_body && has_dynamic_body) {
                if (body_a->sleeping && body_a->motion_type == motion_type::MOTION_DYNAMIC) rigid_body::wake(world, body_a);
                if (body_b->sleeping && body_b->motion_type == motion_type::MOTION_DYNAMIC) rigid_body::wake(world, body_b);
            }
        }
    }
}

/**
 * integrates velocities into positions and rotations. a dynamic linear cast body moving further than its threshold only
 * rotates here and becomes a ccd body; the ccd resolution moves it
 */
static void velocity_integration_update(World &world, float delta_time) {
    for (uint32_t i = 0; i < world.bodies.active_body_count; i++) {
        RigidBody *body = world.bodies.pool[world.bodies.active_body_indices[i]];
        if (body->sleeping) continue;

        motion_properties::MotionProperties &mp = body->motion_properties;

        if (body->motion_type == motion_type::MOTION_DYNAMIC) {
            motion_properties::clamp_linear_velocity(mp);
            motion_properties::clamp_angular_velocity(mp);
        }

        // rotate first, which helps long thin bodies: q' = rotation(w dt) * q
        const Vec3f rotation_vector = vec3f::scale(vec3f::load(mp.angular_velocity), delta_time);
        const float angle = vec3f::length(rotation_vector);
        if (angle > 1e-6f) {
            const Quatf rotation = quatf::set_axis_angle(vec3f::scale(rotation_vector, 1.0f / angle), angle);
            quatf::store(body->quaternion, quatf::normalize(quatf::multiply(rotation, quatf::load(body->quaternion))));
        }

        const Vec3f displacement = vec3f::scale(vec3f::load(mp.linear_velocity), delta_time);

        // ccd only for dynamic linear cast bodies that are not sensors: a kinematic body cannot be stopped by a collision
        bool update_position = true;
        if (mp.motion_quality == motion_properties::MOTION_QUALITY_LINEAR_CAST && body->motion_type == motion_type::MOTION_DYNAMIC && !body->sensor) {
            const float inner_radius = shapes::get_shape_inner_radius(body->shape);
            const float threshold = world.settings.ccd.linear_cast_threshold * inner_radius;
            if (vec3f::squared_length(displacement) > threshold * threshold) {
                ccd::CCDBody ccd_body;
                ccd::reset_ccd_body(ccd_body);
                ccd_body.body_index = body->index;
                ccd_body.hit_body_index = ~0u;
                ccd_body.delta_position = displacement;
                ccd_body.fraction = 1.0f;
                ccd_body.fraction_plus_slop = 1.0f;
                ccd_body.linear_cast_threshold_sq = threshold * threshold;
                ccd_body.max_penetration =
                    scalar::min(world.settings.solver.penetration_slop, world.settings.ccd.linear_cast_max_penetration * inner_radius);
                if (world.ccd.ccd_bodies.push(ccd_body)) {
                    body->ccd_body_index = (int32_t)(world.ccd.ccd_bodies.length - 1);

                    rigid_body::set_quaternion(world, body, quatf::load(body->quaternion), false);

                    // the position waits for the ccd resolution
                    update_position = false;
                }
            }
        }

        if (update_position) {
            // ccd below reads the target's position and aabb, so derive now; the broadphase leaf waits for
            // finish_island_step, the tree only has to be current for the next step
            rigid_body_step::add_position_step(body, displacement);
            rigid_body_step::derive_transform(body);
        }
    }
}

/** a body's motion still to come this step: a dynamic linear cast body has not moved yet, anything else (kinematic too) has */
static void calculate_body_motion(Vec3f &out, const RigidBody &body, float delta_time) {
    if (body.motion_type == motion_type::MOTION_DYNAMIC && body.motion_properties.motion_quality == motion_properties::MOTION_QUALITY_LINEAR_CAST) {
        out = vec3f::scale(vec3f::load(body.motion_properties.linear_velocity), delta_time);
        return;
    }
    out = vec3f::zero();
}

/** one ccd body's sweep through the broadphase: the visitor's and the cast collector's state, and the earliest hit */
struct CCDSweep {
    World *world;
    ccd::CCDBody *ccd_body;
    RigidBody *body_a;
    RigidBody *body_b;
    float delta_time;
    cast_shape::CastShapeSettings cast_settings;
    /** the earliest hit, with the faces the manifold needs */
    cast_shape::CastShapeHit hit;
};

/** the cast collector: keeps the hit that, with the penetration allowance, comes earliest */
static void ccd_add_hit(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeHit &hit) {
    CCDSweep &sweep = *static_cast<CCDSweep *>(collector.context);
    ccd::CCDBody &ccd_body = *sweep.ccd_body;
    const float fraction = hit.fraction;

    const Vec3f normal = vec3f::normalize(hit.penetration_axis);

    // the penetration allowance as a fraction; skips a near-zero denominator
    const float denominator = vec3f::dot(normal, ccd_body.delta_position);
    if (denominator <= ccd_body.max_penetration) return;
    const float fraction_plus_slop = fraction + ccd_body.max_penetration / denominator;

    // only the earliest hit
    if (fraction_plus_slop >= ccd_body.fraction_plus_slop) return;

    cast_shape::copy_hit(sweep.hit, hit);

    ccd_body.hit_body_index = sweep.body_b->index;
    ccd_body.fraction = fraction;
    ccd_body.fraction_plus_slop = fraction_plus_slop;
    ccd_body.contact_normal = normal;
    ccd_body.sub_shape_id_1 = hit.sub_shape_id_a;
    ccd_body.sub_shape_id_2 = hit.sub_shape_id_b;

    // the cast assumed body b still; a linear cast body b has not moved yet, so the hit moves with it to the impact
    Vec3f body_b_motion;
    calculate_body_motion(body_b_motion, *sweep.body_b, sweep.delta_time);
    body_b_motion = vec3f::scale(body_b_motion, fraction);
    if (vec3f::squared_length(body_b_motion) > 1e-12f) {
        sweep.hit.point_a = vec3f::add(sweep.hit.point_a, body_b_motion);
        sweep.hit.point_b = vec3f::add(sweep.hit.point_b, body_b_motion);
        for (uint32_t i = 0; i < sweep.hit.face_a.num_vertices; i++) sweep.hit.face_a.vertices[i] = vec3f::add(sweep.hit.face_a.vertices[i], body_b_motion);
        for (uint32_t i = 0; i < sweep.hit.face_b.num_vertices; i++) sweep.hit.face_b.vertices[i] = vec3f::add(sweep.hit.face_b.vertices[i], body_b_motion);
    }

    contact_constraints::ContactSettings &settings = ccd_body.contact_settings;
    contact_constraints::combine_materials(settings, sweep.world->contact_constraints, *sweep.body_a,
                                           shapes::get_shape_material(sweep.body_a->shape, hit.sub_shape_id_a), *sweep.body_b,
                                           shapes::get_shape_material(sweep.body_b->shape, hit.sub_shape_id_b));
    // ccd supports no sensors; the scales stay 1
    settings.is_sensor = false;

    collector.early_out_fraction = fraction_plus_slop;
}

static bool ccd_should_early_out(const cast_shape::CastShapeCollector &) { return false; }

/** a body the swept bounds reach: the filters, a ray test against its bounds, then the shape cast */
static void visit_ccd_body(body_visitor::BodyVisitor *visitor, RigidBody *body_b) {
    CCDSweep &sweep = *static_cast<CCDSweep *>(visitor->context);
    World &world = *sweep.world;
    ccd::CCDBody &ccd_body = *sweep.ccd_body;
    RigidBody &body_a = *sweep.body_a;

    if (body_a.index == body_b->index) return;

    // two ccd bodies: the lower body index does the pair
    const int32_t ccd_body_b_index = body_b->ccd_body_index;
    if (ccd_body_b_index >= 0 && ccd_body.body_index > world.ccd.ccd_bodies[(uint32_t)ccd_body_b_index].body_index) return;

    if (!filter::should_pair_collide(body_a.collision_groups, body_a.collision_mask, body_b->collision_groups, body_b->collision_mask)) return;

    // ccd supports no sensors
    if (body_b->sensor) return;

    // every dynamic linear cast body counts as moving by velocity * dt, also one below its threshold that already moved
    Vec3f body_b_motion;
    calculate_body_motion(body_b_motion, *body_b, sweep.delta_time);
    const Vec3f relative_displacement = vec3f::subtract(ccd_body.delta_position, body_b_motion);

    if (vec3f::squared_length(relative_displacement) < ccd_body.linear_cast_threshold_sq) return;

    // a ray from a's position along the relative displacement against b's bounds grown by a's half extent. the ray's
    // length is 1, so the slab values and the cap are both sweep fractions
    const Vec3f extent = vec3f::scale(aabox::size(aabox::load(body_a.aabb)), 0.5f);
    const AABox expanded = aabox::expand_by_extents(aabox::load(body_b->aabb), extent);
    const cast_ray::Rayf ray = {vec3f::load(body_a.position), relative_displacement, 1.0f};
    if (!cast_utils::ray_hits_box3(ray, expanded)) return;

    sweep.body_b = body_b;
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CUSTOM);
    collector.body_id_b = body_b->id;
    collector.custom_add_hit = ccd_add_hit;
    collector.custom_should_early_out = ccd_should_early_out;
    collector.context = &sweep;
    // starts from the visitor's best so far
    collector.early_out_fraction = visitor->early_out_fraction;

    // faces for the manifold, the deepest point of a start overlap, the shrunken shape and convex radius, back faces
    // ignored, active edges by the world setting along the relative displacement
    sweep.cast_settings.collect_faces = true;
    sweep.cast_settings.return_deepest_point = true;
    sweep.cast_settings.use_shrunken_shape_and_convex_radius = true;
    sweep.cast_settings.collide_with_backfaces_triangles = world.settings.narrowphase.collide_with_backfaces;
    sweep.cast_settings.collide_with_backfaces_convex = false;
    sweep.cast_settings.collide_only_with_active_edges = world.settings.narrowphase.collide_only_with_active_edges;
    sweep.cast_settings.active_edge_movement_direction = relative_displacement;

    narrowphase::cast_shape_vs_shape(collector, sweep.cast_settings, body_a.shape, sub_shape::builder(), body_transform(body_a),
                                     relative_displacement, body_b->shape, sub_shape::builder(), body_transform(*body_b));

    visitor->early_out_fraction = scalar::min(visitor->early_out_fraction, collector.early_out_fraction);
}

/** the contact of a ccd hit: the combined material, and the contact and its event under the pair (made when the broadphase never found it) */
static void on_ccd_contact_added(World &world, RigidBody &body_a, RigidBody &body_b, const manifold::ContactManifold &contact_manifold,
                                 contact_constraints::ContactSettings &contact_settings) {
    contact_constraints::combine_materials(contact_settings, world.contact_constraints, body_a,
                                           shapes::get_shape_material(body_a.shape, contact_manifold.sub_shape_id_a), body_b,
                                           shapes::get_shape_material(body_b.shape, contact_manifold.sub_shape_id_b));
    contact_settings.is_sensor = body_a.sensor || body_b.sensor;

    // the contact keeps body a's id at most b's, as add_contact_constraint orders them
    RigidBody *ordered_a = &body_a;
    RigidBody *ordered_b = &body_b;
    const manifold::ContactManifold *ordered_manifold = &contact_manifold;
    manifold::ContactManifold &swapped = narrowphase_temp_manifold;
    if (body_a.id > body_b.id) {
        ordered_a = &body_b;
        ordered_b = &body_a;
        swapped = contact_manifold;
        manifold::swap_shapes(swapped);
        ordered_manifold = &swapped;
    }

    // ccd can hit a pair the broadphase never found: a real contact implies a real overlapping pair, and the next sweep
    // keeps the record if the fat bounds overlap or destroys it through the usual cascade
    int32_t record = pairs::find_pair_record(world.pairs, *ordered_a, *ordered_b);
    if (record == pairs::INVALID_PAIR_KEY) record = pairs::add_pair_record(world.pairs, *ordered_a, *ordered_b);
    if (record == pairs::INVALID_PAIR_KEY) return;

    contacts::Contact *contact = contacts::find_contact_in_pair(world.contacts, world.pairs, record, ordered_manifold->sub_shape_id_a,
                                                                ordered_manifold->sub_shape_id_b);
    const bool persisted = contact != nullptr;
    if (persisted) {
        // a ccd contact so its stale impulses never warm start
        contacts::get_write_manifold(*contact, world.contacts).flags |= contacts::MANIFOLD_FLAG_CCD_CONTACT;
    } else {
        contact = contacts::create_contact(world.contacts, world.pairs, record, *ordered_a, *ordered_b, ordered_manifold->sub_shape_id_a,
                                           ordered_manifold->sub_shape_id_b);
        if (contact == nullptr) return;
        contacts::get_write_manifold(*contact, world.contacts).flags = contacts::MANIFOLD_FLAG_CCD_CONTACT;
    }
    contact->last_processed_frame = world.contacts.frame_stamp;

    // the ccd contact as an added or persisted event, after the caller answered this step's overrides
    events::ContactEvent event;
    event.contact_index = contact->contact_index;
    event.body_index_a = ordered_a->index;
    event.body_index_b = ordered_b->index;
    event.body_id_a = ordered_a->id;
    event.body_id_b = ordered_b->id;
    event.user_data_a = ordered_a->user_data;
    event.user_data_b = ordered_b->user_data;
    event.sub_shape_id_a = ordered_manifold->sub_shape_id_a;
    event.sub_shape_id_b = ordered_manifold->sub_shape_id_b;
    event.material_id_a = shapes::get_shape_material(ordered_a->shape, ordered_manifold->sub_shape_id_a);
    event.material_id_b = shapes::get_shape_material(ordered_b->shape, ordered_manifold->sub_shape_id_b);
    vec3f::store(event.normal, ordered_manifold->world_space_normal);
    event.penetration = ordered_manifold->penetration_depth;
    // an unpruned ccd manifold reports its first MAX_CONTACT_POINTS
    event.num_points = ordered_manifold->num_contact_points < manifold::MAX_CONTACT_POINTS ? ordered_manifold->num_contact_points
                                                                                    : manifold::MAX_CONTACT_POINTS;
    for (uint32_t i = 0; i < manifold::MAX_CONTACT_POINTS; i++) {
        if (i < event.num_points) {
            vec3f::store(event.points_a[i], manifold::get_world_space_contact_point_on_a(*ordered_manifold, i));
            vec3f::store(event.points_b[i], manifold::get_world_space_contact_point_on_b(*ordered_manifold, i));
        } else {
            vec3f::store(event.points_a[i], vec3f::zero());
            vec3f::store(event.points_b[i], vec3f::zero());
        }
    }
    event.settings = contact_settings;
    (persisted ? world.events.contacts_persisted : world.events.contacts_added).push(event);
}

/** each ccd body's earliest hit along its displacement, after velocity integration */
static void find_ccd_contacts(World &world, float delta_time) {
    filter::Filter body_filter = filter::create_empty();

    for (uint32_t i = 0; i < world.ccd.ccd_bodies.length; i++) {
        ccd::CCDBody &ccd_body = world.ccd.ccd_bodies[i];
        RigidBody &body_a = *world.bodies.pool[ccd_body.body_index];

        Box3f swept_aabb;
        aabox::store(swept_aabb, ccd::compute_swept_aabb(body_a, ccd_body.delta_position));

        CCDSweep sweep;
        sweep.world = &world;
        sweep.ccd_body = &ccd_body;
        sweep.body_a = &body_a;
        sweep.body_b = nullptr;
        sweep.delta_time = delta_time;
        sweep.cast_settings = cast_shape::default_settings();
        sweep.hit = cast_shape::create_hit();

        body_visitor::BodyVisitor visitor = body_visitor::create(visit_ccd_body, &sweep);
        visitor.early_out_fraction = ccd_body.fraction_plus_slop;

        filter::set_from_body(body_filter, world.settings.layers, &body_a);
        broadphase::cast_aabb(world.broadphase, world.bodies, swept_aabb, ccd_body.delta_position, body_filter, visitor);

        if (ccd_body.fraction_plus_slop < 1.0f) {
            RigidBody &body_b = *world.bodies.pool[ccd_body.hit_body_index];
            const cast_shape::CastShapeHit &hit = sweep.hit;

            // points by clipping, keeping face points within the manifold tolerance of the hit; relative to a's centre
            // of mass at the start of the sweep
            manifold::ContactManifold &contact_manifold = narrowphase_manifolds[0];
            manifold::reset(contact_manifold);
            contact_manifold.base_offset = vec3f::load(body_a.center_of_mass_position);
            manifold::manifold_between_two_faces(contact_manifold, hit.point_a, hit.point_b, hit.penetration_axis,
                                                 world.settings.narrowphase.manifold_tolerance, hit.face_a, hit.face_b);
            contact_manifold.sub_shape_id_a = hit.sub_shape_id_a;
            contact_manifold.sub_shape_id_b = hit.sub_shape_id_b;
            contact_manifold.penetration_depth = hit.penetration_depth;
            contact_manifold.world_space_normal = ccd_body.contact_normal;

            // not pruned: every clipped point counts towards the mean below
            on_ccd_contact_added(world, body_a, body_b, contact_manifold, ccd_body.contact_settings);

            if (ccd_body.contact_settings.is_sensor) {
                ccd_body.fraction_plus_slop = 1.0f;
                ccd_body.hit_body_index = ~0u;
            } else if (contact_manifold.num_contact_points > 1) {
                // the mean of the points gives the same impulse as applying it at every point
                Vec3f average = vec3f::make(0.0f, 0.0f, 0.0f);
                for (uint32_t j = 0; j < contact_manifold.num_contact_points; j++) {
                    average = vec3f::add(average, contact_manifold.relative_contact_points_on_b[j]);
                }
                average = vec3f::scale(average, 1.0f / (float)contact_manifold.num_contact_points);
                ccd_body.contact_point = vec3f::add(contact_manifold.base_offset, average);
            } else {
                ccd_body.contact_point = hit.point_b;
            }
        }
    }

    filter::release(body_filter);
}

/** the world inverse inertia of a dynamic body at its rotation, scaled by a contact's inverse inertia scale */
static Mat4f scaled_inverse_inertia(const RigidBody &body, float scale) {
    const Mat4f inverse_inertia =
        motion_properties::get_inverse_inertia_for_rotation(body.motion_properties, mat4f::from_quat(quatf::load(body.quaternion)));
    return scale == 1.0f ? inverse_inertia : mat4f::multiply_scalar(inverse_inertia, scale);
}

/**
 * the impulse of a ccd contact on its dynamic body a: one velocity iteration of a normal part and, within the friction
 * cone, a friction part
 */
template <motion_type::MotionType TYPE_B>
static void resolve_ccd_contact(World &world, const ccd::CCDBody &ccd_body, RigidBody &body_a, RigidBody &body_b) {
    constexpr motion_type::MotionType TYPE_A = motion_type::MOTION_DYNAMIC;

    // r1 + u: the contact point relative to a's centre of mass at the impact
    const Vec3f contact_point = ccd_body.contact_point;
    const Vec3f contact_normal = ccd_body.contact_normal;
    const Vec3f body_a_position_at_hit =
        vec3f::scale_and_add(vec3f::load(body_a.center_of_mass_position), ccd_body.delta_position, ccd_body.fraction);
    const Vec3f r1_plus_u = vec3f::subtract(contact_point, body_a_position_at_hit);
    const Vec3f r2 = vec3f::subtract(contact_point, vec3f::load(body_b.center_of_mass_position));

    const Vec3f v1 = rigid_body::get_velocity_at_point_com(&body_a, r1_plus_u);
    const Vec3f v2 = TYPE_B != motion_type::MOTION_STATIC ? rigid_body::get_velocity_at_point_com(&body_b, r2) : vec3f::zero();
    const Vec3f relative_velocity = vec3f::subtract(v2, v1);
    const float normal_velocity = vec3f::dot(relative_velocity, contact_normal);

    const contact_constraints::ContactSettings &settings = ccd_body.contact_settings;

    float normal_velocity_bias = 0.0f;
    if (settings.combined_restitution > 0.0f && normal_velocity < -world.settings.ccd.min_velocity_for_restitution) {
        normal_velocity_bias = settings.combined_restitution * normal_velocity;
    }

    const float inv_mass_a = settings.inv_mass_scale_1 * body_a.motion_properties.inv_mass;
    const Mat4f inv_inertia_a = scaled_inverse_inertia(body_a, settings.inv_inertia_scale_1);
    float inv_mass_b = 0.0f;
    Mat4f inv_inertia_b = mat4f::zero();
    if constexpr (TYPE_B == motion_type::MOTION_DYNAMIC) {
        inv_mass_b = settings.inv_mass_scale_2 * body_b.motion_properties.inv_mass;
        inv_inertia_b = scaled_inverse_inertia(body_b, settings.inv_inertia_scale_2);
    }

    Vec3f linear_velocity_a = vec3f::load(body_a.motion_properties.linear_velocity);
    Vec3f angular_velocity_a = vec3f::load(body_a.motion_properties.angular_velocity);
    Vec3f linear_velocity_b = vec3f::zero();
    Vec3f angular_velocity_b = vec3f::zero();
    if constexpr (TYPE_B != motion_type::MOTION_STATIC) {
        linear_velocity_b = vec3f::load(body_b.motion_properties.linear_velocity);
        angular_velocity_b = vec3f::load(body_b.motion_properties.angular_velocity);
    }

    contact_constraint_part::ContactConstraintPart normal_constraint = {};
    contact_constraint_part::calculate_constraint_properties<TYPE_A, TYPE_B>(normal_constraint, inv_mass_a, inv_inertia_a, r1_plus_u, inv_mass_b,
                                                                             inv_inertia_b, r2, contact_normal, normal_velocity_bias);
    contact_constraint_part::solve_velocity_constraint<TYPE_A, TYPE_B>(normal_constraint, linear_velocity_a, angular_velocity_a,
                                                                       linear_velocity_b, angular_velocity_b, inv_mass_a, inv_mass_b,
                                                                       contact_normal, -scalar::FLOAT_MAX, scalar::FLOAT_MAX);

    if (settings.combined_friction > 0.0f) {
        // the tangential velocity, the normal part removed
        const Vec3f tangent_velocity = vec3f::scale_and_add(relative_velocity, contact_normal, -normal_velocity);
        const float tangent_length_sq = vec3f::squared_length(tangent_velocity);
        if (tangent_length_sq > 1e-12f) {
            const Vec3f friction_direction = vec3f::scale(tangent_velocity, 1.0f / scalar::sqrt(tangent_length_sq));

            // the coulomb cone from the normal lambda
            const float max_friction = settings.combined_friction * normal_constraint.total_lambda;

            contact_constraint_part::ContactConstraintPart friction_constraint = {};
            contact_constraint_part::calculate_constraint_properties<TYPE_A, TYPE_B>(
                friction_constraint, inv_mass_a, inv_inertia_a, r1_plus_u, inv_mass_b, inv_inertia_b, r2, friction_direction, 0.0f);
            contact_constraint_part::solve_velocity_constraint<TYPE_A, TYPE_B>(friction_constraint, linear_velocity_a, angular_velocity_a,
                                                                               linear_velocity_b, angular_velocity_b, inv_mass_a, inv_mass_b,
                                                                               friction_direction, -max_friction, max_friction);
        }
    }

    motion_properties::set_linear_velocity(
        body_a.motion_properties,
        motion_properties::apply_translation_dof_constraint(linear_velocity_a, body_a.motion_properties.allowed_degrees_of_freedom));
    motion_properties::set_angular_velocity(body_a.motion_properties, angular_velocity_a);
    if constexpr (TYPE_B == motion_type::MOTION_DYNAMIC) {
        motion_properties::set_linear_velocity(
            body_b.motion_properties,
            motion_properties::apply_translation_dof_constraint(linear_velocity_b, body_b.motion_properties.allowed_degrees_of_freedom));
        motion_properties::set_angular_velocity(body_b.motion_properties, angular_velocity_b);
    }
}

static void apply_ccd(World &world, const ccd::CCDBody &ccd_body, RigidBody &body_a, RigidBody &body_b) {
    if (body_a.motion_type != motion_type::MOTION_DYNAMIC) return;
    switch (body_b.motion_type) {
        case motion_type::MOTION_STATIC: resolve_ccd_contact<motion_type::MOTION_STATIC>(world, ccd_body, body_a, body_b); break;
        case motion_type::MOTION_KINEMATIC: resolve_ccd_contact<motion_type::MOTION_KINEMATIC>(world, ccd_body, body_a, body_b); break;
        case motion_type::MOTION_DYNAMIC: resolve_ccd_contact<motion_type::MOTION_DYNAMIC>(world, ccd_body, body_a, body_b); break;
    }
}

/** earliest impact first, the body index breaking ties */
static bool ccd_body_sorts_before(const ccd::CCDBody &a, const ccd::CCDBody &b) {
    if (a.fraction_plus_slop != b.fraction_plus_slop) return a.fraction_plus_slop < b.fraction_plus_slop;
    return a.body_index < b.body_index;
}

/**
 * the ccd bodies in time of impact order (fraction_plus_slop, which includes the penetration allowance, for stability), as
 * indices into ccd.order so the records and every body's ccd_body_index stay where they are
 */
static void sort_ccd_bodies(ccd::CCD &ccd) {
    collections::Vec<uint32_t> &order = ccd.order;
    order.length = 0;
    if (!order.reserve(ccd.ccd_bodies.length)) return;
    for (uint32_t i = 0; i < ccd.ccd_bodies.length; i++) order.data[i] = i;
    order.length = ccd.ccd_bodies.length;
    sort::insertion_sort(order.data, order.data + order.length,
                         [&](uint32_t a, uint32_t b) { return ccd_body_sorts_before(ccd.ccd_bodies[a], ccd.ccd_bodies[b]); });
}

/** in time of impact order: each ccd body moves to its impact (or the whole way), with the impulse of its contact */
static void resolve_ccd_contacts(World &world) {
    const collections::Vec<uint32_t> &order = world.ccd.order;
    for (uint32_t i = 0; i < order.length; i++) {
        ccd::CCDBody &ccd_body = world.ccd.ccd_bodies[order[i]];
        RigidBody &body_a = *world.bodies.pool[ccd_body.body_index];

        if (ccd_body.hit_body_index != ~0u) {
            RigidBody &body_b = *world.bodies.pool[ccd_body.hit_body_index];
            ccd::CCDBody *ccd_body_b = body_b.ccd_body_index >= 0 ? &world.ccd.ccd_bodies[(uint32_t)body_b.ccd_body_index] : nullptr;

            // hitting a ccd body earlier than it hits anything itself cancels its own hit (we do the response) and shortens
            // its travel to the time of our impact
            if (ccd_body_b != nullptr && ccd_body_b->fraction > ccd_body.fraction) {
                ccd_body_b->hit_body_index = ~0u;
                ccd_body_b->fraction_plus_slop = ccd_body.fraction;
            }

            // a ccd body that stopped against something else before we reached it is not where we hit it, so there is no
            // response. we still only move to the impact, the last position known to be free
            if (ccd_body_b == nullptr || ccd_body_b->fraction >= ccd_body.fraction) {
                apply_ccd(world, ccd_body, body_a, body_b);
                if (body_b.sleeping) rigid_body::wake(world, &body_b);
            }
        }

        // to the impact with the penetration allowance, or as far as an earlier hit by another body allows (only dynamic
        // bodies make ccd bodies, kinematic ones already moved in velocity integration)
        const Vec3f position = vec3f::scale_and_add(vec3f::load(body_a.position), ccd_body.delta_position, ccd_body.fraction_plus_slop);
        rigid_body::set_transform(world, &body_a, position, quatf::load(body_a.quaternion), false);
    }
}

void solve(World &world) {
    const float delta_time = world.delta_time;
    if (delta_time <= 0.0f) return;

    // the caller's answers to this step's contacts: a reject drops the constraint, anything else rebuilds it from the settings
    contact_constraints::apply_contact_overrides(world.contact_constraints, world.contacts, world.bodies, world.events, world.answers,
                                                 world.settings, delta_time);

    wake_bodies_in_user_constraints(world);

    islands::prepare(world.islands, world.bodies, world.contact_constraints.count);
    islands::link_contact_constraints(world.islands, world.contact_constraints, world.contacts, world.bodies);
    islands::link_user_constraints(world.islands, world.constraints, world.bodies);
    islands::finalize(world.islands, world.bodies, world.constraints, world.settings);

    // a constraint whose bodies all sleep sleeps
    constraints::update_sleeping(world.constraints, world.bodies);

    constraints::setup_velocity_constraints(world.constraints, world.bodies, delta_time);

    // the cached impulses scaled by the ratio of this step's delta time to the last, against overshoot when the step length
    // varies; no warm start after a step of 0. the ratio is clamped: a body that just landed carries the large impulse that
    // stopped it, far more than resting needs, and scaled up by a much longer step it would gain a velocity the solver can
    // no longer take back (a stack in equilibrium is a little less stable across a big jump in step length)
    const float warm_start_impulse_ratio =
        world.previous_time_step > 0.0f ? scalar::min(delta_time / world.previous_time_step, 4.0f) : 0.0f;

    world.previous_time_step = delta_time;

    for (uint32_t i = 0; i < world.islands.islands.length; i++) {
        islands::Island &island = world.islands.islands[i];
        if (island.contact_indices.length == 0 && island.constraint_ids.length == 0) continue;

        // sorted for a deterministic simulation; the order holds for the position solve
        constraints::sort_constraint_ids(world.constraints, island.constraint_ids);
        contact_constraints::sort_contact_indices(world.contact_constraints, world.bodies, island.contact_indices);

        // warm started in the sorted order, so the result does not depend on the order the contacts were found
        if (world.settings.solver.warm_starting) {
            constraints::warm_start_velocity_constraints(world.constraints, world.bodies, island.constraint_ids, warm_start_impulse_ratio);
            contact_constraints::warm_start_velocity_constraints(world.contact_constraints, world.bodies, island.contact_indices,
                                                                 warm_start_impulse_ratio);
        }

        for (uint32_t step = 0; step < island.num_velocity_steps; step++) {
            bool applied_impulse = false;
            if (island.contact_indices.length > 0) {
                applied_impulse =
                    contact_constraints::solve_velocity_constraints_for_island(world.contact_constraints, world.bodies, island.contact_indices);
            }
            if (island.constraint_ids.length > 0) {
                applied_impulse = constraints::solve_velocity_constraints_for_island(world.constraints, world.bodies, island.constraint_ids,
                                                                                     delta_time) ||
                                  applied_impulse;
            }
            // converged
            if (!applied_impulse) break;
        }
    }

    // the impulses warm start the next step
    contact_constraints::store_applied_impulses(world.contact_constraints, world.contacts);

    velocity_integration_update(world, delta_time);

    find_ccd_contacts(world, delta_time);
    sort_ccd_bodies(world.ccd);
    resolve_ccd_contacts(world);

    const world_settings::SolverSettings &solver = world.settings.solver;
    for (uint32_t i = 0; i < world.islands.islands.length; i++) {
        islands::Island &island = world.islands.islands[i];
        if (island.contact_indices.length == 0 && island.constraint_ids.length == 0) continue;

        for (uint32_t step = 0; step < island.num_position_steps; step++) {
            bool applied_impulse = false;
            if (island.contact_indices.length > 0) {
                applied_impulse = contact_constraints::solve_position_constraints_for_island(
                    world.contact_constraints, world.bodies, island.contact_indices, solver.penetration_slop, solver.baumgarte_factor,
                    solver.max_penetration_distance);
            }
            if (island.constraint_ids.length > 0) {
                applied_impulse = constraints::solve_position_constraints_for_island(world.constraints, world.bodies, island.constraint_ids,
                                                                                     solver.baumgarte_factor, delta_time) ||
                                  applied_impulse;
            }
            if (!applied_impulse) break;
        }
    }

    for (uint32_t i = 0; i < world.islands.islands.length; i++) {
        islands::finish_island_step(world.islands.islands[i], world.bodies, world.broadphase, world.pairs, world.settings, delta_time,
                                    world.events);
    }
}

void step_end(World &world) {
    if (world.delta_time <= 0.0f) return;

    // this step's cache writes become the next step's reads
    contacts::flip_manifold_cache(world.contacts);
}

}  // namespace update

TUMBLE_NAMESPACE_END
