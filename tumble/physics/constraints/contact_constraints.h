#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/constraints/constraint_part/angular_friction_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/contact_constraint_part.h"
#include "tumble/physics/constraints/contact_settings.h"
#include "tumble/physics/contacts.h"
#include "tumble/physics/events.h"
#include "tumble/physics/manifold/manifold.h"
#include "tumble/physics/pairs.h"
#include "tumble/physics/world_settings.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// the solver's contact constraints, one per manifold, rebuilt every step from the narrowphase or the pair cache: an
// axis part per point, and a friction pair and a twist part per manifold
namespace contact_constraints {

struct WorldContactPoint {
    /** world space */
    Float3 position_a;
    Float3 position_b;
    /** in each body's local centre of mass space, for the cache */
    Float3 local_position_a;
    Float3 local_position_b;
    contact_constraint_part::ContactConstraintPart normal_constraint;
    /** from the point on a to the mean of a's points, in the contact plane: the moment arm of the angular friction cap */
    float distance_to_friction_center;
};

struct ContactConstraint {
    uint32_t body_index_a = ~0u;
    uint32_t body_index_b = ~0u;
    sub_shape::SubShapeId sub_shape_id_a = sub_shape::EMPTY_SUB_SHAPE_ID;
    sub_shape::SubShapeId sub_shape_id_b = sub_shape::EMPTY_SUB_SHAPE_ID;
    /** world space, from a to b */
    Float3 normal;
    Float3 tangent_1;
    Float3 tangent_2;
    float friction = 0.5f;
    float restitution;
    uint32_t num_contact_points;
    WorldContactPoint contact_points[manifold::MAX_CONTACT_POINTS];
    contact_constraint_part::ContactConstraintPart friction_constraint_1;
    contact_constraint_part::ContactConstraintPart friction_constraint_2;
    angular_friction_constraint_part::AngularFrictionConstraintPart angular_friction_constraint;
    /** the bodies' inverse masses after the contact's scales */
    float inv_mass_a;
    float inv_mass_b;
    float inv_inertia_scale_a = 1.0f;
    float inv_inertia_scale_b = 1.0f;
    /** the contact record this constraint stores its impulses back into */
    int32_t contact_index = -1;
    /** for a deterministic solve order */
    uint64_t sort_key;
};

/** a surface's friction and restitution, which stand in for a body's own on the side of a contact whose part reports its material id */
struct Material {
    float friction;
    float restitution;
};

struct ContactConstraints {
    /** count live, the rest reused */
    collections::Vec<ContactConstraint> pool;
    uint32_t count;
    /** by material id; an id past the end, or -1, takes the body's own */
    collections::Vec<Material> materials;
};

ContactConstraints create();
void release(ContactConstraints &contact_constraints);

/**
 * the combined friction and restitution of a contact, each side's from its material when the table holds the id,
 * else from its body; the bodies' combine modes decide how they combine
 */
void combine_materials(ContactSettings &io_settings, const ContactConstraints &contact_constraints, const rigid_body::RigidBody &body_a,
                       int32_t material_id_a, const rigid_body::RigidBody &body_b, int32_t material_id_b);

/**
 * builds a constraint from a fresh manifold, matching its points against the pair's cached contact for warm
 * starting; appends to events.contacts_added or contacts_persisted. false when the manifold made no constraint
 * (a sensor, or no points).
 */
bool add_contact_constraint(ContactConstraints &contact_constraints, contacts::Contacts &contacts, pairs::Pairs &pairs, int32_t pair_record,
                            rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b, const manifold::ContactManifold &manifold,
                            const world_settings::WorldSettings &settings, events::PhysicsEvents &events, float delta_time,
                            uint32_t step_stamp);
/** builds a constraint from last step's cached manifold when the pair barely moved; appends to contacts_persisted */
bool add_contact_constraint_from_cache(ContactConstraints &contact_constraints, contacts::Contacts &contacts, rigid_body::RigidBody &body_a,
                                       rigid_body::RigidBody &body_b, contacts::Contact &contact, const world_settings::WorldSettings &settings,
                                       events::PhysicsEvents &events, float delta_time, uint32_t step_stamp);
/**
 * rebuilds a constraint's friction, restitution, masses and surface velocities from the caller's answer, before the solve,
 * keeping its warm start impulses; friction the build left off starts from the contact's cached friction impulses. the settings must still make a constraint (not a sensor, some dynamic mass); the normal bias is
 * rebuilt for the new restitution, which is why it takes the world settings and the step's delta time.
 */
void apply_contact_override(ContactConstraints &contact_constraints, uint32_t constraint_index, contacts::Contacts &contacts,
                            bodies::Bodies &bodies, const ContactSettings &settings, const world_settings::WorldSettings &world_settings,
                            float delta_time);
/**
 * every written contact_override, before the solve: by its event's contact, a reject, a sensor or no dynamic mass left (no
 * dynamic body with a nonzero inverse mass scale) drops the contact's constraint and zeroes the impulses its contact caches
 * for the next step; any other answer goes through apply_contact_override. a contact that made no constraint cannot gain
 * one: a sensor cannot become a body by override. the dropped constraints are compacted out, keeping order.
 */
void apply_contact_overrides(ContactConstraints &contact_constraints, contacts::Contacts &contacts, bodies::Bodies &bodies,
                             const events::PhysicsEvents &events, const events::PhysicsAnswers &answers,
                             const world_settings::WorldSettings &world_settings, float delta_time);

/** an island's contact constraints, in their solve order */
void warm_start_velocity_constraints(ContactConstraints &contact_constraints, bodies::Bodies &bodies, const collections::Vec<uint32_t> &constraint_indices,
                                     float warm_start_ratio);
/** true when any impulse was applied */
bool solve_velocity_constraints_for_island(ContactConstraints &contact_constraints, bodies::Bodies &bodies,
                                           const collections::Vec<uint32_t> &constraint_indices);
/** copies every constraint's accumulated impulses into its contact's write manifold */
void store_applied_impulses(ContactConstraints &contact_constraints, contacts::Contacts &contacts);
bool solve_position_constraints_for_island(ContactConstraints &contact_constraints, bodies::Bodies &bodies,
                                           const collections::Vec<uint32_t> &constraint_indices, float penetration_slop, float baumgarte_factor,
                                           float max_penetration_distance);
/**
 * the contact's sort key: the 64 bit fnv-1a hash of body id a, sub shape id a, body id b, sub shape id b, each id as
 * its little endian bytes (a body id as 8: index then sequence)
 */
uint64_t compute_contact_sort_key(body_id::BodyId body_id_a, sub_shape::SubShapeId sub_shape_id_a, body_id::BodyId body_id_b,
                                  sub_shape::SubShapeId sub_shape_id_b);
/** by sort key, for a deterministic solve */
void sort_contact_indices(const ContactConstraints &contact_constraints, const bodies::Bodies &bodies, collections::Vec<uint32_t> &contact_indices);

}  // namespace contact_constraints

TUMBLE_NAMESPACE_END
