#pragma once

#include <stdint.h>

#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/mass_properties.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/sleep.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/constraints/combine_material.h"
#include "tumble/physics/constraints/constraint_id.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/layers.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// a body lives in a slot of bodies::Bodies; adding it to and refreshing it in the broadphase is the caller's
namespace rigid_body {

enum MassPropertiesOverride : uint32_t {
    /** both from the shape's density */
    CALCULATE_MASS_AND_INERTIA = 0,
    /** the mass given, the inertia from the shape scaled to it */
    CALCULATE_INERTIA = 1,
    /** both given */
    MASS_AND_INERTIA_PROVIDED = 2,
};

struct RigidBodySettings {
    /** null makes a body with an empty shape */
    shapes::Shape *shape = nullptr;

    uint32_t object_layer;

    motion_type::MotionType motion_type;

    /** the user's own value for the body, reported back with it in events */
    uint64_t user_data = 0;

    Float3 position = {0.0f, 0.0f, 0.0f};

    Float4 quaternion = {0.0f, 0.0f, 0.0f, 1.0f};

    motion_properties::MotionQuality motion_quality = motion_properties::MOTION_QUALITY_DISCRETE;

    /** a dof mask */
    uint32_t allowed_degrees_of_freedom = 0b111111;

    /** scales gravity */
    float gravity_factor = 1.0f;
    float linear_damping = 0.05f;
    float angular_damping = 0.05f;
    float max_linear_velocity = 500.0f;
    float max_angular_velocity = 0.25f * scalar::PI * 60.0f;

    /** for a dynamic body in contact; 0 takes the world's. an island runs the most of its contacts and constraints */
    uint32_t num_velocity_steps_override = 0;
    /** for a dynamic body in contact; 0 takes the world's. an island runs the most of its contacts and constraints */
    uint32_t num_position_steps_override = 0;

    float friction = 0.2f;
    float restitution = 0.0f;

    combine_material::MaterialCombineMode friction_combine_mode = combine_material::GEOMETRIC_MEAN;
    combine_material::MaterialCombineMode restitution_combine_mode = combine_material::MAX;

    /** the groups it is in */
    uint32_t collision_groups = 0xffffffffu;
    /** the groups it collides with */
    uint32_t collision_mask = 0xffffffffu;

    /** see RigidBody::use_manifold_reduction */
    bool use_manifold_reduction = true;

    bool allow_sleeping = true;

    bool sensor = false;

    /** no ghost collisions on internal edges */
    bool enhanced_internal_edge_removal = false;

    /** a kinematic body that also collides with static and kinematic bodies */
    bool collide_kinematic_vs_non_dynamic = false;

    /** the world's contact pairs record the pairs this body is a side of */
    bool observes_contacts = false;

    /** the mass alone, negative for none: the inertia from the shape scaled to it. ignored with mass_properties_override */
    float mass = -1.0f;

    /**
     * mass and inertia, null for none: for shapes they cannot be derived from, such as triangle meshes, or custom values.
     * copied, not referenced; wins over mass
     */
    const mass_properties::MassProperties *mass_properties_override = nullptr;
};

struct RigidBody {
    body_id::BodyId id;

    /** the id's index: the slot in Bodies::pool */
    uint32_t index;

    /** the id's sequence */
    uint32_t sequence;

    uint64_t user_data;

    shapes::Shape *shape;

    /** at the shape's origin, world space */
    Float3 position;

    Float4 quaternion;

    /** world space */
    Float3 center_of_mass_position;

    motion_type::MotionType motion_type;

    mass_properties::MassProperties mass_properties;

    MassPropertiesOverride mass_properties_override;

    motion_properties::MotionProperties motion_properties;

    float friction;

    float restitution;

    combine_material::MaterialCombineMode friction_combine_mode;

    combine_material::MaterialCombineMode restitution_combine_mode;

    /** the groups it is in */
    uint32_t collision_groups;

    /** the groups it collides with */
    uint32_t collision_mask;

    /**
     * contacts of different sub shapes (a mesh's triangles, a compound's children) with similar normals merge. off when
     * exactly which sub shapes touch matters, at a cost
     */
    bool use_manifold_reduction;

    /** detects collisions without responding to them */
    bool sensor;

    /** no ghost collisions on internal edges */
    bool enhanced_internal_edge_removal;

    /** a kinematic body that also collides with static and kinematic bodies */
    bool collide_kinematic_vs_non_dynamic;

    /** the world's contact pairs record the pairs this body is a side of */
    bool observes_contacts;

    /** read only */
    bool sleeping;

    /** world space */
    Box3f aabb;

    uint32_t object_layer;

    /** -1 if none */
    int32_t broadphase_layer;

    /** its place in the broadphase tree (a node and child), -1 if none */
    int32_t broadphase_node;

    /** in the broadphase move set, awaiting pair discovery */
    bool in_move_set;

    /** the head of its list of persistent pairs, a pairs::pair_edge_key; -1 for none */
    int32_t head_pair_key;

    /** set by island building, -1 when in none */
    int32_t island_index;

    /** its place in the active list (dynamic and kinematic bodies); INACTIVE_BODY_INDEX otherwise */
    uint32_t active_index;

    /** its place in the step's ccd bodies, -1 when it has none */
    int32_t ccd_body_index;

    /** the constraints that reference it, removed with it */
    collections::Vec<constraint_id::ConstraintId> constraint_ids;
};

/**
 * a new body in a slot of `bodies`, its mass, centre of mass and aabb from its shape, on the active list unless static.
 * not in the broadphase: broadphase::add_body puts it there. null (error set) when the pool cannot grow. `layers` holds
 * settings.object_layer
 */
RigidBody *create(bodies::Bodies &bodies, const layers::Layers &layers, const RigidBodySettings &settings);

/**
 * off the active list and its slot freed, so its id stops resolving; the record stays in the slot for the next create.
 * the caller removes it from the broadphase, its constraints and its pairs first. false if it was already removed
 */
bool remove(bodies::Bodies &bodies, RigidBody *body);

/** null for a removed body's id */
RigidBody *get(const bodies::Bodies &bodies, body_id::BodyId body_id);

/** world space; zero for a body that is not dynamic */
Float4x4 &get_inverse_inertia(Float4x4 &out, RigidBody *body);

/** from the transform and the shape; after any change to them */
void update_center_of_mass_position(RigidBody *body);

/** from the transform and the shape; after any change to them */
void update_aabb(RigidBody *body);

/** mass, centre of mass and aabb from the shape; after the shape changes. the caller refreshes the broadphase */
void update_shape(RigidBody *body);

/** the centre of mass and aabb follow. the caller refreshes the broadphase */
void set_position(bodies::Bodies &bodies, RigidBody *body, Vec3f position, bool wake);

/** the centre of mass and aabb follow. the caller refreshes the broadphase */
void set_quaternion(bodies::Bodies &bodies, RigidBody *body, Quatf quaternion, bool wake);

/** the centre of mass and aabb follow. the caller refreshes the broadphase */
void set_transform(bodies::Bodies &bodies, RigidBody *body, Vec3f position, Quatf quaternion, bool wake);

/** the caller reinserts the body in the broadphase and marks it moved */
void set_object_layer(const layers::Layers &layers, RigidBody *body, uint32_t object_layer);

/** the active list follows; `wake` wakes one not becoming static, and one becoming static stops. the caller refreshes the broadphase */
void set_motion_type(bodies::Bodies &bodies, RigidBody *body, motion_type::MotionType motion_type, bool wake);

/** at the centre of mass, integrated over the next step; dynamic bodies only */
void add_force(bodies::Bodies &bodies, RigidBody *body, Vec3f force, bool wake);

/** integrated over the next step; dynamic bodies only */
void add_torque(bodies::Bodies &bodies, RigidBody *body, Vec3f torque, bool wake);

/** the force, and the torque r x f about the centre of mass; dynamic bodies only */
void add_force_at_position(bodies::Bodies &bodies, RigidBody *body, Vec3f force, Vec3f world_position, bool wake);

/** at the centre of mass, dv = impulse / mass; dynamic bodies only, always wakes */
void add_impulse(bodies::Bodies &bodies, RigidBody *body, Vec3f impulse);

/** dynamic bodies only, always wakes */
void add_impulse_at_position(bodies::Bodies &bodies, RigidBody *body, Vec3f impulse, Vec3f world_position);

/** dw = I^-1 impulse; dynamic bodies only, always wakes */
void add_angular_impulse(bodies::Bodies &bodies, RigidBody *body, Vec3f impulse);

/** always wakes */
void set_linear_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity);

/** always wakes */
void add_linear_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity_delta);

/** always wakes */
void set_angular_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity);

/** always wakes */
void add_angular_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity_delta);

void clear_forces(RigidBody *body);

/** the velocities that bring a kinematic body to the target in `delta_time` */
void move_kinematic(RigidBody *body, Vec3f target_position, Quatf target_quaternion, float delta_time);

/** world space */
Vec3f get_world_space_surface_normal(const RigidBody *body, sub_shape::SubShapeId sub_shape_id, Vec3f world_position);

/** queried relative to the shape's origin, not the centre of mass */
Float3 &get_surface_normal(Float3 &out, const RigidBody *body, const Float3 &world_position, sub_shape::SubShapeId sub_shape_id);

shapes::GetLeafShapeResult create_get_leaf_shape_result();

/** the leaf the sub shape id's path leads to */
void get_leaf_shape(shapes::GetLeafShapeResult &out, const shapes::Shape *root_shape, sub_shape::SubShapeId sub_shape_id);

shapes::GetSubShapeTransformedShapeResult create_get_sub_shape_transformed_shape_result();

/** the shape the sub shape id's path leads to, transformed from the given pose */
void get_sub_shape_transformed_shape(shapes::GetSubShapeTransformedShapeResult &out, const shapes::Shape *root_shape,
                                     sub_shape::SubShapeId sub_shape_id, Vec3f position, Quatf rotation,
                                     Vec3f scale);

/** of a point relative to the centre of mass; zero for a static body */
Vec3f get_velocity_at_point_com(const RigidBody *body, Vec3f point_relative_to_com);

/** of a world space point; zero for a static body */
Vec3f get_velocity_at_point(const RigidBody *body, Vec3f world_point);

bool bodies_share_constraint(const RigidBody *body_a, const RigidBody *body_b);

inline void sleep(bodies::Bodies &bodies, RigidBody *body) { sleep::sleep(bodies, body); }

inline void wake(bodies::Bodies &bodies, RigidBody *body) { sleep::wake(bodies, body); }

}  // namespace rigid_body

namespace filter {

/** the layers that collide with the body's object layer, and its groups and mask */
inline void set_from_body(Filter &filter, const layers::Layers &layers, const rigid_body::RigidBody *body) {
    set_from_layer(filter, layers, body->object_layer, body->collision_groups, body->collision_mask);
}

}  // namespace filter

TUMBLE_NAMESPACE_END
