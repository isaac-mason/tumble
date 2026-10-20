#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/dof.h"
#include "tumble/physics/body/mass_properties.h"
#include "tumble/physics/constraints/contact_settings.h"
#include "tumble/physics/manifold/manifold.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/offset_center_of_mass.h"
#include "tumble/physics/shapes/plane.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"
#include "tumble/physics/world.h"
#include "tumble/physics/world_settings.h"

// what js calls where the C++ api does not cross as it is: bodies by id (an id naming no live body sets
// ERROR_BAD_HANDLE and the call does nothing), settings holding pointers, the world's settings built call by call
namespace adapters {

namespace wasm {

/** the heap from heap_begin, and every built-in shape type registered */
bool init(uint32_t heap_begin);
uint32_t abi_version();
uint32_t schema_hash();
float simd_check();
uint32_t error();
void error_clear();
void *alloc_bytes(uint32_t bytes);
void free_bytes(void *block, uint32_t bytes);
uint32_t allocated_bytes();

}  // namespace wasm

namespace world_settings {

/** the defaults, on the heap, for the layer calls to fill */
tumble::world_settings::WorldSettings *create();
void destroy(tumble::world_settings::WorldSettings *settings);
uint32_t add_broadphase_layer(tumble::world_settings::WorldSettings *settings);
uint32_t add_object_layer(tumble::world_settings::WorldSettings *settings, uint32_t broadphase_layer);
void enable_collision(tumble::world_settings::WorldSettings *settings, uint32_t object_layer_a, uint32_t object_layer_b);

}  // namespace world_settings

namespace world {

tumble::world::World *create(const tumble::world_settings::WorldSettings *settings);
void destroy(tumble::world::World *world);
void step(tumble::world::World *world, float delta_time);

// the step's phases, as update.h has them
void step_begin(tumble::world::World *world, float delta_time);
void find_pairs(tumble::world::World *world);
void narrowphase(tumble::world::World *world);
void solve(tumble::world::World *world);
void step_end(tumble::world::World *world);
/** the bodies the last narrowphase woke, whose pairs are not found yet */
uint32_t woken(const tumble::world::World *world);

/** the surfaces material ids name, as friction and restitution pairs; false (error set) when the allocator fails */
bool set_materials(tumble::world::World *world, const float *friction_restitution, uint32_t count);

}  // namespace world

/** events::PairEvent as js reads it: bodies by id, user data in halves */
struct PairEvent {
    tumble::body_id::BodyId body_a;
    tumble::body_id::BodyId body_b;
    uint32_t user_data_a_low;
    uint32_t user_data_a_high;
    uint32_t user_data_b_low;
    uint32_t user_data_b_high;
};

/** events::ContactEvent as js reads it */
struct ContactEvent {
    tumble::body_id::BodyId body_a;
    tumble::body_id::BodyId body_b;
    uint32_t user_data_a_low;
    uint32_t user_data_a_high;
    uint32_t user_data_b_low;
    uint32_t user_data_b_high;
    uint32_t sub_shape_a;
    uint32_t sub_shape_b;
    int32_t material_a;
    int32_t material_b;
    tumble::Float3 normal;
    float penetration;
    uint32_t num_points;
    tumble::Float3 points_a[tumble::manifold::MAX_CONTACT_POINTS];
    tumble::Float3 points_b[tumble::manifold::MAX_CONTACT_POINTS];
    tumble::contact_constraints::ContactSettings settings;
};

struct ContactRemovedEvent {
    tumble::body_id::BodyId body_a;
    tumble::body_id::BodyId body_b;
    uint32_t sub_shape_a;
    uint32_t sub_shape_b;
};

/**
 * a renderer's view of bodies: each body it draws in a slot, with the slot's pose in flat arrays here (positions xyz,
 * quaternions xyzw), refreshed by read
 */
struct Poses {
    tumble::world::World *world;
    /** by slot; INVALID_BODY_ID for a free slot */
    tumble::collections::Vec<tumble::body_id::BodyId> bodies;
    tumble::collections::Vec<uint32_t> free_slots;
    tumble::collections::Vec<float> positions;
    tumble::collections::Vec<float> quaternions;
    /** by slot: added since the last read, so it reads as changed */
    tumble::collections::Vec<uint8_t> fresh;
    /** the slots the last read changed */
    tumble::collections::Vec<uint32_t> changed;
};

namespace poses {

/** null when the allocator fails */
Poses *create(tumble::world::World *world);
void destroy(Poses *poses);
/** a slot for the body, its pose filled now; a free slot is reused. UINT32_MAX (error set) for a bad id or out of memory */
uint32_t add(Poses *poses, tumble::body_id::BodyId body);
/** the slot freed, its pose left as it was; false for a free or out of range slot */
bool remove(Poses *poses, uint32_t slot);
/**
 * every live slot's pose brought to its body's, comparing with what is there: exact whatever stepped or teleported
 * since. a slot whose body is gone keeps its pose. how many slots changed, listed in changed
 */
uint32_t read(Poses *poses);
/** slots in use or freed: the length of the arrays, over 3 and 4 */
uint32_t slots(const Poses *poses);
tumble::body_id::BodyId body(const Poses *poses, uint32_t slot);
const float *positions(const Poses *poses);
const float *quaternions(const Poses *poses);
const uint32_t *changed(const Poses *poses);

}  // namespace poses

// a step's events: a count, and a reader of one by index (false past the count)
namespace events {

uint32_t pairs(const tumble::world::World *world);
bool read_pair(PairEvent &out, const tumble::world::World *world, uint32_t index);
uint32_t contacts_added(const tumble::world::World *world);
bool read_contact_added(ContactEvent &out, const tumble::world::World *world, uint32_t index);
uint32_t contacts_persisted(const tumble::world::World *world);
bool read_contact_persisted(ContactEvent &out, const tumble::world::World *world, uint32_t index);
uint32_t contacts_removed(const tumble::world::World *world);
bool read_contact_removed(ContactRemovedEvent &out, const tumble::world::World *world, uint32_t index);
/** the bodies the step moved */
uint32_t moved(const tumble::world::World *world);
tumble::body_id::BodyId moved_body(const tumble::world::World *world, uint32_t index);

}  // namespace events

// answers to this step's events, by the index the reader took
namespace answers {

/** no narrowphase for pairs entry `index`, and its contacts go; false past the count */
bool reject_pair(tumble::world::World *world, uint32_t index);
/** contact event `index` (added, then persisted) solved with these settings; false past the count */
bool override_contact(tumble::world::World *world, uint32_t index, const tumble::contact_constraints::ContactSettings &settings);
/** contact event `index` dropped before the solve */
bool reject_contact(tumble::world::World *world, uint32_t index);

}  // namespace answers

/** RigidBodySettings as js gives it: the mass properties inline, used when has_mass_properties, for the pointer */
struct BodySettings {
    tumble::shapes::Shape *shape;
    uint32_t object_layer;
    uint32_t motion_type;
    uint32_t user_data_low;
    uint32_t user_data_high;
    tumble::Float3 position;
    tumble::Float4 quaternion;
    uint32_t motion_quality;
    uint32_t allowed_degrees_of_freedom;
    float gravity_factor;
    float linear_damping;
    float angular_damping;
    float max_linear_velocity;
    float max_angular_velocity;
    uint32_t num_velocity_steps_override;
    uint32_t num_position_steps_override;
    float friction;
    float restitution;
    uint32_t friction_combine_mode;
    uint32_t restitution_combine_mode;
    uint32_t collision_groups;
    uint32_t collision_mask;
    bool use_manifold_reduction;
    bool allow_sleeping;
    bool sensor;
    bool enhanced_internal_edge_removal;
    bool collide_kinematic_vs_non_dynamic;
    bool observes_contacts;
    float mass;
    /** mass_properties in place of the shape's, for shapes they cannot be derived from */
    bool has_mass_properties;
    tumble::mass_properties::MassProperties mass_properties;
};

namespace rigid_body {

/** the C++ defaults, for js to start a body's settings from */
void default_settings(BodySettings &out);
/** the new body's id, or INVALID_BODY_ID (error set) when the allocator fails */
tumble::body_id::BodyId create(tumble::world::World *world, const BodySettings &settings);
bool remove(tumble::world::World *world, tumble::body_id::BodyId id);
/** whether the id names a live body */
bool exists(tumble::world::World *world, tumble::body_id::BodyId id);

// readers: false (out unchanged) or 0 when the id names no body
bool get_position(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_quaternion(tumble::Float4 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_transform(tumble::Float3 &out_position, tumble::Float4 &out_quaternion, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_center_of_mass_position(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_linear_velocity(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_angular_velocity(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_velocity_at_point(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &point);
bool get_aabb(tumble::Box3f &out, tumble::world::World *world, tumble::body_id::BodyId id);
float get_mass(tumble::world::World *world, tumble::body_id::BodyId id);
uint32_t get_motion_type(tumble::world::World *world, tumble::body_id::BodyId id);
uint32_t get_object_layer(tumble::world::World *world, tumble::body_id::BodyId id);
uint32_t get_user_data_low(tumble::world::World *world, tumble::body_id::BodyId id);
uint32_t get_user_data_high(tumble::world::World *world, tumble::body_id::BodyId id);
tumble::shapes::Shape *get_shape(tumble::world::World *world, tumble::body_id::BodyId id);
bool is_sleeping(tumble::world::World *world, tumble::body_id::BodyId id);

// writers: false when the id names no body
bool set_position(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &position, bool wake);
bool set_quaternion(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float4 &quaternion, bool wake);
bool set_transform(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &position, const tumble::Float4 &quaternion,
                   bool wake);
bool set_linear_velocity(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &velocity);
bool set_angular_velocity(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &velocity);
bool add_linear_velocity(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &velocity);
bool add_angular_velocity(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &velocity);
bool add_force(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &force, bool wake);
bool add_torque(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &torque, bool wake);
bool add_force_at_position(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &force, const tumble::Float3 &position,
                           bool wake);
bool add_impulse(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &impulse);
bool add_impulse_at_position(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &impulse, const tumble::Float3 &position);
bool add_angular_impulse(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &impulse);
bool set_motion_type(tumble::world::World *world, tumble::body_id::BodyId id, uint32_t motion_type, bool wake);
bool set_object_layer(tumble::world::World *world, tumble::body_id::BodyId id, uint32_t object_layer);
bool set_shape(tumble::world::World *world, tumble::body_id::BodyId id, tumble::shapes::Shape *shape);
bool move_kinematic(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &position, const tumble::Float4 &quaternion,
                    float delta_time);
bool sleep(tumble::world::World *world, tumble::body_id::BodyId id);
bool wake(tumble::world::World *world, tumble::body_id::BodyId id);
bool set_user_data(tumble::world::World *world, tumble::body_id::BodyId id, uint32_t low, uint32_t high);
bool set_friction(tumble::world::World *world, tumble::body_id::BodyId id, float friction);
bool set_restitution(tumble::world::World *world, tumble::body_id::BodyId id, float restitution);

}  // namespace rigid_body

// what every shape has
namespace shape {

uint32_t get_type(const tumble::shapes::Shape *shape);
/** local space */
void get_local_bounds(tumble::Box3f &out, const tumble::shapes::Shape *shape);
void get_center_of_mass(tumble::Float3 &out, const tumble::shapes::Shape *shape);
float get_volume(const tumble::shapes::Shape *shape);
/** at the shape's density, about its centre of mass */
void get_mass_properties(tumble::mass_properties::MassProperties &out, const tumble::shapes::Shape *shape);

}  // namespace shape

// each shape's create, null when the allocator fails; the defaults are the C++'s

namespace box {

void default_settings(tumble::box::BoxShapeSettings &out);
tumble::shapes::Shape *create(const tumble::box::BoxShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace box

namespace sphere {

void default_settings(tumble::sphere::SphereShapeSettings &out);
tumble::shapes::Shape *create(const tumble::sphere::SphereShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace sphere

namespace capsule {

void default_settings(tumble::capsule::CapsuleShapeSettings &out);
tumble::shapes::Shape *create(const tumble::capsule::CapsuleShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace capsule

namespace cylinder {

void default_settings(tumble::cylinder::CylinderShapeSettings &out);
tumble::shapes::Shape *create(const tumble::cylinder::CylinderShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace cylinder

namespace plane {

void default_settings(tumble::plane::PlaneShapeSettings &out);
tumble::shapes::Shape *create(const tumble::plane::PlaneShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace plane

namespace empty {

tumble::shapes::Shape *create();
void destroy(tumble::shapes::Shape *shape);

}  // namespace empty

/** a hull build's outcome: the data, or null and why */
struct ConvexHullBuild {
    tumble::convex_hull::ConvexHullData *data;
    int32_t result;
};

namespace convex_hull_data {

void default_settings(tumble::convex_hull::ConvexHullBuildSettings &out);
/** false when the points make no hull or the allocator fails, out.result saying which */
bool build(ConvexHullBuild &out, const tumble::convex_hull::ConvexHullBuildSettings &settings);
void release(tumble::convex_hull::ConvexHullData *data);

}  // namespace convex_hull_data

namespace convex_hull {

void default_settings(tumble::convex_hull::ConvexHullShapeSettings &out);
tumble::shapes::Shape *create(const tumble::convex_hull::ConvexHullShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace convex_hull

namespace triangle_mesh_data {

void default_settings(tumble::triangle_mesh_builder::TriangleMeshBuilderSettings &out);
/** null when the allocator fails */
tumble::triangle_mesh_data::TriangleMeshData *build(const tumble::triangle_mesh_builder::TriangleMeshBuilderSettings &settings);
void release(tumble::triangle_mesh_data::TriangleMeshData *data);

}  // namespace triangle_mesh_data

namespace triangle_mesh {

void default_settings(tumble::triangle_mesh::TriangleMeshShapeSettings &out);
tumble::shapes::Shape *create(const tumble::triangle_mesh::TriangleMeshShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace triangle_mesh

namespace compound {

/** `children` is the address of `count` CompoundShapeChild values, read during the call */
tumble::shapes::Shape *create(const uint8_t *children, uint32_t count);
void destroy(tumble::shapes::Shape *shape);

}  // namespace compound

namespace static_compound {

uint32_t default_bvh_max_leaf_children();
tumble::shapes::Shape *create(const uint8_t *children, uint32_t count, uint32_t bvh_max_leaf_children);
void destroy(tumble::shapes::Shape *shape);

}  // namespace static_compound

namespace transformed {

void default_settings(tumble::transformed::TransformedShapeSettings &out);
tumble::shapes::Shape *create(const tumble::transformed::TransformedShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace transformed

namespace scaled {

void default_settings(tumble::scaled::ScaledShapeSettings &out);
tumble::shapes::Shape *create(const tumble::scaled::ScaledShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace scaled

namespace offset_center_of_mass {

void default_settings(tumble::offset_center_of_mass::OffsetCenterOfMassShapeSettings &out);
tumble::shapes::Shape *create(const tumble::offset_center_of_mass::OffsetCenterOfMassShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace offset_center_of_mass

}  // namespace adapters
