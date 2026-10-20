#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/world.h"
#include "tumble/physics/constraints/contact_settings.h"
#include "tumble/physics/manifold/manifold.h"
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
};

namespace rigid_body {

/** the C++ defaults, for js to start a body's settings from */
void default_settings(BodySettings &out);
/** the new body's id, or INVALID_BODY_ID (error set) when the allocator fails */
tumble::body_id::BodyId create(tumble::world::World *world, const BodySettings &settings);
bool remove(tumble::world::World *world, tumble::body_id::BodyId id);
bool get_position(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_quaternion(tumble::Float4 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_linear_velocity(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool set_linear_velocity(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &velocity);

}  // namespace rigid_body

namespace box {

tumble::shapes::Shape *create(const tumble::box::BoxShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace box

namespace sphere {

tumble::shapes::Shape *create(const tumble::sphere::SphereShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace sphere

}  // namespace adapters
