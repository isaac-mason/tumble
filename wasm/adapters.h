#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/sphere.h"
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

}  // namespace world

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
