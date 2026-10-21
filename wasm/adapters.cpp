// what js calls where the C++ api does not cross as it is: bodies by id, settings holding pointers, the world's
// settings built call by call. wasm/bindings.cpp binds them; the generated extern "C" layer calls them.

#include "wasm/adapters.h"

#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/events.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/utils/triangle_mesh_data.h"
#include "tumble/physics/update.h"
#include "tumble/util/alloc.h"
#include "tumble/util/error.h"
#include "tumble/util/object.h"
#include "wasm/runtime/instance.h"

using namespace tumble;

namespace adapters {

namespace wasm {

bool init(uint32_t heap_begin) {
    if (!runtime::init(heap_begin)) return false;
    dispatch::init();
    return true;
}

uint32_t abi_version() { return runtime::abi_version(); }
uint32_t schema_hash() { return runtime::schema_hash(); }
float simd_check() { return runtime::simd_check(); }
uint32_t error() { return util::error(); }
void error_clear() { util::error_clear(); }
void *alloc_bytes(uint32_t bytes) { return util::alloc_bytes(bytes); }
void free_bytes(void *block, uint32_t bytes) { util::free_bytes(block, bytes); }
uint32_t allocated_bytes() { return (uint32_t)util::allocated_bytes(); }

}  // namespace wasm

namespace world_settings {

tumble::world_settings::WorldSettings *create() {
    tumble::world_settings::WorldSettings *settings = util::create<tumble::world_settings::WorldSettings>();
    if (settings != nullptr) *settings = tumble::world_settings::create();
    return settings;
}

void destroy(tumble::world_settings::WorldSettings *settings) {
    tumble::world_settings::release(*settings);
    util::destroy(settings);
}

uint32_t add_broadphase_layer(tumble::world_settings::WorldSettings *settings) { return tumble::world_settings::add_broadphase_layer(*settings); }

uint32_t add_object_layer(tumble::world_settings::WorldSettings *settings, uint32_t broadphase_layer) {
    return tumble::world_settings::add_object_layer(*settings, broadphase_layer);
}

void enable_collision(tumble::world_settings::WorldSettings *settings, uint32_t object_layer_a, uint32_t object_layer_b) {
    tumble::world_settings::enable_collision(*settings, object_layer_a, object_layer_b);
}

}  // namespace world_settings

namespace world {

tumble::world::World *create(const tumble::world_settings::WorldSettings *settings) { return tumble::world::create(*settings); }
void destroy(tumble::world::World *world) { tumble::world::destroy(world); }
void step(tumble::world::World *world, float delta_time) { tumble::world::step(*world, delta_time); }

void step_begin(tumble::world::World *world, float delta_time) { update::step_begin(*world, delta_time); }
void find_pairs(tumble::world::World *world) { update::find_pairs(*world); }
void narrowphase(tumble::world::World *world) { update::narrowphase(*world); }
void solve(tumble::world::World *world) { update::solve(*world); }
void step_end(tumble::world::World *world) { update::step_end(*world); }
uint32_t woken(const tumble::world::World *world) { return world->events.woken; }

bool set_materials(tumble::world::World *world, const float *friction_restitution, uint32_t count) {
    collections::Owned<contact_constraints::Material> materials;
    if (!materials.v.reserve(count)) return false;
    for (uint32_t i = 0; i < count; i++) materials.v.push({friction_restitution[i * 2], friction_restitution[i * 2 + 1]});
    return tumble::world::set_materials(*world, materials.v.data, count);
}

}  // namespace world

namespace events {

static void split(uint32_t &low, uint32_t &high, uint64_t value) {
    low = (uint32_t)value;
    high = (uint32_t)(value >> 32);
}

uint32_t pairs(const tumble::world::World *world) { return world->events.pairs.length; }

bool read_pair(PairEvent &out, const tumble::world::World *world, uint32_t index) {
    if (index >= world->events.pairs.length) return false;
    const tumble::events::PairEvent &event = world->events.pairs[index];
    out.body_a = event.body_id_a;
    out.body_b = event.body_id_b;
    split(out.user_data_a_low, out.user_data_a_high, event.user_data_a);
    split(out.user_data_b_low, out.user_data_b_high, event.user_data_b);
    return true;
}

static void copy_contact(ContactEvent &out, const tumble::events::ContactEvent &event) {
    out.body_a = event.body_id_a;
    out.body_b = event.body_id_b;
    split(out.user_data_a_low, out.user_data_a_high, event.user_data_a);
    split(out.user_data_b_low, out.user_data_b_high, event.user_data_b);
    out.sub_shape_a = event.sub_shape_id_a;
    out.sub_shape_b = event.sub_shape_id_b;
    out.material_a = event.material_id_a;
    out.material_b = event.material_id_b;
    out.normal = event.normal;
    out.penetration = event.penetration;
    out.num_points = event.num_points;
    for (uint32_t i = 0; i < manifold::MAX_CONTACT_POINTS; i++) {
        out.points_a[i] = event.points_a[i];
        out.points_b[i] = event.points_b[i];
    }
    out.settings = event.settings;
}

uint32_t contacts_added(const tumble::world::World *world) { return world->events.contacts_added.length; }

bool read_contact_added(ContactEvent &out, const tumble::world::World *world, uint32_t index) {
    if (index >= world->events.contacts_added.length) return false;
    copy_contact(out, world->events.contacts_added[index]);
    return true;
}

uint32_t contacts_persisted(const tumble::world::World *world) { return world->events.contacts_persisted.length; }

bool read_contact_persisted(ContactEvent &out, const tumble::world::World *world, uint32_t index) {
    if (index >= world->events.contacts_persisted.length) return false;
    copy_contact(out, world->events.contacts_persisted[index]);
    return true;
}

uint32_t contacts_removed(const tumble::world::World *world) { return world->events.contacts_removed.length; }

bool read_contact_removed(ContactRemovedEvent &out, const tumble::world::World *world, uint32_t index) {
    if (index >= world->events.contacts_removed.length) return false;
    const tumble::events::ContactRemovedEvent &event = world->events.contacts_removed[index];
    out = {event.body_id_a, event.body_id_b, event.sub_shape_id_a, event.sub_shape_id_b};
    return true;
}

uint32_t moved(const tumble::world::World *world) { return world->events.moved.length; }

body_id::BodyId moved_body(const tumble::world::World *world, uint32_t index) {
    if (index >= world->events.moved.length) return body_id::INVALID_BODY_ID;
    return world->bodies.pool[world->events.moved[index]]->id;
}

}  // namespace events

namespace poses {

Poses *create(tumble::world::World *world) {
    Poses *poses = util::alloc<Poses>(1);
    if (poses == nullptr) return nullptr;
    *poses = {};
    poses->world = world;
    return poses;
}

void destroy(Poses *poses) {
    poses->bodies.release();
    poses->free_slots.release();
    poses->positions.release();
    poses->quaternions.release();
    poses->fresh.release();
    poses->changed.release();
    util::free(poses, 1);
}

static void store_pose(Poses *poses, uint32_t slot, const tumble::rigid_body::RigidBody *body) {
    float *position = poses->positions.data + slot * 3;
    float *quaternion = poses->quaternions.data + slot * 4;
    position[0] = body->position.x;
    position[1] = body->position.y;
    position[2] = body->position.z;
    quaternion[0] = body->quaternion.x;
    quaternion[1] = body->quaternion.y;
    quaternion[2] = body->quaternion.z;
    quaternion[3] = body->quaternion.w;
}

uint32_t add(Poses *poses, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = tumble::rigid_body::get(*poses->world, id);
    if (body == nullptr) {
        util::set_error(util::ERROR_BAD_HANDLE);
        return UINT32_MAX;
    }
    uint32_t slot;
    if (poses->free_slots.length > 0) {
        slot = poses->free_slots[--poses->free_slots.length];
    } else {
        slot = poses->bodies.length;
        const uint32_t wanted = slot == poses->bodies.capacity ? (slot == 0 ? 16 : slot * 2) : poses->bodies.capacity;
        if (!poses->bodies.reserve(wanted) || !poses->fresh.reserve(wanted) || !poses->positions.reserve(wanted * 3) ||
            !poses->quaternions.reserve(wanted * 4) || !poses->changed.reserve(wanted)) {
            return UINT32_MAX;
        }
        poses->bodies.push(body_id::INVALID_BODY_ID);
        poses->fresh.push(0);
        poses->positions.length += 3;
        poses->quaternions.length += 4;
    }
    poses->bodies[slot] = id;
    poses->fresh[slot] = 1;
    store_pose(poses, slot, body);
    return slot;
}

bool remove(Poses *poses, uint32_t slot) {
    if (slot >= poses->bodies.length || poses->bodies[slot] == body_id::INVALID_BODY_ID) return false;
    poses->bodies[slot] = body_id::INVALID_BODY_ID;
    poses->fresh[slot] = 0;
    // when this cannot grow, the slot is never reused, which is all it costs
    poses->free_slots.push(slot);
    return true;
}

uint32_t read(Poses *poses) {
    poses->changed.length = 0;
    for (uint32_t slot = 0; slot < poses->bodies.length; slot++) {
        const body_id::BodyId id = poses->bodies[slot];
        if (id == body_id::INVALID_BODY_ID) continue;
        const tumble::rigid_body::RigidBody *body = tumble::rigid_body::get(*poses->world, id);
        if (body == nullptr) continue;
        const float *position = poses->positions.data + slot * 3;
        const float *quaternion = poses->quaternions.data + slot * 4;
        const bool same = position[0] == body->position.x && position[1] == body->position.y && position[2] == body->position.z &&
                          quaternion[0] == body->quaternion.x && quaternion[1] == body->quaternion.y &&
                          quaternion[2] == body->quaternion.z && quaternion[3] == body->quaternion.w;
        if (same && poses->fresh[slot] == 0) continue;
        store_pose(poses, slot, body);
        poses->fresh[slot] = 0;
        // reserved to the slots' capacity in add, so this never grows
        poses->changed.push(slot);
    }
    return poses->changed.length;
}

uint32_t slots(const Poses *poses) { return poses->bodies.length; }

body_id::BodyId body(const Poses *poses, uint32_t slot) {
    return slot < poses->bodies.length ? poses->bodies[slot] : body_id::INVALID_BODY_ID;
}

const float *positions(const Poses *poses) { return poses->positions.data; }
const float *quaternions(const Poses *poses) { return poses->quaternions.data; }
const uint32_t *changed(const Poses *poses) { return poses->changed.data; }

}  // namespace poses

namespace answers {

bool reject_pair(tumble::world::World *world, uint32_t index) {
    if (index >= world->events.pairs.length) return false;
    tumble::events::reject_pair(world->answers, index);
    return true;
}

bool override_contact(tumble::world::World *world, uint32_t index, const contact_constraints::ContactSettings &settings) {
    if (index >= world->answers.contact_override.length) return false;
    tumble::events::override_contact(world->answers, world->events, index).settings = settings;
    return true;
}

bool reject_contact(tumble::world::World *world, uint32_t index) {
    if (index >= world->answers.contact_override.length) return false;
    tumble::events::override_contact(world->answers, world->events, index).reject = true;
    return true;
}

}  // namespace answers

namespace rigid_body {

void default_settings(BodySettings &out) {
    const tumble::rigid_body::RigidBodySettings d = {};
    out = {d.shape, d.object_layer, d.motion_type, (uint32_t)d.user_data, (uint32_t)(d.user_data >> 32), d.position, d.quaternion,
           d.motion_quality, d.allowed_degrees_of_freedom, d.gravity_factor, d.linear_damping, d.angular_damping, d.max_linear_velocity,
           d.max_angular_velocity, d.num_velocity_steps_override, d.num_position_steps_override, d.friction, d.restitution,
           d.friction_combine_mode, d.restitution_combine_mode, d.collision_groups, d.collision_mask, d.use_manifold_reduction,
           d.allow_sleeping, d.sensor, d.enhanced_internal_edge_removal, d.collide_kinematic_vs_non_dynamic, d.observes_contacts,
           d.mass, false, mass_properties::create()};
}

body_id::BodyId create(tumble::world::World *world, const BodySettings &s) {
    tumble::rigid_body::RigidBodySettings settings;
    settings.shape = s.shape;
    settings.object_layer = s.object_layer;
    settings.motion_type = (motion_type::MotionType)s.motion_type;
    settings.user_data = (uint64_t)s.user_data_high << 32 | s.user_data_low;
    settings.position = s.position;
    settings.quaternion = s.quaternion;
    settings.motion_quality = (motion_properties::MotionQuality)s.motion_quality;
    settings.allowed_degrees_of_freedom = s.allowed_degrees_of_freedom;
    settings.gravity_factor = s.gravity_factor;
    settings.linear_damping = s.linear_damping;
    settings.angular_damping = s.angular_damping;
    settings.max_linear_velocity = s.max_linear_velocity;
    settings.max_angular_velocity = s.max_angular_velocity;
    settings.num_velocity_steps_override = s.num_velocity_steps_override;
    settings.num_position_steps_override = s.num_position_steps_override;
    settings.friction = s.friction;
    settings.restitution = s.restitution;
    settings.friction_combine_mode = (combine_material::MaterialCombineMode)s.friction_combine_mode;
    settings.restitution_combine_mode = (combine_material::MaterialCombineMode)s.restitution_combine_mode;
    settings.collision_groups = s.collision_groups;
    settings.collision_mask = s.collision_mask;
    settings.use_manifold_reduction = s.use_manifold_reduction;
    settings.allow_sleeping = s.allow_sleeping;
    settings.sensor = s.sensor;
    settings.enhanced_internal_edge_removal = s.enhanced_internal_edge_removal;
    settings.collide_kinematic_vs_non_dynamic = s.collide_kinematic_vs_non_dynamic;
    settings.observes_contacts = s.observes_contacts;
    settings.mass = s.mass;
    settings.mass_properties_override = s.has_mass_properties ? &s.mass_properties : nullptr;
    tumble::rigid_body::RigidBody *body = tumble::rigid_body::create(*world, settings);
    return body == nullptr ? body_id::INVALID_BODY_ID : body->id;
}

/** the live body an id names, or null with the error set */
static tumble::rigid_body::RigidBody *resolve(tumble::world::World *world, body_id::BodyId id) {
    tumble::rigid_body::RigidBody *body = tumble::rigid_body::get(*world, id);
    if (body == nullptr) util::set_error(util::ERROR_BAD_HANDLE);
    return body;
}

bool remove(tumble::world::World *world, body_id::BodyId id) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body != nullptr && tumble::rigid_body::remove(*world, body);
}

bool exists(tumble::world::World *world, body_id::BodyId id) { return tumble::rigid_body::get(*world, id) != nullptr; }

bool get_position(Float3 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->position;
    return true;
}

bool get_quaternion(Float4 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->quaternion;
    return true;
}

bool get_transform(Float3 &out_position, Float4 &out_quaternion, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out_position = body->position;
    out_quaternion = body->quaternion;
    return true;
}

bool get_center_of_mass_position(Float3 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->center_of_mass_position;
    return true;
}

bool get_linear_velocity(Float3 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->motion_properties.linear_velocity;
    return true;
}

bool get_angular_velocity(Float3 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->motion_properties.angular_velocity;
    return true;
}

bool get_velocity_at_point(Float3 &out, tumble::world::World *world, body_id::BodyId id, const Float3 &point) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    vec3f::store(out, tumble::rigid_body::get_velocity_at_point(body, vec3f::load(point)));
    return true;
}

bool get_aabb(Box3f &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->aabb;
    return true;
}

float get_mass(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr || body->motion_properties.inv_mass <= 0.0f) return 0.0f;
    return 1.0f / body->motion_properties.inv_mass;
}

uint32_t get_motion_type(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : (uint32_t)body->motion_type;
}

uint32_t get_object_layer(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : body->object_layer;
}

uint32_t get_user_data_low(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : (uint32_t)body->user_data;
}

uint32_t get_user_data_high(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : (uint32_t)(body->user_data >> 32);
}

shapes::Shape *get_shape(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body == nullptr ? nullptr : body->shape;
}

bool is_sleeping(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body != nullptr && body->sleeping;
}

bool set_position(tumble::world::World *world, body_id::BodyId id, const Float3 &position, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_position(*world, body, vec3f::load(position), wake);
    return true;
}

bool set_quaternion(tumble::world::World *world, body_id::BodyId id, const Float4 &quaternion, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_quaternion(*world, body, quatf::load(quaternion), wake);
    return true;
}

bool set_transform(tumble::world::World *world, body_id::BodyId id, const Float3 &position, const Float4 &quaternion, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_transform(*world, body, vec3f::load(position), quatf::load(quaternion), wake);
    return true;
}

bool set_linear_velocity(tumble::world::World *world, body_id::BodyId id, const Float3 &velocity) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_linear_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool set_angular_velocity(tumble::world::World *world, body_id::BodyId id, const Float3 &velocity) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_angular_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool add_linear_velocity(tumble::world::World *world, body_id::BodyId id, const Float3 &velocity) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_linear_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool add_angular_velocity(tumble::world::World *world, body_id::BodyId id, const Float3 &velocity) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_angular_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool add_force(tumble::world::World *world, body_id::BodyId id, const Float3 &force, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_force(*world, body, vec3f::load(force), wake);
    return true;
}

bool add_torque(tumble::world::World *world, body_id::BodyId id, const Float3 &torque, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_torque(*world, body, vec3f::load(torque), wake);
    return true;
}

bool add_force_at_position(tumble::world::World *world, body_id::BodyId id, const Float3 &force, const Float3 &position, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_force_at_position(*world, body, vec3f::load(force), vec3f::load(position), wake);
    return true;
}

bool add_impulse(tumble::world::World *world, body_id::BodyId id, const Float3 &impulse) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_impulse(*world, body, vec3f::load(impulse));
    return true;
}

bool add_impulse_at_position(tumble::world::World *world, body_id::BodyId id, const Float3 &impulse, const Float3 &position) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_impulse_at_position(*world, body, vec3f::load(impulse), vec3f::load(position));
    return true;
}

bool add_angular_impulse(tumble::world::World *world, body_id::BodyId id, const Float3 &impulse) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_angular_impulse(*world, body, vec3f::load(impulse));
    return true;
}

bool set_motion_type(tumble::world::World *world, body_id::BodyId id, uint32_t motion_type, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_motion_type(*world, body, (motion_type::MotionType)motion_type, wake);
    return true;
}

bool set_object_layer(tumble::world::World *world, body_id::BodyId id, uint32_t object_layer) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_object_layer(*world, body, object_layer);
    return true;
}

bool set_shape(tumble::world::World *world, body_id::BodyId id, shapes::Shape *shape) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_shape(*world, body, shape);
    return true;
}

bool move_kinematic(tumble::world::World *world, body_id::BodyId id, const Float3 &position, const Float4 &quaternion, float delta_time) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::move_kinematic(*world, body, vec3f::load(position), quatf::load(quaternion), delta_time);
    return true;
}

bool sleep(tumble::world::World *world, body_id::BodyId id) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::sleep(*world, body);
    return true;
}

bool wake(tumble::world::World *world, body_id::BodyId id) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::wake(*world, body);
    return true;
}

bool set_user_data(tumble::world::World *world, body_id::BodyId id, uint32_t low, uint32_t high) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    body->user_data = (uint64_t)high << 32 | low;
    return true;
}

bool set_friction(tumble::world::World *world, body_id::BodyId id, float friction) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    body->friction = friction;
    return true;
}

bool set_restitution(tumble::world::World *world, body_id::BodyId id, float restitution) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    body->restitution = restitution;
    return true;
}

}  // namespace rigid_body

namespace shape {

uint32_t get_type(const shapes::Shape *shape) { return (uint32_t)shape->type; }
void get_local_bounds(Box3f &out, const shapes::Shape *shape) { out = shape->aabb; }
void get_center_of_mass(Float3 &out, const shapes::Shape *shape) { out = shape->center_of_mass; }
float get_volume(const shapes::Shape *shape) { return shape->volume; }
void get_mass_properties(mass_properties::MassProperties &out, const shapes::Shape *shape) { shapes::compute_mass_properties(out, shape); }

}  // namespace shape

namespace box {

void default_settings(tumble::box::BoxShapeSettings &out) { out = {}; }
shapes::Shape *create(const tumble::box::BoxShapeSettings &settings) { return tumble::box::create(settings); }
void destroy(shapes::Shape *shape) { tumble::box::destroy(shape); }

}  // namespace box

namespace sphere {

void default_settings(tumble::sphere::SphereShapeSettings &out) { out = {}; }
shapes::Shape *create(const tumble::sphere::SphereShapeSettings &settings) { return tumble::sphere::create(settings); }
void destroy(shapes::Shape *shape) { tumble::sphere::destroy(shape); }

}  // namespace sphere

namespace capsule {

void default_settings(tumble::capsule::CapsuleShapeSettings &out) { out = {}; }
shapes::Shape *create(const tumble::capsule::CapsuleShapeSettings &settings) { return tumble::capsule::create(settings); }
void destroy(shapes::Shape *shape) { tumble::capsule::destroy(shape); }

}  // namespace capsule

namespace cylinder {

void default_settings(tumble::cylinder::CylinderShapeSettings &out) { out = {}; }
shapes::Shape *create(const tumble::cylinder::CylinderShapeSettings &settings) { return tumble::cylinder::create(settings); }
void destroy(shapes::Shape *shape) { tumble::cylinder::destroy(shape); }

}  // namespace cylinder

namespace plane {

void default_settings(tumble::plane::PlaneShapeSettings &out) { out = {}; }
shapes::Shape *create(const tumble::plane::PlaneShapeSettings &settings) { return tumble::plane::create(settings); }
void destroy(shapes::Shape *shape) { tumble::plane::destroy(shape); }

}  // namespace plane

namespace empty {

shapes::Shape *create() { return tumble::empty::create(); }
void destroy(shapes::Shape *shape) { tumble::empty::destroy(shape); }

}  // namespace empty

namespace convex_hull_data {

void default_settings(tumble::convex_hull::ConvexHullBuildSettings &out) { out = {}; }

bool build(ConvexHullBuild &out, const tumble::convex_hull::ConvexHullBuildSettings &settings) {
    tumble::convex_hull::BuildResult result = tumble::convex_hull::BUILD_SUCCESS;
    out.data = tumble::convex_hull::build(settings, &result);
    out.result = result;
    return out.data != nullptr;
}

void release(tumble::convex_hull::ConvexHullData *data) { tumble::convex_hull::release(data); }

}  // namespace convex_hull_data

namespace convex_hull {

void default_settings(tumble::convex_hull::ConvexHullShapeSettings &out) { out = {}; }
shapes::Shape *create(const tumble::convex_hull::ConvexHullShapeSettings &settings) { return tumble::convex_hull::create(settings); }
void destroy(shapes::Shape *shape) { tumble::convex_hull::destroy(shape); }

}  // namespace convex_hull

namespace triangle_mesh_data {

void default_settings(tumble::triangle_mesh_builder::TriangleMeshBuilderSettings &out) { out = {}; }

tumble::triangle_mesh_data::TriangleMeshData *build(const tumble::triangle_mesh_builder::TriangleMeshBuilderSettings &settings) {
    return tumble::triangle_mesh_builder::build(settings);
}

void release(tumble::triangle_mesh_data::TriangleMeshData *data) { tumble::triangle_mesh_data::destroy(data); }

}  // namespace triangle_mesh_data

namespace triangle_mesh {

void default_settings(tumble::triangle_mesh::TriangleMeshShapeSettings &out) { out = {}; }
shapes::Shape *create(const tumble::triangle_mesh::TriangleMeshShapeSettings &settings) { return tumble::triangle_mesh::create(settings); }
void destroy(shapes::Shape *shape) { tumble::triangle_mesh::destroy(shape); }

}  // namespace triangle_mesh

namespace compound {

shapes::Shape *create(const uint8_t *children, uint32_t count) {
    return tumble::compound::create({.children = (const tumble::compound::CompoundShapeChild *)children, .num_children = count});
}

void destroy(shapes::Shape *shape) { tumble::compound::destroy(shape); }

}  // namespace compound

namespace static_compound {

uint32_t default_bvh_max_leaf_children() { return tumble::static_compound::StaticCompoundShapeSettings{}.bvh_max_leaf_children; }

shapes::Shape *create(const uint8_t *children, uint32_t count, uint32_t bvh_max_leaf_children) {
    return tumble::static_compound::create({.children = (const tumble::compound::CompoundShapeChild *)children,
                                            .num_children = count,
                                            .bvh_max_leaf_children = bvh_max_leaf_children});
}

void destroy(shapes::Shape *shape) { tumble::static_compound::destroy(shape); }

}  // namespace static_compound

namespace transformed {

void default_settings(tumble::transformed::TransformedShapeSettings &out) { out = {}; }
shapes::Shape *create(const tumble::transformed::TransformedShapeSettings &settings) { return tumble::transformed::create(settings); }
void destroy(shapes::Shape *shape) { tumble::transformed::destroy(shape); }

}  // namespace transformed

namespace scaled {

void default_settings(tumble::scaled::ScaledShapeSettings &out) { out = {}; }
shapes::Shape *create(const tumble::scaled::ScaledShapeSettings &settings) { return tumble::scaled::create(settings); }
void destroy(shapes::Shape *shape) { tumble::scaled::destroy(shape); }

}  // namespace scaled

namespace offset_center_of_mass {

void default_settings(tumble::offset_center_of_mass::OffsetCenterOfMassShapeSettings &out) { out = {}; }
shapes::Shape *create(const tumble::offset_center_of_mass::OffsetCenterOfMassShapeSettings &settings) { return tumble::offset_center_of_mass::create(settings); }
void destroy(shapes::Shape *shape) { tumble::offset_center_of_mass::destroy(shape); }

}  // namespace offset_center_of_mass

using tumble::constraint_id::ConstraintId;

/** the constraint record a get found, or null with the error set */
template <typename Record>
static Record *found(Record *record) {
    if (record == nullptr) util::set_error(util::ERROR_BAD_HANDLE);
    return record;
}

template <typename Settings>
static ConstraintId create_between(tumble::world::World *world, body_id::BodyId body_a, body_id::BodyId body_b, const Settings &settings,
                                   ConstraintId (*create)(tumble::world::World &, tumble::rigid_body::RigidBody *,
                                                          tumble::rigid_body::RigidBody *, const Settings &)) {
    tumble::rigid_body::RigidBody *a = rigid_body::resolve(world, body_a);
    tumble::rigid_body::RigidBody *b = rigid_body::resolve(world, body_b);
    if (a == nullptr || b == nullptr) return tumble::constraint_id::INVALID_CONSTRAINT_ID;
    return create(*world, a, b, settings);
}

namespace constraint {

uint32_t get_type(ConstraintId id) { return tumble::constraint_id::type(id); }

bool exists(tumble::world::World *world, ConstraintId id) { return constraints::get(world->constraints, id) != nullptr; }

bool set_enabled(tumble::world::World *world, ConstraintId id, bool enabled) {
    constraints::ConstraintBase *base = found(constraints::get(world->constraints, id));
    if (base == nullptr) return false;
    base->enabled = enabled;
    return true;
}

bool is_enabled(tumble::world::World *world, ConstraintId id) {
    const constraints::ConstraintBase *base = found(constraints::get(world->constraints, id));
    return base != nullptr && base->enabled;
}

}  // namespace constraint

namespace point_constraint {

void default_settings(tumble::point_constraint::PointConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::world::World *world, body_id::BodyId body_a, body_id::BodyId body_b,
                    const tumble::point_constraint::PointConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::point_constraint::create);
}

bool remove(tumble::world::World *world, ConstraintId id) { return found(tumble::point_constraint::get(*world, id)) && tumble::point_constraint::remove(*world, id); }

bool get_total_lambda(Float3 &out, tumble::world::World *world, ConstraintId id) {
    const tumble::point_constraint::PointConstraint *constraint = found(tumble::point_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::point_constraint::get_total_lambda(*constraint));
    return true;
}

}  // namespace point_constraint

namespace distance_constraint {

void default_settings(tumble::distance_constraint::DistanceConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::world::World *world, body_id::BodyId body_a, body_id::BodyId body_b,
                    const tumble::distance_constraint::DistanceConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::distance_constraint::create);
}

bool remove(tumble::world::World *world, ConstraintId id) {
    return found(tumble::distance_constraint::get(*world, id)) && tumble::distance_constraint::remove(*world, id);
}

float get_total_lambda(tumble::world::World *world, ConstraintId id) {
    const tumble::distance_constraint::DistanceConstraint *constraint = found(tumble::distance_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::distance_constraint::get_total_lambda(*constraint);
}

bool set_distance(tumble::world::World *world, ConstraintId id, float min_distance, float max_distance) {
    tumble::distance_constraint::DistanceConstraint *constraint = found(tumble::distance_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::distance_constraint::set_distance(*constraint, min_distance, max_distance);
    return true;
}

bool set_limits_spring_settings(tumble::world::World *world, ConstraintId id, const spring_settings::SpringSettings &settings) {
    tumble::distance_constraint::DistanceConstraint *constraint = found(tumble::distance_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->limits_spring_settings = settings;
    return true;
}

}  // namespace distance_constraint

namespace hinge_constraint {

void default_settings(tumble::hinge_constraint::HingeConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::world::World *world, body_id::BodyId body_a, body_id::BodyId body_b,
                    const tumble::hinge_constraint::HingeConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::hinge_constraint::create);
}

bool remove(tumble::world::World *world, ConstraintId id) { return found(tumble::hinge_constraint::get(*world, id)) && tumble::hinge_constraint::remove(*world, id); }

bool set_limits(tumble::world::World *world, ConstraintId id, float min, float max) {
    tumble::hinge_constraint::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::hinge_constraint::set_limits(*constraint, min, max);
    return true;
}

bool set_limits_spring_settings(tumble::world::World *world, ConstraintId id, const spring_settings::SpringSettings &settings) {
    tumble::hinge_constraint::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->limits_spring_settings = settings;
    return true;
}

bool set_max_friction_torque(tumble::world::World *world, ConstraintId id, float torque) {
    tumble::hinge_constraint::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->max_friction_torque = torque;
    return true;
}

bool set_motor_settings(tumble::world::World *world, ConstraintId id, const motor_settings::MotorSettings &settings) {
    tumble::hinge_constraint::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->motor_settings = settings;
    return true;
}

bool set_motor_state(tumble::world::World *world, ConstraintId id, uint32_t state) {
    tumble::hinge_constraint::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::hinge_constraint::set_motor_state(*constraint, (motor_settings::MotorState)state);
    return true;
}

bool set_target_angular_velocity(tumble::world::World *world, ConstraintId id, float velocity) {
    tumble::hinge_constraint::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::hinge_constraint::set_target_angular_velocity(*constraint, velocity);
    return true;
}

bool set_target_angle(tumble::world::World *world, ConstraintId id, float angle) {
    tumble::hinge_constraint::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::hinge_constraint::set_target_angle(*constraint, angle);
    return true;
}

float get_current_angle(tumble::world::World *world, ConstraintId id) {
    const tumble::hinge_constraint::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::hinge_constraint::get_current_angle(*constraint);
}

}  // namespace hinge_constraint

namespace slider_constraint {

void default_settings(tumble::slider_constraint::SliderConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::world::World *world, body_id::BodyId body_a, body_id::BodyId body_b,
                    const tumble::slider_constraint::SliderConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::slider_constraint::create);
}

bool remove(tumble::world::World *world, ConstraintId id) {
    return found(tumble::slider_constraint::get(*world, id)) && tumble::slider_constraint::remove(*world, id);
}

float get_current_position(tumble::world::World *world, ConstraintId id) {
    const tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::slider_constraint::get_current_position(*world, *constraint);
}

bool set_limits(tumble::world::World *world, ConstraintId id, float min, float max) {
    tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::set_limits(*constraint, min, max);
    return true;
}

bool set_limits_spring_settings(tumble::world::World *world, ConstraintId id, const spring_settings::SpringSettings &settings) {
    tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->limits_spring_settings = settings;
    return true;
}

bool set_motor_settings(tumble::world::World *world, ConstraintId id, const motor_settings::MotorSettings &settings) {
    tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->motor_settings = settings;
    return true;
}

bool set_motor_state(tumble::world::World *world, ConstraintId id, uint32_t state) {
    tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::set_motor_state(*constraint, (motor_settings::MotorState)state);
    return true;
}

bool set_target_velocity(tumble::world::World *world, ConstraintId id, float velocity) {
    tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::set_target_velocity(*constraint, velocity);
    return true;
}

bool set_target_position(tumble::world::World *world, ConstraintId id, float position) {
    tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::set_target_position(*constraint, position);
    return true;
}

bool set_max_friction_force(tumble::world::World *world, ConstraintId id, float force) {
    tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::set_max_friction_force(*constraint, force);
    return true;
}

float get_max_friction_force(tumble::world::World *world, ConstraintId id) {
    const tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::slider_constraint::get_max_friction_force(*constraint);
}

bool get_total_lambda_position(Vec2f &out, tumble::world::World *world, ConstraintId id) {
    const tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::get_total_lambda_position(out, *constraint);
    return true;
}

float get_total_lambda_position_limits(tumble::world::World *world, ConstraintId id) {
    const tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::slider_constraint::get_total_lambda_position_limits(*constraint);
}

bool get_total_lambda_rotation(Float3 &out, tumble::world::World *world, ConstraintId id) {
    const tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::slider_constraint::get_total_lambda_rotation(*constraint));
    return true;
}

float get_total_lambda_motor(tumble::world::World *world, ConstraintId id) {
    const tumble::slider_constraint::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::slider_constraint::get_total_lambda_motor(*constraint);
}

}  // namespace slider_constraint

namespace fixed_constraint {

void default_settings(tumble::fixed_constraint::FixedConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::world::World *world, body_id::BodyId body_a, body_id::BodyId body_b,
                    const tumble::fixed_constraint::FixedConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::fixed_constraint::create);
}

bool remove(tumble::world::World *world, ConstraintId id) { return found(tumble::fixed_constraint::get(*world, id)) && tumble::fixed_constraint::remove(*world, id); }

bool get_total_lambda_position(Float3 &out, tumble::world::World *world, ConstraintId id) {
    const tumble::fixed_constraint::FixedConstraint *constraint = found(tumble::fixed_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::fixed_constraint::get_total_lambda_position(*constraint));
    return true;
}

bool get_total_lambda_rotation(Float3 &out, tumble::world::World *world, ConstraintId id) {
    const tumble::fixed_constraint::FixedConstraint *constraint = found(tumble::fixed_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::fixed_constraint::get_total_lambda_rotation(*constraint));
    return true;
}

}  // namespace fixed_constraint

namespace cone_constraint {

void default_settings(tumble::cone_constraint::ConeConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::world::World *world, body_id::BodyId body_a, body_id::BodyId body_b,
                    const tumble::cone_constraint::ConeConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::cone_constraint::create);
}

bool remove(tumble::world::World *world, ConstraintId id) { return found(tumble::cone_constraint::get(*world, id)) && tumble::cone_constraint::remove(*world, id); }

bool set_half_cone_angle(tumble::world::World *world, ConstraintId id, float half_cone_angle) {
    tumble::cone_constraint::ConeConstraint *constraint = found(tumble::cone_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::cone_constraint::set_half_cone_angle(*constraint, half_cone_angle);
    return true;
}

float get_half_cone_angle(tumble::world::World *world, ConstraintId id) {
    const tumble::cone_constraint::ConeConstraint *constraint = found(tumble::cone_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::cone_constraint::get_half_cone_angle(*constraint);
}

float get_cos_half_cone_angle(tumble::world::World *world, ConstraintId id) {
    const tumble::cone_constraint::ConeConstraint *constraint = found(tumble::cone_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::cone_constraint::get_cos_half_cone_angle(*constraint);
}

bool get_total_lambda_position(Float3 &out, tumble::world::World *world, ConstraintId id) {
    const tumble::cone_constraint::ConeConstraint *constraint = found(tumble::cone_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::cone_constraint::get_total_lambda_position(*constraint));
    return true;
}

float get_total_lambda_rotation(tumble::world::World *world, ConstraintId id) {
    const tumble::cone_constraint::ConeConstraint *constraint = found(tumble::cone_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::cone_constraint::get_total_lambda_rotation(*constraint);
}

}  // namespace cone_constraint

namespace swing_twist_constraint {

void default_settings(tumble::swing_twist_constraint::SwingTwistConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::world::World *world, body_id::BodyId body_a, body_id::BodyId body_b,
                    const tumble::swing_twist_constraint::SwingTwistConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::swing_twist_constraint::create);
}

bool remove(tumble::world::World *world, ConstraintId id) {
    return found(tumble::swing_twist_constraint::get(*world, id)) && tumble::swing_twist_constraint::remove(*world, id);
}

bool set_normal_half_cone_angle(tumble::world::World *world, ConstraintId id, float angle) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_normal_half_cone_angle(*constraint, angle);
    return true;
}

bool set_plane_half_cone_angle(tumble::world::World *world, ConstraintId id, float angle) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_plane_half_cone_angle(*constraint, angle);
    return true;
}

bool set_twist_min_angle(tumble::world::World *world, ConstraintId id, float angle) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_twist_min_angle(*constraint, angle);
    return true;
}

bool set_twist_max_angle(tumble::world::World *world, ConstraintId id, float angle) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_twist_max_angle(*constraint, angle);
    return true;
}

bool set_max_friction_torque(tumble::world::World *world, ConstraintId id, float torque) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_max_friction_torque(*constraint, torque);
    return true;
}

bool set_swing_motor_settings(tumble::world::World *world, ConstraintId id, const motor_settings::MotorSettings &settings) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->swing_motor_settings = settings;
    return true;
}

bool set_twist_motor_settings(tumble::world::World *world, ConstraintId id, const motor_settings::MotorSettings &settings) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->twist_motor_settings = settings;
    return true;
}

bool set_swing_motor_state(tumble::world::World *world, ConstraintId id, uint32_t state) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_swing_motor_state(*constraint, (motor_settings::MotorState)state);
    return true;
}

bool set_twist_motor_state(tumble::world::World *world, ConstraintId id, uint32_t state) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_twist_motor_state(*constraint, (motor_settings::MotorState)state);
    return true;
}

bool set_target_angular_velocity_cs(tumble::world::World *world, ConstraintId id, const Float3 &velocity) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_target_angular_velocity_cs(*constraint, vec3f::load(velocity));
    return true;
}

bool set_target_orientation_cs(tumble::world::World *world, ConstraintId id, const Float4 &orientation) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_target_orientation_cs(*constraint, quatf::load(orientation));
    return true;
}

bool set_target_orientation_bs(tumble::world::World *world, ConstraintId id, const Float4 &orientation) {
    tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_target_orientation_bs(*constraint, quatf::load(orientation));
    return true;
}

bool get_rotation_in_constraint_space(Float4 &out, tumble::world::World *world, ConstraintId id) {
    const tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    quatf::store(out, tumble::swing_twist_constraint::get_rotation_in_constraint_space(*constraint, world->bodies));
    return true;
}

bool get_total_lambda_position(Float3 &out, tumble::world::World *world, ConstraintId id) {
    const tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::swing_twist_constraint::get_total_lambda_position(*constraint));
    return true;
}

float get_total_lambda_twist(tumble::world::World *world, ConstraintId id) {
    const tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::swing_twist_constraint::get_total_lambda_twist(*constraint);
}

float get_total_lambda_swing_y(tumble::world::World *world, ConstraintId id) {
    const tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::swing_twist_constraint::get_total_lambda_swing_y(*constraint);
}

float get_total_lambda_swing_z(tumble::world::World *world, ConstraintId id) {
    const tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::swing_twist_constraint::get_total_lambda_swing_z(*constraint);
}

bool get_total_lambda_motor(Float3 &out, tumble::world::World *world, ConstraintId id) {
    const tumble::swing_twist_constraint::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::swing_twist_constraint::get_total_lambda_motor(*constraint));
    return true;
}

}  // namespace swing_twist_constraint

namespace six_dof_constraint {

using tumble::six_dof_constraint::SixDOFConstraint;

/** the constraint, or null: with the error set for a bad id, without it for an axis past the last */
static SixDOFConstraint *found_axis(tumble::world::World *world, ConstraintId id, uint32_t axis, uint32_t axes) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    return axis < axes ? constraint : nullptr;
}

void default_settings(tumble::six_dof_constraint::SixDOFConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::world::World *world, body_id::BodyId body_a, body_id::BodyId body_b,
                    const tumble::six_dof_constraint::SixDOFConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::six_dof_constraint::create);
}

bool remove(tumble::world::World *world, ConstraintId id) {
    return found(tumble::six_dof_constraint::get(*world, id)) && tumble::six_dof_constraint::remove(*world, id);
}

bool set_translation_limits(tumble::world::World *world, ConstraintId id, const Float3 &limit_min, const Float3 &limit_max) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_translation_limits(*constraint, vec3f::load(limit_min), vec3f::load(limit_max));
    return true;
}

bool set_rotation_limits(tumble::world::World *world, ConstraintId id, const Float3 &limit_min, const Float3 &limit_max) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_rotation_limits(*constraint, vec3f::load(limit_min), vec3f::load(limit_max));
    return true;
}

bool set_limits_spring_settings(tumble::world::World *world, ConstraintId id, uint32_t axis, const spring_settings::SpringSettings &settings) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::six_dof_constraint::NUM_TRANSLATION);
    if (constraint == nullptr) return false;
    constraint->limits_spring_settings[axis] = settings;
    return true;
}

bool set_motor_settings(tumble::world::World *world, ConstraintId id, uint32_t axis, const motor_settings::MotorSettings &settings) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::six_dof_constraint::NUM);
    if (constraint == nullptr) return false;
    constraint->motor_settings[axis] = settings;
    return true;
}

bool set_motor_state(tumble::world::World *world, ConstraintId id, uint32_t axis, uint32_t state) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::six_dof_constraint::NUM);
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_motor_state(*constraint, (tumble::six_dof_constraint::SixDOFAxis)axis, (motor_settings::MotorState)state);
    return true;
}

bool set_target_velocity_cs(tumble::world::World *world, ConstraintId id, const Float3 &velocity) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_target_velocity_cs(*constraint, vec3f::load(velocity));
    return true;
}

bool set_target_angular_velocity_cs(tumble::world::World *world, ConstraintId id, const Float3 &velocity) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_target_angular_velocity_cs(*constraint, vec3f::load(velocity));
    return true;
}

bool set_target_position_cs(tumble::world::World *world, ConstraintId id, const Float3 &position) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_target_position_cs(*constraint, vec3f::load(position));
    return true;
}

bool set_target_orientation_cs(tumble::world::World *world, ConstraintId id, const Float4 &orientation) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_target_orientation_cs(*constraint, quatf::load(orientation));
    return true;
}

bool set_max_friction(tumble::world::World *world, ConstraintId id, uint32_t axis, float friction) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::six_dof_constraint::NUM);
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_max_friction(*constraint, axis, friction);
    return true;
}

bool make_free_axis(tumble::world::World *world, ConstraintId id, uint32_t axis) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::six_dof_constraint::NUM);
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::make_free_axis(*constraint, axis);
    return true;
}

bool make_fixed_axis(tumble::world::World *world, ConstraintId id, uint32_t axis) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::six_dof_constraint::NUM);
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::make_fixed_axis(*constraint, axis);
    return true;
}

bool set_limited_axis(tumble::world::World *world, ConstraintId id, uint32_t axis, float min, float max) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::six_dof_constraint::NUM);
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_limited_axis(*constraint, axis, min, max);
    return true;
}

}  // namespace six_dof_constraint

namespace filter {

/** whether the tables hold every layer: a fill that could not grow leaves them short, with the error set */
static bool whole(const tumble::filter::Filter &filter, const layers::Layers &layers) {
    return filter.enabled_object_layers.length == layers.object_layers && filter.enabled_broadphase_layers.length == layers.broadphase_layers;
}

tumble::filter::Filter *create(tumble::world::World *world) {
    tumble::filter::Filter *filter = util::create<tumble::filter::Filter>();
    if (filter == nullptr) return nullptr;
    *filter = tumble::filter::create(world->settings.layers);
    if (!whole(*filter, world->settings.layers)) {
        destroy(filter);
        return nullptr;
    }
    return filter;
}

void destroy(tumble::filter::Filter *filter) {
    tumble::filter::release(*filter);
    util::destroy(filter);
}

void enable_all_layers(tumble::filter::Filter *filter, tumble::world::World *world) {
    tumble::filter::enable_all_layers(*filter, world->settings.layers);
}

void disable_all_layers(tumble::filter::Filter *filter, tumble::world::World *world) {
    tumble::filter::disable_all_layers(*filter, world->settings.layers);
}

/** the filter's tables are the world's size, and the layer is one of them */
static bool fits(const tumble::filter::Filter *filter, const tumble::world::World *world, uint32_t layer, uint32_t layers) {
    return whole(*filter, world->settings.layers) && layer < layers;
}

bool enable_object_layer(tumble::filter::Filter *filter, tumble::world::World *world, uint32_t object_layer) {
    if (!fits(filter, world, object_layer, world->settings.layers.object_layers)) return false;
    tumble::filter::enable_object_layer(*filter, world->settings.layers, object_layer);
    return true;
}

bool disable_object_layer(tumble::filter::Filter *filter, tumble::world::World *world, uint32_t object_layer) {
    if (!fits(filter, world, object_layer, world->settings.layers.object_layers)) return false;
    tumble::filter::disable_object_layer(*filter, world->settings.layers, object_layer);
    return true;
}

bool enable_broadphase_layer(tumble::filter::Filter *filter, tumble::world::World *world, uint32_t broadphase_layer) {
    if (!fits(filter, world, broadphase_layer, world->settings.layers.broadphase_layers)) return false;
    tumble::filter::enable_broadphase_layer(*filter, world->settings.layers, broadphase_layer);
    return true;
}

bool disable_broadphase_layer(tumble::filter::Filter *filter, tumble::world::World *world, uint32_t broadphase_layer) {
    if (!fits(filter, world, broadphase_layer, world->settings.layers.broadphase_layers)) return false;
    tumble::filter::disable_broadphase_layer(*filter, world->settings.layers, broadphase_layer);
    return true;
}

bool is_object_layer_enabled(const tumble::filter::Filter *filter, uint32_t object_layer) {
    return tumble::filter::filter_object_layer(*filter, object_layer);
}

bool is_broadphase_layer_enabled(const tumble::filter::Filter *filter, uint32_t broadphase_layer) {
    return tumble::filter::filter_broadphase_layer(*filter, broadphase_layer);
}

bool set_from_body(tumble::filter::Filter *filter, tumble::world::World *world, body_id::BodyId body) {
    const tumble::rigid_body::RigidBody *found = rigid_body::resolve(world, body);
    if (found == nullptr || !whole(*filter, world->settings.layers)) return false;
    tumble::filter::set_from_body(*filter, world->settings.layers, found);
    return true;
}

uint32_t get_collision_groups(const tumble::filter::Filter *filter) { return filter->collision_groups; }
uint32_t get_collision_mask(const tumble::filter::Filter *filter) { return filter->collision_mask; }

bool copy(tumble::filter::Filter *out, const tumble::filter::Filter *source) {
    tumble::filter::copy(*out, *source);
    return out->enabled_object_layers.length == source->enabled_object_layers.length &&
           out->enabled_broadphase_layers.length == source->enabled_broadphase_layers.length;
}

}  // namespace filter

namespace query {

// one collector per query type, reused: its hits grow to the most a query has held. zeroed, not made by
// create_collector, as nothing runs static constructors here; each query sets its kind and resets it
static cast_ray::CastRayCollector ray_collector = {};
static cast_shape::CastShapeCollector cast_shape_collector = {};
static collide_shape::CollideShapeCollector collide_shape_collector = {};
static collide_point::CollidePointCollector collide_point_collector = {};

/** ALL, ANY or CLOSEST: a CUSTOM collector's function pointers are not js's to give */
static bool built_in(uint32_t kind) { return kind <= collector::COLLECT_CLOSEST; }

/** the filter with the query's group and mask */
static const tumble::filter::Filter &with_groups(tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    filter->collision_groups = collision_groups;
    filter->collision_mask = collision_mask;
    return *filter;
}

static Transformf transform_of(const Float3 &position, const Float4 &quaternion, const Float3 &scale) { return {position, quaternion, scale}; }

float initial_early_out_fraction() { return collector::INITIAL_EARLY_OUT_FRACTION; }
float unbounded_early_out() { return collector::UNBOUNDED; }

void default_cast_ray_settings(cast_ray::CastRaySettings &out) { out = cast_ray::default_settings(); }

uint32_t cast_ray(tumble::world::World *world, uint32_t kind, float early_out_fraction, const cast_ray::CastRaySettings &settings,
                  const Float3 &origin, const Float3 &direction, float length, tumble::filter::Filter *filter, uint32_t collision_groups,
                  uint32_t collision_mask) {
    if (!built_in(kind)) return 0;
    ray_collector.kind = (collector::CollectorKind)kind;
    cast_ray::reset(ray_collector);
    ray_collector.early_out_fraction = early_out_fraction;
    const cast_ray::Rayf ray = {vec3f::load(origin), vec3f::load(direction), length};
    tumble::query::cast_ray(*world, ray_collector, settings, ray, with_groups(filter, collision_groups, collision_mask));
    return ray_collector.hits.length;
}

bool read_cast_ray_hit(cast_ray::CastRayHit &out, uint32_t index) {
    if (index >= ray_collector.hits.length) return false;
    out = ray_collector.hits[index];
    return true;
}

void default_cast_shape_settings(CastShapeSettings &out) {
    const cast_shape::CastShapeSettings d = cast_shape::default_settings();
    out = {d.collide_with_backfaces_triangles,
           d.collide_with_backfaces_convex,
           d.collision_tolerance,
           d.penetration_tolerance,
           d.return_deepest_point,
           d.use_shrunken_shape_and_convex_radius,
           d.collide_only_with_active_edges,
           {0.0f, 0.0f, 0.0f}};
    vec3f::store(out.active_edge_movement_direction, d.active_edge_movement_direction);
}

uint32_t cast_shape(tumble::world::World *world, uint32_t kind, float early_out_fraction, const CastShapeSettings &s,
                    const shapes::Shape *shape, const Float3 &position, const Float4 &quaternion, const Float3 &scale,
                    const Float3 &displacement, tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    if (!built_in(kind)) return 0;
    cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    settings.collide_with_backfaces_triangles = s.collide_with_backfaces_triangles;
    settings.collide_with_backfaces_convex = s.collide_with_backfaces_convex;
    settings.collect_faces = false;
    settings.collision_tolerance = s.collision_tolerance;
    settings.penetration_tolerance = s.penetration_tolerance;
    settings.return_deepest_point = s.return_deepest_point;
    settings.use_shrunken_shape_and_convex_radius = s.use_shrunken_shape_and_convex_radius;
    settings.collide_only_with_active_edges = s.collide_only_with_active_edges;
    settings.active_edge_movement_direction = vec3f::load(s.active_edge_movement_direction);
    cast_shape_collector.kind = (collector::CollectorKind)kind;
    cast_shape::reset(cast_shape_collector);
    cast_shape_collector.early_out_fraction = early_out_fraction;
    tumble::query::cast_shape(*world, cast_shape_collector, settings, shape, transform_of(position, quaternion, scale), vec3f::load(displacement),
                              with_groups(filter, collision_groups, collision_mask));
    return cast_shape_collector.hits.length;
}

bool read_cast_shape_hit(CastShapeHit &out, uint32_t index) {
    if (index >= cast_shape_collector.hits.length) return false;
    const cast_shape::CastShapeHit &hit = cast_shape_collector.hits[index];
    out.status = hit.status;
    out.fraction = hit.fraction;
    vec3f::store(out.point_a, hit.point_a);
    vec3f::store(out.point_b, hit.point_b);
    out.penetration_depth = hit.penetration_depth;
    vec3f::store(out.penetration_axis, hit.penetration_axis);
    vec3f::store(out.normal, hit.normal);
    out.sub_shape_id_a = hit.sub_shape_id_a;
    out.sub_shape_id_b = hit.sub_shape_id_b;
    out.material_id_a = hit.material_id_a;
    out.material_id_b = hit.material_id_b;
    out.body_id_b = hit.body_id_b;
    return true;
}

void default_collide_shape_settings(CollideShapeSettings &out) {
    const collide_shape::CollideShapeSettings d = collide_shape::default_settings();
    out = {d.max_separation_distance,
           d.collision_tolerance,
           d.penetration_tolerance,
           d.collide_with_backfaces,
           d.collide_only_with_active_edges,
           {0.0f, 0.0f, 0.0f},
           d.internal_edge_removal_vertex_tolerance_sq};
    vec3f::store(out.active_edge_movement_direction, d.active_edge_movement_direction);
}

uint32_t collide_shape(tumble::world::World *world, uint32_t kind, float early_out_fraction, const CollideShapeSettings &s,
                       const shapes::Shape *shape, const Float3 &position, const Float4 &quaternion, const Float3 &scale,
                       bool internal_edge_removal, tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    if (!built_in(kind)) return 0;
    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.max_separation_distance = s.max_separation_distance;
    settings.collision_tolerance = s.collision_tolerance;
    settings.penetration_tolerance = s.penetration_tolerance;
    settings.collide_with_backfaces = s.collide_with_backfaces;
    settings.collide_only_with_active_edges = s.collide_only_with_active_edges;
    settings.active_edge_movement_direction = vec3f::load(s.active_edge_movement_direction);
    settings.collect_faces = false;
    settings.internal_edge_removal_vertex_tolerance_sq = s.internal_edge_removal_vertex_tolerance_sq;
    collide_shape_collector.kind = (collector::CollectorKind)kind;
    collide_shape::reset(collide_shape_collector);
    collide_shape_collector.early_out_fraction = early_out_fraction;
    const Transformf transform = transform_of(position, quaternion, scale);
    const tumble::filter::Filter &query_filter = with_groups(filter, collision_groups, collision_mask);
    if (internal_edge_removal) {
        tumble::query::collide_shape_with_internal_edge_removal(*world, collide_shape_collector, settings, shape, transform, query_filter);
    } else {
        tumble::query::collide_shape(*world, collide_shape_collector, settings, shape, transform, query_filter);
    }
    return collide_shape_collector.hits.length;
}

bool read_collide_shape_hit(CollideShapeHit &out, uint32_t index) {
    if (index >= collide_shape_collector.hits.length) return false;
    const collide_shape::CollideShapeHit &hit = collide_shape_collector.hits[index];
    vec3f::store(out.point_a, hit.point_a);
    vec3f::store(out.point_b, hit.point_b);
    vec3f::store(out.penetration_axis, hit.penetration_axis);
    out.penetration = hit.penetration;
    out.sub_shape_id_a = hit.sub_shape_id_a;
    out.sub_shape_id_b = hit.sub_shape_id_b;
    out.material_id_a = hit.material_id_a;
    out.material_id_b = hit.material_id_b;
    out.body_id_b = hit.body_id_b;
    return true;
}

void default_collide_point_settings(collide_point::CollidePointSettings &out) { out = collide_point::default_settings(); }

uint32_t collide_point(tumble::world::World *world, uint32_t kind, const collide_point::CollidePointSettings &settings, const Float3 &point,
                       tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    if (!built_in(kind)) return 0;
    collide_point_collector.kind = (collector::CollectorKind)kind;
    collide_point::reset(collide_point_collector);
    tumble::query::collide_point(*world, collide_point_collector, settings, vec3f::load(point), with_groups(filter, collision_groups, collision_mask));
    return collide_point_collector.hits.length;
}

bool read_collide_point_hit(collide_point::CollidePointHit &out, uint32_t index) {
    if (index >= collide_point_collector.hits.length) return false;
    out = collide_point_collector.hits[index];
    return true;
}

}  // namespace query

namespace kcc_policy {

void default_body_policy(tumble::kcc_policy::BodyPolicy &out) { out = {}; }

tumble::kcc_policy::Policy *create() {
    tumble::kcc_policy::Policy *policy = util::create<tumble::kcc_policy::Policy>();
    if (policy != nullptr) *policy = tumble::kcc_policy::create();
    return policy;
}

void destroy(tumble::kcc_policy::Policy *policy) {
    tumble::kcc_policy::release(*policy);
    util::destroy(policy);
}

void get_defaults(tumble::kcc_policy::BodyPolicy &out, const tumble::kcc_policy::Policy *policy) { out = policy->defaults; }
void set_defaults(tumble::kcc_policy::Policy *policy, const tumble::kcc_policy::BodyPolicy &body_policy) { policy->defaults = body_policy; }

bool set(tumble::kcc_policy::Policy *policy, body_id::BodyId body, const tumble::kcc_policy::BodyPolicy &body_policy) {
    return tumble::kcc_policy::set(*policy, body, body_policy);
}

bool remove(tumble::kcc_policy::Policy *policy, body_id::BodyId body) { return tumble::kcc_policy::remove(*policy, body); }

void get(tumble::kcc_policy::BodyPolicy &out, const tumble::kcc_policy::Policy *policy, body_id::BodyId body) {
    out = tumble::kcc_policy::get(policy, body);
}

}  // namespace kcc_policy

namespace kcc {

using tumble::kcc::KCC;

static Float3 float3_of(Vec3f v) {
    Float3 out;
    return vec3f::store(out, v);
}

void default_settings(KCCSettings &out) {
    const tumble::kcc::KCCSettings d;
    out = {d.shape, d.shape_offset, d.mass, d.max_strength, d.up, d.max_slope_angle, d.predictive_contact_distance, d.character_padding,
           d.max_collision_iterations, d.max_constraint_iterations, d.collision_tolerance, d.penetration_recovery_speed, d.min_time_remaining,
           d.max_num_hits, d.hit_reduction_cos_max_angle, d.back_face_mode, d.enhanced_internal_edge_removal, d.has_supporting_volume_plane,
           d.supporting_volume_plane, (uint32_t)d.user_data, (uint32_t)(d.user_data >> 32), d.observes_contacts, d.inner_rigid_body_shape,
           d.inner_rigid_body_object_layer};
}

void default_update_settings(tumble::kcc::UpdateSettings &out) { out = {}; }

KCC *create(const KCCSettings &s, const Float3 &position, const Float4 &quaternion) {
    tumble::kcc::KCCSettings settings;
    settings.shape = s.shape;
    settings.shape_offset = s.shape_offset;
    settings.mass = s.mass;
    settings.max_strength = s.max_strength;
    settings.up = s.up;
    settings.max_slope_angle = s.max_slope_angle;
    settings.predictive_contact_distance = s.predictive_contact_distance;
    settings.character_padding = s.character_padding;
    settings.max_collision_iterations = s.max_collision_iterations;
    settings.max_constraint_iterations = s.max_constraint_iterations;
    settings.collision_tolerance = s.collision_tolerance;
    settings.penetration_recovery_speed = s.penetration_recovery_speed;
    settings.min_time_remaining = s.min_time_remaining;
    settings.max_num_hits = s.max_num_hits;
    settings.hit_reduction_cos_max_angle = s.hit_reduction_cos_max_angle;
    settings.back_face_mode = (tumble::kcc::BackFaceMode)s.back_face_mode;
    settings.enhanced_internal_edge_removal = s.enhanced_internal_edge_removal;
    settings.has_supporting_volume_plane = s.has_supporting_volume_plane;
    settings.supporting_volume_plane = s.supporting_volume_plane;
    settings.user_data = (uint64_t)s.user_data_high << 32 | s.user_data_low;
    settings.observes_contacts = s.observes_contacts;
    settings.inner_rigid_body_shape = s.inner_rigid_body_shape;
    settings.inner_rigid_body_object_layer = s.inner_rigid_body_object_layer;
    return tumble::kcc::create(settings, vec3f::load(position), quatf::load(quaternion));
}

void destroy(KCC *character) { tumble::kcc::destroy(character); }
void add(tumble::world::World *world, KCC *character) { tumble::kcc::add(*world, *character); }
void remove(tumble::world::World *world, KCC *character) { tumble::kcc::remove(*world, *character); }

void read_state(KCCState &out, const KCC *character) {
    const tumble::kcc::GroundInfo &ground = character->ground;
    out.position = float3_of(character->position);
    quatf::store(out.quaternion, character->quaternion);
    out.linear_velocity = float3_of(character->linear_velocity);
    out.up = float3_of(character->up);
    out.ground_state = ground.state;
    out.ground_normal = float3_of(ground.normal);
    out.ground_velocity = float3_of(ground.velocity);
    out.ground_body_id = ground.body_id;
    out.ground_sub_shape_id = ground.sub_shape_id;
    out.ground_position = float3_of(ground.position);
    out.shape = character->shape;
    out.inner_rigid_body_id = character->inner_rigid_body_id;
    out.contact_count = character->contacts.length;
    out.contact_event_count = character->contact_events.length;
}

void set_linear_velocity(KCC *character, const Float3 &velocity) { character->linear_velocity = vec3f::load(velocity); }

uint8_t *contacts(KCC *character) { return reinterpret_cast<uint8_t *>(character->contacts.data); }

bool read_contact_event(KCCContactEvent &out, const KCC *character, uint32_t index) {
    if (index >= character->contact_events.length) return false;
    const tumble::kcc::ContactEvent &event = character->contact_events[index];
    out = {event.kind, event.body_id, event.sub_shape_id, float3_of(event.position), float3_of(event.normal)};
    return true;
}

void set_position(tumble::world::World *world, KCC *character, const Float3 &position) {
    tumble::kcc::set_position(*world, *character, vec3f::load(position));
}

void set_quaternion(tumble::world::World *world, KCC *character, const Float4 &quaternion) {
    tumble::kcc::set_quaternion(*world, *character, quatf::load(quaternion));
}

bool is_slope_too_steep(const KCC *character, const Float3 &normal) { return tumble::kcc::is_slope_too_steep(*character, vec3f::load(normal)); }

void get_center_of_mass_transform(Float3 &out_position, Float4 &out_quaternion, const KCC *character) {
    Vec3f position;
    Quatf quaternion;
    tumble::kcc::get_center_of_mass_transform(*character, position, quaternion);
    vec3f::store(out_position, position);
    quatf::store(out_quaternion, quaternion);
}

bool has_collided_with(const KCC *character, body_id::BodyId body) { return tumble::kcc::has_collided_with(*character, body); }

bool can_walk_stairs(const KCC *character, const Float3 &velocity) { return tumble::kcc::can_walk_stairs(*character, vec3f::load(velocity)); }

void cancel_velocity_towards_steep_slopes(Float3 &out, const KCC *character, const Float3 &velocity) {
    out = float3_of(tumble::kcc::cancel_velocity_towards_steep_slopes(*character, vec3f::load(velocity)));
}

void update_ground_velocity(tumble::world::World *world, KCC *character, const tumble::kcc_policy::Policy *policy) {
    tumble::kcc::update_ground_velocity(*world, *character, policy);
}

void refresh_contacts(tumble::world::World *world, KCC *character, tumble::filter::Filter *filter, uint32_t collision_groups,
                      uint32_t collision_mask, const tumble::kcc_policy::Policy *policy) {
    tumble::kcc::refresh_contacts(*world, *character, query::with_groups(filter, collision_groups, collision_mask), policy);
}

bool set_shape(tumble::world::World *world, KCC *character, shapes::Shape *shape, tumble::filter::Filter *filter, uint32_t collision_groups,
               uint32_t collision_mask, const tumble::kcc_policy::Policy *policy, float max_penetration_depth) {
    return tumble::kcc::set_shape(*world, *character, shape, query::with_groups(filter, collision_groups, collision_mask), policy,
                                  max_penetration_depth);
}

void move(tumble::world::World *world, KCC *character, float delta_time, const Float3 &gravity, const tumble::kcc_policy::Policy *policy,
          tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    tumble::kcc::move(*world, *character, delta_time, vec3f::load(gravity), policy, query::with_groups(filter, collision_groups, collision_mask));
}

bool stick_to_floor(tumble::world::World *world, KCC *character, const Float3 &step_down, tumble::filter::Filter *filter,
                    uint32_t collision_groups, uint32_t collision_mask, const tumble::kcc_policy::Policy *policy) {
    return tumble::kcc::stick_to_floor(*world, *character, vec3f::load(step_down), query::with_groups(filter, collision_groups, collision_mask),
                                       policy);
}

bool walk_stairs(tumble::world::World *world, KCC *character, float delta_time, const Float3 &step_up, const Float3 &step_forward,
                 const Float3 &step_forward_test, const Float3 &step_down_extra, tumble::filter::Filter *filter, uint32_t collision_groups,
                 uint32_t collision_mask, const tumble::kcc_policy::Policy *policy) {
    return tumble::kcc::walk_stairs(*world, *character, delta_time, vec3f::load(step_up), vec3f::load(step_forward), vec3f::load(step_forward_test),
                                    vec3f::load(step_down_extra), query::with_groups(filter, collision_groups, collision_mask), policy);
}

void update(tumble::world::World *world, KCC *character, float delta_time, const Float3 &gravity, const tumble::kcc::UpdateSettings &settings,
            const tumble::kcc_policy::Policy *policy, tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    tumble::kcc::update(*world, *character, delta_time, vec3f::load(gravity), settings, policy,
                        query::with_groups(filter, collision_groups, collision_mask));
}

void collide(tumble::world::World *world, KCC *character, const Float3 &position, const Float3 &movement_direction, tumble::filter::Filter *filter,
             uint32_t collision_groups, uint32_t collision_mask, const tumble::kcc_policy::Policy *policy) {
    tumble::kcc::collide(*world, *character, vec3f::load(position), vec3f::load(movement_direction),
                         query::with_groups(filter, collision_groups, collision_mask), policy);
}

void solve(KCCSolve &out, tumble::world::World *world, KCC *character, const Float3 &velocity, float delta_time, float time_remaining,
           const tumble::kcc_policy::Policy *policy) {
    Vec3f solved = vec3f::load(velocity);
    Vec3f displacement;
    out.time_simulated = tumble::kcc::solve(*world, *character, solved, delta_time, time_remaining, policy, displacement);
    out.velocity = float3_of(solved);
    out.displacement = float3_of(displacement);
}

bool cast(KCCContact &out, tumble::world::World *world, KCC *character, const Float3 &position, const Float3 &displacement,
          tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask, const tumble::kcc_policy::Policy *policy) {
    tumble::kcc::CharacterContact hit;
    if (!tumble::kcc::cast(hit, *world, *character, vec3f::load(position), vec3f::load(displacement),
                           query::with_groups(filter, collision_groups, collision_mask), policy)) {
        return false;
    }
    out = {float3_of(hit.position), float3_of(hit.linear_velocity), float3_of(hit.contact_normal), float3_of(hit.surface_normal), hit.distance,
           hit.fraction, hit.body_id, hit.sub_shape_id, hit.material_id, hit.motion_type, hit.is_sensor};
    return true;
}

void update_ground(tumble::world::World *world, KCC *character, float delta_time, bool skip_contact_velocity_check,
                   const tumble::kcc_policy::Policy *policy) {
    tumble::kcc::update_ground(*world, *character, delta_time, skip_contact_velocity_check, policy);
}

void apply_weight(tumble::world::World *world, KCC *character, const Float3 &gravity, float delta_time) {
    tumble::kcc::apply_weight(*world, *character, vec3f::load(gravity), delta_time);
}

void begin_contact_tracking(KCC *character) { tumble::kcc::begin_contact_tracking(*character); }
void end_contact_tracking(KCC *character) { tumble::kcc::end_contact_tracking(*character); }

}  // namespace kcc

namespace debug {

tumble::debug::DebugBuffers *create() { return tumble::debug::create(); }
void destroy(tumble::debug::DebugBuffers *buffers) { tumble::debug::release(buffers); }
void clear(tumble::debug::DebugBuffers *buffers) { tumble::debug::clear(*buffers); }
const float *lines(const tumble::debug::DebugBuffers *buffers) { return buffers->lines.data; }
uint32_t line_count(const tumble::debug::DebugBuffers *buffers) { return buffers->line_count; }
const float *triangles(const tumble::debug::DebugBuffers *buffers) { return buffers->triangles.data; }
uint32_t triangle_count(const tumble::debug::DebugBuffers *buffers) { return buffers->triangle_count; }

void default_bodies_options(tumble::debug::BodiesOptions &out) { out = {}; }
void default_body_options(tumble::debug::BodyOptions &out) { out = {}; }
void default_shape_options(tumble::debug::ShapeOptions &out) { out = {}; }
void default_joints_options(tumble::debug::JointsOptions &out) { out = {}; }

void bodies(tumble::debug::DebugBuffers *out, const tumble::world::World *world, const tumble::debug::BodiesOptions &options) {
    tumble::debug::bodies(*out, *world, options);
}

bool body(tumble::debug::DebugBuffers *out, tumble::world::World *world, body_id::BodyId body, const tumble::debug::BodyOptions &options) {
    const tumble::rigid_body::RigidBody *found = rigid_body::resolve(world, body);
    if (found == nullptr) return false;
    tumble::debug::body(*out, *found, options);
    return true;
}

void line(tumble::debug::DebugBuffers *out, const Float3 &a, const Float3 &b, const Float3 &color) {
    tumble::debug::line(*out, vec3f::load(a), vec3f::load(b), vec3f::load(color));
}

void shape(tumble::debug::DebugBuffers *out, const shapes::Shape *shape, const Float3 &position, const Float4 &quaternion,
           const tumble::debug::ShapeOptions &options) {
    tumble::debug::shape(*out, shape, vec3f::load(position), quatf::load(quaternion), options);
}

void contacts(tumble::debug::DebugBuffers *out, const tumble::world::World *world) { tumble::debug::contacts(*out, *world); }
void character_contacts(tumble::debug::DebugBuffers *out, const tumble::kcc::KCC *character) { tumble::debug::character_contacts(*out, *character); }
void contact_constraints(tumble::debug::DebugBuffers *out, const tumble::world::World *world) { tumble::debug::contact_constraints(*out, *world); }

void joints(tumble::debug::DebugBuffers *out, const tumble::world::World *world, const tumble::debug::JointsOptions &options) {
    tumble::debug::joints(*out, *world, options);
}

}  // namespace debug

namespace contact_pairs {

void begin(tumble::world::World *world) { tumble::contact_pairs::begin(*world); }

void record_world_contacts(tumble::world::World *world, uint32_t first_added, uint32_t first_persisted) {
    tumble::contact_pairs::record_world_contacts(*world, first_added, first_persisted);
}

void record_character_contacts(tumble::world::World *world, const tumble::kcc::KCC *character) {
    tumble::contact_pairs::record_character_contacts(*world, *character);
}

void end(tumble::world::World *world) { tumble::contact_pairs::end(*world); }

static void read_pair(ContactPair &out, const tumble::world::World *world, uint32_t handle) {
    const tumble::contact_pairs::ContactPair &pair = world->contact_pairs.pairs[handle];
    out = {handle,
           pair.a.body_id,
           pair.a.sub_shape_id,
           (uint32_t)pair.a.user_data,
           (uint32_t)(pair.a.user_data >> 32),
           pair.a.is_sensor,
           pair.b.body_id,
           pair.b.sub_shape_id,
           (uint32_t)pair.b.user_data,
           (uint32_t)(pair.b.user_data >> 32),
           pair.b.is_sensor,
           pair.point,
           pair.normal,
           pair.penetration_depth,
           pair.relative_velocity,
           pair.added};
}

uint32_t active_count(const tumble::world::World *world) { return world->contact_pairs.active.length; }

bool read_active(ContactPair &out, const tumble::world::World *world, uint32_t index) {
    if (index >= world->contact_pairs.active.length) return false;
    read_pair(out, world, world->contact_pairs.active[index]);
    return true;
}

uint32_t removed_count(const tumble::world::World *world) { return world->contact_pairs.removed.length; }

bool read_removed(ContactPair &out, const tumble::world::World *world, uint32_t index) {
    if (index >= world->contact_pairs.removed.length) return false;
    read_pair(out, world, world->contact_pairs.removed[index]);
    return true;
}

}  // namespace contact_pairs

}  // namespace adapters
