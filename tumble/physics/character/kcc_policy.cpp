#include "tumble/physics/character/kcc_policy.h"

#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"

TUMBLE_NAMESPACE_BEGIN

namespace kcc_policy {

static constexpr BodyPolicy DEFAULTS = {};

void release(Policy &policy) { policy.bodies.release(); }

bool set(Policy &policy, body_id::BodyId body_id, const BodyPolicy &body_policy) { return policy.bodies.set({body_id}, body_policy); }

bool remove(Policy &policy, body_id::BodyId body_id) { return policy.bodies.remove({body_id}); }

const BodyPolicy &get(const Policy *policy, body_id::BodyId body_id) {
    if (policy == nullptr) return DEFAULTS;
    const BodyPolicy *own = policy->bodies.get({body_id});
    return own != nullptr ? *own : policy->defaults;
}

static bool is_zero(const Float3 &v) { return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f; }

static const BodyPolicy &own(void *context, body_id::BodyId body_id) { return get(static_cast<const Policy *>(context), body_id); }

/** a zero surface velocity adds nothing, so a body's -0 stays -0 */
static void adjust_body_velocity(const kcc::KCC &, const rigid_body::RigidBody &body, Vec3f &io_linear_velocity, Vec3f &io_angular_velocity,
                                 void *context) {
    const BodyPolicy &body_policy = own(context, body.id);
    if (!is_zero(body_policy.surface_linear_velocity)) {
        io_linear_velocity = vec3f::add(io_linear_velocity, vec3f::load(body_policy.surface_linear_velocity));
    }
    if (!is_zero(body_policy.surface_angular_velocity)) {
        io_angular_velocity = vec3f::add(io_angular_velocity, vec3f::load(body_policy.surface_angular_velocity));
    }
}

static bool validate(const kcc::KCC &, const rigid_body::RigidBody &body, sub_shape::SubShapeId, Vec3f, Vec3f, void *context) {
    return !own(context, body.id).ignore;
}

static void met(const kcc::KCC &, const rigid_body::RigidBody &body, sub_shape::SubShapeId, Vec3f, Vec3f,
                kcc::CharacterContactSettings &io_settings, void *context) {
    const BodyPolicy &body_policy = own(context, body.id);
    io_settings.can_push_character = body_policy.can_push_character;
    io_settings.can_receive_impulses = body_policy.can_receive_impulses;
}

/** on a still, walkable surface of a body that does not let it slide, the character stops */
static void solve(const kcc::KCC &character, const rigid_body::RigidBody &body, sub_shape::SubShapeId, Vec3f, Vec3f contact_normal,
                  Vec3f contact_velocity, Vec3f, Vec3f &io_new_character_velocity, void *context) {
    if (own(context, body.id).slide_on_walkable) return;
    if (vec3f::squared_length(contact_velocity) <= 1e-12f && !kcc::is_slope_too_steep(character, contact_normal)) {
        io_new_character_velocity = vec3f::make(0.0f, 0.0f, 0.0f);
    }
}

kcc::CharacterListener listener(const Policy &policy) {
    return {adjust_body_velocity, validate, met, met, solve, nullptr, const_cast<Policy *>(&policy)};
}

}  // namespace kcc_policy

TUMBLE_NAMESPACE_END
