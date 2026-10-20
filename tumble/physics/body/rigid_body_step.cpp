#include "tumble/physics/body/rigid_body_step.h"

#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body_step {

/** the centre of mass moved by step along the allowed translation axes; a locked axis keeps its value bit for bit */
static void move_center_of_mass(rigid_body::RigidBody *body, Vec3f moved) {
    const uint32_t allowed = body->motion_properties.allowed_degrees_of_freedom & 0b111;
    if (allowed == 0b111) {
        vec3f::store(body->center_of_mass_position, moved);
        return;
    }
    const int32_t x = (allowed & 0b001) ? -1 : 0;
    const int32_t y = (allowed & 0b010) ? -1 : 0;
    const int32_t z = (allowed & 0b100) ? -1 : 0;
    const Vec3f before = vec3f::load(body->center_of_mass_position);
    vec3f::store(body->center_of_mass_position, {v4f::select(moved.v, before.v, v4i::make(x, y, z, z))});
}

void add_position_step(rigid_body::RigidBody *body, Vec3f linear_velocity_times_delta_time) {
    move_center_of_mass(body, vec3f::add(vec3f::load(body->center_of_mass_position), linear_velocity_times_delta_time));
}

void sub_position_step(rigid_body::RigidBody *body, Vec3f linear_velocity_times_delta_time) {
    move_center_of_mass(body, vec3f::subtract(vec3f::load(body->center_of_mass_position), linear_velocity_times_delta_time));
}

/** the body's rotation turned by the rotation vector, angle sign times its length about its direction */
static void rotate_by(rigid_body::RigidBody *body, Vec3f angular_velocity_times_delta_time, float sign) {
    // this used to use the equation: d/dt R(t) = 1/2 * w(t) * R(t) so that R(t + dt) = R(t) + 1/2 * w(t) * R(t) * dt
    // see: appendix b of an introduction to physically based modeling: rigid body simulation ii-nonpenetration constraints
    // url: https://www.cs.cmu.edu/~baraff/sigcourse/notesd2.pdf
    // but this is a first order approximation and does not work well for kinematic ragdolls that are driven to a new
    // pose if the poses differ enough. so now we split w(t) * dt into an axis and angle part and create a quaternion with it.
    // note that the resulting quaternion is normalized since otherwise numerical drift will eventually make the rotation non-normalized.
    const float len = vec3f::length(angular_velocity_times_delta_time);
    if (len > 1.0e-6f) {
        const Vec3f axis = vec3f::scale(angular_velocity_times_delta_time, 1.0f / len);
        const Quatf rotation = quatf::set_axis_angle(axis, sign > 0.0f ? len : -len);
        quatf::store(body->quaternion, quatf::normalize(quatf::multiply(rotation, quatf::load(body->quaternion))));
    }
}

void add_rotation_step(rigid_body::RigidBody *body, Vec3f angular_velocity_times_delta_time) {
    rotate_by(body, angular_velocity_times_delta_time, 1.0f);
}

void sub_rotation_step(rigid_body::RigidBody *body, Vec3f angular_velocity_times_delta_time) {
    rotate_by(body, angular_velocity_times_delta_time, -1.0f);
}

void derive_transform(rigid_body::RigidBody *body) {
    const Vec3f shape_center_of_mass_in_world_space =
        vec3f::transform_quat(vec3f::load(body->shape->center_of_mass), quatf::load(body->quaternion));
    vec3f::store(body->position, vec3f::subtract(vec3f::load(body->center_of_mass_position), shape_center_of_mass_in_world_space));

    rigid_body::update_aabb(body);
}

}  // namespace rigid_body_step

TUMBLE_NAMESPACE_END
