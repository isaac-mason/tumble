#pragma once

// the constraint tests' world without a world: bodies, layers and constraints, and a step that runs the
// solver phases by hand (gravity, setup, warm start, velocity iterations, integration, position iterations).

#include "tumble/instance.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/rigid_body_step.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

struct ConstraintScene {
    bodies::Bodies bodies;
    layers::Layers layers;
    constraints::Constraints constraints;
    uint32_t layer;
    collections::Vec<constraint_id::ConstraintId> ids;
};

inline ConstraintScene make_constraint_scene(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    ConstraintScene scene = {};
    scene.bodies = bodies::init();
    scene.layers = layers::create();
    const uint32_t broadphase_layer = layers::add_broadphase_layer(scene.layers);
    scene.layer = layers::add_object_layer(scene.layers, broadphase_layer);
    scene.constraints = constraints::create();
    return scene;
}

inline void release_constraint_scene(ConstraintScene &scene) {
    scene.ids.release();
    constraints::release(scene.constraints);
    bodies::release(scene.bodies);
    layers::release(scene.layers);
}

/** a sphere of radius 0.5 (mass 1000 * 4/3 pi 0.125, about 523.6), or a shapeless body when static */
inline rigid_body::RigidBody *add_body(ConstraintScene &scene, motion_type::MotionType motion_type, const Float3 &position,
                                       const Float4 &quaternion = {0.0f, 0.0f, 0.0f, 1.0f}) {
    rigid_body::RigidBodySettings settings;
    settings.shape = motion_type == motion_type::MOTION_STATIC ? nullptr : sphere::create({.radius = 0.5f});
    settings.object_layer = scene.layer;
    settings.motion_type = motion_type;
    settings.position = position;
    settings.quaternion = quaternion;
    settings.linear_damping = 0.0f;
    settings.angular_damping = 0.0f;
    return rigid_body::create(scene.bodies, scene.layers, settings);
}

/** turns the body by rotation, in world space: q = rotation * q */
inline void rotate_body(rigid_body::RigidBody *body, Quatf rotation) {
    quatf::store(body->quaternion, quatf::multiply(rotation, quatf::load(body->quaternion)));
}

/** a world point on the body, given relative to its center of mass in body space */
inline Vec3f world_point(const rigid_body::RigidBody *body, Vec3f local) {
    return vec3f::add(vec3f::transform_quat(local, quatf::load(body->quaternion)), vec3f::load(body->center_of_mass_position));
}

/** one step: ten velocity iterations and two position iterations over scene.ids, gravity on dynamic bodies */
inline void step_constraint_scene(ConstraintScene &scene, float delta_time, Vec3f gravity) {
    bodies::Bodies &bodies = scene.bodies;
    bodies.step_stamp++;

    for (uint32_t i = 0; i < bodies.pool.length; i++) {
        rigid_body::RigidBody *body = bodies.pool[i];
        if (body == nullptr || body->motion_type != motion_type::MOTION_DYNAMIC) continue;
        Float3 &velocity = body->motion_properties.linear_velocity;
        vec3f::store(velocity, vec3f::scale_and_add(vec3f::load(velocity), gravity, delta_time));
    }

    constraints::update_sleeping(scene.constraints, bodies);
    constraints::setup_velocity_constraints(scene.constraints, bodies, delta_time);
    constraints::warm_start_velocity_constraints(scene.constraints, bodies, scene.ids, 1.0f);
    for (uint32_t i = 0; i < 10; i++) {
        constraints::solve_velocity_constraints_for_island(scene.constraints, bodies, scene.ids, delta_time);
    }

    for (uint32_t i = 0; i < bodies.pool.length; i++) {
        rigid_body::RigidBody *body = bodies.pool[i];
        if (body == nullptr || body->motion_type == motion_type::MOTION_STATIC) continue;
        rigid_body_step::add_position_step(body, vec3f::scale(vec3f::load(body->motion_properties.linear_velocity), delta_time));
        rigid_body_step::add_rotation_step(body, vec3f::scale(vec3f::load(body->motion_properties.angular_velocity), delta_time));
    }

    for (uint32_t i = 0; i < 2; i++) {
        constraints::solve_position_constraints_for_island(scene.constraints, bodies, scene.ids, 0.2f, delta_time);
    }

    for (uint32_t i = 0; i < bodies.pool.length; i++) {
        rigid_body::RigidBody *body = bodies.pool[i];
        if (body != nullptr && body->motion_type != motion_type::MOTION_STATIC) rigid_body_step::derive_transform(body);
    }
}
