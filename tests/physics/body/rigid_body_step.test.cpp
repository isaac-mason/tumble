#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/dof.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/rigid_body_step.h"
#include "tumble/physics/shapes/box.h"
#include "test.h"
#include "test_math.h"

using rigid_body::RigidBody;

static RigidBody make_body(uint32_t allowed_degrees_of_freedom) {
    RigidBody body = {};
    body.quaternion = {0.0f, 0.0f, 0.0f, 1.0f};
    body.motion_properties = motion_properties::create();
    body.motion_properties.allowed_degrees_of_freedom = allowed_degrees_of_freedom;
    return body;
}

void test_rigid_body_step_position_on_locked_axes(TestCtx &ctx) {
    // the dof mask applies to the step, a locked axis stays where it is (not zero)
    RigidBody body = make_body(dof::DOF_ALL & ~dof::TRANSLATION_X);
    Float3 &com = body.center_of_mass_position;
    com = {5.0f, 10.0f, -3.0f};
    rigid_body_step::add_position_step(&body, vec3f::make(1.0f, 2.0f, 4.0f));
    CHECK(ctx, com.x == 5.0f && com.y == 12.0f && com.z == 1.0f);

    com = {5.0f, 10.0f, -3.0f};
    rigid_body_step::sub_position_step(&body, vec3f::make(1.0f, 2.0f, 4.0f));
    CHECK(ctx, com.x == 5.0f && com.y == 8.0f && com.z == -7.0f);

    // rotation bits play no part; everything free moves every axis
    com = {0.0f, 0.0f, 0.0f};
    body.motion_properties.allowed_degrees_of_freedom = dof::DOF_ROTATION_ONLY;
    rigid_body_step::add_position_step(&body, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, com.x == 0.0f && com.y == 0.0f && com.z == 0.0f);
    body.motion_properties.allowed_degrees_of_freedom = dof::DOF_ALL;
    rigid_body_step::add_position_step(&body, vec3f::make(1.0f, -2.0f, 3.0f));
    CHECK(ctx, com.x == 1.0f && com.y == -2.0f && com.z == 3.0f);
}

void test_rigid_body_step_rotation(TestCtx &ctx) {
    // a quarter turn about z in one step, composed on the left of the current rotation
    RigidBody body = make_body(dof::DOF_ALL);
    Float4 &q = body.quaternion;
    rigid_body_step::add_rotation_step(&body, vec3f::make(0.0f, 0.0f, scalar::PI * 0.5f));
    Vec3f x_axis = vec3f::transform_quat(vec3f::make(1.0f, 0.0f, 0.0f), quatf::load(q));
    CHECK(ctx, vec3f::distance(x_axis, vec3f::make(0.0f, 1.0f, 0.0f)) < 1e-5f);
    CHECK_NEAR(ctx, quat_length(quatf::load(q)), 1.0f, 1e-6f);

    // then a quarter turn about x, applied in world space: the rotated x axis (now y) goes to z
    rigid_body_step::add_rotation_step(&body, vec3f::make(scalar::PI * 0.5f, 0.0f, 0.0f));
    x_axis = vec3f::transform_quat(vec3f::make(1.0f, 0.0f, 0.0f), quatf::load(q));
    CHECK(ctx, vec3f::distance(x_axis, vec3f::make(0.0f, 0.0f, 1.0f)) < 1e-5f);

    // sub undoes add, in reverse order
    rigid_body_step::sub_rotation_step(&body, vec3f::make(scalar::PI * 0.5f, 0.0f, 0.0f));
    rigid_body_step::sub_rotation_step(&body, vec3f::make(0.0f, 0.0f, scalar::PI * 0.5f));
    CHECK(ctx, scalar::abs(q.w) > 1.0f - 1e-6f);

    // a step at or under 1e-6 rad is no step at all
    Float4 tilted = stored_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), 0.3f);
    q = tilted;
    rigid_body_step::add_rotation_step(&body, vec3f::make(1e-7f, 0.0f, 0.0f));
    CHECK(ctx, q.x == tilted.x && q.y == tilted.y && q.z == tilted.z && q.w == tilted.w);
}

void test_rigid_body_step_derive_transform(TestCtx &ctx) {
    // a box whose centre of mass sits off its origin: position is the centre of mass minus the
    // rotated offset, and the aabb follows the derived position
    shapes::Shape *shape = box::create({.half_extents = {1.0f, 0.5f, 0.5f}});
    shape->center_of_mass = {1.0f, 0.0f, 0.0f};
    RigidBody body = make_body(dof::DOF_ALL);
    body.shape = shape;
    body.quaternion = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI * 0.5f);
    body.center_of_mass_position = {3.0f, 4.0f, 5.0f};
    rigid_body_step::derive_transform(&body);
    // the offset (1, 0, 0) turned a quarter about z is (0, 1, 0)
    CHECK(ctx, vec3f::distance(vec3f::load(body.position), vec3f::make(3.0f, 3.0f, 5.0f)) < 1e-5f);
    // the 2 x 1 x 1 box turned onto y, about the position
    CHECK(ctx, vec3f::distance(vec3f::load(body.aabb.min), vec3f::make(2.5f, 2.0f, 4.5f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(vec3f::load(body.aabb.max), vec3f::make(3.5f, 4.0f, 5.5f)) < 1e-5f);
    box::destroy(shape);
}
