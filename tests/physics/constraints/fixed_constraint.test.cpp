#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/constraints/constraint_part/rotation_euler_constraint_part.h"
#include "physics/constraints/constraint_test_scene.h"
#include "tumble/physics/constraints/fixed_constraint.h"
#include "test.h"
#include "test_math.h"


static const Float3 GRAVITY = {0.0f, -9.81f, 0.0f};

/** b's center of mass in a's frame */
static Vec3f offset_in_a(const RigidBody *a, const RigidBody *b) {
    const Vec3f offset = vec3f::subtract(vec3f::load(b->center_of_mass_position), vec3f::load(a->center_of_mass_position));
    return vec3f::transform_quat(offset, quatf::conjugate(quatf::load(a->quaternion)));
}

/** the angle of the rotation q2 * inv_initial_orientation * q1^-1, the fixed constraint's rotation error */
static float rotation_error(const FixedConstraint &constraint, const RigidBody *a, const RigidBody *b) {
    const Quatf inv_a = quatf::conjugate(quatf::load(a->quaternion));
    const Quatf diff = quatf::multiply(quatf::multiply(quatf::load(b->quaternion), constraint.inv_initial_orientation), inv_a);
    const float w = scalar::min(1.0f, scalar::abs(quatf::w(diff)));
    return 2.0f * scalar::acos(w);
}

void test_fixed_constraint_create_world_space(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, MotionType::DYNAMIC, {0.0f, 1.0f, 0.0f}, stored_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), 0.7f));
    RigidBody *b = add_body(scene, MotionType::DYNAMIC, {2.0f, 1.0f, 0.0f}, stored_axis_angle(vec3f::make(1.0f, 1.0f, 0.0f), -1.1f));
    const ConstraintId id =
        fixed_constraint::create(*scene.world, a, b, {.point_a = {1.0f, 1.0f, 0.0f}, .point_b = {1.0f, 1.0f, 0.0f}});
    FixedConstraint *constraint = fixed_constraint::get(*scene.world, id);
    CHECK(ctx, constraint != nullptr && constraint_id::type(id) == ConstraintType::FIXED);

    // both attachments are the same world point, and the current relative pose is the rest pose
    CHECK(ctx, vec3f::distance(world_point(a, constraint->local_space_position1), vec3f::make(1.0f, 1.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(world_point(b, constraint->local_space_position2), vec3f::make(1.0f, 1.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, rotation_error(*constraint, a, b) < 1e-3f);
    CHECK(ctx, a->constraint_ids.length == 1 && b->constraint_ids.length == 1);

    // the rest pose survives a rotation of the pair as a whole
    const Float4 turn = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.9f);
    rotate_body(a, quatf::load(turn));
    rotate_body(b, quatf::load(turn));
    CHECK(ctx, rotation_error(*constraint, a, b) < 1e-3f);
    release_constraint_scene(scene);
}

void test_fixed_constraint_create_local_space(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, MotionType::DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, MotionType::DYNAMIC, {2.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), 0.5f));

    // local space: points as given, and the inverse initial orientation straight from the axes
    const Float3 x2 = {0.0f, 2.0f, 0.0f};
    const Float3 y2 = {-3.0f, 0.0f, 0.0f};
    const ConstraintId id = fixed_constraint::create(
        *scene.world, a, b,
        {.point_a = {1.0f, 0.0f, 0.0f}, .point_b = {-1.0f, 0.0f, 0.0f}, .axis_x_b = x2, .axis_y_b = y2, .space = ConstraintSpace::LOCAL});
    const FixedConstraint *constraint = fixed_constraint::get(*scene.world, id);
    CHECK(ctx, vec3f::exact_equals(constraint->local_space_position1, vec3f::make(1.0f, 0.0f, 0.0f)));
    CHECK(ctx, vec3f::exact_equals(constraint->local_space_position2, vec3f::make(-1.0f, 0.0f, 0.0f)));
    const Quatf expected = rotation_euler_constraint_part::get_inv_initial_orientation_xy(
        vec3f::make(1.0f, 0.0f, 0.0f), vec3f::make(0.0f, 1.0f, 0.0f), vec3f::make(0.0f, 1.0f, 0.0f), vec3f::make(-1.0f, 0.0f, 0.0f));
    CHECK(ctx, same_rotation(constraint->inv_initial_orientation, expected));
    // body 2's x axis along body 1's y: a quarter turn about z
    CHECK_NEAR(ctx, scalar::abs(quatf::dot(constraint->inv_initial_orientation, axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI / 2.0f))), 1.0f,
               1e-5f);

    // default axes in local space: no relative rotation
    const ConstraintId plain = fixed_constraint::create(*scene.world, a, b, {.space = ConstraintSpace::LOCAL});
    CHECK(ctx, same_rotation(fixed_constraint::get(*scene.world, plain)->inv_initial_orientation, quatf::identity()));
    release_constraint_scene(scene);
}

void test_fixed_constraint_auto_detect_point(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, MotionType::DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, MotionType::DYNAMIC, {2.0f, 0.0f, 0.0f});
    RigidBody *ground = add_body(scene, MotionType::STATIC, {0.0f, -5.0f, 0.0f});

    // equal masses: the midpoint
    const ConstraintId id =
        fixed_constraint::create(*scene.world, a, b, {.point_a = {9.0f, 9.0f, 9.0f}, .auto_detect_point = true});
    const FixedConstraint *constraint = fixed_constraint::get(*scene.world, id);
    CHECK(ctx, vec3f::distance(constraint->local_space_position1, vec3f::make(1.0f, 0.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::distance(constraint->local_space_position2, vec3f::make(-1.0f, 0.0f, 0.0f)) < 1e-5f);

    // a static body has no inverse mass: the anchor sits on the dynamic body
    const ConstraintId to_ground =
        fixed_constraint::create(*scene.world, ground, b, {.auto_detect_point = true});
    const FixedConstraint *grounded = fixed_constraint::get(*scene.world, to_ground);
    CHECK(ctx, vec3f::distance(grounded->local_space_position1, vec3f::make(2.0f, 5.0f, 0.0f)) < 1e-5f);
    CHECK(ctx, vec3f::length(grounded->local_space_position2) < 1e-5f);
    release_constraint_scene(scene);
}

void test_fixed_constraint_auto_detect_point_static(TestCtx &ctx) {
    ConstraintScene scene = make_constraint_scene();
    RigidBody *static_a = add_body(scene, MotionType::STATIC, {0.0f, 0.0f, 0.0f});
    RigidBody *static_b = add_body(scene, MotionType::STATIC, {4.0f, 0.0f, 0.0f});

    // body a is static, so the anchor is body b's center of mass even though neither has an inverse mass
    const FixedConstraint *both_static = fixed_constraint::get(
        *scene.world, fixed_constraint::create(*scene.world, static_a, static_b, {.auto_detect_point = true}));
    CHECK(ctx, vec3f::exact_equals(both_static->local_space_position1, vec3f::make(4.0f, 0.0f, 0.0f)));
    CHECK(ctx, vec3f::length(both_static->local_space_position2) == 0.0f);

    // a body made static keeps the inverse mass it had; being static, it still anchors on the other body
    RigidBody *made_static = add_body(scene, MotionType::DYNAMIC, {0.0f, 2.0f, 0.0f});
    RigidBody *dynamic = add_body(scene, MotionType::DYNAMIC, {4.0f, 2.0f, 0.0f});
    rigid_body::set_motion_type(*scene.world, made_static, MotionType::STATIC, false);
    CHECK(ctx, made_static->motion_properties.inv_mass > 0.0f);
    const FixedConstraint *anchored = fixed_constraint::get(
        *scene.world, fixed_constraint::create(*scene.world, made_static, dynamic, {.auto_detect_point = true}));
    CHECK(ctx, vec3f::exact_equals(anchored->local_space_position1, vec3f::make(4.0f, 0.0f, 0.0f)));
    CHECK(ctx, vec3f::length(anchored->local_space_position2) == 0.0f);
    release_constraint_scene(scene);
}

void test_fixed_constraint_holds_pose(TestCtx &ctx) {
    // a body welded to a static one, spun, thrown and pulled by gravity, stays where it was put
    ConstraintScene scene = make_constraint_scene();
    RigidBody *wall = add_body(scene, MotionType::STATIC, {0.0f, 5.0f, 0.0f});
    const Float4 rest_rotation = stored_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), 0.3f);
    RigidBody *body = add_body(scene, MotionType::DYNAMIC, {1.5f, 5.0f, 0.0f}, rest_rotation);
    const Float3 rest_position = body->center_of_mass_position;
    const ConstraintId id =
        fixed_constraint::create(*scene.world, wall, body, {.point_a = {0.5f, 5.0f, 0.0f}, .point_b = {0.5f, 5.0f, 0.0f}});
    scene.ids.push(id);
    body->motion_properties.linear_velocity = {0.0f, 3.0f, 1.0f};
    body->motion_properties.angular_velocity = {1.0f, 2.0f, 0.5f};

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 120; i++) step_constraint_scene(scene, dt, vec3f::load(GRAVITY));

    const FixedConstraint *constraint = fixed_constraint::get(*scene.world, id);
    CHECK(ctx, vec3f::distance(vec3f::load(body->center_of_mass_position), vec3f::load(rest_position)) < 1e-2f);
    CHECK(ctx, scalar::abs(quatf::dot(quatf::load(body->quaternion), quatf::load(rest_rotation))) > scalar::cos(0.5f * 1e-2f));
    CHECK(ctx, rotation_error(*constraint, wall, body) < 1e-2f);
    CHECK(ctx, vec3f::length(vec3f::load(body->motion_properties.angular_velocity)) < 1e-2f);

    // the impulses that hold it: the translation one carries its weight each step
    const Vec3f lambda_position = fixed_constraint::get_total_lambda_position(*constraint);
    const Vec3f lambda_rotation = fixed_constraint::get_total_lambda_rotation(*constraint);
    const float weight_impulse = 9.81f * dt / body->motion_properties.inv_mass;
    CHECK_NEAR(ctx, scalar::abs(vec3f::y(lambda_position)), weight_impulse, weight_impulse * 0.05f);
    CHECK(ctx, vec3f::exact_equals(lambda_rotation, constraint->rotation_constraint_part.total_lambda));
    release_constraint_scene(scene);
}

void test_fixed_constraint_joins_two_dynamic_bodies(TestCtx &ctx) {
    // two welded bodies thrown with spin move as one: the relative pose is kept
    ConstraintScene scene = make_constraint_scene();
    RigidBody *a = add_body(scene, MotionType::DYNAMIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, MotionType::DYNAMIC, {1.0f, 0.5f, 0.0f}, stored_axis_angle(vec3f::make(1.0f, 0.0f, 1.0f), 0.8f));
    const ConstraintId id = fixed_constraint::create(*scene.world, a, b, {.auto_detect_point = true});
    scene.ids.push(id);
    a->motion_properties.angular_velocity = {0.0f, 4.0f, 0.0f};
    b->motion_properties.linear_velocity = {-2.0f, 0.0f, 1.0f};

    // b's center of mass in a's frame at rest
    const Vec3f rest_offset = offset_in_a(a, b);

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 90; i++) step_constraint_scene(scene, dt, vec3f::zero());

    const FixedConstraint *constraint = fixed_constraint::get(*scene.world, id);
    CHECK(ctx, vec3f::distance(offset_in_a(a, b), rest_offset) < 1e-2f);
    CHECK(ctx, rotation_error(*constraint, a, b) < 1e-2f);
    // the pair has turned as a whole
    CHECK(ctx, scalar::abs(quatf::dot(quatf::load(a->quaternion), quatf::identity())) < 0.99f);
    release_constraint_scene(scene);
}
