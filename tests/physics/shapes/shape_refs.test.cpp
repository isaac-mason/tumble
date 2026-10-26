#include "physics/world_test_scene.h"
#include "test.h"
#include "tumble/physics/character/kcc.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/offset_center_of_mass.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/util/alloc.h"
#include "tumble/physics/shapes/capsule.h"


void test_shape_refs_bodies(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    // the caller's Ref and each body's count
    Ref<Shape> crate = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    CHECK(ctx, crate->ref_count == 1);
    RigidBody *a = add_body(scene, crate, MotionType::DYNAMIC, {0.0f, 1.0f, 0.0f});
    RigidBody *b = add_body(scene, crate, MotionType::DYNAMIC, {3.0f, 1.0f, 0.0f});
    CHECK(ctx, crate->ref_count == 3);

    // set_shape swaps what is held; the same shape again changes nothing
    Ref<Shape> ball = sphere::create({.radius = 0.5f});
    rigid_body::set_shape(*scene.world, a, ball, false);
    CHECK(ctx, crate->ref_count == 2 && ball->ref_count == 2);
    rigid_body::set_shape(*scene.world, a, ball, false);
    CHECK(ctx, ball->ref_count == 2);

    // a removed body lets go; the caller letting go leaves a's ball held by a alone
    rigid_body::remove(*scene.world, b);
    CHECK(ctx, crate->ref_count == 1);
    ball = nullptr;
    CHECK(ctx, a->shape->ref_count == 1);

    // a shape no one but a body ever held goes with that body
    RigidBody *loner = add_body(scene, capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f}), MotionType::DYNAMIC, {6.0f, 1.0f, 0.0f});
    CHECK(ctx, loner->shape->ref_count == 1);
    const size_t before_removing = util::allocated_bytes();
    rigid_body::remove(*scene.world, loner);
    CHECK(ctx, util::allocated_bytes() < before_removing);

    release_world_scene(scene);
    CHECK(ctx, crate->ref_count == 1);
}

void test_shape_refs_free_everything_with_the_world(TestCtx &ctx) {
    const size_t bytes = util::allocated_bytes();
    const size_t blocks = util::allocated_blocks();
    WorldScene scene = make_world_scene(ctx);
    add_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), MotionType::DYNAMIC, {0.0f, 1.0f, 0.0f});
    RigidBody *removed = add_body(scene, sphere::create({.radius = 0.5f}), MotionType::DYNAMIC, {3.0f, 1.0f, 0.0f});
    rigid_body::remove(*scene.world, removed);
    release_world_scene(scene);
    CHECK(ctx, util::allocated_bytes() == bytes && util::allocated_blocks() == blocks);
}

void test_shape_refs_shapes_over_shapes(TestCtx &ctx) {
    dispatch::init();
    const size_t bytes = util::allocated_bytes();
    {
        Ref<Shape> crate = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
        Ref<Shape> raised = transformed::create({.shape = crate, .position = {0.0f, 1.0f, 0.0f}});
        Ref<Shape> big = scaled::create({.shape = crate, .scale = {2.0f, 2.0f, 2.0f}});
        Ref<Shape> ballasted = offset_center_of_mass::create({.shape = crate, .offset = {0.0f, -0.2f, 0.0f}});
        const CompoundShapeSettings::Child children[2] = {{{-1.0f, 0.0f, 0.0f}, {0, 0, 0, 1}, crate}, {{1.0f, 0.0f, 0.0f}, {0, 0, 0, 1}, raised}};
        Ref<Shape> pair = compound::create({.children = children, .num_children = 2});
        Ref<Shape> fence = static_compound::create({.children = children, .num_children = 2});

        // every shape over it holds it: the wrappers once, each compound once per child, and the caller
        CHECK(ctx, crate->ref_count == 6 && raised->ref_count == 3);

        // let go of from the outside in: each frees what it alone held
        fence = nullptr;
        pair = nullptr;
        CHECK(ctx, crate->ref_count == 4 && raised->ref_count == 1);
        ballasted = nullptr;
        big = nullptr;
        raised = nullptr;
        CHECK(ctx, crate->ref_count == 1);
    }
    CHECK(ctx, util::allocated_bytes() == bytes);
}

void test_shape_refs_characters(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    Ref<Shape> standing = box::create({.half_extents = {0.3f, 0.9f, 0.3f}});
    Ref<Shape> crouching = box::create({.half_extents = {0.3f, 0.6f, 0.3f}});
    Ref<Shape> presence = box::create({.half_extents = {0.25f, 0.85f, 0.25f}});
    KCCSettings settings;
    settings.shape = standing;
    settings.inner_rigid_body_shape = presence;
    settings.inner_rigid_body_object_layer = scene.moving;
    Ref<KCC> character = kcc::create(settings, vec3f::make(0.0f, 2.0f, 0.0f), quatf::identity());

    // the character holds its shape and its inner body's, and its inner body holds that too once added
    CHECK(ctx, standing->ref_count == 2 && presence->ref_count == 2);
    kcc::add(*scene.world, *character);
    CHECK(ctx, presence->ref_count == 3);

    // a set_shape that goes through swaps what the character and its inner body hold
    Filter filter = filter::create(scene.world->settings.layers);
    CHECK(ctx, kcc::set_shape(*scene.world, *character, crouching, filter, nullptr, scalar::FLOAT_MAX));
    CHECK(ctx, standing->ref_count == 1 && crouching->ref_count == 3 && presence->ref_count == 2);

    kcc::remove(*scene.world, *character);
    character = nullptr;
    CHECK(ctx, crouching->ref_count == 1 && presence->ref_count == 1);
    release_world_scene(scene);
}

void test_shape_refs_character_freed_in_a_world(TestCtx &ctx) {
    dispatch::init();
    WorldScene scene = make_world_scene(ctx);
    Ref<Shape> capsule = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
    KCCSettings settings;
    settings.shape = capsule;
    settings.inner_rigid_body_shape = capsule;
    settings.inner_rigid_body_object_layer = scene.moving;
    Ref<KCC> character = kcc::create(settings, vec3f::make(0.0f, 0.0f, 0.0f), quatf::identity());
    kcc::add(*scene.world, *character);
    const BodyId inner = character->inner_rigid_body_id;
    // this Ref, the character's shape and inner body shape, and the inner body's
    CHECK(ctx, rigid_body::get(*scene.world, inner) != nullptr && capsule->ref_count == 4);

    // its last Ref takes the inner body out of the world it was added to, as jolt's CharacterVirtual does
    character = nullptr;
    CHECK(ctx, rigid_body::get(*scene.world, inner) == nullptr && capsule->ref_count == 1);
    release_world_scene(scene);
}

namespace {

/** where the point constraint's anchor on body b is, in world space */
Vec3f anchor_b(World &world, ConstraintId id, const RigidBody &b) {
    const PointConstraint &joint = *point_constraint::get(world, id);
    return vec3f::add(vec3f::load(b.center_of_mass_position), vec3f::transform_quat(joint.local_space_position2, quatf::load(b.quaternion)));
}

}  // namespace

void test_shape_refs_set_shape_refreshes_a_shape_changed_in_place(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    Ref<Shape> crate = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    const CompoundShapeSettings::Child children[2] = {{{0.0f, 0.0f, 0.0f}, {0, 0, 0, 1}, crate}, {{0.0f, 1.0f, 0.0f}, {0, 0, 0, 1}, crate}};
    Ref<Shape> stack = compound::create({.children = children, .num_children = 2});
    RigidBody *anchor = add_body(scene, crate, MotionType::STATIC, {0.0f, 5.0f, 0.0f});
    RigidBody *body = add_body(scene, stack, MotionType::DYNAMIC, {0.0f, 3.0f, 0.0f});
    rigid_body::sleep(*scene.world, body);
    const ConstraintId id =
        point_constraint::create(*scene.world, anchor, body, {.point_a = {0.0f, 4.5f, 0.0f}, .point_b = {0.0f, 4.5f, 0.0f}});
    const Vec3f before = anchor_b(*scene.world, id, *body);

    // the stack's top box moves up a metre in place: its centre of mass rises half a metre
    CompoundShape::Child &top = shapes::as<CompoundShape>(stack).data->children[1];
    top.position = {0.0f, 2.0f, 0.0f};
    compound::update(stack);
    CHECK_NEAR(ctx, stack->center_of_mass.y, 1.0f, 1e-5f);

    // taking its own shape again, the body follows the change: the joint's anchor stays where it was on it, and it wakes
    rigid_body::set_shape(*scene.world, body, stack, true);
    CHECK_NEAR(ctx, body->center_of_mass_position.y, 4.0f, 1e-5f);
    CHECK(ctx, vec3f::distance(anchor_b(*scene.world, id, *body), before) < 1e-5f);
    CHECK(ctx, rigid_body::is_active(*body) && stack->ref_count == 2);

    release_world_scene(scene);
    stack = nullptr;
    crate = nullptr;
}
