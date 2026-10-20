#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/estimate_collision_response.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/world.h"
#include "test.h"
#include "test_math.h"

using estimate_collision_response::CollisionEstimationResult;
using rigid_body::RigidBody;

struct Scene {
    world::World *world;
    shapes::Shape *ball_shape;
    shapes::Shape *ground_shape;
    RigidBody *ball;
    RigidBody *ground;
    manifold::ContactManifold manifold;
    float mass;
};

static constexpr float RADIUS = 0.5f;

/** a dynamic ball of radius 0.5 resting its lowest point on a static box's top face at y = 0, the contact at the origin */
static void make_scene(Scene &scene, const Float3 &velocity) {
    dispatch::init();
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t object_layer = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, object_layer, object_layer);
    scene.world = world::create(settings);
    world_settings::release(settings);

    scene.ball_shape = sphere::create({.radius = RADIUS, .density = 1000.0f});
    scene.ground_shape = box::create({.half_extents = {5, 1, 5}});
    scene.ball = rigid_body::create(*scene.world,
                                    {.shape = scene.ball_shape, .object_layer = object_layer, .motion_type = motion_type::MOTION_DYNAMIC, .position = {0, RADIUS, 0}});
    scene.ground = rigid_body::create(*scene.world,
                                      {.shape = scene.ground_shape, .object_layer = object_layer, .motion_type = motion_type::MOTION_STATIC, .position = {0, -1, 0}});
    scene.ball->motion_properties.linear_velocity = velocity;
    scene.mass = 4.0f / 3.0f * scalar::PI * RADIUS * RADIUS * RADIUS * 1000.0f;

    // one contact at the origin, the normal from the ball (a) into the ground (b)
    scene.manifold = manifold::create();
    scene.manifold.num_contact_points = 1;
    scene.manifold.world_space_normal = vec3f::make(0, -1, 0);
}

static void release_scene(Scene &scene) {
    world::destroy(scene.world);
    sphere::destroy(scene.ball_shape);
    box::destroy(scene.ground_shape);
}

void test_estimate_collision_response_sphere_drop(TestCtx &ctx) {
    Scene scene;
    const float v0 = 4.0f;
    make_scene(scene, {0, -v0, 0});
    CHECK_NEAR(ctx, 1.0f / scene.ball->motion_properties.inv_mass, scene.mass, 0.05f);

    // restitution 0.5 above the 1 m/s threshold: the normal impulse is m v0 (1 + e) and the ball leaves at e v0
    CollisionEstimationResult result = estimate_collision_response::create_result();
    estimate_collision_response::estimate_collision_response(result, *scene.ball, *scene.ground, scene.manifold, 0.0f, 0.5f, 1.0f, 10);
    CHECK(ctx, result.num_impulses == 1);
    CHECK_NEAR(ctx, result.contact_impulse[0], scene.mass * v0 * 1.5f, 0.05f);
    CHECK_NEAR(ctx, vec3f::y(result.linear_velocity_1), 0.5f * v0, 1e-5f);
    CHECK(ctx, vec3f::x(result.linear_velocity_1) == 0.0f && vec3f::z(result.linear_velocity_1) == 0.0f);
    CHECK(ctx, vec3f::length(result.angular_velocity_1) == 0.0f);
    // the static ground is not moved
    CHECK(ctx, vec3f::length(result.linear_velocity_2) == 0.0f && vec3f::length(result.angular_velocity_2) == 0.0f);
    // no friction asked for
    CHECK(ctx, vec3f::length(result.friction_impulse) == 0.0f);

    // inelastic: the impulse stops the ball dead
    estimate_collision_response::estimate_collision_response(result, *scene.ball, *scene.ground, scene.manifold, 0.0f, 0.0f, 1.0f, 10);
    CHECK_NEAR(ctx, result.contact_impulse[0], scene.mass * v0, 0.05f);
    CHECK_NEAR(ctx, vec3f::y(result.linear_velocity_1), 0.0f, 1e-5f);

    // slower than the restitution threshold: no bounce
    scene.ball->motion_properties.linear_velocity = {0, -0.5f, 0};
    estimate_collision_response::estimate_collision_response(result, *scene.ball, *scene.ground, scene.manifold, 0.0f, 0.5f, 1.0f, 10);
    CHECK_NEAR(ctx, result.contact_impulse[0], scene.mass * 0.5f, 0.01f);
    CHECK_NEAR(ctx, vec3f::y(result.linear_velocity_1), 0.0f, 1e-5f);

    // moving apart: no impulse at all
    scene.ball->motion_properties.linear_velocity = {0, 2, 0};
    estimate_collision_response::estimate_collision_response(result, *scene.ball, *scene.ground, scene.manifold, 0.5f, 0.5f, 1.0f, 10);
    CHECK(ctx, result.contact_impulse[0] == 0.0f);
    CHECK_NEAR(ctx, vec3f::y(result.linear_velocity_1), 2.0f, 1e-6f);

    // the bodies themselves are untouched
    CHECK(ctx, scene.ball->motion_properties.linear_velocity.y == 2.0f);
    release_scene(scene);
}

void test_estimate_collision_response_sliding_sphere_rolls(TestCtx &ctx) {
    Scene scene;
    const float v0 = 4.0f;
    const float slide = 1.0f;
    make_scene(scene, {slide, -v0, 0});

    // enough friction to grip: the contact point stops slipping, so a solid sphere keeps 5/7 of its speed and spins at
    // v / r. the friction impulse takes m slide 2/7, within mu times the normal impulse
    CollisionEstimationResult result = estimate_collision_response::create_result();
    estimate_collision_response::estimate_collision_response(result, *scene.ball, *scene.ground, scene.manifold, 0.5f, 0.0f, 1.0f, 10);
    CHECK_NEAR(ctx, result.contact_impulse[0], scene.mass * v0, 0.05f);
    CHECK_NEAR(ctx, vec3f::x(result.linear_velocity_1), slide * 5.0f / 7.0f, 1e-4f);
    CHECK_NEAR(ctx, vec3f::y(result.linear_velocity_1), 0.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::z(result.angular_velocity_1), -slide * 5.0f / 7.0f / RADIUS, 1e-3f);
    // the friction impulse lies in the contact plane
    CHECK_NEAR(ctx, vec3f::dot(result.friction_impulse, scene.manifold.world_space_normal), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::length(result.friction_impulse), scene.mass * slide * 2.0f / 7.0f, 0.05f);
    // the single point has no moment arm about the normal
    CHECK(ctx, result.angular_friction_impulse == 0.0f);
    CHECK(ctx, vec3f::x(result.friction_point) == 0.0f && vec3f::y(result.friction_point) == 0.0f && vec3f::z(result.friction_point) == 0.0f);

    // too little friction to grip: the friction impulse saturates at mu times the normal impulse
    scene.ball->motion_properties.linear_velocity = {20, -v0, 0};
    estimate_collision_response::estimate_collision_response(result, *scene.ball, *scene.ground, scene.manifold, 0.1f, 0.0f, 1.0f, 10);
    const float saturated = vec3f::length(result.friction_impulse);
    CHECK_NEAR(ctx, saturated, 0.1f * result.contact_impulse[0], 0.01f);
    CHECK_NEAR(ctx, vec3f::x(result.linear_velocity_1), 20.0f - 0.1f * v0, 1e-3f);

    release_scene(scene);
}
