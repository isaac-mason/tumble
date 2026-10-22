#include "physics/world_test_scene.h"
#include "test.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/debug.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/scaled.h"

// a sphere of the caller's own, in user slot 1: its functions hand the built-in sphere's an inner shape, and count the
// supports the narrowphase asks for

namespace {

struct BallShape : shapes::Shape {
    static constexpr shapes::ShapeType TYPE = shapes::SHAPE_USER_1;
    shapes::Shape *inner;
};

struct Counters {
    uint32_t supports;
    uint32_t own_collides;
};

Counters counters;

const shapes::Shape *inner_of(const shapes::Shape *shape) { return shapes::as<BallShape>(shape).inner; }

void ball_set_support(support::Support &out, const shapes::Shape *shape, support::SupportFunctionMode mode, Vec3f scale) {
    counters.supports++;
    sphere::set_support(out, inner_of(shape), mode, scale);
}
void ball_compute_mass_properties(mass_properties::MassProperties &out, const shapes::Shape *shape) {
    sphere::compute_mass_properties(out, inner_of(shape));
}
void ball_get_surface_normal(shapes::SurfaceNormalResult &io, const shapes::Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    sphere::get_surface_normal(io, inner_of(shape), sub_shape_id);
}
void ball_get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const shapes::Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    sphere::get_supporting_face(io, direction, inner_of(shape), sub_shape_id);
}
int32_t ball_get_material(const shapes::Shape *shape, sub_shape::SubShapeId) { return shape->material_id; }
float ball_get_inner_radius(const shapes::Shape *shape) { return sphere::get_inner_radius(inner_of(shape)); }
void ball_collide_point(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings, Vec3f point,
                        const shapes::Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    sphere::collide_point_vs_sphere(collector, settings, point, inner_of(shape), sub_shape_id, transform);
}
shapes::ShapeDef ball_def(bool skips_pair_cache) {
    shapes::ShapeDef def = {};
    def.type = shapes::SHAPE_USER_1;
    def.set_support = ball_set_support;
    def.compute_mass_properties = ball_compute_mass_properties;
    def.get_surface_normal = ball_get_surface_normal;
    def.get_supporting_face = ball_get_supporting_face;
    def.get_material = ball_get_material;
    def.get_inner_radius = ball_get_inner_radius;
    def.cast_ray = convex::cast_ray_vs_convex;
    def.collide_point = ball_collide_point;
    def.skips_pair_cache = skips_pair_cache;
    return shapes::define_shape(def);
}

/** registers the ball beside the built-ins and starts the counts over */
void init_with_ball(bool skips_pair_cache = false) {
    const shapes::ShapeDef def = ball_def(skips_pair_cache);
    dispatch::init(&def, 1);
    counters = {};
}

shapes::Shape *create_ball(float radius) {
    BallShape *shape = shapes::allocate<BallShape>();
    shape->inner = sphere::create({.radius = radius});
    shape->aabb = shape->inner->aabb;
    shape->center_of_mass = shape->inner->center_of_mass;
    shape->volume = shape->inner->volume;
    shape->material_id = 7;
    return shape;
}

void destroy_ball(shapes::Shape *shape) {
    sphere::destroy(shapes::as<BallShape>(shape).inner);
    shapes::release<BallShape>(shape);
}

WorldScene make_floor_scene() {
    WorldScene scene = {};
    world_settings::WorldSettings settings = make_world_test_settings(scene.moving, scene.not_moving);
    scene.world = world::create(settings);
    world_settings::release(settings);
    add_body(scene, box::create({.half_extents = {10.0f, 0.5f, 10.0f}}), motion_type::MOTION_STATIC, {0.0f, -0.5f, 0.0f});
    return scene;
}

rigid_body::RigidBody *add_awake_body(WorldScene &scene, shapes::Shape *shape, const Float3 &position) {
    rigid_body::RigidBodySettings settings = body_settings(shape, scene.moving, motion_type::MOTION_DYNAMIC, position);
    settings.allow_sleeping = false;
    return rigid_body::create(*scene.world, settings);
}

}  // namespace

void test_user_shape_rests_through_the_generic_rows(TestCtx &ctx) {
    init_with_ball();
    shapes::Shape *shape = create_ball(0.5f);

    // on its own it collides with the box as any convex shape does, and its contacts report its material
    WorldScene scene = make_floor_scene();
    rigid_body::RigidBody *alone = add_awake_body(scene, shape, {0.0f, 2.0f, 0.0f});
    for (uint32_t i = 0; i < 120; i++) step_world(*scene.world);
    CHECK_NEAR(ctx, alone->position.y, 0.5f, 0.02f);
    bool material_reported = scene.world->events.contacts_persisted.length > 0;
    for (uint32_t i = 0; i < scene.world->events.contacts_persisted.length; i++) {
        const events::ContactEvent &event = scene.world->events.contacts_persisted[i];
        const int32_t material = event.body_id_a == alone->id ? event.material_id_a : event.material_id_b;
        material_reported = material_reported && material == 7;
    }
    CHECK(ctx, material_reported);
    release_world_scene(scene);

    // as a compound's child and under a scale it gets the compound's and the wrapper's rows
    const compound::CompoundShapeChild child = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, shape};
    shapes::Shape *compound_shape = compound::create({.children = &child, .num_children = 1});
    shapes::Shape *scaled_shape = scaled::create({.shape = shape, .scale = {2.0f, 2.0f, 2.0f}});
    scene = make_floor_scene();
    rigid_body::RigidBody *in_compound = add_awake_body(scene, compound_shape, {-3.0f, 2.0f, 0.0f});
    rigid_body::RigidBody *scaled_body = add_awake_body(scene, scaled_shape, {3.0f, 3.0f, 0.0f});
    for (uint32_t i = 0; i < 120; i++) step_world(*scene.world);
    CHECK_NEAR(ctx, in_compound->position.y, 0.5f, 0.02f);
    CHECK_NEAR(ctx, scaled_body->position.y, 1.0f, 0.02f);
    release_world_scene(scene);

    compound::destroy(compound_shape);
    scaled::destroy(scaled_shape);
    destroy_ball(shape);
}

/** steps a ball resting on the floor and counts the steps whose narrowphase asked it for no support: the cached ones */
uint32_t cached_steps_of_a_resting_ball(bool skips_pair_cache) {
    init_with_ball(skips_pair_cache);
    shapes::Shape *shape = create_ball(0.5f);
    WorldScene scene = make_floor_scene();
    add_awake_body(scene, shape, {0.0f, 0.5f, 0.0f});
    for (uint32_t i = 0; i < 60; i++) step_world(*scene.world);
    uint32_t cached = 0;
    for (uint32_t i = 0; i < 30; i++) {
        const uint32_t before = counters.supports;
        step_world(*scene.world);
        if (counters.supports == before) cached++;
    }
    release_world_scene(scene);
    destroy_ball(shape);
    return cached;
}

void test_user_shape_skipping_the_pair_cache_collides_every_step(TestCtx &ctx) {
    CHECK(ctx, cached_steps_of_a_resting_ball(false) > 0);
    CHECK(ctx, cached_steps_of_a_resting_ball(true) == 0);
}

void test_user_shape_skipping_the_cache_reaches_through_compounds_and_wrappers(TestCtx &ctx) {
    init_with_ball(true);
    shapes::Shape *shape = create_ball(0.5f);
    shapes::Shape *plain = sphere::create({.radius = 0.5f});
    const compound::CompoundShapeChild children[2] = {{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, plain},
                                                      {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, shape}};
    shapes::Shape *with_ball = compound::create({.children = children, .num_children = 2});
    shapes::Shape *without_ball = compound::create({.children = children, .num_children = 1});
    shapes::Shape *scaled_ball = scaled::create({.shape = shape, .scale = {2.0f, 2.0f, 2.0f}});
    CHECK(ctx, !shapes::caches_pairs(shape));
    CHECK(ctx, !shapes::caches_pairs(with_ball) && shapes::caches_pairs(without_ball));
    CHECK(ctx, !shapes::caches_pairs(scaled_ball) && shapes::caches_pairs(plain));
    scaled::destroy(scaled_ball);
    compound::destroy(with_ball);
    compound::destroy(without_ball);
    sphere::destroy(plain);
    destroy_ball(shape);
}

void counting_collide(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                      const shapes::Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                      const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    counters.own_collides++;
    convex::collide_convex_vs_convex(collector, settings, shape_a, sub_shape_id_a, transform_a, shape_b, sub_shape_id_b, transform_b);
}

void test_user_shape_own_pair_row_replaces_the_generic_one(TestCtx &ctx) {
    init_with_ball(true);
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_USER_1, shapes::SHAPE_BOX) == convex::collide_convex_vs_convex);
    shapes::set_collide_shape_fn(shapes::SHAPE_USER_1, shapes::SHAPE_BOX, counting_collide);
    shapes::set_collide_shape_fn(shapes::SHAPE_BOX, shapes::SHAPE_USER_1, collide_shape::reversed_collide_shape_vs_shape<counting_collide>);

    shapes::Shape *shape = create_ball(0.5f);
    WorldScene scene = make_floor_scene();
    rigid_body::RigidBody *body = add_awake_body(scene, shape, {0.0f, 0.5f, 0.0f});
    for (uint32_t i = 0; i < 10; i++) step_world(*scene.world);
    CHECK(ctx, counters.own_collides == 10);
    CHECK_NEAR(ctx, body->position.y, 0.5f, 0.02f);
    release_world_scene(scene);
    destroy_ball(shape);

    // a second init starts the tables over: the generic row is back
    init_with_ball();
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_USER_1, shapes::SHAPE_BOX) == convex::collide_convex_vs_convex);
}

void draw_cross(debug::DebugBuffers &out, const shapes::Shape *, const Transformf &transform, Vec3f color) {
    const Vec3f at = vec3f::load(transform.position);
    debug::line(out, vec3f::subtract(at, vec3f::make(1.0f, 0.0f, 0.0f)), vec3f::add(at, vec3f::make(1.0f, 0.0f, 0.0f)), color);
    debug::line(out, vec3f::subtract(at, vec3f::make(0.0f, 1.0f, 0.0f)), vec3f::add(at, vec3f::make(0.0f, 1.0f, 0.0f)), color);
}

void test_user_shape_draws_its_bounds_or_its_own_lines(TestCtx &ctx) {
    init_with_ball();
    shapes::Shape *shape = create_ball(0.5f);
    debug::DebugBuffers *buffers = debug::create();

    // no draw: the bounds' twelve edges, at the pose
    debug::shape(*buffers, shape, vec3f::make(0.0f, 2.0f, 0.0f), quatf::identity(), {});
    CHECK(ctx, buffers->line_count == 12);
    float lowest = scalar::INF;
    for (uint32_t i = 0; i < buffers->line_count * 2; i++) lowest = scalar::min(lowest, buffers->lines[i * debug::VERTEX_STRIDE + 1]);
    CHECK_NEAR(ctx, lowest, 1.5f, 1e-6f);

    // a def's draw replaces them
    shapes::ShapeDef def = ball_def(false);
    def.draw = draw_cross;
    dispatch::init(&def, 1);
    debug::clear(*buffers);
    debug::shape(*buffers, shape, vec3f::make(0.0f, 2.0f, 0.0f), quatf::identity(), {});
    CHECK(ctx, buffers->line_count == 2);
    CHECK_NEAR(ctx, buffers->lines[1], 2.0f, 1e-6f);

    debug::release(buffers);
    destroy_ball(shape);
}
