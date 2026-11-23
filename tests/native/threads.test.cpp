#include <thread>

#include "physics/world_test_scene.h"
#include "test.h"
#include "test_math.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/triangle_mesh.h"

namespace {

constexpr uint32_t THREAD_COUNT = 4;
constexpr uint32_t QUERIES_PER_ROUND = 64;
constexpr uint32_t ROUNDS = 6;
constexpr uint32_t STEPS = 80;

struct Hash {
    uint32_t value = 2166136261u;

    void word(uint32_t w) {
        for (int i = 0; i < 4; i++) {
            value ^= (w >> (i * 8)) & 0xffu;
            value *= 16777619u;
        }
    }
    void real(float f) { word(f != f ? 0x7fc00000u : __builtin_bit_cast(uint32_t, f)); }
    void vec3(Vec3f v) {
        real(vec3f::x(v));
        real(vec3f::y(v));
        real(vec3f::z(v));
    }
    void quat(Quatf q) {
        real(quatf::x(q));
        real(quatf::y(q));
        real(quatf::z(q));
        real(quatf::w(q));
    }
};

constexpr uint32_t GROUND_CELLS = 6;
constexpr uint32_t GROUND_SIDE = GROUND_CELLS + 1;

float ground_positions[GROUND_SIDE * GROUND_SIDE * 3];
uint32_t ground_indices[GROUND_CELLS * GROUND_CELLS * 6];

void fill_ground_arrays() {
    for (uint32_t z = 0; z < GROUND_SIDE; z++) {
        for (uint32_t x = 0; x < GROUND_SIDE; x++) {
            float *p = ground_positions + (z * GROUND_SIDE + x) * 3;
            p[0] = 2.0f * (float)x - (float)GROUND_CELLS;
            p[1] = 0.15f * scalar::sin(1.3f * (float)x) * scalar::cos(0.9f * (float)z);
            p[2] = 2.0f * (float)z - (float)GROUND_CELLS;
        }
    }
    uint32_t *index = ground_indices;
    for (uint32_t z = 0; z < GROUND_CELLS; z++) {
        for (uint32_t x = 0; x < GROUND_CELLS; x++) {
            const uint32_t corner = z * GROUND_SIDE + x;
            const uint32_t quad[6] = {corner, corner + GROUND_SIDE, corner + 1, corner + 1, corner + GROUND_SIDE, corner + GROUND_SIDE + 1};
            for (uint32_t k = 0; k < 6; k++) *index++ = quad[k];
        }
    }
}

Shape *make_ground() {
    TriangleMeshShapeSettings settings;
    settings.positions = ground_positions;
    settings.num_positions = GROUND_SIDE * GROUND_SIDE;
    settings.indices = ground_indices;
    settings.num_indices = GROUND_CELLS * GROUND_CELLS * 6;
    return triangle_mesh::create(settings);
}

WorldScene make_pile(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_body(scene, make_ground(), MotionType::STATIC, {0.0f, 0.0f, 0.0f});

    Vec3f hull_points[8];
    for (uint32_t i = 0; i < 8; i++) {
        hull_points[i] = vec3f::make((i & 1) != 0 ? 0.4f : -0.4f, (i & 2) != 0 ? 0.3f : -0.3f, (i & 4) != 0 ? 0.35f : -0.35f);
    }
    const ConvexHullShapeSettings hull = {
        .positions = hull_points, .num_positions = 8, .bake_support_adjacency = ConvexHullShape::BakeSupportAdjacency::NEVER};

    for (uint32_t i = 0; i < 14; i++) {
        const Float3 position = {0.09f * (float)(i % 3) - 0.1f, 1.0f + 1.1f * (float)i, 0.05f * (float)(i % 4) - 0.08f};
        Shape *shape;
        switch (i % 4) {
            case 0: shape = box::create({.half_extents = vec3f::make(0.5f, 0.4f, 0.3f)}); break;
            case 1: shape = sphere::create({.radius = 0.45f}); break;
            case 2: shape = capsule::create({.half_height_of_cylinder = 0.3f, .radius = 0.3f}); break;
            default: shape = convex_hull::create(hull); break;
        }
        RigidBodySettings settings = body_settings(shape, scene.moving, MotionType::DYNAMIC, position);
        settings.quaternion = axis_angle(vec3f::make(0.3f, 1.0f, 0.2f * (float)i), 0.37f * (float)i);
        rigid_body::create(*scene.world, settings);
    }
    return scene;
}

uint32_t hash_bodies(const World &world) {
    Hash hash;
    for (uint32_t i = 0; i < world.bodies.pool.length; i++) {
        const RigidBody *body = world.bodies.pool[i];
        if (body == nullptr) continue;
        hash.vec3(body->position);
        hash.quat(body->quaternion);
        hash.vec3(body->motion_properties.linear_velocity);
        hash.vec3(body->motion_properties.angular_velocity);
    }
    return hash.value;
}

uint32_t step_and_hash(World *world) {
    Hash hash;
    for (uint32_t step = 0; step < STEPS; step++) {
        step_world(*world);
        hash.word(hash_bodies(*world));
    }
    return hash.value;
}

struct QueryShapes {
    Shape *sphere_shape;
    Shape *box_shape;
    Shape *capsule_shape;
};

uint32_t run_queries(World &world, const QueryShapes &shapes, uint32_t seed) {
    Hash hash;
    const Filter query_filter = filter::create(world.settings.layers);
    for (uint32_t round = 0; round < ROUNDS; round++) {
        for (uint32_t q = 0; q < QUERIES_PER_ROUND; q++) {
            const float a = 0.37f * (float)(q + seed);
            const Vec3f origin = vec3f::make(6.0f * scalar::sin(a), 2.0f + 6.0f * scalar::cos(1.7f * a), 6.0f * scalar::cos(a));
            const Vec3f target = vec3f::make(1.5f * scalar::cos(2.1f * a), 0.5f + 4.0f * scalar::sin(0.3f * a), 1.5f * scalar::sin(a));
            const Vec3f direction = vec3f::subtract(target, origin);
            const Transform at = pose(origin, axis_angle(vec3f::make(1.0f, 0.4f, 0.2f), a));

            CastRayCollector ray_collector = cast_ray::create_collector(CollectorKind::ALL);
            query::cast_ray(world, ray_collector, cast_ray::default_settings(), Rayf{origin, vec3f::normalize(direction), 20.0f}, query_filter);
            hash.word(ray_collector.hits.length);
            for (uint32_t i = 0; i < ray_collector.hits.length; i++) {
                hash.real(ray_collector.hits[i].fraction);
                hash.word(ray_collector.hits[i].body_id_b);
            }

            CastShapeCollector cast_collector = cast_shape::create_collector(CollectorKind::ALL);
            const Shape *cast_shape_choice = q % 3 == 0 ? shapes.sphere_shape : q % 3 == 1 ? shapes.box_shape : shapes.capsule_shape;
            query::cast_shape(world, cast_collector, cast_shape::default_settings(), cast_shape_choice, at, direction, query_filter);
            hash.word(cast_collector.hits.length);
            for (uint32_t i = 0; i < cast_collector.hits.length; i++) {
                hash.real(cast_collector.hits[i].fraction);
                hash.vec3(cast_collector.hits[i].point_a);
                hash.vec3(cast_collector.hits[i].point_b);
            }

            const Transform inside = pose(target, axis_angle(vec3f::make(0.2f, 1.0f, 0.4f), a));
            CollideShapeCollector collide_collector = collide_shape::create_collector(CollectorKind::ALL);
            const Shape *collide_shape_choice = q % 3 == 0 ? shapes.capsule_shape : q % 3 == 1 ? shapes.sphere_shape : shapes.box_shape;
            query::collide_shape(world, collide_collector, collide_shape::default_settings(), collide_shape_choice, inside, query_filter);
            hash.word(collide_collector.hits.length);
            for (uint32_t i = 0; i < collide_collector.hits.length; i++) {
                hash.real(collide_collector.hits[i].penetration);
                hash.vec3(collide_collector.hits[i].point_a);
                hash.vec3(collide_collector.hits[i].penetration_axis);
            }
        }
    }
    return hash.value;
}

}  // namespace

void test_threads_concurrent_queries_on_one_world_match_a_single_thread(TestCtx &ctx) {
    fill_ground_arrays();
    WorldScene scene = make_pile(ctx);
    for (uint32_t step = 0; step < 30; step++) step_world(*scene.world);

    const QueryShapes shapes = {sphere::create({.radius = 0.4f}), box::create({.half_extents = vec3f::make(0.3f, 0.2f, 0.4f)}),
                                capsule::create({.half_height_of_cylinder = 0.3f, .radius = 0.25f})};

    uint32_t expected[THREAD_COUNT];
    for (uint32_t t = 0; t < THREAD_COUNT; t++) expected[t] = run_queries(*scene.world, shapes, t * 5);

    uint32_t actual[THREAD_COUNT] = {};
    std::thread threads[THREAD_COUNT];
    for (uint32_t t = 0; t < THREAD_COUNT; t++) {
        threads[t] = std::thread([&, t] { actual[t] = run_queries(*scene.world, shapes, t * 5); });
    }
    for (uint32_t t = 0; t < THREAD_COUNT; t++) threads[t].join();
    for (uint32_t t = 0; t < THREAD_COUNT; t++) CHECK(ctx, actual[t] == expected[t]);

    release_world_scene(scene);
}

void test_threads_worlds_stepped_on_separate_threads_match_a_single_thread(TestCtx &ctx) {
    fill_ground_arrays();
    WorldScene reference = make_pile(ctx);
    const uint32_t expected = step_and_hash(reference.world);
    release_world_scene(reference);

    WorldScene first = make_pile(ctx);
    WorldScene second = make_pile(ctx);
    uint32_t first_hash = 0;
    uint32_t second_hash = 0;
    std::thread first_thread([&] { first_hash = step_and_hash(first.world); });
    std::thread second_thread([&] { second_hash = step_and_hash(second.world); });
    first_thread.join();
    second_thread.join();
    CHECK(ctx, first_hash == expected);
    CHECK(ctx, second_hash == expected);

    release_world_scene(first);
    release_world_scene(second);
}
