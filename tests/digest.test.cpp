#include "test.h"

#include "physics/world_test_scene.h"
#include "test_math.h"
#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"
#include "digest.h"


namespace {

struct Hash {
    uint32_t value = 2166136261u;

    void word(uint32_t w) {
        for (int i = 0; i < 4; i++) {
            value ^= (w >> (i * 8)) & 0xffu;
            value *= 16777619u;
        }
    }
    /** every nan alike: their payloads are the platform's */
    void real(float f) { word(f != f ? 0x7fc00000u : __builtin_bit_cast(uint32_t, f)); }
    void lanes(V128 v) {
        real(v4f::lane<0>(v));
        real(v4f::lane<1>(v));
        real(v4f::lane<2>(v));
        real(v4f::lane<3>(v));
    }
    void mask(V128 m) {
        word((uint32_t)v4i::lane<0>(m));
        word((uint32_t)v4i::lane<1>(m));
        word((uint32_t)v4i::lane<2>(m));
        word((uint32_t)v4i::lane<3>(m));
    }
    void float3(const Float3 &f) {
        real(f.x);
        real(f.y);
        real(f.z);
    }
};

constexpr uint32_t VALUE_COUNT = 16;
const float VALUES[VALUE_COUNT] = {
    0.0f, -0.0f, 1.0f, -1.0f, 0.5f, 3.75f, -2.25f, 1e-40f, -1e-40f, scalar::FLOAT_MAX, -scalar::FLOAT_MAX, scalar::INF, -scalar::INF,
    __builtin_nanf(""), 1e30f, 7e-3f,
};

void hash_primitives(Hash &hash) {
    for (uint32_t i = 0; i < VALUE_COUNT; i++) {
        for (uint32_t j = 0; j < VALUE_COUNT; j++) {
            const V128 a = v4f::make(VALUES[i], VALUES[j], VALUES[(i + j) % VALUE_COUNT], VALUES[(i * 3 + 1) % VALUE_COUNT]);
            const V128 b = v4f::make(VALUES[j], VALUES[(j + 5) % VALUE_COUNT], VALUES[i], VALUES[(i + 2 * j) % VALUE_COUNT]);
            hash.lanes(v4f::add(a, b));
            hash.lanes(v4f::sub(a, b));
            hash.lanes(v4f::mul(a, b));
            hash.lanes(v4f::div(a, b));
            hash.lanes(v4f::min(a, b));
            hash.lanes(v4f::max(a, b));
            hash.lanes(v4f::fmin(a, b));
            hash.lanes(v4f::fmax(a, b));
            hash.lanes(v4f::neg(a));
            hash.lanes(v4f::abs(a));
            hash.lanes(v4f::sqrt(a));
            hash.lanes(v4f::neg_xyz(a));
            hash.lanes(v4f::mul_add(a, b, a));
            hash.mask(v4f::lt(a, b));
            hash.mask(v4f::le(a, b));
            hash.mask(v4f::gt(a, b));
            hash.mask(v4f::ge(a, b));
            hash.mask(v4f::eq(a, b));
            hash.mask(v4f::bit_and(a, b));
            hash.mask(v4f::bit_or(a, b));
            hash.mask(v4f::bit_xor(a, b));
            hash.lanes(v4f::select(a, b, v4f::lt(a, b)));
            hash.word(v4f::all_true(v4f::le(a, b)) | v4f::any_true(v4f::lt(a, b)) << 1 | v4f::bitmask(v4f::gt(a, b)) << 2);
            hash.lanes(v4f::shuffle<3, 1, 0, 2>(a));
            hash.lanes(v4f::shuffle2<6, 1, 4, 3>(a, b));
            hash.lanes(v4f::set_lane<2>(a, VALUES[j]));
            hash.real(v4f::hsum3(a));
            hash.real(v4f::hsum4(a));
            hash.real(v4f::dot3(a, b));
            hash.real(v4f::dot4(a, b));
            hash.real(scalar::min(VALUES[i], VALUES[j]));
            hash.real(scalar::max(VALUES[i], VALUES[j]));
            hash.mask(v4i::add(a, b));
        }
    }
    float packed[12];
    for (uint32_t i = 0; i < 12; i++) packed[i] = VALUES[(i * 7) % VALUE_COUNT];
    V128 x, y, z;
    v4f::load3x4(packed, x, y, z);
    hash.lanes(x);
    hash.lanes(y);
    hash.lanes(z);
    v4f::transpose3x4(v4f::load4(packed), v4f::load4(packed + 4), v4f::load4(packed + 8), v4f::load3(packed + 1), x, y, z);
    hash.lanes(x);
    hash.lanes(y);
    hash.lanes(z);
    hash.lanes(v4f::load3_xyzz(packed + 3));
}

void fibonacci_sphere(float *out, uint32_t n, float radius) {
    const float phi = scalar::PI * (3.0f - scalar::sqrt(5.0f));
    for (uint32_t i = 0; i < n; i++) {
        const float y = 1.0f - ((float)i / (float)(n - 1)) * 2.0f;
        const float r = scalar::sqrt(scalar::max(0.0f, 1.0f - y * y));
        const float theta = phi * (float)i;
        out[i * 3] = scalar::cos(theta) * r * radius;
        out[i * 3 + 1] = y * radius;
        out[i * 3 + 2] = scalar::sin(theta) * r * radius;
    }
}

/** a bumpy grid of triangles, 8 by 8 cells of 2 units */
shapes::Shape *make_ground() {
    constexpr uint32_t CELLS = 8;
    constexpr uint32_t SIDE = CELLS + 1;
    static float positions[SIDE * SIDE * 3];
    static uint32_t indices[CELLS * CELLS * 6];
    for (uint32_t z = 0; z < SIDE; z++) {
        for (uint32_t x = 0; x < SIDE; x++) {
            float *p = positions + (z * SIDE + x) * 3;
            p[0] = 2.0f * (float)x - (float)CELLS;
            p[1] = 0.15f * scalar::sin(1.3f * (float)x) * scalar::cos(0.9f * (float)z);
            p[2] = 2.0f * (float)z - (float)CELLS;
        }
    }
    uint32_t *index = indices;
    for (uint32_t z = 0; z < CELLS; z++) {
        for (uint32_t x = 0; x < CELLS; x++) {
            const uint32_t corner = z * SIDE + x;
            const uint32_t quad[6] = {corner, corner + SIDE, corner + 1, corner + 1, corner + SIDE, corner + SIDE + 1};
            for (uint32_t k = 0; k < 6; k++) *index++ = quad[k];
        }
    }
    triangle_mesh_builder::TriangleMeshBuilderSettings settings;
    settings.positions = positions;
    settings.num_positions = SIDE * SIDE;
    settings.indices = indices;
    settings.num_indices = CELLS * CELLS * 6;
    return triangle_mesh::create({.data = triangle_mesh_builder::build(settings)});
}

void hash_pile(Hash &hash) {
    dispatch::init();
    WorldScene scene = {};
    world_settings::WorldSettings settings = make_world_test_settings(scene.moving, scene.not_moving);
    scene.world = world::create(settings);
    add_body(scene, make_ground(), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});

    // a small hull walks every point; a large one climbs its adjacency
    float small_points[24 * 3];
    float large_points[60 * 3];
    fibonacci_sphere(small_points, 24, 0.5f);
    fibonacci_sphere(large_points, 60, 0.6f);
    convex_hull::ConvexHullData *small_hull = convex_hull::build(
        {.positions = small_points, .num_positions = 24, .bake_support_adjacency = convex_hull::BAKE_SUPPORT_ADJACENCY_NEVER});
    convex_hull::ConvexHullData *large_hull = convex_hull::build(
        {.positions = large_points, .num_positions = 60, .bake_support_adjacency = convex_hull::BAKE_SUPPORT_ADJACENCY_ALWAYS});

    for (uint32_t i = 0; i < 20; i++) {
        const Float3 position = {0.09f * (float)(i % 3) - 0.1f, 1.0f + 1.15f * (float)i, 0.05f * (float)(i % 4) - 0.08f};
        shapes::Shape *shape;
        switch (i % 5) {
            case 0: shape = box::create({.half_extents = {0.5f, 0.4f, 0.3f}}); break;
            case 1: shape = sphere::create({.radius = 0.45f}); break;
            case 2: shape = capsule::create({.half_height_of_cylinder = 0.3f, .radius = 0.3f}); break;
            case 3: shape = convex_hull::create({.data = small_hull}); break;
            default: shape = convex_hull::create({.data = large_hull}); break;
        }
        rigid_body::RigidBodySettings settings = body_settings(shape, scene.moving, motion_type::MOTION_DYNAMIC, position);
        settings.quaternion = stored_axis_angle(vec3f::normalize(vec3f::make(0.3f, 1.0f, 0.2f * (float)i)), 0.37f * (float)i);
        add_body(*scene.world, settings);
    }

    for (uint32_t step = 0; step < 300; step++) {
        step_world(*scene.world);
        const bodies::Bodies &bodies = scene.world->bodies;
        for (uint32_t i = 0; i < bodies.pool.length; i++) {
            const rigid_body::RigidBody *body = bodies.pool[i];
            if (body == nullptr) continue;
            hash.float3(body->position);
            hash.real(body->quaternion.x);
            hash.real(body->quaternion.y);
            hash.real(body->quaternion.z);
            hash.real(body->quaternion.w);
            hash.float3(body->motion_properties.linear_velocity);
            hash.float3(body->motion_properties.angular_velocity);
        }
    }
}

}  // namespace

uint32_t digest() {
    Hash hash;
    hash_primitives(hash);
    hash_pile(hash);
    return hash.value;
}

/** a change that moves any bit of the primitives or the pile changes this; the runners print the new value */
constexpr uint32_t PINNED_DIGEST = 0xf510af7cu;

void test_digest_matches_the_pin(TestCtx &ctx) { CHECK(ctx, digest() == PINNED_DIGEST); }
