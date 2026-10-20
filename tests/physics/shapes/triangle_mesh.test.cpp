#include "physics/shapes/triangle_mesh.h"
#include "math/aabox.h"

#include "instance.h"
#include "math/quatf.h"
#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/collision/internal_edge_removing_collector.h"
#include "physics/shapes/box.h"
#include "physics/shapes/capsule.h"
#include "physics/shapes/convex.h"
#include "physics/shapes/dispatch.h"
#include "physics/shapes/sphere.h"
#include "physics/shapes/utils/triangle_mesh_builder.h"
#include "test.h"
#include "test_math.h"
#include "util/alloc.h"
#include "util/collections.h"

using shapes::Shape;
using triangle_mesh_data::TriangleMeshData;

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};

static uint32_t heap_bytes_in_use() {
    util::HeapStats stats;
    util::heap_stats(&stats);
    return stats.bytes_in_use;
}

/** builds the mesh with the builder's defaults and a shape over it */
static Shape *mesh(const float *positions, uint32_t num_floats, const uint32_t *indices, uint32_t num_indices,
                   const int32_t *material_ids = nullptr, int32_t material_id = -1) {
    triangle_mesh_builder::TriangleMeshBuilderSettings settings;
    settings.positions = positions;
    settings.num_positions = num_floats / 3;
    settings.indices = indices;
    settings.num_indices = num_indices;
    settings.material_ids = material_ids;
    TriangleMeshData *data = triangle_mesh_builder::build(settings);
    if (data == nullptr) return nullptr;
    return triangle_mesh::create({.data = data, .material_id = material_id});
}

template <uint32_t NUM_FLOATS, uint32_t NUM_INDICES>
static Shape *mesh(const float (&positions)[NUM_FLOATS], const uint32_t (&indices)[NUM_INDICES], const int32_t *material_ids = nullptr) {
    return mesh(positions, NUM_FLOATS, indices, NUM_INDICES, material_ids);
}

/** destroys a shape made by `mesh` and the mesh data it was the only user of */
static void drop(Shape *s) {
    TriangleMeshData *data = s->triangle_mesh.data;
    triangle_mesh::destroy(s);
    triangle_mesh_data::destroy(data);
}

/** an n by n grid of square cells in the xz plane at y = 0 from the origin, two triangles a cell, facing +y */
static Shape *grid(uint32_t n, float cell) {
    collections::Vec<float> positions = {};
    collections::Vec<uint32_t> indices = {};
    for (uint32_t z = 0; z <= n; z++) {
        for (uint32_t x = 0; x <= n; x++) {
            positions.push((float)x * cell);
            positions.push(0.0f);
            positions.push((float)z * cell);
        }
    }
    for (uint32_t z = 0; z < n; z++) {
        for (uint32_t x = 0; x < n; x++) {
            const uint32_t i0 = z * (n + 1) + x;
            const uint32_t i1 = i0 + 1;
            const uint32_t i2 = i0 + n + 1;
            const uint32_t i3 = i2 + 1;
            const uint32_t cell_indices[6] = {i0, i2, i1, i1, i2, i3};
            for (uint32_t i = 0; i < 6; i++) indices.push(cell_indices[i]);
        }
    }
    Shape *s = mesh(positions.data, positions.length, indices.data, indices.length);
    positions.release();
    indices.release();
    return s;
}

/** unit quad in the xz plane at y = 1, normal +y, spanning [0,1] x [0,1] */
static const float QUAD_POSITIONS[12] = {0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1};
static const uint32_t QUAD_INDICES[6] = {0, 2, 1, 0, 3, 2};

/** closed unit cube spanning [0,1]^3, counter clockwise seen from outside */
static const float CUBE_POSITIONS[24] = {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1};
static const uint32_t CUBE_INDICES[36] = {
    0, 2, 1, 0, 3, 2,  // z = 0, normal -z
    4, 5, 6, 4, 6, 7,  // z = 1, normal +z
    0, 1, 5, 0, 5, 4,  // y = 0, normal -y
    3, 7, 6, 3, 6, 2,  // y = 1, normal +y
    0, 4, 7, 0, 7, 3,  // x = 0, normal -x
    1, 2, 6, 1, 6, 5,  // x = 1, normal +x
};

/** one triangle in the xy plane, normal +z */
static const float TRIANGLE_POSITIONS[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
static const uint32_t TRIANGLE_INDICES[3] = {0, 1, 2};
static const uint32_t TRIANGLE_INDICES_FLIPPED[3] = {0, 2, 1};

/** the same triangle twice the size */
static const float BIG_TRIANGLE_POSITIONS[9] = {0, 0, 0, 2, 0, 0, 0, 2, 0};

static void collide(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings, const Shape *a,
                    const Transformf &transform_a, const Shape *b, const Transformf &transform_b) {
    shapes::collide_fn(a->type, b->type)(collector, settings, a, NO_SUB_SHAPE, transform_a, b, NO_SUB_SHAPE, transform_b);
}

static void cast(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *a,
                 const Transformf &transform_a, const Float3 &displacement, const Shape *b, const Transformf &transform_b) {
    shapes::cast_fn(a->type, b->type)(collector, settings, a, NO_SUB_SHAPE, transform_a, vec3f::load(displacement), b, NO_SUB_SHAPE, transform_b);
}

static void ray_cast(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const Shape *s,
                     const Transformf &transform, const Float3 &origin, const Float3 &direction, float length) {
    const cast_ray::Rayf ray = {vec3f::load(origin), vec3f::load(direction), length};
    shapes::def_of(s).cast_ray(collector, settings, ray, s, NO_SUB_SHAPE, transform);
}

static bool point_inside(const Shape *s, const Transformf &transform, const Float3 &point) {
    collide_point::CollidePointCollector collector = collide_point::create_collector(collector::COLLECT_ANY);
    shapes::def_of(s).collide_point(collector, collide_point::default_settings(), vec3f::load(point), s, NO_SUB_SHAPE, transform);
    const bool inside = collector.has_hit;
    collide_point::release(collector);
    return inside;
}

static float deepest_penetration(const collide_shape::CollideShapeCollector &collector) {
    float deepest = -1e30f;
    for (uint32_t i = 0; i < collector.hits.length; i++) deepest = scalar::max(deepest, collector.hits[i].penetration);
    return deepest;
}

static float first_fraction(const cast_shape::CastShapeCollector &collector) {
    float first = 1e30f;
    for (uint32_t i = 0; i < collector.hits.length; i++) first = scalar::min(first, collector.hits[i].fraction);
    return first;
}

/** the triangle a hit's sub shape id names, popped from a top-level mesh */
static uint32_t triangle_of(const Shape *s, sub_shape::SubShapeId id) {
    sub_shape::PopResult pop;
    sub_shape::pop_index(pop, id, s->triangle_mesh.data->triangle_count);
    return pop.value;
}

/* the shape */

void test_triangle_mesh_create_and_mass(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    const uint32_t heap_before = heap_bytes_in_use();

    Shape *s = mesh(TRIANGLE_POSITIONS, 9, TRIANGLE_INDICES, 3, nullptr, 4);
    CHECK(ctx, s != nullptr);
    CHECK(ctx, s->type == shapes::SHAPE_TRIANGLE_MESH);
    CHECK(ctx, s->triangle_mesh.data->triangle_count == 1);
    CHECK(ctx, s->material_id == 4);
    // vertices (0,0,0), (1,0,0), (0,1,0): the bounds are exactly [0,0,0] to [1,1,0]
    CHECK(ctx, s->aabb.min.x == 0.0f && s->aabb.min.y == 0.0f && s->aabb.min.z == 0.0f);
    CHECK(ctx, s->aabb.max.x == 1.0f && s->aabb.max.y == 1.0f && s->aabb.max.z == 0.0f);
    CHECK(ctx, s->center_of_mass.x == 0.0f && s->center_of_mass.y == 0.0f && s->center_of_mass.z == 0.0f);
    CHECK(ctx, s->volume == 0.0f);

    // static geometry: zero mass, unit inertia; no sphere fits inside a sheet
    mass_properties::MassProperties props;
    shapes::compute_mass_properties(props, s);
    CHECK(ctx, props.mass == 0.0f);
    CHECK(ctx, props.inertia.m[0] == 1.0f && props.inertia.m[5] == 1.0f && props.inertia.m[10] == 1.0f && props.inertia.m[1] == 0.0f);
    CHECK(ctx, shapes::get_shape_inner_radius(s) == 0.0f);

    // update recomputes the bounds from the bvh root
    s->aabb = {{9, 9, 9}, {9, 9, 9}};
    triangle_mesh::update(s);
    CHECK(ctx, s->aabb.max.x == 1.0f && s->aabb.max.y == 1.0f && s->aabb.min.z == 0.0f);
    drop(s);

    const float two_positions[12] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1};
    const uint32_t two_indices[6] = {0, 1, 2, 1, 2, 3};
    Shape *two = mesh(two_positions, two_indices);
    CHECK(ctx, two->triangle_mesh.data->triangle_count == 2);
    CHECK(ctx, two->aabb.max.z == 1.0f);
    drop(two);

    // shape and mesh return every byte
    CHECK(ctx, heap_bytes_in_use() == heap_before);
}

void test_triangle_mesh_shared_data(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    TriangleMeshData *data = triangle_mesh_builder::build({.positions = QUAD_POSITIONS, .num_positions = 4, .indices = QUAD_INDICES, .num_indices = 6});
    Shape *first = triangle_mesh::create({.data = data});
    Shape *second = triangle_mesh::create({.data = data, .material_id = 2});
    CHECK(ctx, first->triangle_mesh.data == second->triangle_mesh.data);

    // the first shape going leaves the mesh to the second
    triangle_mesh::destroy(first);
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    ray_cast(collector, cast_ray::default_settings(), second, pose({0, 0, 0}), {0.3f, 3, 0.6f}, {0, -1, 0}, 4);
    CHECK(ctx, collector.has_hit);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.5f, 1e-6f);
    cast_ray::release(collector);
    triangle_mesh::destroy(second);
    triangle_mesh_data::destroy(data);
}

void test_triangle_mesh_surface_normal(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();

    // one triangle in the xy plane: its normal is +z
    Shape *s = mesh(TRIANGLE_POSITIONS, TRIANGLE_INDICES);
    sub_shape::SubShapeIdBuilder id;
    sub_shape::push_index(id, NO_SUB_SHAPE, 0, 1);
    Float3 normal;
    vec3f::store(normal, shapes::get_shape_surface_normal(s, vec3f::make(0.3f, 0.3f, 0), id.value));
    CHECK(ctx, near3(normal, 0, 0, 1, 1e-6f));
    drop(s);

    // every triangle of the cube by its sub shape id: the outward normal of its face
    Shape *cube = mesh(CUBE_POSITIONS, CUBE_INDICES);
    const TriangleMeshData &data = *cube->triangle_mesh.data;
    CHECK(ctx, data.triangle_count == 12);
    for (uint32_t t = 0; t < data.triangle_count; t++) {
        sub_shape::push_index(id, NO_SUB_SHAPE, t, data.triangle_count);
        Vec3f a, b, c;
        triangle_mesh_data::get_triangle_vertices(a, b, c, data, t);
        const Vec3f centroid = vec3f::scale(vec3f::add(vec3f::add(a, b), c), 1.0f / 3.0f);
        vec3f::store(normal, shapes::get_shape_surface_normal(cube, centroid, id.value));
        // the face is the axis the centroid sits on (0 or 1), the normal points away from the centre
        const Vec3f outward = vec3f::subtract(centroid, vec3f::make(0.5f, 0.5f, 0.5f));
        const float ax = scalar::abs(vec3f::x(outward)), ay = scalar::abs(vec3f::y(outward)), az = scalar::abs(vec3f::z(outward));
        Float3 expected = {0, 0, 0};
        if (ax >= ay && ax >= az) expected.x = vec3f::x(outward) > 0 ? 1.0f : -1.0f;
        else if (ay >= az) expected.y = vec3f::y(outward) > 0 ? 1.0f : -1.0f;
        else expected.z = vec3f::z(outward) > 0 ? 1.0f : -1.0f;
        CHECK(ctx, near3(normal, expected, 1e-6f));
    }
    drop(cube);
}

void test_triangle_mesh_supporting_face(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *s = mesh(TRIANGLE_POSITIONS, TRIANGLE_INDICES);
    sub_shape::SubShapeIdBuilder id;
    sub_shape::push_index(id, NO_SUB_SHAPE, 0, 1);

    const Mat4f transform = mat4f::from_rotation_translation(quatf::identity(), vec3f::make(1, 2, 3));
    face::Face face;

    // scaled then moved: a, b, c in order
    shapes::get_shape_supporting_face(face, s, id.value, vec3f::make(0, 0, -1), transform, vec3f::make(2, 1, 1));
    CHECK(ctx, face.num_vertices == 3);
    CHECK(ctx, near3(face.vertices[0], 1, 2, 3, 1e-6f));
    CHECK(ctx, near3(face.vertices[1], 3, 2, 3, 1e-6f));
    CHECK(ctx, near3(face.vertices[2], 1, 3, 3, 1e-6f));

    // a mirrored scale turns the triangle inside out: c, b, a keeps it counter clockwise from the front
    shapes::get_shape_supporting_face(face, s, id.value, vec3f::make(0, 0, -1), transform, vec3f::make(-1, 1, 1));
    CHECK(ctx, face.num_vertices == 3);
    CHECK(ctx, near3(face.vertices[0], 1, 3, 3, 1e-6f));
    CHECK(ctx, near3(face.vertices[1], 0, 2, 3, 1e-6f));
    CHECK(ctx, near3(face.vertices[2], 1, 2, 3, 1e-6f));
    drop(s);
}

void test_triangle_mesh_dispatch_wiring(TestCtx &ctx) {
    dispatch::init();
    const shapes::ShapeDef &mesh_def = shapes::shape_defs[shapes::shape_type_index(shapes::SHAPE_TRIANGLE_MESH)];
    CHECK(ctx, mesh_def.type == shapes::SHAPE_TRIANGLE_MESH && mesh_def.set_support == nullptr);
    CHECK(ctx, mesh_def.cast_ray == triangle_mesh::cast_ray_vs_triangle_mesh);
    CHECK(ctx, mesh_def.collide_point == triangle_mesh::collide_point_vs_triangle_mesh);
    CHECK(ctx, mesh_def.get_inner_radius == shapes::default_get_inner_radius);

    const shapes::ShapeType mesh_type = shapes::SHAPE_TRIANGLE_MESH;
    const shapes::ShapeType convex_types[] = {shapes::SHAPE_BOX, shapes::SHAPE_CAPSULE, shapes::SHAPE_CONVEX_HULL, shapes::SHAPE_CYLINDER};
    for (shapes::ShapeType t : convex_types) {
        CHECK(ctx, shapes::collide_fn(t, mesh_type) == triangle_mesh::collide_convex_vs_triangle_mesh);
        CHECK(ctx, shapes::collide_fn(mesh_type, t) ==
                       collide_shape::reversed_collide_shape_vs_shape<triangle_mesh::collide_convex_vs_triangle_mesh>);
        CHECK(ctx, shapes::cast_fn(t, mesh_type) == triangle_mesh::cast_convex_vs_triangle_mesh);
        CHECK(ctx, shapes::cast_fn(mesh_type, t) == cast_shape::reversed_cast_shape_vs_shape<triangle_mesh::cast_convex_vs_triangle_mesh>);
    }
    // the sphere takes the closed-form paths
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_SPHERE, mesh_type) == triangle_mesh::collide_sphere_vs_triangle_mesh);
    CHECK(ctx, shapes::collide_fn(mesh_type, shapes::SHAPE_SPHERE) ==
                   collide_shape::reversed_collide_shape_vs_shape<triangle_mesh::collide_sphere_vs_triangle_mesh>);
    CHECK(ctx, shapes::cast_fn(shapes::SHAPE_SPHERE, mesh_type) == triangle_mesh::cast_sphere_vs_triangle_mesh);
    CHECK(ctx, shapes::cast_fn(mesh_type, shapes::SHAPE_SPHERE) ==
                   cast_shape::reversed_cast_shape_vs_shape<triangle_mesh::cast_sphere_vs_triangle_mesh>);
    // mesh vs mesh and mesh vs empty have no handler; the convex pairs keep theirs
    CHECK(ctx, shapes::collide_fn(mesh_type, mesh_type) == nullptr);
    CHECK(ctx, shapes::cast_fn(mesh_type, shapes::SHAPE_EMPTY) == nullptr);
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_BOX, shapes::SHAPE_CAPSULE) == convex::collide_convex_vs_convex);
}

/* cast ray */

static const cast_ray::CastRaySettings RAY_FRONT = {false, false, true};
static const cast_ray::CastRaySettings RAY_BACKFACES = {true, false, true};

/** every hit of a ray against the mesh in a pose (rotation identity), as crashcat's scaled mesh ray tests cast it */
static uint32_t ray_hits(cast_ray::CastRayCollector &collector, const Shape *s, const Float3 &scale, const Float3 &origin,
                         const Float3 &direction, float length, const cast_ray::CastRaySettings &settings = RAY_FRONT) {
    cast_ray::reset(collector);
    ray_cast(collector, settings, s, pose({0, 0, 0}, {0, 0, 0, 1}, scale), origin, direction, length);
    return collector.hits.length;
}

void test_triangle_mesh_ray_scaled_quad(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *quad = mesh(QUAD_POSITIONS, QUAD_INDICES);
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);

    // unit scale: the surface at y = 1, 10 of 20 down
    CHECK(ctx, ray_hits(collector, quad, {1, 1, 1}, {0.3f, 11, 0.6f}, {0, -1, 0}, 20) == 1);
    CHECK(ctx, collector.hits[0].status == cast_ray::COLLIDING);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 10.0f / 20.0f, 1e-6f);

    // scale 2: the quad spans [0,2] x [0,2] at y = 2; x = 1.5 is off the unscaled quad
    CHECK(ctx, ray_hits(collector, quad, {2, 2, 2}, {1.5f, 11, 1.8f}, {0, -1, 0}, 20) == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 9.0f / 20.0f, 1e-6f);
    CHECK(ctx, ray_hits(collector, quad, {1, 1, 1}, {1.5f, 11, 1.8f}, {0, -1, 0}, 20) == 0);

    // scale (3, 0.5, 1): [0,3] x [0,1] at y = 0.5; z is unscaled, so z = 1.5 misses
    CHECK(ctx, ray_hits(collector, quad, {3, 0.5f, 1}, {2.5f, 5.5f, 0.5f}, {0, -1, 0}, 10) == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 5.0f / 10.0f, 1e-6f);
    CHECK(ctx, ray_hits(collector, quad, {3, 0.5f, 1}, {2.5f, 5.5f, 1.5f}, {0, -1, 0}, 10) == 0);

    // a diagonal ray against the scale 2 quad: y drops by 2 after sqrt(5)
    const float sqrt5 = scalar::sqrt(5.0f);
    CHECK(ctx, ray_hits(collector, quad, {2, 2, 2}, {0.2f, 4, 0.7f}, {1 / sqrt5, -2 / sqrt5, 0}, 10) == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, sqrt5 / 10.0f, 1e-6f);

    // a hit at the ray's end counts, one just short of the surface does not, scaled or not
    CHECK(ctx, ray_hits(collector, quad, {1, 1, 1}, {0.3f, 6, 0.6f}, {0, -1, 0}, 5) == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 1.0f, 1e-6f);
    CHECK(ctx, ray_hits(collector, quad, {1, 1, 1}, {0.3f, 6, 0.6f}, {0, -1, 0}, 5 - 1e-5f) == 0);
    CHECK(ctx, ray_hits(collector, quad, {2, 2, 2}, {0.3f, 7, 0.6f}, {0, -1, 0}, 5 - 1e-5f) == 0);

    // a mirrored scale puts the quad at y = -1 facing -y: a ray from above meets its back
    CHECK(ctx, ray_hits(collector, quad, {1, -1, 1}, {0.3f, 9, 0.6f}, {0, -1, 0}, 20) == 0);
    CHECK(ctx, ray_hits(collector, quad, {1, -1, 1}, {0.3f, 9, 0.6f}, {0, -1, 0}, 20, RAY_BACKFACES) == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 10.0f / 20.0f, 1e-6f);

    // from below, unscaled: the back face, culled unless asked for
    CHECK(ctx, ray_hits(collector, quad, {1, 1, 1}, {0.3f, -1, 0.6f}, {0, 1, 0}, 5) == 0);
    CHECK(ctx, ray_hits(collector, quad, {1, 1, 1}, {0.3f, -1, 0.6f}, {0, 1, 0}, 5, RAY_BACKFACES) == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 2.0f / 5.0f, 1e-6f);

    // parallel to the quad: nothing
    CHECK(ctx, ray_hits(collector, quad, {1, 1, 1}, {-1, 1, 0.5f}, {1, 0, 0}, 5, RAY_BACKFACES) == 0);

    cast_ray::release(collector);
    drop(quad);
}

void test_triangle_mesh_ray_closed_cube(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *cube = mesh(CUBE_POSITIONS, CUBE_INDICES);

    // through the cube along +z at (0.3, 0.6), off every face's diagonal: in at z = 0 after 2 of 4, out at z = 1
    cast_ray::CastRayCollector closest = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    ray_cast(closest, RAY_FRONT, cube, pose({0, 0, 0}), {0.3f, 0.6f, -2}, {0, 0, 1}, 4);
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 0.5f, 1e-6f);
    // the near face: its triangle's normal is -z
    Float3 normal;
    vec3f::store(normal, shapes::get_shape_surface_normal(cube, vec3f::make(0.3f, 0.6f, 0), closest.hits[0].sub_shape_id));
    CHECK(ctx, near3(normal, 0, 0, -1, 1e-6f));

    // every hit, front faces only: the exit is a back face
    cast_ray::CastRayCollector all = cast_ray::create_collector(collector::COLLECT_ALL);
    ray_cast(all, RAY_FRONT, cube, pose({0, 0, 0}), {0.3f, 0.6f, -2}, {0, 0, 1}, 4);
    CHECK(ctx, all.hits.length == 1);
    CHECK_NEAR(ctx, all.hits[0].fraction, 0.5f, 1e-6f);

    // with the triangles' back faces: in and out
    cast_ray::reset(all);
    ray_cast(all, RAY_BACKFACES, cube, pose({0, 0, 0}), {0.3f, 0.6f, -2}, {0, 0, 1}, 4);
    CHECK(ctx, all.hits.length == 2);
    if (all.hits.length == 2) {
        const float f0 = scalar::min(all.hits[0].fraction, all.hits[1].fraction);
        const float f1 = scalar::max(all.hits[0].fraction, all.hits[1].fraction);
        CHECK_NEAR(ctx, f0, 0.5f, 1e-6f);
        CHECK_NEAR(ctx, f1, 0.75f, 1e-6f);
        const uint32_t far_hit = all.hits[0].fraction > all.hits[1].fraction ? 0 : 1;
        vec3f::store(normal, shapes::get_shape_surface_normal(cube, vec3f::make(0.3f, 0.6f, 1), all.hits[far_hit].sub_shape_id));
        CHECK(ctx, near3(normal, 0, 0, 1, 1e-6f));
    }

    // the closest collector with back faces still reports the near face
    cast_ray::reset(closest);
    ray_cast(closest, RAY_BACKFACES, cube, pose({0, 0, 0}), {0.3f, 0.6f, -2}, {0, 0, 1}, 4);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 0.5f, 1e-6f);

    // the cube turned a quarter about y and moved to x = 10: (x, y, z) -> (z, y, -x) + (10, 0, 0), so it spans
    // x in [10, 11], z in [-1, 0]; a ray down from above at (10.4, 3, -0.3) meets the top at y = 1 after 2 of 4
    const Transformf turned = pose({10, 0, 0}, stored_axis_angle(vec3f::make(0, 1, 0), scalar::PI * 0.5f));
    cast_ray::reset(closest);
    ray_cast(closest, RAY_FRONT, cube, turned, {10.4f, 3, -0.3f}, {0, -1, 0}, 4);
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 0.5f, 1e-5f);
    cast_ray::reset(closest);
    ray_cast(closest, RAY_FRONT, cube, turned, {10.4f, 3, 0.3f}, {0, -1, 0}, 4);
    CHECK(ctx, !closest.has_hit);

    // an any collector stops at its first hit
    cast_ray::CastRayCollector any = cast_ray::create_collector(collector::COLLECT_ANY);
    ray_cast(any, RAY_BACKFACES, cube, pose({0, 0, 0}), {0.3f, 0.6f, -2}, {0, 0, 1}, 4);
    CHECK(ctx, any.has_hit && any.hits.length == 1);

    cast_ray::release(closest);
    cast_ray::release(all);
    cast_ray::release(any);
    drop(cube);
}

void test_triangle_mesh_ray_materials_and_sub_shapes(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // the quad's first input triangle (0, 2, 1) covers x > z, the second (0, 3, 2) covers x < z
    const int32_t materials[2] = {3, 7};
    Shape *quad = mesh(QUAD_POSITIONS, QUAD_INDICES, materials);
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);

    ray_cast(collector, RAY_FRONT, quad, pose({0, 0, 0}), {0.8f, 3, 0.2f}, {0, -1, 0}, 4);
    CHECK(ctx, collector.has_hit && collector.hits[0].material_id == 3);
    const uint32_t first = triangle_of(quad, collector.hits[0].sub_shape_id);
    CHECK(ctx, triangle_mesh_data::get_material_id(*quad->triangle_mesh.data, first) == 3);

    cast_ray::reset(collector);
    ray_cast(collector, RAY_FRONT, quad, pose({0, 0, 0}), {0.2f, 3, 0.8f}, {0, -1, 0}, 4);
    CHECK(ctx, collector.has_hit && collector.hits[0].material_id == 7);
    const uint32_t second = triangle_of(quad, collector.hits[0].sub_shape_id);
    CHECK(ctx, second != first && triangle_mesh_data::get_material_id(*quad->triangle_mesh.data, second) == 7);

    // a parent's bits below the mesh's level survive: the mesh pushes its triangle above them
    const sub_shape::SubShapeIdBuilder parent = {0xfffffff5u, 4};
    cast_ray::reset(collector);
    const cast_ray::Rayf ray = {vec3f::make(0.2f, 3, 0.8f), vec3f::make(0, -1, 0), 4};
    triangle_mesh::cast_ray_vs_triangle_mesh(collector, RAY_FRONT, ray, quad, parent, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit);
    sub_shape::PopResult pop;
    sub_shape::pop(pop, collector.hits[0].sub_shape_id, 4);
    CHECK(ctx, pop.value == 5);
    CHECK(ctx, triangle_of(quad, pop.remainder) == second);

    cast_ray::release(collector);
    drop(quad);
}

/* collide point */

void test_triangle_mesh_collide_point(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *cube = mesh(CUBE_POSITIONS, CUBE_INDICES);

    // the parity ray goes up from the point; samples stay off the top face's diagonal
    CHECK(ctx, point_inside(cube, pose({0, 0, 0}), {0.3f, 0.5f, 0.6f}));
    CHECK(ctx, point_inside(cube, pose({0, 0, 0}), {0.7f, 0.1f, 0.2f}));
    CHECK(ctx, !point_inside(cube, pose({0, 0, 0}), {1.3f, 0.5f, 0.6f}));
    CHECK(ctx, !point_inside(cube, pose({0, 0, 0}), {0.3f, 1.5f, 0.6f}));
    CHECK(ctx, !point_inside(cube, pose({0, 0, 0}), {0.3f, -0.5f, 0.6f}));

    // scale (3, 1, 1): the cube spans x in [0, 3]
    CHECK(ctx, point_inside(cube, pose({0, 0, 0}, {0, 0, 0, 1}, {3, 1, 1}), {2.5f, 0.3f, 0.6f}));
    CHECK(ctx, !point_inside(cube, pose({0, 0, 0}, {0, 0, 0, 1}, {3, 1, 1}), {3.5f, 0.3f, 0.6f}));
    CHECK(ctx, !point_inside(cube, pose({0, 0, 0}), {2.5f, 0.3f, 0.6f}));

    // turned a quarter about y and moved to x = 10: spans x in [10, 11], z in [-1, 0]
    const Transformf turned = pose({10, 0, 0}, stored_axis_angle(vec3f::make(0, 1, 0), scalar::PI * 0.5f));
    CHECK(ctx, point_inside(cube, turned, {10.6f, 0.3f, -0.2f}));
    CHECK(ctx, !point_inside(cube, turned, {10.6f, 0.3f, 0.2f}));

    // a mesh with no height has nothing inside it, not even a point on its surface
    Shape *flat = mesh(QUAD_POSITIONS, QUAD_INDICES);
    CHECK(ctx, !point_inside(flat, pose({0, 0, 0}), {0.25f, 1, 0.75f}));
    drop(flat);

    // the hit reports the crossing's triangle material and the shape's sub shape id
    const int32_t materials[12] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6};
    Shape *painted = mesh(CUBE_POSITIONS, CUBE_INDICES, materials);
    collide_point::CollidePointCollector collector = collide_point::create_collector(collector::COLLECT_ALL);
    shapes::def_of(painted).collide_point(collector, collide_point::default_settings(), vec3f::make(0.3f, 0.5f, 0.6f), painted, NO_SUB_SHAPE,
                                          pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    // the only crossing going up is the top face (y = 1), material 4
    CHECK(ctx, collector.hits[0].material_id == 4);
    const uint32_t top = triangle_of(painted, collector.hits[0].sub_shape_id_b);
    CHECK(ctx, triangle_mesh_data::get_material_id(*painted->triangle_mesh.data, top) == 4);
    collide_point::release(collector);
    drop(painted);
    drop(cube);
}

/* collide */

void test_triangle_mesh_sphere_resting_on_quad(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // a 2 by 2 quad at y = 0 facing +y, its diagonal from (0, 0) to (2, 2)
    const float positions[12] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2};
    const uint32_t indices[6] = {0, 2, 1, 0, 3, 2};
    const int32_t materials[2] = {8, 9};
    Shape *quad = mesh(positions, indices, materials);
    Shape *ball = sphere::create({.radius = 0.5f, .material_id = 1});

    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // over the interior of one triangle, 0.1 deep, far from the diagonal: one contact straight down
    collide(collector, settings, ball, pose({1.6f, 0.4f, 0.3f}), quad, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const collide_shape::CollideShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.penetration, 0.1f, 1e-5f);
    CHECK(ctx, near3(unit(hit.penetration_axis), 0, -1, 0, 1e-5f));
    CHECK(ctx, near3(hit.point_a, 1.6f, -0.1f, 0.3f, 1e-5f));
    CHECK(ctx, near3(hit.point_b, 1.6f, 0, 0.3f, 1e-5f));
    CHECK(ctx, hit.material_id_a == 1 && hit.material_id_b == 8);

    // next to the diagonal (0.14 from it): the other triangle is touched at its diagonal edge too. with active
    // edges (the default) that inactive edge gives the face normal, so no contact pushes sideways
    collide_shape::reset(collector);
    collide(collector, settings, ball, pose({1.4f, 0.4f, 1.2f}), quad, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 2);
    CHECK_NEAR(ctx, deepest_penetration(collector), 0.1f, 1e-5f);
    for (uint32_t i = 0; i < collector.hits.length; i++) CHECK(ctx, near3(unit(collector.hits[i].penetration_axis), 0, -1, 0, 1e-5f));

    // without active edges the diagonal contact is a ghost tilted towards the edge
    collide_shape::CollideShapeSettings every_edge = settings;
    every_edge.collide_only_with_active_edges = false;
    collide_shape::reset(collector);
    collide(collector, every_edge, ball, pose({1.4f, 0.4f, 1.2f}), quad, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 2);
    bool tilted = false;
    for (uint32_t i = 0; i < collector.hits.length; i++) tilted |= vec3f::y(unit(collector.hits[i].penetration_axis)) > -0.99f;
    CHECK(ctx, tilted);

    // jolt's internal edge removal keeps one contact, the face's, even with every edge active
    internal_edge_removing::InternalEdgeRemovingCollector remover = internal_edge_removing::create();
    collide_shape::reset(collector);
    internal_edge_removing::set(remover, collector, settings.internal_edge_removal_vertex_tolerance_sq);
    every_edge.collect_faces = true;
    collide(remover.collector, every_edge, ball, pose({1.4f, 0.4f, 1.2f}), quad, pose({0, 0, 0}));
    internal_edge_removing::flush(remover);
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-5f);
    CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, -1, 0, 1e-5f));
    internal_edge_removing::release(remover);

    // the closest collector keeps the deepest
    collide_shape::CollideShapeCollector closest = collide_shape::create_collector(collector::COLLECT_CLOSEST);
    collide(closest, settings, ball, pose({1.4f, 0.4f, 1.2f}), quad, pose({0, 0, 0}));
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, closest.hits[0].penetration, 0.1f, 1e-5f);

    // with faces: the sphere has none, the triangle is the hit triangle in world space
    settings.collect_faces = true;
    collide_shape::reset(collector);
    collide(collector, settings, ball, pose({1.6f, 0.4f, 0.3f}), quad, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1 && collector.hits[0].face_a.num_vertices == 0 && collector.hits[0].face_b.num_vertices == 3);
    for (uint32_t v = 0; v < 3; v++) CHECK(ctx, vec3f::y(collector.hits[0].face_b.vertices[v]) == 0.0f);

    // above the separation, or under the quad with back faces off: nothing
    collide_shape::reset(collector);
    collide(collector, collide_shape::default_settings(), ball, pose({1.6f, 0.6f, 0.3f}), quad, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);
    collide_shape::reset(collector);
    collide(collector, collide_shape::default_settings(), ball, pose({1.6f, -0.4f, 0.3f}), quad, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);
    collide_shape::CollideShapeSettings backfaces = collide_shape::default_settings();
    backfaces.collide_with_backfaces = true;
    collide(collector, backfaces, ball, pose({1.6f, -0.4f, 0.3f}), quad, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, 1, 0, 1e-5f));

    // within a max separation distance: a negative penetration
    collide_shape::CollideShapeSettings separated = collide_shape::default_settings();
    separated.max_separation_distance = 0.2f;
    collide_shape::reset(collector);
    collide(collector, separated, ball, pose({1.6f, 0.6f, 0.3f}), quad, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, -0.1f, 1e-5f);

    collide_shape::release(collector);
    collide_shape::release(closest);
    sphere::destroy(ball);
    drop(quad);
}

void test_triangle_mesh_sphere_collide_features(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *tri = mesh(BIG_TRIANGLE_POSITIONS, TRIANGLE_INDICES);
    const collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // past the edge at x = 2 along the x axis: the closest point is the vertex (2, 0, 0), 0.3 away
    Shape *ball = sphere::create({.radius = 0.5f});
    collide(collector, settings, ball, pose({2.3f, 0, 0}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.2f, 1e-5f);
    CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), -1, 0, 0, 1e-5f));

    // centred on the vertex at the origin: the full radius, the arbitrary axis
    Shape *big = sphere::create({.radius = 0.6f});
    collide_shape::reset(collector);
    collide(collector, settings, big, pose({0, 0, 0}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.6f, 1e-5f);

    // a sphere scaled by 2 (radius 1) 0.4 above the face: 0.6 deep
    collide_shape::reset(collector);
    collide(collector, settings, ball, pose({0.5f, 0.5f, 0.4f}, {0, 0, 0, 1}, {2, 2, 2}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.6f, 1e-5f);

    // far away: nothing
    collide_shape::reset(collector);
    collide(collector, settings, ball, pose({10, 10, 10}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    collide_shape::release(collector);
    sphere::destroy(ball);
    sphere::destroy(big);
    drop(tri);
}

void test_triangle_mesh_convex_collide(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *tri = mesh(BIG_TRIANGLE_POSITIONS, TRIANGLE_INDICES);
    const collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // a box 0.3 into the face from above: the axis down into the triangle
    Shape *cube = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.0f, .material_id = 6});
    collide(collector, settings, cube, pose({0.5f, 0.5f, 0.2f}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.3f, 1e-4f);
    CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, 0, -1, 1e-4f));
    CHECK(ctx, collector.hits[0].material_id_a == 6 && collector.hits[0].material_id_b == -1);
    CHECK(ctx, triangle_of(tri, collector.hits[0].sub_shape_id_b) == 0);

    // a capsule lying in the face (its axis is y), a box past an edge, a box far off
    Shape *pill = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
    collide_shape::reset(collector);
    collide(collector, settings, pill, pose({0.5f, 0.5f, 0.15f}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.15f, 1e-4f);
    Shape *small = box::create({.half_extents = {0.3f, 0.3f, 0.3f}});
    collide_shape::reset(collector);
    collide(collector, settings, small, pose({2.2f, 0, 0}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    collide_shape::reset(collector);
    collide(collector, settings, small, pose({10, 10, 10}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // deep: a box sunk to its centre needs epa
    collide_shape::reset(collector);
    collide(collector, settings, cube, pose({0.5f, 0.5f, 0.0f}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-3f);

    // back faces: under the triangle only with them on
    collide_shape::reset(collector);
    collide(collector, settings, cube, pose({0.5f, 0.5f, -0.2f}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);
    collide_shape::CollideShapeSettings backfaces = settings;
    backfaces.collide_with_backfaces = true;
    collide(collector, backfaces, cube, pose({0.5f, 0.5f, -0.2f}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.3f, 1e-4f);

    // faces: the box's bottom and the triangle, both in world space
    collide_shape::CollideShapeSettings faces = settings;
    faces.collect_faces = true;
    collide_shape::reset(collector);
    collide(collector, faces, cube, pose({0.5f, 0.5f, 0.2f}), tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1 && collector.hits[0].face_a.num_vertices == 4 && collector.hits[0].face_b.num_vertices == 3);
    for (uint32_t v = 0; v < 4; v++) CHECK_NEAR(ctx, vec3f::z(collector.hits[0].face_a.vertices[v]), -0.3f, 1e-5f);
    for (uint32_t v = 0; v < 3; v++) CHECK_NEAR(ctx, vec3f::z(collector.hits[0].face_b.vertices[v]), 0.0f, 1e-6f);

    // a 10 x 10 grid: the bvh walk finds the triangles under a box in its middle
    Shape *floor = grid(10, 1.0f);
    collide_shape::reset(collector);
    collide(collector, settings, small, pose({5.3f, 0.2f, 5.6f}), floor, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length > 0);
    CHECK_NEAR(ctx, deepest_penetration(collector), 0.1f, 1e-4f);
    for (uint32_t i = 0; i < collector.hits.length; i++) {
        const uint32_t t = triangle_of(floor, collector.hits[i].sub_shape_id_b);
        Box3f bounds;
        triangle_mesh_data::calculate_triangle_aabb(bounds, *floor->triangle_mesh.data, t);
        CHECK(ctx, bounds.max.x >= 5.0f - 1e-6f && bounds.min.x <= 5.6f && bounds.max.z >= 5.3f && bounds.min.z <= 5.9f);
    }

    collide_shape::release(collector);
    box::destroy(cube);
    box::destroy(small);
    capsule::destroy(pill);
    drop(tri);
    drop(floor);
}

void test_triangle_mesh_box_sliding_on_grid(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *floor = grid(6, 1.0f);
    Shape *crate = box::create({.half_extents = {0.3f, 0.3f, 0.3f}, .convex_radius = 0.0f});
    const Float3 slide = {1, 0, 0};

    // active edges with no movement hint (the default), with the slide as the hint, and off
    const collide_shape::CollideShapeSettings no_hint = collide_shape::default_settings();
    collide_shape::CollideShapeSettings hinted = no_hint;
    hinted.active_edge_movement_direction = vec3f::load(slide);
    collide_shape::CollideShapeSettings every_edge = no_hint;
    every_edge.collide_only_with_active_edges = false;
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // the box 0.05 into the grid, slid along x across cell edges and diagonals. where it overlaps a triangle by
    // a thin sliver the shortest way out is sideways through that triangle's inner edge: a ghost. active edges
    // give those contacts the face normal; with the slide as the hint they keep the edge normal only where it
    // points away from the motion (jolt's fix_normal), so nothing pushes against the slide
    uint32_t hits = 0;
    bool all_face = true;
    bool none_against_hinted = true;
    bool ghost_against = false;
    for (uint32_t step = 0; step <= 100; step++) {
        const Float3 position = {1.0f + 0.02f * (float)step, 0.25f, 2.43f};
        collide_shape::reset(collector);
        collide(collector, no_hint, crate, pose(position), floor, pose({0, 0, 0}));
        hits += collector.hits.length;
        for (uint32_t i = 0; i < collector.hits.length; i++) all_face &= vec3f::y(unit(collector.hits[i].penetration_axis)) < -0.9999f;

        collide_shape::reset(collector);
        collide(collector, hinted, crate, pose(position), floor, pose({0, 0, 0}));
        for (uint32_t i = 0; i < collector.hits.length; i++) none_against_hinted &= vec3f::dot(unit(collector.hits[i].penetration_axis), vec3f::load(slide)) <= 1e-4f;

        collide_shape::reset(collector);
        collide(collector, every_edge, crate, pose(position), floor, pose({0, 0, 0}));
        for (uint32_t i = 0; i < collector.hits.length; i++) ghost_against |= vec3f::dot(unit(collector.hits[i].penetration_axis), vec3f::load(slide)) > 0.5f;
    }
    CHECK(ctx, hits > 100);
    CHECK(ctx, all_face);
    CHECK(ctx, none_against_hinted);
    CHECK(ctx, ghost_against);

    // the same slide as a cast from 0.1 above, dipping 0.4 over 3: it lands after a quarter, and the leading
    // edge then meets the triangles ahead from the side. with no hint every hit's axis goes down into the grid
    cast_shape::CastShapeSettings cast_settings = cast_shape::default_settings();
    cast_shape::CastShapeCollector cast_collector = cast_shape::create_collector(collector::COLLECT_ALL);
    cast(cast_collector, cast_settings, crate, pose({1.0f, 0.4f, 2.43f}), {3, -0.4f, 0}, floor, pose({0, 0, 0}));
    CHECK(ctx, cast_collector.hits.length > 2);
    CHECK_NEAR(ctx, first_fraction(cast_collector), 0.25f, 1e-3f);
    for (uint32_t i = 0; i < cast_collector.hits.length; i++) {
        CHECK(ctx, vec3f::y(unit(cast_collector.hits[i].penetration_axis)) < -0.9999f);
        CHECK(ctx, vec3f::y(cast_collector.hits[i].normal) > 0.9999f);
    }
    // with the slide as the hint: nothing against it
    cast_settings.active_edge_movement_direction = vec3f::load(slide);
    cast_shape::reset(cast_collector);
    cast(cast_collector, cast_settings, crate, pose({1.0f, 0.4f, 2.43f}), {3, -0.4f, 0}, floor, pose({0, 0, 0}));
    CHECK(ctx, cast_collector.hits.length > 2);
    for (uint32_t i = 0; i < cast_collector.hits.length; i++) CHECK(ctx, vec3f::dot(unit(cast_collector.hits[i].penetration_axis), vec3f::load(slide)) <= 1e-4f);
    // the grid turned half way round about y and moved back over [0, 6]^2: the hint is world space, so it is
    // turned into the mesh's space with it and the slide is still unimpeded
    const Transformf turned = pose({6, 0, 6}, stored_axis_angle(vec3f::make(0, 1, 0), scalar::PI));
    cast_shape::reset(cast_collector);
    cast(cast_collector, cast_settings, crate, pose({1.0f, 0.4f, 2.43f}), {3, -0.4f, 0}, floor, turned);
    CHECK(ctx, cast_collector.hits.length > 2);
    for (uint32_t i = 0; i < cast_collector.hits.length; i++) CHECK(ctx, vec3f::dot(unit(cast_collector.hits[i].penetration_axis), vec3f::load(slide)) <= 1e-4f);
    // and without active edges the leading edge's hits push straight back against it
    cast_settings.collide_only_with_active_edges = false;
    cast_shape::reset(cast_collector);
    cast(cast_collector, cast_settings, crate, pose({1.0f, 0.4f, 2.43f}), {3, -0.4f, 0}, floor, pose({0, 0, 0}));
    bool cast_ghost = false;
    for (uint32_t i = 0; i < cast_collector.hits.length; i++) cast_ghost |= vec3f::dot(unit(cast_collector.hits[i].penetration_axis), vec3f::load(slide)) > 0.5f;
    CHECK(ctx, cast_ghost);

    cast_shape::release(cast_collector);
    collide_shape::release(collector);
    box::destroy(crate);
    drop(floor);
}

void test_triangle_mesh_active_edges_ported(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // crashcat's active edge integration tests, their xy grids laid in xz with y up
    Shape *floor = grid(4, 1.0f);
    Shape *ball = sphere::create({.radius = 0.4f});
    cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    settings.active_edge_movement_direction = vec3f::make(0, -1, 0);
    cast_shape::CastShapeCollector closest = cast_shape::create_collector(collector::COLLECT_CLOSEST);

    // straight down onto the grid vertex at (2, 2): the axis is the face normal, a to b down into the grid
    cast(closest, settings, ball, pose({2, 2, 2}), {0, -3, 0}, floor, pose({0, 0, 0}));
    CHECK(ctx, closest.has_hit);
    const cast_shape::CastShapeHit *hit = cast_shape::hit(closest);
    CHECK_NEAR(ctx, hit->fraction, 1.6f / 3.0f, 1e-5f);
    CHECK(ctx, vec3f::y(unit(hit->penetration_axis)) < -0.9999f);
    CHECK(ctx, vec3f::y(hit->normal) > 0.9999f && scalar::abs(vec3f::length(hit->normal) - 1.0f) < 1e-5f);

    // sliding right while dipping: still the face normal
    Shape *strip = grid(5, 1.0f);
    settings.active_edge_movement_direction = vec3f::make(1, 0, 0);
    cast_shape::reset(closest);
    cast(closest, settings, ball, pose({0.5f, 0.5f, 1}), {4, -0.2f, 0}, strip, pose({0, 0, 0}));
    CHECK(ctx, closest.has_hit);
    CHECK(ctx, vec3f::y(unit(cast_shape::hit(closest)->penetration_axis)) < -0.9999f);
    CHECK(ctx, vec3f::y(cast_shape::hit(closest)->normal) > 0.9999f);

    // the feature off: still a hit
    settings.collide_only_with_active_edges = false;
    cast_shape::reset(closest);
    cast(closest, settings, ball, pose({1, 2, 1}), {0, -3, 0}, floor, pose({0, 0, 0}));
    CHECK(ctx, closest.has_hit);

    // missing the mesh entirely
    cast_shape::reset(closest);
    cast(closest, settings, ball, pose({10, 2, 10}), {1, 0, 0}, floor, pose({0, 0, 0}));
    CHECK(ctx, !closest.has_hit);

    // collide on a 2-wide grid over the cell edge at x = 2: the contact pushes straight down
    Shape *coarse = grid(2, 2.0f);
    Shape *big = sphere::create({.radius = 0.5f});
    collide_shape::CollideShapeSettings collide_settings = collide_shape::default_settings();
    collide_settings.active_edge_movement_direction = vec3f::make(0, -1, 0);
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collide(collector, collide_settings, big, pose({2, 0.3f, 1}), coarse, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length > 0);
    for (uint32_t i = 0; i < collector.hits.length; i++) CHECK(ctx, vec3f::y(unit(collector.hits[i].penetration_axis)) < -0.9999f);

    collide_shape::release(collector);
    cast_shape::release(closest);
    sphere::destroy(ball);
    sphere::destroy(big);
    drop(floor);
    drop(strip);
    drop(coarse);
}

/* cast */

void test_triangle_mesh_sphere_cast(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    const cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);
    Shape *down = mesh(TRIANGLE_POSITIONS, TRIANGLE_INDICES_FLIPPED);  // faces -z
    Shape *up = mesh(TRIANGLE_POSITIONS, TRIANGLE_INDICES);            // faces +z
    Shape *ball = sphere::create({.radius = 0.5f, .material_id = 2});

    // head on from z = -2 by 2 into the face: touches with the centre at -0.5, after 0.75
    cast(collector, settings, ball, pose({0.25f, 0.25f, -2}), {0, 0, 2}, down, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const cast_shape::CastShapeHit &hit = collector.hits[0];
    CHECK(ctx, hit.status == cast_shape::COLLIDING);
    CHECK_NEAR(ctx, hit.fraction, 0.75f, 1e-5f);
    CHECK(ctx, near3(hit.point_a, 0.25f, 0.25f, 0, 1e-5f));
    CHECK(ctx, near3(hit.point_b, 0.25f, 0.25f, 0, 1e-5f));
    // the axis points a to b, +z; the normal is its negated unit
    CHECK(ctx, near3(unit(hit.penetration_axis), 0, 0, 1, 1e-5f));
    CHECK(ctx, near3(hit.normal, 0, 0, -1, 1e-5f));
    CHECK(ctx, hit.material_id_a == 2 && hit.material_id_b == -1);
    CHECK(ctx, hit.face_a.num_vertices == 0 && hit.face_b.num_vertices == 0);

    // sideways past it: nothing
    cast_shape::reset(collector);
    cast(collector, settings, ball, pose({0, 0, -2}), {1, 0, 0}, down, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // into the back of the face: culled, unless back faces are on
    cast_shape::reset(collector);
    cast(collector, settings, ball, pose({0.25f, 0.25f, -1}), {0, 0, 1.5f}, up, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);
    cast_shape::CastShapeSettings backfaces = settings;
    backfaces.collide_with_backfaces_triangles = true;
    cast(collector, backfaces, ball, pose({0.25f, 0.25f, -1}), {0, 0, 1.5f}, up, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 1.0f / 3.0f, 1e-5f);
    CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, 0, 1, 1e-5f));

    // already 0.2 through the face and moving deeper: fraction 0, the overlap's depth
    Shape *unit_ball = sphere::create({.radius = 1.0f});
    cast_shape::reset(collector);
    cast(collector, settings, unit_ball, pose({0.3f, 0.3f, -0.3f}), {0, 0, -0.5f}, up, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].fraction == 0.0f);
    CHECK_NEAR(ctx, collector.hits[0].penetration_depth, 0.7f, 1e-5f);
    // and moving away from it
    cast_shape::reset(collector);
    cast(collector, settings, unit_ball, pose({0.3f, 0.3f, -0.4f}), {0, 0, 0.5f}, down, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1 && collector.hits[0].fraction == 0.0f);

    // onto an edge and onto a vertex
    Shape *big_tri = mesh(BIG_TRIANGLE_POSITIONS, TRIANGLE_INDICES);
    Shape *small = sphere::create({.radius = 0.4f});
    cast_shape::reset(collector);
    cast(collector, settings, small, pose({1.5f, 1.5f, 0}), {-1, -1, 0}, big_tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    // the hypotenuse x + y = 2 is sqrt(2) / 2 from (1.5, 1.5); the centre closes at sqrt(2) a unit fraction
    CHECK_NEAR(ctx, collector.hits[0].fraction, (scalar::sqrt(0.5f) - 0.4f) / scalar::sqrt(2.0f), 1e-5f);
    CHECK(ctx, near3(collector.hits[0].point_b, 1, 1, 0, 1e-4f));
    Shape *tiny = sphere::create({.radius = 0.3f});
    cast_shape::reset(collector);
    cast(collector, settings, tiny, pose({-1, -1, 0}), {1.5f, 1.5f, 0}, big_tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, (scalar::sqrt(2.0f) - 0.3f) / scalar::sqrt(4.5f), 1e-5f);
    CHECK(ctx, near3(collector.hits[0].point_b, 0, 0, 0, 1e-5f));

    // two stacked triangles at z = -1 and -2: the closest collector keeps the upper
    const float stacked_positions[18] = {0, 0, -1, 1, 0, -1, 0, 1, -1, 0, 0, -2, 1, 0, -2, 0, 1, -2};
    const uint32_t stacked_indices[6] = {0, 1, 2, 3, 4, 5};
    Shape *stacked = mesh(stacked_positions, stacked_indices);
    cast_shape::CastShapeCollector closest = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    cast(closest, settings, tiny, pose({0.25f, 0.25f, 0}), {0, 0, -3}, stacked, pose({0, 0, 0}));
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, cast_shape::hit(closest)->fraction, 0.7f / 3.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::z(cast_shape::hit(closest)->point_b), -1.0f, 1e-5f);

    cast_shape::release(collector);
    cast_shape::release(closest);
    sphere::destroy(ball);
    sphere::destroy(unit_ball);
    sphere::destroy(small);
    sphere::destroy(tiny);
    drop(down);
    drop(up);
    drop(big_tri);
    drop(stacked);
}

/** a 10 by 10 quad tilted by `angle` about z through the origin, normal (-sin, cos, 0) */
static Shape *ramp(float angle) {
    const float c = scalar::cos(angle), s = scalar::sin(angle);
    const float positions[12] = {-5 * c, -5 * s, -5, 5 * c, 5 * s, -5, 5 * c, 5 * s, 5, -5 * c, -5 * s, 5};
    const uint32_t indices[6] = {0, 2, 1, 0, 3, 2};
    return mesh(positions, indices);
}

void test_triangle_mesh_cast_onto_ramp(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    const float angle = scalar::PI / 6.0f;
    const Float3 n = {-scalar::sin(angle), scalar::cos(angle), 0};
    Shape *slope = ramp(angle);
    const cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CLOSEST);

    // a sphere dropped 8 straight down: it touches when its centre is r from the plane,
    // n.(c0 - t (0, 8, 0)) = r, so t = (n.c0 - r) / (8 cos)
    const float radius = 0.5f;
    const Float3 start = {0.3f, 4, 0.7f};
    Shape *ball = sphere::create({.radius = radius});
    cast(collector, settings, ball, pose(start), {0, -8, 0}, slope, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit);
    const float expected = (vec3f::dot(vec3f::load(n), vec3f::load(start)) - radius) / (8 * n.y);
    const cast_shape::CastShapeHit *hit = cast_shape::hit(collector);
    CHECK_NEAR(ctx, hit->fraction, expected, 1e-5f);
    CHECK(ctx, near3(hit->normal, n, 1e-5f));
    const Vec3f centre_at_hit = vec3f::scale_and_add(vec3f::load(start), vec3f::make(0, -8, 0), expected);
    const Vec3f contact = vec3f::scale_and_add(centre_at_hit, vec3f::load(n), -radius);
    CHECK(ctx, near3(hit->point_b, contact, 1e-4f));
    CHECK_NEAR(ctx, vec3f::dot(vec3f::load(n), hit->point_b), 0.0f, 1e-5f);

    // the same ramp as a flat 10 by 10 quad, turned 30 degrees about z and lifted by 1: the plane through (0, 1, 0)
    const float flat_positions[12] = {-5, 0, -5, 5, 0, -5, 5, 0, 5, -5, 0, 5};
    const uint32_t flat_indices[6] = {0, 2, 1, 0, 3, 2};
    Shape *flat = mesh(flat_positions, flat_indices);
    const Transformf tilted = pose({0, 1, 0}, stored_axis_angle(vec3f::make(0, 0, 1), angle));
    cast_shape::reset(collector);
    cast(collector, settings, ball, pose(start), {0, -8, 0}, flat, tilted);
    CHECK(ctx, collector.has_hit);
    const float lifted = (vec3f::dot(vec3f::load(n), vec3f::make(start.x, start.y - 1, start.z)) - radius) / (8 * n.y);
    CHECK_NEAR(ctx, cast_shape::hit(collector)->fraction, lifted, 1e-5f);
    CHECK(ctx, near3(cast_shape::hit(collector)->normal, n, 1e-5f));

    // a capsule upright through gjk: its lower cap's centre sits half_height below, so
    // n.(c0 - (0, h, 0)) - 8 cos t = r
    Shape *pill = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.25f});
    cast_shape::reset(collector);
    cast(collector, settings, pill, pose(start), {0, -8, 0}, slope, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit);
    const float capsule_expected = (vec3f::dot(vec3f::load(n), vec3f::load(start)) - 0.5f * n.y - 0.25f) / (8 * n.y);
    CHECK_NEAR(ctx, cast_shape::hit(collector)->fraction, capsule_expected, 2e-4f);
    CHECK(ctx, vec3f::dot(cast_shape::hit(collector)->normal, vec3f::load(n)) > 0.9999f);

    cast_shape::release(collector);
    sphere::destroy(ball);
    capsule::destroy(pill);
    drop(slope);
    drop(flat);
}

void test_triangle_mesh_convex_cast(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    const cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);
    Shape *down = mesh(TRIANGLE_POSITIONS, TRIANGLE_INDICES_FLIPPED);
    Shape *cube = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.0f, .material_id = 3});

    // up from z = -2 by 2.5 into the face: its top at -1.5 meets z = 0 after 1.5 of 2.5
    cast(collector, settings, cube, pose({0.25f, 0.25f, -2}), {0, 0, 2.5f}, down, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const cast_shape::CastShapeHit &hit = collector.hits[0];
    CHECK(ctx, hit.status == cast_shape::COLLIDING);
    CHECK_NEAR(ctx, hit.fraction, 0.6f, 1e-3f);
    CHECK(ctx, vec3f::z(unit(hit.penetration_axis)) > 0.9999f);
    CHECK(ctx, near3(hit.normal, 0, 0, -1, 1e-4f));
    CHECK(ctx, hit.material_id_a == 3 && hit.material_id_b == -1);
    CHECK(ctx, triangle_of(down, hit.sub_shape_id_b) == 0);
    CHECK(ctx, hit.face_a.num_vertices == 0 && hit.face_b.num_vertices == 0);

    // sideways and far off: nothing
    cast_shape::reset(collector);
    cast(collector, settings, cube, pose({5, 5, 0}), {1, 0, 0}, down, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // into the back of the face: culled unless asked for
    Shape *up = mesh(TRIANGLE_POSITIONS, TRIANGLE_INDICES);
    cast_shape::reset(collector);
    cast(collector, settings, cube, pose({0.25f, 0.25f, -2}), {0, 0, 2.5f}, up, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);
    cast_shape::CastShapeSettings backfaces = settings;
    backfaces.collide_with_backfaces_triangles = true;
    cast(collector, backfaces, cube, pose({0.25f, 0.25f, -2}), {0, 0, 2.5f}, up, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);

    // already through the face: fraction 0
    Shape *big_tri = mesh(BIG_TRIANGLE_POSITIONS, TRIANGLE_INDICES);
    cast_shape::reset(collector);
    cast(collector, settings, cube, pose({1, 1, 0.2f}), {0, 0, -0.5f}, big_tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1 && collector.hits[0].fraction == 0.0f);

    // faces: the box's top and the triangle at the contact
    cast_shape::CastShapeSettings faces = settings;
    faces.collect_faces = true;
    cast_shape::reset(collector);
    cast(collector, faces, cube, pose({0.25f, 0.25f, -2}), {0, 0, 2.5f}, down, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1 && collector.hits[0].face_a.num_vertices == 4 && collector.hits[0].face_b.num_vertices == 3);
    for (uint32_t v = 0; v < 4; v++) CHECK_NEAR(ctx, vec3f::z(collector.hits[0].face_a.vertices[v]), 0.0f, 2e-3f);
    for (uint32_t v = 0; v < 3; v++) CHECK(ctx, vec3f::z(collector.hits[0].face_b.vertices[v]) == 0.0f);

    // a box scaled by 2 (half extent 1) from y = 5 by -5 onto the quad at y = 1: its bottom at 4 closes 3, fraction
    // 0.6. a's scale is applied once, by its support
    Shape *quad = mesh(QUAD_POSITIONS, QUAD_INDICES);
    cast_shape::reset(collector);
    cast(collector, settings, cube, pose({0.5f, 5, 0.5f}, {0, 0, 0, 1}, {2, 2, 2}), {0, -5, 0}, quad, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length > 0);
    CHECK_NEAR(ctx, first_fraction(collector), 0.6f, 1e-3f);

    cast_shape::release(collector);
    box::destroy(cube);
    drop(down);
    drop(up);
    drop(big_tri);
    drop(quad);
}

void test_triangle_mesh_closest_cast_orders_overlaps(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    cast_shape::CastShapeSettings deepest = cast_shape::default_settings();
    deepest.return_deepest_point = true;

    // two upward quads side by side: the left at y = 0 over x in [-2, 0], the right at y = 0.3 over [0, 2]
    const float left_first[24] = {-2, 0, -1, -2, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0.3f, -1, 0, 0.3f, 1, 2, 0.3f, 1, 2, 0.3f, -1};
    const float right_first[24] = {0, 0.3f, -1, 0, 0.3f, 1, 2, 0.3f, 1, 2, 0.3f, -1, -2, 0, -1, -2, 0, 1, 0, 0, 1, 0, 0, -1};
    const uint32_t indices[12] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    Shape *meshes[2] = {mesh(left_first, indices), mesh(right_first, indices)};
    Shape *ball = sphere::create({.radius = 0.5f});
    Shape *crate = box::create({.half_extents = {0.25f, 0.5f, 0.25f}, .convex_radius = 0.0f});

    for (Shape *m : meshes) {
        // a radius 0.5 sphere at (0, 0.4) starts 0.1 into the left quad and 0.4 into the right: the deeper wins
        cast_shape::CastShapeCollector closest = cast_shape::create_collector(collector::COLLECT_CLOSEST);
        cast(closest, deepest, ball, pose({0, 0.4f, 0}), {0, -1, 0}, m, pose({0, 0, 0}));
        CHECK(ctx, closest.has_hit && cast_shape::hit(closest)->fraction == 0.0f);
        CHECK_NEAR(ctx, cast_shape::hit(closest)->penetration_depth, 0.4f, 1e-4f);
        CHECK_NEAR(ctx, vec3f::y(cast_shape::hit(closest)->point_b), 0.3f, 1e-4f);
        CHECK_NEAR(ctx, closest.early_out_fraction, -0.4f, 1e-4f);

        // a box spanning x in [-0.25, 0.25], y in [-0.1, 0.9]: 0.1 into the left quad (out upwards), 0.25 into the
        // right (out sideways past its x = 0 edge)
        cast_shape::reset(closest);
        cast(closest, deepest, crate, pose({0, 0.4f, 0}), {0, -1, 0}, m, pose({0, 0, 0}));
        CHECK(ctx, closest.has_hit && cast_shape::hit(closest)->fraction == 0.0f);
        CHECK_NEAR(ctx, cast_shape::hit(closest)->penetration_depth, 0.25f, 1e-4f);
        CHECK_NEAR(ctx, vec3f::y(cast_shape::hit(closest)->point_b), 0.3f, 1e-4f);
        cast_shape::release(closest);
    }

    sphere::destroy(ball);
    box::destroy(crate);
    drop(meshes[0]);
    drop(meshes[1]);
}

/* scaled and transformed poses: the mesh handles its scale itself */

void test_triangle_mesh_scaled_poses(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *quad = mesh(QUAD_POSITIONS, QUAD_INDICES);
    Shape *cube = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    Shape *ball = sphere::create({.radius = 1.0f});
    const collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    const Float4 identity = {0, 0, 0, 1};

    // scale (3, 2, 1): the surface at y = 2 over x in [0, 3]. the cube's bottom at 1.8, where the unscaled quad is not
    collide(collector, settings, cube, pose({2.5f, 2.3f, 0.5f}), quad, pose({0, 0, 0}, identity, {3, 2, 1}));
    CHECK(ctx, collector.hits.length > 0);
    CHECK_NEAR(ctx, deepest_penetration(collector), 0.2f, 1e-4f);
    collide_shape::reset(collector);
    collide(collector, settings, cube, pose({2.5f, 1.3f, 0.5f}), quad, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // turned 90 degrees about y after the scale, (x, y, z) -> (z, y, -x): the surface spans x in [0, 1], z in [-3, 0].
    // rotating before scaling would span x in [0, 3], z in [-1, 0] instead
    const Float4 turned = stored_axis_angle(vec3f::make(0, 1, 0), scalar::PI * 0.5f);
    collide_shape::reset(collector);
    collide(collector, settings, cube, pose({0.5f, 2.3f, -2.5f}), quad, pose({0, 0, 0}, turned, {3, 2, 1}));
    CHECK(ctx, collector.hits.length > 0);
    CHECK_NEAR(ctx, deepest_penetration(collector), 0.2f, 1e-4f);
    collide_shape::reset(collector);
    collide(collector, settings, cube, pose({2.5f, 2.3f, -0.5f}), quad, pose({0, 0, 0}, turned, {3, 2, 1}));
    CHECK(ctx, collector.hits.length == 0);

    // mirrored in y: the quad at y = -1 facing -y, a cube below it 0.2 through its front
    collide_shape::reset(collector);
    collide(collector, settings, cube, pose({0.5f, -1.3f, 0.5f}), quad, pose({0, 0, 0}, identity, {1, -1, 1}));
    CHECK(ctx, collector.hits.length > 0);
    CHECK_NEAR(ctx, deepest_penetration(collector), 0.2f, 1e-4f);
    CHECK(ctx, vec3f::y(unit(collector.hits[0].penetration_axis)) > 0.9999f);

    // the sphere's own path, scale (3, 2, 1): a unit sphere's bottom at 1.8
    collide_shape::reset(collector);
    collide(collector, settings, ball, pose({2.5f, 2.8f, 0.5f}), quad, pose({0, 0, 0}, identity, {3, 2, 1}));
    CHECK(ctx, collector.hits.length > 0);
    CHECK_NEAR(ctx, deepest_penetration(collector), 0.2f, 1e-5f);
    // and mirrored, from below
    collide_shape::reset(collector);
    collide(collector, settings, ball, pose({0.5f, -1.8f, 0.5f}), quad, pose({0, 0, 0}, identity, {1, -1, 1}));
    CHECK(ctx, collector.hits.length > 0);
    CHECK_NEAR(ctx, deepest_penetration(collector), 0.2f, 1e-5f);

    // casts onto the (3, 2, 1) quad: the cube from y = 5 by -5, bottom at 4.5, meets y = 2 after 2.5 of 5; the unit
    // sphere from 6, bottom at 5, after 3
    const cast_shape::CastShapeSettings cast_settings = cast_shape::default_settings();
    cast_shape::CastShapeCollector cast_collector = cast_shape::create_collector(collector::COLLECT_ALL);
    cast(cast_collector, cast_settings, cube, pose({2.5f, 5, 0.5f}), {0, -5, 0}, quad, pose({0, 0, 0}, identity, {3, 2, 1}));
    CHECK(ctx, cast_collector.hits.length > 0 && cast_collector.hits[0].status == cast_shape::COLLIDING);
    CHECK_NEAR(ctx, first_fraction(cast_collector), 0.5f, 1e-3f);
    cast_shape::reset(cast_collector);
    cast(cast_collector, cast_settings, ball, pose({2.5f, 6, 0.5f}), {0, -5, 0}, quad, pose({0, 0, 0}, identity, {3, 2, 1}));
    CHECK(ctx, cast_collector.hits.length > 0);
    CHECK_NEAR(ctx, first_fraction(cast_collector), 0.6f, 1e-5f);
    // turned and moved: (x, y, z) -> (z, y, -x) + (10, 0, 0), the surface over x in [10, 11], z in [-3, 0]
    cast_shape::reset(cast_collector);
    cast(cast_collector, cast_settings, ball, pose({10.5f, 6, -2.5f}), {0, -5, 0}, quad, pose({10, 0, 0}, turned, {3, 2, 1}));
    CHECK(ctx, cast_collector.hits.length > 0);
    CHECK_NEAR(ctx, first_fraction(cast_collector), 0.6f, 1e-5f);
    CHECK(ctx, near3(cast_collector.hits[0].normal, 0, 1, 0, 1e-5f));
    cast_shape::reset(cast_collector);
    cast(cast_collector, cast_settings, cube, pose({10.5f, 5, -2.5f}), {0, -5, 0}, quad, pose({10, 0, 0}, turned, {3, 2, 1}));
    CHECK(ctx, cast_collector.hits.length > 0);
    CHECK_NEAR(ctx, first_fraction(cast_collector), 0.5f, 1e-3f);

    // faces under a non-uniform mesh scale: the triangle scaled once, not twice
    collide_shape::CollideShapeSettings faces = settings;
    faces.collect_faces = true;
    Shape *tri = mesh(TRIANGLE_POSITIONS, TRIANGLE_INDICES);
    Shape *sharp = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.0f});
    collide_shape::reset(collector);
    collide(collector, faces, sharp, pose({1, 1, 0.4f}), tri, pose({0, 0, 0}, identity, {2, 3, 1}));
    CHECK(ctx, collector.hits.length > 0 && collector.hits[0].face_b.num_vertices == 3);
    float max_x = -1e30f, max_y = -1e30f;
    for (uint32_t v = 0; v < 3; v++) {
        max_x = scalar::max(max_x, vec3f::x(collector.hits[0].face_b.vertices[v]));
        max_y = scalar::max(max_y, vec3f::y(collector.hits[0].face_b.vertices[v]));
    }
    CHECK_NEAR(ctx, max_x, 2.0f, 1e-5f);
    CHECK_NEAR(ctx, max_y, 3.0f, 1e-5f);

    cast_shape::release(cast_collector);
    collide_shape::release(collector);
    box::destroy(cube);
    box::destroy(sharp);
    sphere::destroy(ball);
    drop(quad);
    drop(tri);
}

/* the reversed pairs */

void test_triangle_mesh_reversed_pairs(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // a 2 by 2 quad at y = 0 facing +y
    const float positions[12] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2};
    const uint32_t indices[6] = {0, 2, 1, 0, 3, 2};
    const int32_t materials[2] = {5, 5};
    Shape *quad = mesh(positions, indices, materials);
    Shape *ball = sphere::create({.radius = 0.5f, .material_id = 1});
    Shape *crate = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.0f, .material_id = 2});

    // the mesh as a against a sphere 0.1 into it: the axis moves the sphere (b) out, up
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collide(collector, collide_shape::default_settings(), quad, pose({0, 0, 0}), ball, pose({1.6f, 0.4f, 0.3f}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-5f);
    CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, 1, 0, 1e-5f));
    CHECK(ctx, near3(collector.hits[0].point_a, 1.6f, 0, 0.3f, 1e-5f));
    CHECK(ctx, near3(collector.hits[0].point_b, 1.6f, -0.1f, 0.3f, 1e-5f));
    CHECK(ctx, collector.hits[0].material_id_a == 5 && collector.hits[0].material_id_b == 1);
    CHECK(ctx, triangle_of(quad, collector.hits[0].sub_shape_id_a) < 2);

    // and against a box through gjk
    collide_shape::reset(collector);
    collide(collector, collide_shape::default_settings(), quad, pose({0, 0, 0}), crate, pose({1, 0.4f, 1}));
    CHECK(ctx, collector.hits.length > 0);
    CHECK_NEAR(ctx, deepest_penetration(collector), 0.1f, 1e-4f);
    for (uint32_t i = 0; i < collector.hits.length; i++) {
        CHECK(ctx, vec3f::y(unit(collector.hits[i].penetration_axis)) > 0.9999f);
        CHECK(ctx, collector.hits[i].material_id_a == 5 && collector.hits[i].material_id_b == 2);
    }

    // the mesh swept down onto a resting body: the same contact as the body swept up into the mesh, seen from
    // the mesh. the quad faces -y here so the body meets its front
    const uint32_t down_indices[6] = {0, 1, 2, 0, 2, 3};
    Shape *ceiling = mesh(positions, down_indices);
    const cast_shape::CastShapeSettings cast_settings = cast_shape::default_settings();
    cast_shape::CastShapeCollector cast_collector = cast_shape::create_collector(collector::COLLECT_ALL);
    // the quad from y = 2 by -3 onto the sphere's top at 0.5: after 1.5
    cast(cast_collector, cast_settings, ceiling, pose({0, 2, 0}), {0, -3, 0}, ball, pose({1.6f, 0, 0.3f}));
    CHECK(ctx, cast_collector.hits.length == 1);
    const cast_shape::CastShapeHit &swept = cast_collector.hits[0];
    CHECK_NEAR(ctx, swept.fraction, 0.5f, 1e-5f);
    // a is the mesh now: the axis points a to b, down; points are where each shape is at the contact
    CHECK(ctx, vec3f::y(unit(swept.penetration_axis)) < -0.9999f);
    CHECK(ctx, near3(swept.normal, 0, 1, 0, 1e-5f));
    CHECK(ctx, near3(swept.point_a, 1.6f, 0.5f, 0.3f, 1e-4f));
    CHECK(ctx, near3(swept.point_b, 1.6f, 0.5f, 0.3f, 1e-4f));
    CHECK(ctx, swept.material_id_a == -1 && swept.material_id_b == 1);
    CHECK(ctx, triangle_of(ceiling, swept.sub_shape_id_a) < 2 && swept.sub_shape_id_b == sub_shape::EMPTY_SUB_SHAPE_ID);

    // and onto the box's top at 0.5, through the convex sweep
    cast_shape::reset(cast_collector);
    cast(cast_collector, cast_settings, ceiling, pose({0, 2, 0}), {0, -3, 0}, crate, pose({1, 0, 1}));
    CHECK(ctx, cast_collector.hits.length > 0);
    CHECK_NEAR(ctx, first_fraction(cast_collector), 0.5f, 1e-3f);
    for (uint32_t i = 0; i < cast_collector.hits.length; i++) {
        CHECK(ctx, vec3f::y(unit(cast_collector.hits[i].penetration_axis)) < -0.9999f);
        CHECK_NEAR(ctx, vec3f::y(cast_collector.hits[i].point_a), 0.5f, 2e-3f);
    }

    cast_shape::release(cast_collector);
    collide_shape::release(collector);
    sphere::destroy(ball);
    box::destroy(crate);
    drop(quad);
    drop(ceiling);
}

void test_triangle_mesh_sphere_cast_inactive_edge_no_hint(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *floor = grid(4, 1.0f);
    Shape *ball = sphere::create({.radius = 0.4f});
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // 0.3 above the grid beside the interior edge at x = 2: the triangles across it are touched only at that
    // edge, so their closest point is off to the side. every edge here is inactive, so each contact takes the
    // face normal, a to b down into the grid (jolt's fix_normal with no movement hint)
    cast(collector, cast_shape::default_settings(), ball, pose({1.9f, 0.3f, 2.5f}), {0, -0.1f, 0}, floor, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length > 1);
    for (uint32_t i = 0; i < collector.hits.length; i++) CHECK(ctx, vec3f::y(unit(collector.hits[i].penetration_axis)) < -0.9999f);

    cast_shape::release(collector);
    sphere::destroy(ball);
    drop(floor);
}

/** the normal a face's winding gives, (v1 - v0) x (v2 - v0) */
static Vec3f winding_normal(const face::Face &face) {
    const Vec3f *v = face.vertices;
    return vec3f::cross(vec3f::subtract(v[1], v[0]), vec3f::subtract(v[2], v[0]));
}

void test_triangle_mesh_mirrored_faces(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // mirrored in x the quad spans x in [-1, 0] and still faces +y; the triangle face each hit reports must too
    Shape *quad = mesh(QUAD_POSITIONS, QUAD_INDICES);
    const Transformf mirrored = pose({0, 0, 0}, {0, 0, 0, 1}, {-1, 1, 1});
    Shape *crate = box::create({.half_extents = {0.25f, 0.25f, 0.25f}, .convex_radius = 0.0f});
    Shape *ball = sphere::create({.radius = 0.25f});

    cast_shape::CastShapeSettings cast_settings = cast_shape::default_settings();
    cast_settings.collect_faces = true;
    cast_shape::CastShapeCollector cast_collector = cast_shape::create_collector(collector::COLLECT_ALL);
    cast(cast_collector, cast_settings, crate, pose({-0.5f, 3, 0.5f}), {0, -3, 0}, quad, mirrored);
    CHECK(ctx, cast_collector.hits.length > 0);
    for (uint32_t i = 0; i < cast_collector.hits.length; i++) {
        CHECK(ctx, cast_collector.hits[i].face_b.num_vertices == 3);
        CHECK(ctx, vec3f::y(unit(winding_normal(cast_collector.hits[i].face_b))) > 0.9999f);
    }

    collide_shape::CollideShapeSettings collide_settings = collide_shape::default_settings();
    collide_settings.collect_faces = true;
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    const Shape *bodies[2] = {crate, ball};
    for (const Shape *body : bodies) {
        collide_shape::reset(collector);
        collide(collector, collide_settings, body, pose({-0.5f, 1.2f, 0.5f}), quad, mirrored);
        CHECK(ctx, collector.hits.length > 0);
        for (uint32_t i = 0; i < collector.hits.length; i++) {
            CHECK(ctx, collector.hits[i].face_b.num_vertices == 3);
            CHECK(ctx, vec3f::y(unit(winding_normal(collector.hits[i].face_b))) > 0.9999f);
        }
    }

    collide_shape::release(collector);
    cast_shape::release(cast_collector);
    box::destroy(crate);
    sphere::destroy(ball);
    drop(quad);
}

void test_triangle_mesh_default_cast_tolerance(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *big_tri = mesh(BIG_TRIANGLE_POSITIONS, TRIANGLE_INDICES);
    Shape *pill = capsule::create({.half_height_of_cylinder = 0.01f, .radius = 0.5f});
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // sideways past the triangle's corner at the origin: the upper cap's centre, (x, -0.29, 0.2), is first within
    // the radius of that vertex when x = -sqrt(0.25 - 0.29^2 - 0.2^2). the default tolerance resolves it to 1e-4
    cast(collector, cast_shape::default_settings(), pill, pose({-2, -0.3f, 0.2f}), {3, 0, 0}, big_tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length > 0);
    const float expected = (2.0f - scalar::sqrt(0.25f - 0.29f * 0.29f - 0.2f * 0.2f)) / 3.0f;
    CHECK_NEAR(ctx, first_fraction(collector), expected, 1e-4f);

    cast_shape::release(collector);
    capsule::destroy(pill);
    drop(big_tri);
}

void test_triangle_mesh_sphere_cast_zero_closest_point_falls_back_to_normal(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // a zero radius sphere cast in the triangle's plane straight onto a vertex: the centre at the hit is the vertex
    // itself, so the closest point is zero and has no direction of its own
    static const float positions[9] = {2, 0, 0, 3, 0, 1, 3, 0, -1};
    Shape *tri = mesh(positions, TRIANGLE_INDICES);
    Shape *point = sphere::create({.radius = 0.0f});
    cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    settings.collide_with_backfaces_triangles = true;
    settings.collide_only_with_active_edges = false;
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    cast(collector, settings, point, pose({0, 0, 0}), {5, 0, 0}, tri, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const cast_shape::CastShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.fraction, 0.4f, 1e-6f);
    CHECK(ctx, near3(hit.point_b, 2, 0, 0, 1e-6f));
    CHECK_NEAR(ctx, vec3f::length(hit.normal), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, scalar::abs(vec3f::y(hit.normal)), 1.0f, 1e-6f);

    cast_shape::release(collector);
    sphere::destroy(point);
    drop(tri);
}

/* material by sub shape id */

void test_triangle_mesh_material(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // the shape's own material is not what a triangle reports
    const int32_t materials[12] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6};
    Shape *painted = mesh(CUBE_POSITIONS, 24, CUBE_INDICES, 36, materials, 9);
    const TriangleMeshData &data = *painted->triangle_mesh.data;
    CHECK(ctx, data.triangle_count == 12);

    // every triangle's id names that triangle's material, whatever order the builder left them in
    uint32_t matched = 0;
    for (uint32_t triangle = 0; triangle < data.triangle_count; triangle++) {
        sub_shape::SubShapeIdBuilder id;
        sub_shape::push_index(id, sub_shape::builder(), triangle, data.triangle_count);
        const int32_t material = shapes::get_shape_material(painted, id.value);
        if (material == triangle_mesh_data::get_material_id(data, triangle) && material >= 1 && material <= 6) matched++;
    }
    CHECK(ctx, matched == 12);

    // the id a ray hit carries looks up the material the hit reported: the top face (y = 1) is 4
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    ray_cast(collector, RAY_FRONT, painted, pose({0, 0, 0}), {0.3f, 3, 0.1f}, {0, -1, 0}, 4);
    CHECK(ctx, collector.has_hit && collector.hits[0].material_id == 4);
    CHECK(ctx, collector.has_hit && shapes::get_shape_material(painted, collector.hits[0].sub_shape_id) == 4);
    cast_ray::release(collector);
    drop(painted);
}
