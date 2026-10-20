#include "tumble/physics/shapes/plane.h"

#include "tumble/instance.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/transformed.h"
#include "test.h"
#include "test_math.h"

using shapes::Shape;

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};
static const Float4 IDENTITY = {0, 0, 0, 1};

/** surface at y = -3, normal +y */
static const Transformf LOWERED_POSE = {{0, -3, 0}, {0, 0, 0, 1}, {1, 1, 1}};
/** turned a quarter about z the normal becomes -x: surface at x = 4, solid towards +x */
static const Transformf ROTATED_POSE = {{4, 0, 0}, {0, 0, 0.70710678f, 0.70710678f}, {1, 1, 1}};

static Float3 face_vertex(const face::Face &face, uint32_t i) { return {vec3f::x(face.vertices[i]), vec3f::y(face.vertices[i]), vec3f::z(face.vertices[i])}; }

/** a y-up plane through its origin, so the pose alone places it */
static Shape *make_ground(int32_t material_id = -1) { return plane::create({.plane = {{0, 1, 0}, 0}, .material_id = material_id}); }

/** the closest ray hit's fraction, or -1 for none */
static float ray_fraction(const Shape *shape, const Transformf &transform, const Float3 &origin, const Float3 &direction, float length,
                          const cast_ray::CastRaySettings &settings = cast_ray::default_settings()) {
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    const cast_ray::Rayf ray = {vec3f::load(origin), unit(vec3f::load(direction)), length};
    shapes::def_of(shape).cast_ray(collector, settings, ray, shape, NO_SUB_SHAPE, transform);
    const float fraction = collector.has_hit ? collector.hits[0].fraction : -1.0f;
    cast_ray::release(collector);
    return fraction;
}

static bool point_inside(const Shape *shape, const Transformf &transform, const Float3 &point) {
    collide_point::CollidePointCollector collector = collide_point::create_collector(collector::COLLECT_ALL);
    shapes::def_of(shape).collide_point(collector, collide_point::default_settings(), vec3f::load(point), shape, NO_SUB_SHAPE, transform);
    const bool inside = collector.hits.length > 0;
    collide_point::release(collector);
    return inside;
}

static void collide(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings, const Shape *a,
                    const Transformf &transform_a, const Shape *b, const Transformf &transform_b) {
    shapes::collide_fn(a->type, b->type)(collector, settings, a, NO_SUB_SHAPE, transform_a, b, NO_SUB_SHAPE, transform_b);
}

static void cast(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *a,
                 const Transformf &transform_a, const Float3 &displacement, const Shape *b, const Transformf &transform_b) {
    shapes::cast_fn(a->type, b->type)(collector, settings, a, NO_SUB_SHAPE, transform_a, vec3f::load(displacement), b, NO_SUB_SHAPE, transform_b);
}

/* the shape */

void test_plane_create(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // the normal is normalised and the constant with it, so the plane stays at y = 0.5; the defaults
    Shape *p = plane::create({.plane = {{0, 2, 0}, -1}, .material_id = 3});
    CHECK(ctx, p->type == shapes::SHAPE_PLANE);
    CHECK(ctx, near3(p->plane.plane.normal, 0, 1, 0, 0.0f));
    CHECK(ctx, p->plane.plane.constant == -0.5f);
    CHECK(ctx, p->plane.half_extent == plane::DEFAULT_PLANE_HALF_EXTENT);
    CHECK(ctx, p->material_id == 3);
    CHECK(ctx, p->volume == 0 && near3(p->center_of_mass, 0, 0, 0, 0.0f));

    // the bounds: the quad at y = 0.5 out to the half extent, and as far again behind it
    CHECK(ctx, near3(p->aabb.min, -1000, -999.5f, -1000, 1e-3f));
    CHECK(ctx, near3(p->aabb.max, 1000, 0.5f, 1000, 1e-3f));
    CHECK(ctx, point_inside(p, pose({0, 0, 0}), {0, 0.4f, 0}));
    CHECK(ctx, !point_inside(p, pose({0, 0, 0}), {0, 0.75f, 0}));

    // no mass and no inertia, no inner radius, the normal everywhere
    mass_properties::MassProperties mp = mass_properties::create();
    mp.mass = 5.0f;
    for (uint32_t i = 0; i < 16; i++) mp.inertia.m[i] = float(i + 1);
    shapes::compute_mass_properties(mp, p);
    CHECK(ctx, mp.mass == 0.0f);
    for (uint32_t i = 0; i < 16; i++) CHECK(ctx, mp.inertia.m[i] == 0.0f);
    CHECK(ctx, shapes::get_shape_inner_radius(p) == 0.0f);
    const Vec3f normal = shapes::get_shape_surface_normal(p, vec3f::make(12, -40, 3), sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, near3(normal, 0, 1, 0, 0.0f));

    // a tilted plane's bounds cover its quad
    Shape *tilted = plane::create({.plane = {{1, 1, 0}, 0}, .half_extent = 10});
    CHECK(ctx, tilted->aabb.max.x > 0 && tilted->aabb.min.x < 0);
    CHECK_NEAR(ctx, tilted->aabb.max.z, 10.0f, 1e-5f);

    plane::destroy(p);
    plane::destroy(tilted);
}

void test_plane_supporting_face(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *p = plane::create({.plane = {{0, 1, 0}, 0}, .half_extent = 10});
    Float4x4 identity;
    mat4f::store(identity, mat4f::identity());

    face::Face face;
    shapes::get_shape_supporting_face(face, p, 0, vec3f::make(0, -1, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < 4; i++) {
        const Float3 v = face_vertex(face, i);
        CHECK(ctx, v.y == 0.0f && scalar::abs(v.x) == 10.0f && scalar::abs(v.z) == 10.0f);
    }
    // wound counter clockwise seen from above (+y)
    const Float3 a = face_vertex(face, 0), b = face_vertex(face, 1), c = face_vertex(face, 2);
    const Vec3f ab = vec3f::subtract(vec3f::load(b), vec3f::load(a));
    const Vec3f ac = vec3f::subtract(vec3f::load(c), vec3f::load(a));
    const Vec3f n = vec3f::cross(ab, ac);
    CHECK(ctx, vec3f::y(n) > 0.0f);

    // a scale that turns it inside out reverses the order
    face::Face mirrored;
    shapes::get_shape_supporting_face(mirrored, p, 0, vec3f::make(0, -1, 0), mat4f::load(identity), vec3f::make(1, 1, -1));
    CHECK(ctx, mirrored.num_vertices == 4);
    for (uint32_t i = 0; i < 4; i++) {
        const Float3 v = face_vertex(face, 3 - i);
        CHECK(ctx, near3(face_vertex(mirrored, i), v.x, v.y, -v.z, 0.0f));
    }

    plane::destroy(p);
}

/* ray */

void test_plane_cast_ray(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *ground = make_ground();
    const Transformf origin = pose({0, 0, 0});

    // from above: 5 of 1000
    CHECK_NEAR(ctx, ray_fraction(ground, origin, {0, 5, 0}, {0, -1, 0}, 1000.0f), 5.0f / 1000.0f, 1e-7f);
    // from inside the solid half space: at once
    CHECK(ctx, ray_fraction(ground, origin, {0, -5, 0}, {0, 1, 0}, 1000.0f) == 0.0f);
    // parallel, and pointing away
    CHECK(ctx, ray_fraction(ground, origin, {0, 5, 0}, {1, 0, 0}, 1000.0f) < 0.0f);
    CHECK(ctx, ray_fraction(ground, origin, {0, 5, 0}, {0, 1, 0}, 1000.0f) < 0.0f);

    // a tilted plane: the distance along the ray is the signed distance over the cosine
    const Vec3f tilted_normal = unit(vec3f::make(0.1f, 1, 0));
    Plane3f tilted_plane = {{}, 1};
    vec3f::store(tilted_plane.normal, tilted_normal);
    Shape *tilted = plane::create({.plane = tilted_plane});
    CHECK_NEAR(ctx, ray_fraction(tilted, origin, {0, 5, 0}, {0, -1, 0}, 1000.0f), (vec3f::y(tilted_normal) * 5 + 1) / vec3f::y(tilted_normal) / 1000.0f,
               1e-7f);

    // the constant: a plane at y = 3
    Shape *raised = plane::create({.plane = {{0, 1, 0}, -3}});
    CHECK_NEAR(ctx, ray_fraction(raised, origin, {0, 10, 0}, {0, -1, 0}, 1000.0f), 7.0f / 1000.0f, 1e-7f);

    // a non-uniform scale leaves a plane through the origin where it is; it moves an offset one
    CHECK_NEAR(ctx, ray_fraction(ground, pose({0, 0, 0}, IDENTITY, {2, 0.5f, 1}), {0, 5, 0}, {0, -1, 0}, 1000.0f), 5.0f / 1000.0f, 1e-7f);
    CHECK_NEAR(ctx, ray_fraction(raised, pose({0, 0, 0}, IDENTITY, {2, 0.5f, 1}), {0, 10, 0}, {0, -1, 0}, 1000.0f), 8.5f / 1000.0f, 1e-6f);

    // at a pose: along +x onto the rotated plane at x = 4, 0.4 of 10
    CHECK_NEAR(ctx, ray_fraction(ground, ROTATED_POSE, {0, 0, 0}, {1, 0, 0}, 10.0f), 0.4f, 1e-5f);

    // back faces: from below with the inside not solid, only the back face mode hits
    cast_ray::CastRaySettings hollow = cast_ray::default_settings();
    hollow.treat_convex_as_solid = false;
    CHECK(ctx, ray_fraction(ground, origin, {0, -5, 0}, {0, 1, 0}, 1000.0f, hollow) < 0.0f);
    hollow.collide_with_backfaces_convex = true;
    CHECK_NEAR(ctx, ray_fraction(ground, origin, {0, -5, 0}, {0, 1, 0}, 1000.0f, hollow), 5.0f / 1000.0f, 1e-7f);

    // the hit carries the plane's material
    Shape *stone = make_ground(6);
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    const cast_ray::Rayf ray = {vec3f::make(0, 5, 0), vec3f::make(0, -1, 0), 10.0f};
    shapes::def_of(stone).cast_ray(collector, cast_ray::default_settings(), ray, stone, NO_SUB_SHAPE, origin);
    CHECK(ctx, collector.has_hit && collector.hits[0].material_id == 6);
    cast_ray::release(collector);

    // from inside the solid with back faces on: the hit at 0, and the surface still tested from behind
    cast_ray::CastRaySettings solid_back_faces = cast_ray::default_settings();
    solid_back_faces.treat_convex_as_solid = true;
    solid_back_faces.collide_with_backfaces_convex = true;
    cast_ray::CastRayCollector all = cast_ray::create_collector(collector::COLLECT_ALL);
    const cast_ray::Rayf from_inside = {vec3f::make(0, -1, 0), vec3f::make(0, 1, 0), 10.0f};
    shapes::def_of(ground).cast_ray(all, solid_back_faces, from_inside, ground, NO_SUB_SHAPE, origin);
    cast_ray::sort(all);
    CHECK(ctx, all.hits.length == 2);
    if (all.hits.length == 2) {
        CHECK(ctx, all.hits[0].fraction == 0.0f);
        CHECK_NEAR(ctx, all.hits[1].fraction, 0.1f, 1e-6f);
    }
    cast_ray::release(all);

    // only an exactly parallel ray misses: one barely tilted still meets a plane it barely clears
    cast_ray::CastRayCollector grazing = cast_ray::create_collector(collector::COLLECT_ALL);
    const cast_ray::Rayf barely_tilted = {vec3f::make(0, 1e-12f, 0), vec3f::make(1, -1e-11f, 0), 1.0f};
    shapes::def_of(ground).cast_ray(grazing, cast_ray::default_settings(), barely_tilted, ground, NO_SUB_SHAPE, origin);
    CHECK(ctx, grazing.hits.length == 1);
    if (grazing.hits.length == 1) CHECK_NEAR(ctx, grazing.hits[0].fraction, 0.1f, 1e-5f);
    cast_ray::release(grazing);

    plane::destroy(ground);
    plane::destroy(tilted);
    plane::destroy(raised);
    plane::destroy(stone);
}

void test_plane_collide_point(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *ground = make_ground();
    CHECK(ctx, point_inside(ground, pose({0, 0, 0}), {0, -0.1f, 0}));
    CHECK(ctx, !point_inside(ground, pose({0, 0, 0}), {0, 0.1f, 0}));
    CHECK(ctx, point_inside(ground, pose({0, 0, 0}), {5000, -0.1f, -5000}));

    // either side of the rotated plane at x = 4
    CHECK(ctx, point_inside(ground, ROTATED_POSE, {4.5f, 0, 0}));
    CHECK(ctx, !point_inside(ground, ROTATED_POSE, {3.5f, 0, 0}));

    // the hit carries the plane's material
    Shape *stone = make_ground(6);
    collide_point::CollidePointCollector collector = collide_point::create_collector(collector::COLLECT_ALL);
    shapes::def_of(stone).collide_point(collector, collide_point::default_settings(), vec3f::make(0, -1, 0), stone, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) CHECK(ctx, collector.hits[0].material_id == 6);
    collide_point::release(collector);

    plane::destroy(ground);
    plane::destroy(stone);
}

/* collide */

void test_plane_collide_sphere_and_box(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *ground = make_ground();
    Shape *s = sphere::create({.radius = 1.0f});
    Shape *b = box::create({.half_extents = {1, 1, 1}});
    const collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // the plane first, through the reversed row: above, just touching, deep
    const Shape *shapes_b[] = {s, b};
    for (const Shape *other : shapes_b) {
        collide_shape::reset(collector);
        collide(collector, settings, ground, pose({0, 0, 0}), other, pose({0, 2.0f, 0}));
        CHECK(ctx, collector.hits.length == 0);

        collide_shape::reset(collector);
        collide(collector, settings, ground, pose({0, 0, 0}), other, pose({0, 0.95f, 0}));
        CHECK(ctx, collector.hits.length == 1);
        if (collector.hits.length == 1) {
            CHECK_NEAR(ctx, collector.hits[0].penetration, 0.05f, 1e-5f);
            CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, 1, 0, 1e-5f));
        }

        collide_shape::reset(collector);
        collide(collector, settings, ground, pose({0, 0, 0}), other, pose({0, -0.5f, 0}));
        CHECK(ctx, collector.hits.length == 1);
        if (collector.hits.length == 1) CHECK_NEAR(ctx, collector.hits[0].penetration, 1.5f, 1e-5f);
    }

    // the sphere first, at poses: 0.5 into a plane at y = -0.5
    collide_shape::reset(collector);
    collide(collector, settings, s, pose({0, 0, 0}), ground, pose({0, -0.5f, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        const collide_shape::CollideShapeHit &hit = collector.hits[0];
        CHECK_NEAR(ctx, hit.penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(hit.penetration_axis, 0, -1, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_a, 0, -1, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_b, 0, -0.5f, 0, 1e-5f));
    }

    // 0.5 into the rotated plane at x = 4
    collide_shape::reset(collector);
    collide(collector, settings, s, pose({3.5f, 0, 0}), ground, ROTATED_POSE);
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        const collide_shape::CollideShapeHit &hit = collector.hits[0];
        CHECK_NEAR(ctx, hit.penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(hit.penetration_axis, 1, 0, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_a, 4.5f, 0, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_b, 4, 0, 0, 1e-5f));
    }

    // clear of it
    collide_shape::reset(collector);
    collide(collector, settings, s, pose({2.5f, 0, 0}), ground, ROTATED_POSE);
    CHECK(ctx, collector.hits.length == 0);

    // a turned box resting on its corner: a sharp unit box turned 45 degrees about z reaches sqrt 2 below its centre
    Shape *sharp = box::create({.half_extents = {1, 1, 1}, .convex_radius = 0.0f});
    collide_shape::reset(collector);
    collide(collector, settings, sharp, pose({0, 1.3f, 0}, {0, 0, 0.38268343f, 0.92387953f}), ground, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) CHECK_NEAR(ctx, collector.hits[0].penetration, scalar::sqrt(2.0f) - 1.3f, 1e-5f);

    collide_shape::release(collector);
    plane::destroy(ground);
    sphere::destroy(s);
    box::destroy(b);
    box::destroy(sharp);
}

void test_plane_collide_faces(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *ground = make_ground(2);
    Shape *b = box::create({.half_extents = {1, 1, 1}, .material_id = 7});
    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.collect_faces = true;
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // a unit box 0.05 into the ground: its bottom face, and a plane face under it sized by the box's diagonal
    collide(collector, settings, b, pose({0, 0.95f, 0}), ground, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        const collide_shape::CollideShapeHit &hit = collector.hits[0];
        CHECK(ctx, hit.material_id_a == 7 && hit.material_id_b == 2);
        CHECK(ctx, hit.face_a.num_vertices == 4 && hit.face_b.num_vertices == 4);
        const float size = scalar::sqrt(12.0f);
        for (uint32_t i = 0; i < 4; i++) {
            CHECK_NEAR(ctx, face_vertex(hit.face_a, i).y, -0.05f, 1e-5f);
            const Float3 v = face_vertex(hit.face_b, i);
            CHECK_NEAR(ctx, v.y, 0.0f, 1e-6f);
            CHECK_NEAR(ctx, scalar::abs(v.x), size, 1e-4f);
            CHECK_NEAR(ctx, scalar::abs(v.z), size, 1e-4f);
        }
    }

    collide_shape::release(collector);
    plane::destroy(ground);
    box::destroy(b);
}

/* cast */

void test_plane_cast_sphere_at_a_pose(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *ground = make_ground();
    Shape *s = sphere::create({.radius = 1.0f});
    const cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CLOSEST);

    // dropped onto a plane at y = -3: the bottom travels from -1 to -3, 2 of 10
    cast(collector, settings, s, pose({0, 0, 0}), {0, -10, 0}, ground, LOWERED_POSE);
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) {
        const cast_shape::CastShapeHit &hit = collector.hits[0];
        CHECK(ctx, hit.status == cast_shape::COLLIDING);
        CHECK_NEAR(ctx, hit.fraction, 0.2f, 1e-5f);
        CHECK(ctx, near3(hit.normal, 0, 1, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_a, 0, -3, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_b, 0, -3, 0, 1e-5f));
    }

    // along +x onto the plane turned about z at x = 4: the leading point travels from 1 to 4, 3 of 10
    cast_shape::reset(collector);
    cast(collector, settings, s, pose({0, 0, 0}), {10, 0, 0}, ground, ROTATED_POSE);
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) {
        const cast_shape::CastShapeHit &hit = collector.hits[0];
        CHECK_NEAR(ctx, hit.fraction, 0.3f, 1e-5f);
        CHECK(ctx, near3(hit.normal, -1, 0, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_a, 4, 0, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_b, 4, 0, 0, 1e-5f));
        CHECK_NEAR(ctx, hit.penetration_depth, 0.0f, 1e-5f);
    }

    // starting 0.5 into the turned plane: the depth at fraction 0
    cast_shape::reset(collector);
    cast(collector, settings, s, pose({3.5f, 0, 0}), {1, 0, 0}, ground, ROTATED_POSE);
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) {
        const cast_shape::CastShapeHit &hit = collector.hits[0];
        CHECK(ctx, hit.fraction == 0.0f);
        CHECK_NEAR(ctx, hit.penetration_depth, 0.5f, 1e-5f);
        CHECK(ctx, near3(hit.normal, -1, 0, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_a, 4.5f, 0, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_b, 4, 0, 0, 1e-5f));
    }

    // at the origin: from y = 3 falling 5, contact at y = 1 after 0.4; moving sideways above it never hits
    cast_shape::reset(collector);
    cast(collector, settings, s, pose({0, 3, 0}), {0, -5, 0}, ground, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) CHECK_NEAR(ctx, collector.hits[0].fraction, 0.4f, 1e-5f);
    cast_shape::reset(collector);
    cast(collector, settings, s, pose({0, 2, 0}), {5, 0, 0}, ground, pose({0, 0, 0}));
    CHECK(ctx, !collector.has_hit);

    // moving away while overlapping: a back face, ignored unless asked for
    cast_shape::reset(collector);
    cast(collector, settings, s, pose({0, 0.5f, 0}), {0, 5, 0}, ground, pose({0, 0, 0}));
    CHECK(ctx, !collector.has_hit);
    cast_shape::CastShapeSettings back_faces = settings;
    back_faces.collide_with_backfaces_convex = true;
    cast_shape::reset(collector);
    cast(collector, back_faces, s, pose({0, 0.5f, 0}), {0, 5, 0}, ground, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit && collector.hits[0].fraction == 0.0f);

    // the any collector stops at a starting overlap 0.1 deep
    cast_shape::CastShapeCollector any = cast_shape::create_collector(collector::COLLECT_ANY);
    cast(any, settings, s, pose({0, 0.9f, 0}), {0, -1, 0}, ground, pose({0, 0, 0}));
    CHECK(ctx, any.has_hit && any.hits[0].fraction == 0.0f);
    if (any.has_hit) CHECK_NEAR(ctx, any.hits[0].penetration_depth, 0.1f, 1e-5f);
    CHECK(ctx, cast_shape::should_early_out(any));

    cast_shape::release(any);
    cast_shape::release(collector);
    plane::destroy(ground);
    sphere::destroy(s);
}

void test_plane_cast_box_faces_at_a_pose(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // a unit box at (0, 2, 0) cast +10 x onto the turned plane at x = 4: its +x face travels from 1 to 4, and both faces
    // lie on the plane at the hit
    Shape *ground = make_ground();
    Shape *b = box::create({.half_extents = {1, 1, 1}});
    cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    settings.collect_faces = true;
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    cast(collector, settings, b, pose({0, 2, 0}), {10, 0, 0}, ground, ROTATED_POSE);
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) {
        const cast_shape::CastShapeHit &hit = collector.hits[0];
        CHECK_NEAR(ctx, hit.fraction, 0.3f, 1e-5f);
        CHECK(ctx, hit.face_a.num_vertices == 4 && hit.face_b.num_vertices == 4);
        for (uint32_t i = 0; i < 4; i++) {
            CHECK_NEAR(ctx, vec3f::x(hit.face_a.vertices[i]), 4.0f, 1e-5f);
            CHECK_NEAR(ctx, vec3f::x(hit.face_b.vertices[i]), 4.0f, 1e-5f);
        }
    }

    cast_shape::release(collector);
    plane::destroy(ground);
    box::destroy(b);
}

void test_plane_reversed_cast(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // the plane swept +5 in y meets the bottom of a radius 0.5 sphere at (0, 3, 0) (y = 2.5) at fraction 0.5; the hit
    // comes back in the plane-first order
    Shape *ground = make_ground(5);
    Shape *s = sphere::create({.radius = 0.5f, .material_id = 3});
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    cast(collector, cast_shape::default_settings(), ground, pose({0, 0, 0}), {0, 5, 0}, s, pose({0, 3, 0}));
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) {
        const cast_shape::CastShapeHit &hit = collector.hits[0];
        CHECK(ctx, hit.status == cast_shape::COLLIDING);
        CHECK_NEAR(ctx, hit.fraction, 0.5f, 1e-5f);
        CHECK_NEAR(ctx, vec3f::y(hit.point_a), 2.5f, 1e-5f);
        CHECK_NEAR(ctx, vec3f::y(hit.point_b), 2.5f, 1e-5f);
        // the normal points from b (the sphere) towards a (the plane)
        CHECK(ctx, near3(hit.normal, 0, -1, 0, 1e-5f));
        CHECK(ctx, vec3f::y(hit.penetration_axis) > 0.0f);
        CHECK(ctx, hit.material_id_a == 5 && hit.material_id_b == 3);
    }

    // and a collide through the transformed wrapper: the sphere moved to (2, 0, 0) sits with its centre on the ground,
    // its radius deep
    Shape *moved = transformed::create({.shape = s, .position = {2, 0, 0}, .quaternion = IDENTITY});
    collide_shape::CollideShapeCollector collide_collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collide(collide_collector, collide_shape::default_settings(), moved, pose({0, 0, 0}), ground, pose({0, 0, 0}));
    CHECK(ctx, collide_collector.hits.length == 1);
    if (collide_collector.hits.length == 1) CHECK_NEAR(ctx, collide_collector.hits[0].penetration, 0.5f, 1e-5f);

    collide_shape::release(collide_collector);
    cast_shape::release(collector);
    transformed::destroy(moved);
    plane::destroy(ground);
    sphere::destroy(s);
}

void test_plane_dispatch_rows(TestCtx &ctx) {
    dispatch::init();
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_SPHERE, shapes::SHAPE_PLANE) == plane::collide_convex_vs_plane);
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_PLANE, shapes::SHAPE_BOX) ==
                   collide_shape::reversed_collide_shape_vs_shape<plane::collide_convex_vs_plane>);
    CHECK(ctx, shapes::cast_fn(shapes::SHAPE_CONVEX_HULL, shapes::SHAPE_PLANE) == plane::cast_convex_vs_plane);
    CHECK(ctx, shapes::cast_fn(shapes::SHAPE_PLANE, shapes::SHAPE_CYLINDER) ==
                   cast_shape::reversed_cast_shape_vs_shape<plane::cast_convex_vs_plane>);
    // two planes, and a plane against a mesh, have nothing
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_PLANE, shapes::SHAPE_PLANE) == nullptr);
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_PLANE, shapes::SHAPE_TRIANGLE_MESH) == nullptr);
}

/* material by sub shape id */

void test_plane_material(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    Shape *painted = plane::create({.material_id = 4});
    Shape *plain = plane::create({});
    CHECK(ctx, shapes::get_shape_material(painted, sub_shape::EMPTY_SUB_SHAPE_ID) == 4);
    CHECK(ctx, shapes::get_shape_material(plain, sub_shape::EMPTY_SUB_SHAPE_ID) == -1);
    plane::destroy(painted);
    plane::destroy(plain);
}
