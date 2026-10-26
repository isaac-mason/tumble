#include "tumble/physics/shapes/scaled.h"

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


static const SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};
static const Float4 IDENTITY = {0, 0, 0, 1};

static void collide(CollideShapeCollector &collector, const Shape *a, const Transformf &transform_a, const Shape *b,
                    const Transformf &transform_b) {
    shapes::collide_fn(a->type, b->type)(collector, collide_shape::default_settings(), a, NO_SUB_SHAPE, transform_a, b, NO_SUB_SHAPE,
                                         transform_b);
}

/** the closest ray hit's fraction, or -1 for none */
static float ray_fraction(const Shape *shape, const Transformf &transform, const Float3 &origin, const Float3 &direction, float length) {
    CastRayCollector collector = cast_ray::create_collector(CollectorKind::CLOSEST);
    const Rayf ray = {vec3f::load(origin), unit(vec3f::load(direction)), length};
    shapes::def_of(shape).cast_ray(collector, cast_ray::default_settings(), ray, shape, NO_SUB_SHAPE, transform);
    const float fraction = collector.has_hit ? collector.hits[0].fraction : -1.0f;
    return fraction;
}

static bool point_inside(const Shape *shape, const Transformf &transform, const Float3 &point) {
    CollidePointCollector collector = collide_point::create_collector(CollectorKind::ALL);
    shapes::def_of(shape).collide_point(collector, collide_point::default_settings(), vec3f::load(point), shape, NO_SUB_SHAPE, transform);
    const bool inside = collector.hits.length > 0;
    return inside;
}

/* creation, bounds, centre of mass, volume */

void test_scaled_create_and_aabb(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> ss = scaled::create({.shape = s, .scale = {2, 3, 4}});

    CHECK(ctx, ss->type == ShapeType::SCALED);
    CHECK(ctx, shapes::as<ScaledShape>(ss).shape == s);
    CHECK(ctx, shapes::as<ScaledShape>(ss).scale.x == 2 && shapes::as<ScaledShape>(ss).scale.y == 3 && shapes::as<ScaledShape>(ss).scale.z == 4);

    // [-2, -3, -4] to [2, 3, 4]
    CHECK(ctx, ss->aabb.min.x == -2 && ss->aabb.min.y == -3 && ss->aabb.min.z == -4);
    CHECK(ctx, ss->aabb.max.x == 2 && ss->aabb.max.y == 3 && ss->aabb.max.z == 4);

    // a symmetric shape's centre of mass stays at the origin
    CHECK(ctx, ss->center_of_mass.x == 0 && ss->center_of_mass.y == 0 && ss->center_of_mass.z == 0);

    // the volume scales by |sx sy sz|
    CHECK_NEAR(ctx, ss->volume, s->volume * 24.0f, 1e-4f);

    ss = nullptr;
    s = nullptr;
}

void test_scaled_offset_inner_shape(TestCtx &ctx) {
    dispatch::init();
    // an inner shape whose centre of mass and bounds sit off the origin: a unit box moved to (1, 1, 1)
    Ref<Shape> s = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> moved = transformed::create({.shape = s, .position = {1, 1, 1}, .quaternion = IDENTITY});

    // the centre of mass scales with the inner shape's
    Ref<Shape> ss = scaled::create({.shape = moved, .scale = {2, 3, 4}});
    CHECK(ctx, near3(ss->center_of_mass, 2, 3, 4, 1e-6f));
    CHECK(ctx, near3(ss->aabb.min, 0, 0, 0, 1e-6f));
    CHECK(ctx, near3(ss->aabb.max, 4, 6, 8, 1e-6f));

    // a mirroring scale mirrors the bounds: x over [0, 2] goes to [-4, 0]
    Ref<Shape> mirrored = scaled::create({.shape = moved, .scale = {-2, 1, 1}});
    CHECK(ctx, near3(mirrored->aabb.min, -4, 0, 0, 1e-6f));
    CHECK(ctx, near3(mirrored->aabb.max, 0, 2, 2, 1e-6f));
    CHECK(ctx, near3(mirrored->center_of_mass, -2, 1, 1, 1e-6f));
    CHECK_NEAR(ctx, mirrored->volume, s->volume * 2.0f, 1e-4f);

    // the bounds agree with the geometry: a ray along -x from the right meets the mirrored box's face at 0
    CHECK_NEAR(ctx, ray_fraction(mirrored, pose({0, 0, 0}), {5, 1, 1}, {-1, 0, 0}, 10.0f), 0.5f, 1e-5f);

    ss = nullptr;
    mirrored = nullptr;
    moved = nullptr;
    s = nullptr;
}

void test_scaled_mass_properties(TestCtx &ctx) {
    dispatch::init();

    // a unit box scaled (2, 1, 1) is a (2, 1, 1) box
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> sb = scaled::create({.shape = b, .scale = {2, 1, 1}});
    Ref<Shape> wide = box::create({.half_extents = {2, 1, 1}});
    MassProperties scaled_mass, wide_mass;
    shapes::compute_mass_properties(scaled_mass, sb);
    shapes::compute_mass_properties(wide_mass, wide);
    CHECK_NEAR(ctx, scaled_mass.mass, wide_mass.mass, 1e-1f);
    CHECK_NEAR(ctx, scaled_mass.inertia.m[0], wide_mass.inertia.m[0], 1.0f);
    CHECK_NEAR(ctx, scaled_mass.inertia.m[5], wide_mass.inertia.m[5], 1.0f);
    CHECK_NEAR(ctx, scaled_mass.inertia.m[10], wide_mass.inertia.m[10], 1.0f);

    // a sphere scaled by 2: mass x 8, inertia x 32
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> ss = scaled::create({.shape = s, .scale = {2, 2, 2}});
    MassProperties inner, outer;
    shapes::compute_mass_properties(inner, s);
    shapes::compute_mass_properties(outer, ss);
    CHECK_NEAR(ctx, outer.mass, inner.mass * 8.0f, 1e-1f);
    CHECK_NEAR(ctx, outer.inertia.m[0], inner.inertia.m[0] * 32.0f, 1.0f);

    sb = nullptr;
    ss = nullptr;
    b = nullptr;
    wide = nullptr;
    s = nullptr;
}

void test_scaled_inner_radius_and_leaf(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 10.0f});

    // the smallest absolute scale times the inner radius
    Ref<Shape> ss = scaled::create({.shape = s, .scale = {2, 3, 4}});
    CHECK(ctx, shapes::get_shape_inner_radius(ss) == 20.0f);
    Ref<Shape> negative = scaled::create({.shape = s, .scale = {-2, 3, 4}});
    CHECK(ctx, shapes::get_shape_inner_radius(negative) == 20.0f);

    // the leaf is the inner shape, the id untouched
    GetLeafShapeResult leaf;
    shapes::get_leaf_shape(leaf, ss, 0x3u);
    CHECK(ctx, leaf.shape == s && leaf.remainder == 0x3u);

    // the sub shape's transform accumulates the scale
    GetSubShapeTransformedShapeResult out = {};
    out.position = vec3f::make(1, 2, 3);
    out.rotation = quatf::load(IDENTITY);
    out.scale = vec3f::make(1, 2, 3);
    shapes::get_sub_shape_transformed_shape(out, ss, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, out.shape == s);
    CHECK(ctx, near3(out.position, 1, 2, 3, 0.0f));
    CHECK(ctx, near3(out.scale, 2, 6, 12, 0.0f));

    ss = nullptr;
    negative = nullptr;
    s = nullptr;
}

/* surface normal and supporting face */

void test_scaled_surface_normal(TestCtx &ctx) {
    dispatch::init();
    Float3 normal;

    // outside the +x face of a box scaled (2, 3, 1): still +x
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> sb = scaled::create({.shape = b, .scale = {2, 3, 1}});
    vec3f::store(normal, shapes::get_shape_surface_normal(sb, vec3f::make(2.1f, 1, 0), sub_shape::EMPTY_SUB_SHAPE_ID));
    CHECK(ctx, near3(normal, 1, 0, 0, 1e-6f));

    // on a sphere scaled by 2, normalised and outward
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> ss = scaled::create({.shape = s, .scale = {2, 2, 2}});
    vec3f::store(normal, shapes::get_shape_surface_normal(ss, vec3f::make(2, 2, 0), sub_shape::EMPTY_SUB_SHAPE_ID));
    const float r = 1.0f / scalar::sqrt(2.0f);
    CHECK(ctx, near3(normal, r, r, 0, 1e-6f));

    // an ellipsoid (a sphere scaled (2, 1, 1)): at (sqrt 2, 1 / sqrt 2, 0) the normal is the inverse-transpose of the sphere's
    Ref<Shape> ellipsoid = scaled::create({.shape = s, .scale = {2, 1, 1}});
    vec3f::store(normal, shapes::get_shape_surface_normal(ellipsoid, vec3f::make(2.0f * r, r, 0), sub_shape::EMPTY_SUB_SHAPE_ID));
    CHECK(ctx, near3(normal, unit(vec3f::make(r / 2.0f, r, 0)), 1e-6f));

    sb = nullptr;
    ss = nullptr;
    ellipsoid = nullptr;
    b = nullptr;
    s = nullptr;
}

void test_scaled_supporting_face(TestCtx &ctx) {
    dispatch::init();
    Float4x4 identity;
    mat4f::store(identity, mat4f::identity());
    SupportingFace face;

    // uniform: the face at x = -1 scales to -2, y and z to +-2
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> uniform = scaled::create({.shape = b, .scale = {2, 2, 2}});
    shapes::get_shape_supporting_face(face, uniform, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < face.num_vertices; i++) {
        CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), -2.0f, 1e-6f);
        CHECK_NEAR(ctx, scalar::abs(vec3f::y(face.vertices[i])), 2.0f, 1e-6f);
        CHECK_NEAR(ctx, scalar::abs(vec3f::z(face.vertices[i])), 2.0f, 1e-6f);
    }

    // non-uniform
    Ref<Shape> b123 = box::create({.half_extents = {1, 2, 3}});
    Ref<Shape> non_uniform = scaled::create({.shape = b123, .scale = {2, 0.5f, 4}});
    shapes::get_shape_supporting_face(face, non_uniform, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < face.num_vertices; i++) {
        CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), -2.0f, 1e-6f);
        CHECK_NEAR(ctx, scalar::abs(vec3f::y(face.vertices[i])), 1.0f, 1e-6f);
        CHECK_NEAR(ctx, scalar::abs(vec3f::z(face.vertices[i])), 12.0f, 1e-6f);
    }

    // a mirrored box is the same box: the face opposite the direction stays at x = -1
    Ref<Shape> mirrored = scaled::create({.shape = b, .scale = {-1, 1, 1}});
    shapes::get_shape_supporting_face(face, mirrored, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < face.num_vertices; i++) CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), -1.0f, 1e-6f);

    // the shape's scale combines with the world's: x -1 * 2 * 3, y over 2 * 0.5
    shapes::get_shape_supporting_face(face, uniform, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(3, 0.5f, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < face.num_vertices; i++) {
        CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), -6.0f, 1e-5f);
        CHECK_NEAR(ctx, scalar::abs(vec3f::y(face.vertices[i])), 1.0f, 1e-6f);
    }

    // a sphere has none
    Ref<Shape> s = sphere::create({.radius = 2.0f});
    Ref<Shape> ss = scaled::create({.shape = s, .scale = {3, 3, 3}});
    shapes::get_shape_supporting_face(face, ss, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 0);

    uniform = nullptr;
    non_uniform = nullptr;
    mirrored = nullptr;
    ss = nullptr;
    b = nullptr;
    b123 = nullptr;
    s = nullptr;
}

/* ray and point */

void test_scaled_cast_ray(TestCtx &ctx) {
    dispatch::init();

    // a unit sphere scaled by 2 at x = 5: its surface at 3
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> ss = scaled::create({.shape = s, .scale = {2, 2, 2}});
    CHECK_NEAR(ctx, ray_fraction(ss, pose({5, 0, 0}), {0, 0, 0}, {1, 0, 0}, 10.0f), 0.3f, 1e-5f);

    // a unit box scaled (2, 1, 1) at x = 5: its -x face at 3; from above, its top at 1
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> sb = scaled::create({.shape = b, .scale = {2, 1, 1}});
    CHECK_NEAR(ctx, ray_fraction(sb, pose({5, 0, 0}), {0, 0, 0}, {1, 0, 0}, 10.0f), 0.3f, 1e-5f);
    CHECK_NEAR(ctx, ray_fraction(sb, pose({5, 0, 0}), {6.5f, 5, 0}, {0, -1, 0}, 10.0f), 0.4f, 1e-5f);
    CHECK(ctx, ray_fraction(sb, pose({5, 0, 0}), {7.5f, 5, 0}, {0, -1, 0}, 10.0f) < 0.0f);

    // scaled down too small to reach: a box 0.01 across at x = 5 misses a ray straight down at x = 4.9
    Ref<Shape> tiny = scaled::create({.shape = b, .scale = {0.01f, 0.01f, 0.01f}});
    CHECK(ctx, ray_fraction(tiny, pose({5, 0, 0}), {4.9f, 5, 0}, {0, -1, 0}, 10.0f) < 0.0f);
    CHECK_NEAR(ctx, ray_fraction(tiny, pose({5, 0, 0}), {5, 5, 0}, {0, -1, 0}, 10.0f), 0.499f, 1e-5f);

    // nested in a transformed shape moved to 5
    Ref<Shape> moved = transformed::create({.shape = sb, .position = {5, 0, 0}, .quaternion = IDENTITY});
    CHECK_NEAR(ctx, ray_fraction(moved, pose({0, 0, 0}), {0, 0, 0}, {1, 0, 0}, 10.0f), 0.3f, 1e-5f);

    moved = nullptr;
    ss = nullptr;
    sb = nullptr;
    tiny = nullptr;
    s = nullptr;
    b = nullptr;
}

void test_scaled_collide_point(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> ss = scaled::create({.shape = s, .scale = {2, 2, 2}});
    CHECK(ctx, point_inside(ss, pose({0, 0, 0}), {1.9f, 0, 0}));
    CHECK(ctx, !point_inside(ss, pose({0, 0, 0}), {2.1f, 0, 0}));

    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> sb = scaled::create({.shape = b, .scale = {2, 1, 1}});
    CHECK(ctx, point_inside(sb, pose({0, 0, 0}), {1.9f, 0.9f, 0}));
    CHECK(ctx, !point_inside(sb, pose({0, 0, 0}), {2.1f, 0, 0}));
    CHECK(ctx, !point_inside(sb, pose({0, 0, 0}), {0, 1.1f, 0}));

    ss = nullptr;
    sb = nullptr;
    s = nullptr;
    b = nullptr;
}

/* collide and cast through dispatch */

void test_scaled_sphere_and_box_vs_sphere(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    CollideShapeCollector collector = collide_shape::create_collector(CollectorKind::ALL);

    // a unit sphere scaled by 2 against a unit sphere 2.5 away: 0.5 deep, points on each surface
    Ref<Shape> ss = scaled::create({.shape = s, .scale = {2, 2, 2}});
    collide(collector, ss, pose({0, 0, 0}), s, pose({2.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 1, 0, 0, 1e-5f));
        CHECK(ctx, near3(collector.hits[0].point_a, 2, 0, 0, 1e-5f));
        CHECK(ctx, near3(collector.hits[0].point_b, 1.5f, 0, 0, 1e-5f));
    }

    // a unit box scaled (2, 1, 1): 0.5 into its +x face at 2 and into its top at 1
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> sb = scaled::create({.shape = b, .scale = {2, 1, 1}});
    collide_shape::reset(collector);
    collide(collector, sb, pose({0, 0, 0}), s, pose({2.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 1, 0, 0, 1e-5f));
        CHECK(ctx, near3(collector.hits[0].point_a, 2, 0, 0, 1e-5f));
    }
    collide_shape::reset(collector);
    collide(collector, sb, pose({0, 0, 0}), s, pose({1.5f, 1.5f, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, 1, 0, 1e-5f));
    }
    // the sphere first: the same contact seen from the other side
    collide_shape::reset(collector);
    collide(collector, s, pose({1.5f, 1.5f, 0}), sb, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, -1, 0, 1e-5f));
    }
    // past the unscaled box's face but clear of the scaled one's
    collide_shape::reset(collector);
    collide(collector, sb, pose({0, 0, 0}), s, pose({3.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // a sphere swept from -10 by 20 along x meets the scaled box's -x face at -2 after 7
    CastShapeCollector cast_collector = cast_shape::create_collector(CollectorKind::CLOSEST);
    shapes::cast_fn(s->type, sb->type)(cast_collector, cast_shape::default_settings(), s, NO_SUB_SHAPE, pose({-10, 0, 0}), vec3f::make(20, 0, 0), sb,
                                       NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, cast_collector.has_hit);
    if (cast_collector.has_hit) CHECK_NEAR(ctx, cast_collector.hits[0].fraction, 0.35f, 1e-4f);
    // and the scaled box swept into the sphere
    cast_shape::reset(cast_collector);
    shapes::cast_fn(sb->type, s->type)(cast_collector, cast_shape::default_settings(), sb, NO_SUB_SHAPE, pose({0, 0, 0}), vec3f::make(20, 0, 0), s,
                                       NO_SUB_SHAPE, pose({10, 0, 0}));
    CHECK(ctx, cast_collector.has_hit);
    if (cast_collector.has_hit) CHECK_NEAR(ctx, cast_collector.hits[0].fraction, 0.35f, 1e-4f);

    ss = nullptr;
    sb = nullptr;
    s = nullptr;
    b = nullptr;
}

void test_scaled_dispatch_rows(TestCtx &ctx) {
    dispatch::init();
    CHECK(ctx, shapes::collide_fn(ShapeType::SCALED, ShapeType::BOX) == scaled::collide_scaled_vs_shape);
    CHECK(ctx, shapes::collide_fn(ShapeType::BOX, ShapeType::SCALED) == scaled::collide_shape_vs_scaled);
    CHECK(ctx, shapes::cast_fn(ShapeType::SCALED, ShapeType::PLANE) == scaled::cast_scaled_vs_shape);
    CHECK(ctx, shapes::cast_fn(ShapeType::TRIANGLE_MESH, ShapeType::SCALED) == scaled::cast_shape_vs_scaled);
    // a later wrapper than the transformed shape: scaled peels first where the two meet
    CHECK(ctx, shapes::collide_fn(ShapeType::TRANSFORMED, ShapeType::SCALED) == scaled::collide_shape_vs_scaled);
    CHECK(ctx, shapes::collide_fn(ShapeType::SCALED, ShapeType::TRANSFORMED) == scaled::collide_scaled_vs_shape);
    CHECK(ctx, shapes::cast_fn(ShapeType::SCALED, ShapeType::EMPTY) == nullptr);
}

/* material by sub shape id */

void test_scaled_material(TestCtx &ctx) {
    dispatch::init();
    // through the inner shape, sub shape id and all
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}, .material_id = 6});
    Ref<Shape> s = sphere::create({.radius = 1.0f, .material_id = 3});
    Ref<Shape> moved = transformed::create({.shape = s, .position = {3, 0, 0}});
    Ref<Shape> scaled_box = scaled::create({.shape = b, .scale = {2, 3, 4}});
    Ref<Shape> scaled_moved = scaled::create({.shape = moved, .scale = {-1, 2, 2}});
    CHECK(ctx, shapes::get_shape_material(scaled_box, sub_shape::EMPTY_SUB_SHAPE_ID) == 6);
    CHECK(ctx, shapes::get_shape_material(scaled_moved, sub_shape::EMPTY_SUB_SHAPE_ID) == 3);
    scaled_moved = nullptr;
    scaled_box = nullptr;
    moved = nullptr;
    s = nullptr;
    b = nullptr;
}
