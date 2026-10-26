#include "tumble/physics/shapes/transformed.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"


static const SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};
static const Float4 IDENTITY = {0, 0, 0, 1};

static Float3 face_vertex(const SupportingFace &face, uint32_t i) { return {vec3f::x(face.vertices[i]), vec3f::y(face.vertices[i]), vec3f::z(face.vertices[i])}; }

static void collide(CollideShapeCollector &collector, const Shape *a, const Transformf &transform_a, const Shape *b,
                    const Transformf &transform_b) {
    shapes::collide_fn(a->type, b->type)(collector, collide_shape::default_settings(), a, NO_SUB_SHAPE, transform_a, b, NO_SUB_SHAPE,
                                         transform_b);
}

static void cast(CastShapeCollector &collector, const Shape *a, const Transformf &transform_a, const Float3 &displacement,
                 const Shape *b, const Transformf &transform_b) {
    shapes::cast_fn(a->type, b->type)(collector, cast_shape::default_settings(), a, NO_SUB_SHAPE, transform_a, vec3f::load(displacement), b,
                                      NO_SUB_SHAPE, transform_b);
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

/* creation, bounds, centre of mass */

void test_transformed_create_and_aabb(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> t = transformed::create({.shape = s, .position = {5, 3, 2}, .quaternion = IDENTITY});

    CHECK(ctx, t->type == ShapeType::TRANSFORMED);
    CHECK(ctx, shapes::as<TransformedShape>(t).shape == s);
    CHECK(ctx, shapes::as<TransformedShape>(t).position.x == 5 && shapes::as<TransformedShape>(t).quaternion.w == 1);
    CHECK(ctx, t->volume == s->volume);

    // the sphere's [-1, 1] moved by (5, 3, 2)
    CHECK(ctx, t->aabb.min.x == 4 && t->aabb.min.y == 2 && t->aabb.min.z == 1);
    CHECK(ctx, t->aabb.max.x == 6 && t->aabb.max.y == 4 && t->aabb.max.z == 3);

    // a box turned a quarter about z swaps its x and y extents
    Ref<Shape> b = box::create({.half_extents = {1, 2, 3}});
    Ref<Shape> turned = transformed::create({.shape = b, .position = {10, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});
    CHECK(ctx, near3(turned->aabb.min, 8, -1, -3, 1e-5f));
    CHECK(ctx, near3(turned->aabb.max, 12, 1, 3, 1e-5f));

    t = nullptr;
    turned = nullptr;
    s = nullptr;
    b = nullptr;
}

void test_transformed_center_of_mass(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 1.0f});

    // at the translation
    Ref<Shape> t = transformed::create({.shape = s, .position = {5, 3, 2}, .quaternion = IDENTITY});
    CHECK(ctx, near3(t->center_of_mass, 5, 3, 2, 1e-6f));

    // the sphere's centre at the origin rotated stays there, plus the translation
    Ref<Shape> r = transformed::create({.shape = s, .position = {1, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});
    CHECK(ctx, near3(r->center_of_mass, 1, 0, 0, 1e-6f));

    // an inner shape with its own centre of mass: rotated, then translated
    Ref<Shape> inner = transformed::create({.shape = s, .position = {1, 0, 0}, .quaternion = IDENTITY});
    Ref<Shape> outer = transformed::create({.shape = inner, .position = {0, 0, 5}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});
    CHECK(ctx, near3(outer->center_of_mass, 0, 1, 5, 1e-6f));

    t = nullptr;
    r = nullptr;
    outer = nullptr;
    inner = nullptr;
    s = nullptr;
}

void test_transformed_mass_properties(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> b = box::create({.half_extents = {1, 2, 3}});
    Ref<Shape> t = transformed::create({.shape = b, .position = {7, 8, 9}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});

    MassProperties inner, outer;
    shapes::compute_mass_properties(inner, b);
    shapes::compute_mass_properties(outer, t);

    // same mass; a quarter turn about z swaps ixx and iyy, the translation leaves the inertia about the centre of mass alone
    CHECK_NEAR(ctx, outer.mass, inner.mass, 1e-2f);
    CHECK_NEAR(ctx, outer.inertia.m[0], inner.inertia.m[5], 1.0f);
    CHECK_NEAR(ctx, outer.inertia.m[5], inner.inertia.m[0], 1.0f);
    CHECK_NEAR(ctx, outer.inertia.m[10], inner.inertia.m[10], 1.0f);
    CHECK_NEAR(ctx, outer.inertia.m[4], 0.0f, 1e-1f);

    t = nullptr;
    b = nullptr;
}

void test_transformed_inner_radius_and_leaf(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 7.0f});
    Ref<Shape> t = transformed::create({.shape = s, .position = {10, 20, 30}, .quaternion = IDENTITY});
    CHECK(ctx, shapes::get_shape_inner_radius(t) == 7.0f);

    // the leaf is the inner shape, with the id untouched (a decorator consumes no bits)
    GetLeafShapeResult leaf;
    shapes::get_leaf_shape(leaf, t, 0x5u);
    CHECK(ctx, leaf.shape == s && leaf.remainder == 0x5u);

    t = nullptr;
    s = nullptr;
}

/* surface normal */

void test_transformed_surface_normal(TestCtx &ctx) {
    dispatch::init();
    Float3 normal;

    // translated box: outside its +x face in the parent's space
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> t = transformed::create({.shape = b, .position = {3, 2, 1}, .quaternion = IDENTITY});
    vec3f::store(normal, shapes::get_shape_surface_normal(t, vec3f::make(4.1f, 2, 1), sub_shape::EMPTY_SUB_SHAPE_ID));
    CHECK(ctx, near3(normal, 1, 0, 0, 1e-6f));

    // a quarter turn about z: the +x face becomes the +y face
    Ref<Shape> long_box = box::create({.half_extents = {2, 1, 1}});
    Ref<Shape> r = transformed::create({.shape = long_box, .position = {0, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});
    vec3f::store(normal, shapes::get_shape_surface_normal(r, vec3f::make(0, 2.1f, 0), sub_shape::EMPTY_SUB_SHAPE_ID));
    CHECK(ctx, near3(normal, 0, 1, 0, 1e-5f));

    // nested: the outer turns a quarter about z, the inner turns the box a quarter about x. the box's +y face centre
    // (0, 2, 0) goes to (0, 0, 2) under x, which z leaves alone; each level rotates the normal by its own rotation only
    Ref<Shape> b123 = box::create({.half_extents = {1, 2, 3}});
    Ref<Shape> inner = transformed::create({.shape = b123, .position = {0, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(1, 0, 0), scalar::PI * 0.5f)});
    Ref<Shape> outer = transformed::create({.shape = inner, .position = {0, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});
    vec3f::store(normal, shapes::get_shape_surface_normal(outer, vec3f::make(0, 0, 2), sub_shape::EMPTY_SUB_SHAPE_ID));
    CHECK(ctx, near3(normal, 0, 0, 1, 1e-6f));

    t = nullptr;
    r = nullptr;
    outer = nullptr;
    inner = nullptr;
    b = nullptr;
    long_box = nullptr;
    b123 = nullptr;
}

/* supporting face, the values pinned */

void test_transformed_supporting_face(TestCtx &ctx) {
    dispatch::init();
    Float4x4 identity;
    mat4f::store(identity, mat4f::identity());
    SupportingFace face;

    // translated box: the face opposite the direction, x = -1 + 5
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> t = transformed::create({.shape = b, .position = {5, 3, -2}, .quaternion = IDENTITY});
    shapes::get_shape_supporting_face(face, t, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < face.num_vertices; i++) {
        const Float3 v = face_vertex(face, i);
        CHECK_NEAR(ctx, v.x, 4.0f, 1e-5f);
        CHECK(ctx, v.y >= 1.99f && v.y <= 4.01f && v.z >= -3.01f && v.z <= -0.99f);
    }

    // a quarter turn about z: world +x is local -y, the local +y face lands at world x = -1, spanning y over [-2, 2]
    Ref<Shape> long_box = box::create({.half_extents = {2, 1, 1}});
    Ref<Shape> r = transformed::create({.shape = long_box, .position = {0, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});
    shapes::get_shape_supporting_face(face, r, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    float min_y = 100, max_y = -100;
    for (uint32_t i = 0; i < face.num_vertices; i++) {
        const Float3 v = face_vertex(face, i);
        CHECK_NEAR(ctx, v.x, -1.0f, 1e-5f);
        min_y = scalar::min(min_y, v.y);
        max_y = scalar::max(max_y, v.y);
    }
    CHECK_NEAR(ctx, min_y, -2.0f, 1e-5f);
    CHECK_NEAR(ctx, max_y, 2.0f, 1e-5f);

    // a sphere has no face
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> ts = transformed::create({.shape = s, .position = {5, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});
    shapes::get_shape_supporting_face(face, ts, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 0);

    // under a world pose and scale: the box's -x face at x = -1 scales to -2, the shape's position (2, 0, 0) scales to
    // (4, 0, 0), so the face sits at x = 2 spanning y, z over [-2, 2]; a quarter turn about z takes (x, y) to (-y, x),
    // and (3, 1, -1) moves it: x over [1, 5], y = 3, z over [-3, 1]
    Ref<Shape> offset_box = transformed::create({.shape = b, .position = {2, 0, 0}, .quaternion = IDENTITY});
    const Mat4f world = mat4f::from_rotation_translation(axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f), vec3f::make(3, 1, -1));
    shapes::get_shape_supporting_face(face, offset_box, 0, vec3f::make(1, 0, 0), world, vec3f::make(2, 2, 2));
    CHECK(ctx, face.num_vertices == 4);
    float min_x = 100, max_x = -100, min_z = 100, max_z = -100;
    for (uint32_t i = 0; i < face.num_vertices; i++) {
        const Float3 v = face_vertex(face, i);
        CHECK_NEAR(ctx, v.y, 3.0f, 1e-5f);
        min_x = scalar::min(min_x, v.x);
        max_x = scalar::max(max_x, v.x);
        min_z = scalar::min(min_z, v.z);
        max_z = scalar::max(max_z, v.z);
    }
    CHECK_NEAR(ctx, min_x, 1.0f, 1e-5f);
    CHECK_NEAR(ctx, max_x, 5.0f, 1e-5f);
    CHECK_NEAR(ctx, min_z, -3.0f, 1e-5f);
    CHECK_NEAR(ctx, max_z, 1.0f, 1e-5f);

    t = nullptr;
    r = nullptr;
    ts = nullptr;
    offset_box = nullptr;
    b = nullptr;
    long_box = nullptr;
    s = nullptr;
}

void test_transformed_supporting_face_with_scaled(TestCtx &ctx) {
    dispatch::init();
    Float4x4 identity;
    mat4f::store(identity, mat4f::identity());
    SupportingFace face;
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});

    // transformed over scaled: the face at -1 scales to -2, then moves by 5
    Ref<Shape> scaled_box = scaled::create({.shape = b, .scale = {2, 2, 2}});
    Ref<Shape> moved_scaled = transformed::create({.shape = scaled_box, .position = {5, 0, 0}, .quaternion = IDENTITY});
    shapes::get_shape_supporting_face(face, moved_scaled, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < face.num_vertices; i++) CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), 3.0f, 1e-5f);

    // scaled over transformed: the scale applies to the box's position too, as it does to every query, so the box
    // sits at 10 with half extent 2 and its -x face is at 8
    Ref<Shape> moved = transformed::create({.shape = b, .position = {5, 0, 0}, .quaternion = IDENTITY});
    Ref<Shape> scaled_moved = scaled::create({.shape = moved, .scale = {2, 2, 2}});
    shapes::get_shape_supporting_face(face, scaled_moved, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < face.num_vertices; i++) CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), 8.0f, 1e-5f);
    // and the ray agrees: from the origin along +x the face at 8 is 0.8 of 10
    CHECK_NEAR(ctx, ray_fraction(scaled_moved, pose({0, 0, 0}), {0, 0, 0}, {1, 0, 0}, 10.0f), 0.8f, 1e-5f);

    scaled_moved = nullptr;
    moved = nullptr;
    moved_scaled = nullptr;
    scaled_box = nullptr;
    b = nullptr;
}

void test_transformed_sub_shape_transformed_shape(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> b = box::create({.half_extents = {1, 2, 3}});
    const Float4 quarter_z = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f);
    Ref<Shape> t = transformed::create({.shape = b, .position = {1, 0, 0}, .quaternion = quarter_z});

    // from (1, 2, 3) at scale (2, 3, 4): the position scales to (2, 0, 0), the rotation composes, the scale turns into
    // the box's frame
    GetSubShapeTransformedShapeResult out = {};
    out.position = vec3f::make(1, 2, 3);
    out.rotation = quatf::load(IDENTITY);
    out.scale = vec3f::make(2, 3, 4);
    shapes::get_sub_shape_transformed_shape(out, t, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, out.shape == b);
    CHECK(ctx, near3(out.position, 3, 2, 3, 1e-5f));
    CHECK(ctx, scalar::abs(quatf::dot(out.rotation, quatf::load(quarter_z))) > 1.0f - 1e-6f);
    CHECK(ctx, near3(out.scale, 3, 2, 4, 1e-5f));

    t = nullptr;
    b = nullptr;
}

/* ray and point */

void test_transformed_cast_ray(TestCtx &ctx) {
    dispatch::init();

    // a sphere moved to x = 5: its surface at 4 is 0.4 of 10
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> ts = transformed::create({.shape = s, .position = {5, 0, 0}, .quaternion = IDENTITY});
    CHECK_NEAR(ctx, ray_fraction(ts, pose({0, 0, 0}), {0, 0, 0}, {1, 0, 0}, 10.0f), 0.4f, 1e-5f);

    // a unit box turned 22.5 degrees about z at x = 5: along its centre line the boundary is 1 / cos(22.5) away
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    const float angle = scalar::PI / 8.0f;
    Ref<Shape> tb = transformed::create({.shape = b, .position = {5, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), angle)});
    CHECK_NEAR(ctx, ray_fraction(tb, pose({0, 0, 0}), {0, 0, 0}, {1, 0, 0}, 10.0f), (5.0f - 1.0f / scalar::cos(angle)) / 10.0f, 1e-5f);

    // the parent pose turns the shape: rotated a quarter about z, the sphere's centre is at (0, 5, 0)
    CHECK_NEAR(ctx, ray_fraction(ts, pose({0, 0, 0}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)), {0, 0, 0}, {0, 1, 0}, 10.0f), 0.4f, 1e-5f);
    CHECK(ctx, ray_fraction(ts, pose({0, 0, 0}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)), {0, 0, 0}, {1, 0, 0}, 10.0f) < 0.0f);

    ts = nullptr;
    tb = nullptr;
    s = nullptr;
    b = nullptr;
}

void test_transformed_collide_point(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> ts = transformed::create({.shape = s, .position = {5, 0, 0}, .quaternion = IDENTITY});

    CHECK(ctx, point_inside(ts, pose({0, 0, 0}), {5.5f, 0, 0}));
    CHECK(ctx, !point_inside(ts, pose({0, 0, 0}), {6.5f, 0, 0}));
    CHECK(ctx, !point_inside(ts, pose({0, 0, 0}), {0, 0, 0}));

    // under a parent scale of 2 the sphere sits at 10 with radius 2
    CHECK(ctx, point_inside(ts, pose({0, 0, 0}, IDENTITY, {2, 2, 2}), {11.5f, 0, 0}));
    CHECK(ctx, !point_inside(ts, pose({0, 0, 0}, IDENTITY, {2, 2, 2}), {12.5f, 0, 0}));

    // under a parent rotation and translation
    CHECK(ctx, point_inside(ts, pose({1, 1, 1}, stored_axis_angle(vec3f::make(0, 1, 0), scalar::PI * 0.5f)), {1, 1, -4}));

    ts = nullptr;
    s = nullptr;
}

/* collide and cast through dispatch */

void test_transformed_box_vs_sphere_collide(TestCtx &ctx) {
    dispatch::init();
    // a (1, 2, 3) box turned a quarter about z at (5, 0, 0) spans x over [3, 7], y over [-1, 1]; a unit sphere at (5, 1.5, 0)
    // is 0.5 into its top face
    Ref<Shape> b = box::create({.half_extents = {1, 2, 3}, .material_id = 4});
    Ref<Shape> t = transformed::create({.shape = b, .position = {5, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});
    Ref<Shape> s = sphere::create({.radius = 1.0f, .material_id = 9});

    CollideShapeCollector collector = collide_shape::create_collector(CollectorKind::ALL);
    collide(collector, t, pose({0, 0, 0}), s, pose({5, 1.5f, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        const CollideShapeHit &hit = collector.hits[0];
        CHECK_NEAR(ctx, hit.penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(unit(hit.penetration_axis), 0, 1, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_a, 5, 1, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_b, 5, 0.5f, 0, 1e-5f));
        CHECK(ctx, shapes::get_shape_material(t, hit.sub_shape_id_a) == 4 && shapes::get_shape_material(s, hit.sub_shape_id_b) == 9);
    }

    // the other way round: the axis flips, the points and materials swap
    collide_shape::reset(collector);
    collide(collector, s, pose({5, 1.5f, 0}), t, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        const CollideShapeHit &hit = collector.hits[0];
        CHECK_NEAR(ctx, hit.penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(unit(hit.penetration_axis), 0, -1, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_a, 5, 0.5f, 0, 1e-5f));
        CHECK(ctx, near3(hit.point_b, 5, 1, 0, 1e-5f));
        CHECK(ctx, shapes::get_shape_material(s, hit.sub_shape_id_a) == 9 && shapes::get_shape_material(t, hit.sub_shape_id_b) == 4);
    }

    // over the x face rather than the y face: the unturned box would reach x = 6, the turned one only to 7 - so 0.5 into
    // the turned box's +x face at 7 means a sphere at 7.5
    collide_shape::reset(collector);
    collide(collector, t, pose({0, 0, 0}), s, pose({7.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 1, 0, 0, 1e-5f));
    }

    // clear of it
    collide_shape::reset(collector);
    collide(collector, t, pose({0, 0, 0}), s, pose({5, 2.5f, 0}));
    CHECK(ctx, collector.hits.length == 0);

    t = nullptr;
    b = nullptr;
    s = nullptr;
}

void test_transformed_cast(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> b = box::create({.half_extents = {1, 2, 3}});
    Ref<Shape> t = transformed::create({.shape = b, .position = {5, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});
    Ref<Shape> s = sphere::create({.radius = 1.0f});

    // a sphere dropped from (5, 5, 0) by 10 meets the turned box's top at y = 1 after 3
    CastShapeCollector collector = cast_shape::create_collector(CollectorKind::CLOSEST);
    cast(collector, s, pose({5, 5, 0}), {0, -10, 0}, t, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) {
        CHECK_NEAR(ctx, collector.hits[0].fraction, 0.3f, 1e-4f);
        CHECK(ctx, near3(collector.hits[0].normal, 0, 1, 0, 1e-4f));
    }

    // the transformed box swept up by 10 into the resting sphere
    cast_shape::reset(collector);
    cast(collector, t, pose({0, 0, 0}), {0, 10, 0}, s, pose({5, 5, 0}));
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) CHECK_NEAR(ctx, collector.hits[0].fraction, 0.3f, 1e-4f);

    // under a parent turned a quarter about z the box sits at (0, 5, 0), extents (1, 2, 3) again; a world displacement of
    // +10 x carries its +x face at 1 to the sphere at (5, 5, 0) after 3. the displacement stays world space
    cast_shape::reset(collector);
    cast(collector, t, pose({0, 0, 0}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)), {10, 0, 0}, s, pose({5, 5, 0}));
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) {
        CHECK_NEAR(ctx, collector.hits[0].fraction, 0.3f, 1e-4f);
        CHECK(ctx, near3(collector.hits[0].point_b, 4, 5, 0, 1e-3f));
    }

    t = nullptr;
    b = nullptr;
    s = nullptr;
}

/* a rotated child under a non-uniform scale */

void test_transformed_scaled_compound_rotated_child(TestCtx &ctx) {
    dispatch::init();
    // a (1, 2, 0.5) box turned a quarter about z inside a transformed child at (1, 0, 0) of a compound, under a
    // (2, 1, 1) scale, the whole moved to (10, 0, 0). the box's frame sees the scale as (1, 2, 1): half extents
    // (1, 4, 0.5), turned to (4, 1, 0.5) about its centre at (10 + 2, 0, 0), so it spans x over [8, 16], y over [-1, 1].
    // passing the parent's scale through unrotated would give (2, 2, 0.5) about 12, x over [10, 14]
    Ref<Shape> b = box::create({.half_extents = {1, 2, 0.5f}, .convex_radius = 0.0f});
    Ref<Shape> turned = transformed::create({.shape = b, .position = {0, 0, 0}, .quaternion = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)});
    Ref<Shape> other = sphere::create({.radius = 0.25f});
    const CompoundShapeSettings::Child children[] = {{{-20, 0, 0}, IDENTITY, other}, {{1, 0, 0}, IDENTITY, turned}};
    Ref<Shape> c = compound::create({.children = children, .num_children = 2});
    Ref<Shape> sc = scaled::create({.shape = c, .scale = {2, 1, 1}});
    Ref<Shape> outer = transformed::create({.shape = sc, .position = {10, 0, 0}, .quaternion = IDENTITY});

    // the bounds, child by child: the turned box's (2, 1, 0.5) about (1, 0, 0), scaled, moved
    CHECK(ctx, near3(outer->aabb.max, 16, 1, 0.5f, 1e-5f));

    // rays onto the -x face and the top face
    CHECK_NEAR(ctx, ray_fraction(outer, pose({0, 0, 0}), {0, 0, 0}, {1, 0, 0}, 20.0f), 0.4f, 1e-5f);
    CHECK_NEAR(ctx, ray_fraction(outer, pose({0, 0, 0}), {15.5f, 5, 0}, {0, -1, 0}, 10.0f), 0.4f, 1e-5f);
    CHECK(ctx, point_inside(outer, pose({0, 0, 0}), {15.5f, 0.5f, 0}));
    CHECK(ctx, !point_inside(outer, pose({0, 0, 0}), {12, 1.5f, 0}));

    // a unit sphere 0.5 past the +x face at 16 and 0.5 into the top at y = 1, through every wrapper's collide row
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    CollideShapeCollector collector = collide_shape::create_collector(CollectorKind::ALL);
    collide(collector, outer, pose({0, 0, 0}), s, pose({16.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-4f);
        CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 1, 0, 0, 1e-4f));
        // the hit names compound child 1
        SubShapeIdBuilder::PopResult popped;
        sub_shape::pop_index(popped, collector.hits[0].sub_shape_id_a, 2);
        CHECK(ctx, popped.value == 1);
        GetLeafShapeResult leaf;
        shapes::get_leaf_shape(leaf, outer, collector.hits[0].sub_shape_id_a);
        CHECK(ctx, leaf.shape == b);
    }
    collide_shape::reset(collector);
    collide(collector, s, pose({12, 1.5f, 0}), outer, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-4f);
        CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, -1, 0, 1e-4f));
    }

    // the supporting face of the leaf: its top face at y = 1 spanning x over [8, 16]
    if (collector.hits.length == 1) {
        Float4x4 identity;
        mat4f::store(identity, mat4f::identity());
        SupportingFace face;
        shapes::get_shape_supporting_face(face, outer, collector.hits[0].sub_shape_id_b, vec3f::make(0, -1, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
        CHECK(ctx, face.num_vertices == 4);
        float min_x = 100, max_x = -100;
        for (uint32_t i = 0; i < face.num_vertices; i++) {
            const Float3 v = face_vertex(face, i);
            CHECK_NEAR(ctx, v.y, 1.0f, 1e-5f);
            min_x = scalar::min(min_x, v.x);
            max_x = scalar::max(max_x, v.x);
        }
        CHECK_NEAR(ctx, min_x, 8.0f, 1e-4f);
        CHECK_NEAR(ctx, max_x, 16.0f, 1e-4f);
    }

    outer = nullptr;
    sc = nullptr;
    c = nullptr;
    turned = nullptr;
    b = nullptr;
    other = nullptr;
    s = nullptr;
}

void test_transformed_dispatch_rows(TestCtx &ctx) {
    dispatch::init();
    // the wrapper peels against every shape but the empty one, both ways
    CHECK(ctx, shapes::collide_fn(ShapeType::TRANSFORMED, ShapeType::SPHERE) == transformed::collide_transformed_vs_shape);
    CHECK(ctx, shapes::collide_fn(ShapeType::SPHERE, ShapeType::TRANSFORMED) == transformed::collide_shape_vs_transformed);
    CHECK(ctx, shapes::cast_fn(ShapeType::TRANSFORMED, ShapeType::TRIANGLE_MESH) == transformed::cast_transformed_vs_shape);
    CHECK(ctx, shapes::cast_fn(ShapeType::PLANE, ShapeType::TRANSFORMED) == transformed::cast_shape_vs_transformed);
    // against itself the second write of the pair wins, b peeled first
    CHECK(ctx, shapes::collide_fn(ShapeType::TRANSFORMED, ShapeType::TRANSFORMED) == transformed::collide_shape_vs_transformed);
    // over the compound's rows, under the static compound's, under a later wrapper's
    CHECK(ctx, shapes::collide_fn(ShapeType::COMPOUND, ShapeType::TRANSFORMED) == transformed::collide_shape_vs_transformed);
    CHECK(ctx, shapes::collide_fn(ShapeType::TRANSFORMED, ShapeType::STATIC_COMPOUND) != transformed::collide_transformed_vs_shape);
    CHECK(ctx, shapes::collide_fn(ShapeType::TRANSFORMED, ShapeType::SCALED) != transformed::collide_transformed_vs_shape);
    CHECK(ctx, shapes::collide_fn(ShapeType::TRANSFORMED, ShapeType::EMPTY) == nullptr);
}

/* material by sub shape id */

void test_transformed_material(TestCtx &ctx) {
    dispatch::init();
    // through the inner shape, sub shape id and all
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}, .material_id = 6});
    Ref<Shape> s = sphere::create({.radius = 1.0f, .material_id = 3});
    const CompoundShapeSettings::Child children[] = {{{0, 0, 0}, IDENTITY, b}, {{3, 0, 0}, IDENTITY, s}};
    Ref<Shape> c = compound::create({.children = children, .num_children = 2});
    Ref<Shape> moved_box = transformed::create({.shape = b, .position = {1, 2, 3}, .quaternion = stored_axis_angle(vec3f::make(0, 1, 0), 0.4f)});
    Ref<Shape> moved_compound = transformed::create({.shape = c, .position = {1, 2, 3}});
    CHECK(ctx, shapes::get_shape_material(moved_box, sub_shape::EMPTY_SUB_SHAPE_ID) == 6);
    SubShapeIdBuilder id;
    sub_shape::push_index(id, sub_shape::builder(), 1, 2);
    CHECK(ctx, shapes::get_shape_material(moved_compound, id.value) == 3);
    moved_compound = nullptr;
    moved_box = nullptr;
    c = nullptr;
    s = nullptr;
    b = nullptr;
}
