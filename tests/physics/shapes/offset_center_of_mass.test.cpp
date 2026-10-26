#include "tumble/physics/shapes/offset_center_of_mass.h"

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"


static const SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};
static const Float4 IDENTITY = {0, 0, 0, 1};

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

/** the inertia `inner` would have about a point `offset` away: I + m (|d|^2 E - d d^T) */
static void check_parallel_axis(TestCtx &ctx, const MassProperties &outer, const MassProperties &inner,
                                const Float3 &offset, float eps) {
    const float d[3] = {offset.x, offset.y, offset.z};
    const float d2 = vec3f::dot(vec3f::load(offset), vec3f::load(offset));
    CHECK_NEAR(ctx, outer.mass, inner.mass, 1e-2f);
    for (int col = 0; col < 3; col++) {
        for (int row = 0; row < 3; row++) {
            const float expected = inner.inertia.m[col * 4 + row] + inner.mass * ((row == col ? d2 : 0.0f) - d[row] * d[col]);
            CHECK_NEAR(ctx, outer.inertia.m[col * 4 + row], expected, eps);
        }
    }
}

/* creation */

void test_offset_center_of_mass_create(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> o = offset_center_of_mass::create({.shape = s, .offset = {0, -2, 0}});

    CHECK(ctx, o->type == ShapeType::OFFSET_CENTER_OF_MASS);
    CHECK(ctx, shapes::as<OffsetCenterOfMassShape>(o).shape == s);
    CHECK(ctx, near3(shapes::as<OffsetCenterOfMassShape>(o).offset, 0, -2, 0, 0.0f));

    // the centre of mass moves; the geometry, so the bounds and volume, does not
    CHECK(ctx, near3(o->center_of_mass, 0, -2, 0, 1e-6f));
    CHECK(ctx, near3(o->aabb.min, -1, -1, -1, 0.0f) && near3(o->aabb.max, 1, 1, 1, 0.0f));
    CHECK(ctx, o->volume == s->volume);

    // a zero offset behaves as the inner shape
    Ref<Shape> zero = offset_center_of_mass::create({.shape = s, .offset = {0, 0, 0}});
    CHECK(ctx, near3(zero->center_of_mass, s->center_of_mass, 0.0f));
    CHECK(ctx, near3(zero->aabb.min, s->aabb.min, 0.0f) && near3(zero->aabb.max, s->aabb.max, 0.0f));

    // nested offsets add; the bounds stay at the origin
    Ref<Shape> first = offset_center_of_mass::create({.shape = s, .offset = {0, 0.5f, 0}});
    Ref<Shape> second = offset_center_of_mass::create({.shape = first, .offset = {0, 0.3f, 0}});
    CHECK_NEAR(ctx, second->center_of_mass.y, 0.8f, 1e-6f);
    CHECK(ctx, second->aabb.min.y == -1 && second->aabb.max.y == 1);

    o = nullptr;
    zero = nullptr;
    second = nullptr;
    first = nullptr;
    s = nullptr;
}

void test_offset_center_of_mass_mass_properties(TestCtx &ctx) {
    dispatch::init();
    MassProperties inner, outer;

    // a unit box offset up by 2: the same mass, ixx and izz grow by m * 4, iyy stays
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> up = offset_center_of_mass::create({.shape = b, .offset = {0, 2, 0}});
    shapes::compute_mass_properties(inner, b);
    shapes::compute_mass_properties(outer, up);
    check_parallel_axis(ctx, outer, inner, {0, 2, 0}, 1.0f);
    CHECK_NEAR(ctx, outer.inertia.m[0], inner.inertia.m[0] + 4.0f * inner.mass, 1.0f);
    CHECK_NEAR(ctx, outer.inertia.m[5], inner.inertia.m[5], 1e-2f);

    // an offset along every axis: the full parallel axis tensor, off-diagonals included
    Ref<Shape> flat = box::create({.half_extents = {2, 1, 0.5f}});
    Ref<Shape> skew = offset_center_of_mass::create({.shape = flat, .offset = {1, -1, 0.5f}});
    shapes::compute_mass_properties(inner, flat);
    shapes::compute_mass_properties(outer, skew);
    check_parallel_axis(ctx, outer, inner, {1, -1, 0.5f}, 1.0f);
    CHECK(ctx, outer.inertia.m[0] > inner.inertia.m[0] && outer.inertia.m[5] > inner.inertia.m[5] && outer.inertia.m[10] > inner.inertia.m[10]);

    // a unit sphere offset down by 2: m = 4/3 pi rho, ixx = izz = (2/5 + 4) m, iyy = 2/5 m
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> low = offset_center_of_mass::create({.shape = s, .offset = {0, -2, 0}});
    shapes::compute_mass_properties(outer, low);
    const float mass = (4.0f / 3.0f) * scalar::PI * shapes::DEFAULT_SHAPE_DENSITY;
    CHECK_NEAR(ctx, outer.mass, mass, 1e-1f);
    CHECK_NEAR(ctx, outer.inertia.m[0], 4.4f * mass, 1.0f);
    CHECK_NEAR(ctx, outer.inertia.m[5], 0.4f * mass, 1.0f);
    CHECK_NEAR(ctx, outer.inertia.m[10], 4.4f * mass, 1.0f);
    CHECK_NEAR(ctx, outer.inertia.m[4], 0.0f, 1e-2f);

    up = nullptr;
    skew = nullptr;
    low = nullptr;
    b = nullptr;
    flat = nullptr;
    s = nullptr;
}

void test_offset_center_of_mass_pass_through(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> o = offset_center_of_mass::create({.shape = s, .offset = {0, -0.5f, 0}});

    // the inner radius, the surface normal, ray and point: the geometry has not moved
    CHECK(ctx, shapes::get_shape_inner_radius(o) == 1.0f);
    const Vec3f normal = shapes::get_shape_surface_normal(o, vec3f::make(0, 2, 0), sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, near3(normal, 0, 1, 0, 1e-6f));

    CastRayCollector ray_collector = cast_ray::create_collector(CollectorKind::CLOSEST);
    const Rayf ray = {vec3f::make(0, 5, 0), vec3f::make(0, -1, 0), 10.0f};
    shapes::def_of(o).cast_ray(ray_collector, cast_ray::default_settings(), ray, o, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, ray_collector.has_hit);
    if (ray_collector.has_hit) CHECK_NEAR(ctx, ray_collector.hits[0].fraction, 0.4f, 1e-4f);

    CollidePointCollector point_collector = collide_point::create_collector(CollectorKind::ALL);
    shapes::def_of(o).collide_point(point_collector, collide_point::default_settings(), vec3f::make(0, 0.9f, 0), o, NO_SUB_SHAPE, pose({0, 0, 0}));
    shapes::def_of(o).collide_point(point_collector, collide_point::default_settings(), vec3f::make(0, -1.1f, 0), o, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, point_collector.hits.length == 1);

    // the leaf and the sub shape's transform are the inner shape's, not the wrapper's
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeSettings::Child children[] = {{{0, 0, 0}, IDENTITY, s}, {{3, 0, 0}, IDENTITY, b}};
    Ref<Shape> c = compound::create({.children = children, .num_children = 2});
    Ref<Shape> oc = offset_center_of_mass::create({.shape = c, .offset = {0, -1, 0}});
    SubShapeIdBuilder child_one;
    sub_shape::push_index(child_one, sub_shape::builder(), 1, 2);
    GetLeafShapeResult leaf;
    shapes::get_leaf_shape(leaf, oc, child_one.value);
    CHECK(ctx, leaf.shape == b);
    GetSubShapeTransformedShapeResult sub = {};
    sub.position = vec3f::make(0, 0, 0);
    sub.rotation = quatf::load(IDENTITY);
    sub.scale = vec3f::make(1, 1, 1);
    shapes::get_sub_shape_transformed_shape(sub, oc, child_one.value);
    CHECK(ctx, sub.shape == b);
    CHECK(ctx, near3(sub.position, 3, 0, 0, 1e-6f));

    o = nullptr;
    oc = nullptr;
    c = nullptr;
    b = nullptr;
    s = nullptr;
}

/* collide and cast through dispatch */

void test_offset_center_of_mass_sphere_vs_box(TestCtx &ctx) {
    dispatch::init();
    // a unit sphere with its centre of mass 2 below: collides where the sphere is, not where the mass is. a unit box at
    // (0, 1.5, 0) reaches down to 0.5, 0.5 into the sphere's top
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> o = offset_center_of_mass::create({.shape = s, .offset = {0, -2, 0}});
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});

    CollideShapeCollector collector = collide_shape::create_collector(CollectorKind::ALL);
    collide(collector, o, pose({0, 0, 0}), b, pose({0, 1.5f, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, 1, 0, 1e-5f));
        CHECK(ctx, near3(collector.hits[0].point_a, 0, 1, 0, 1e-5f));
        CHECK(ctx, near3(collector.hits[0].point_b, 0, 0.5f, 0, 1e-5f));
    }

    collide_shape::reset(collector);
    collide(collector, b, pose({0, 1.5f, 0}), o, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-5f);
        CHECK(ctx, near3(unit(collector.hits[0].penetration_axis), 0, -1, 0, 1e-5f));
    }

    // where the mass is there is nothing: a box around (0, -2, 0) clear of the sphere
    collide_shape::reset(collector);
    collide(collector, o, pose({0, 0, 0}), b, pose({0, -2.5f, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // against a sphere just touching: 0.1 deep, both orders
    collide_shape::reset(collector);
    collide(collector, o, pose({0, 0, 0}), s, pose({0, 1.9f, 0}));
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-5f);
    collide_shape::reset(collector);
    collide(collector, s, pose({0, -1.9f, 0}), o, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);

    // casts both ways: a sphere sweeping from -5 by 10 meets the other at the origin after 3
    CastShapeCollector cast_collector = cast_shape::create_collector(CollectorKind::CLOSEST);
    cast(cast_collector, o, pose({-5, 0, 0}), {10, 0, 0}, s, pose({0, 0, 0}));
    CHECK(ctx, cast_collector.has_hit);
    if (cast_collector.has_hit) CHECK_NEAR(ctx, cast_collector.hits[0].fraction, 0.3f, 1e-4f);
    cast_shape::reset(cast_collector);
    cast(cast_collector, s, pose({-5, 0, 0}), {10, 0, 0}, o, pose({0, 0, 0}));
    CHECK(ctx, cast_collector.has_hit);
    if (cast_collector.has_hit) CHECK_NEAR(ctx, cast_collector.hits[0].fraction, 0.3f, 1e-4f);

    o = nullptr;
    s = nullptr;
    b = nullptr;
}

void test_offset_center_of_mass_dispatch_rows(TestCtx &ctx) {
    dispatch::init();
    CHECK(ctx, shapes::collide_fn(ShapeType::OFFSET_CENTER_OF_MASS, ShapeType::BOX) ==
                   offset_center_of_mass::collide_offset_center_of_mass_vs_shape);
    CHECK(ctx, shapes::collide_fn(ShapeType::BOX, ShapeType::OFFSET_CENTER_OF_MASS) ==
                   offset_center_of_mass::collide_shape_vs_offset_center_of_mass);
    CHECK(ctx, shapes::cast_fn(ShapeType::OFFSET_CENTER_OF_MASS, ShapeType::SCALED) ==
                   offset_center_of_mass::cast_offset_center_of_mass_vs_shape);
    CHECK(ctx, shapes::cast_fn(ShapeType::TRANSFORMED, ShapeType::OFFSET_CENTER_OF_MASS) ==
                   offset_center_of_mass::cast_shape_vs_offset_center_of_mass);
    CHECK(ctx, shapes::collide_fn(ShapeType::EMPTY, ShapeType::OFFSET_CENTER_OF_MASS) == nullptr);
}

/* material by sub shape id */

void test_offset_center_of_mass_material(TestCtx &ctx) {
    dispatch::init();
    // through the inner shape, sub shape id and all
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}, .material_id = 6});
    Ref<Shape> s = sphere::create({.radius = 1.0f, .material_id = 8});
    const CompoundShapeSettings::Child children[] = {{{0, 0, 0}, IDENTITY, b}, {{3, 0, 0}, IDENTITY, s}};
    Ref<Shape> c = compound::create({.children = children, .num_children = 2});
    Ref<Shape> offset_sphere = offset_center_of_mass::create({.shape = s, .offset = {0, -1, 0}});
    Ref<Shape> offset_compound = offset_center_of_mass::create({.shape = c, .offset = {1, 0, 0}});
    CHECK(ctx, shapes::get_shape_material(offset_sphere, sub_shape::EMPTY_SUB_SHAPE_ID) == 8);
    SubShapeIdBuilder id;
    sub_shape::push_index(id, sub_shape::builder(), 0, 2);
    CHECK(ctx, shapes::get_shape_material(offset_compound, id.value) == 6);
    sub_shape::push_index(id, sub_shape::builder(), 1, 2);
    CHECK(ctx, shapes::get_shape_material(offset_compound, id.value) == 8);
    offset_compound = nullptr;
    offset_sphere = nullptr;
    c = nullptr;
    s = nullptr;
    b = nullptr;
}
