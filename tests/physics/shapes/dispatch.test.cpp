#include "tumble/physics/shapes/dispatch.h"

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

using namespace shapes;

static const ShapeType CONVEX_TYPES[] = {ShapeType::SPHERE, ShapeType::BOX, ShapeType::CAPSULE, ShapeType::CYLINDER};
static const ShapeType ALL_TYPES[] = {ShapeType::SPHERE,     ShapeType::BOX,       ShapeType::CAPSULE,    ShapeType::CONVEX_HULL,   ShapeType::TRIANGLE_MESH,
                                      ShapeType::COMPOUND,   ShapeType::TRANSFORMED, ShapeType::SCALED,   ShapeType::EMPTY,         ShapeType::CYLINDER,
                                      ShapeType::OFFSET_CENTER_OF_MASS, ShapeType::PLANE, ShapeType::STATIC_COMPOUND};

static const SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};

static void collide(CollideShapeCollector &collector, const CollideShapeSettings &settings, const Shape *a,
                    const Transformf &transform_a, const Shape *b, const Transformf &transform_b) {
    collide_fn(a->type, b->type)(collector, settings, a, NO_SUB_SHAPE, transform_a, b, NO_SUB_SHAPE, transform_b);
}

static void ray(CastRayCollector &collector, const CastRaySettings &settings, const Float3 &origin, const Float3 &direction,
                float length, const Shape *shape, const Transformf &transform) {
    const Rayf r = {vec3f::load(origin), unit(vec3f::load(direction)), length};
    def_of(shape).cast_ray(collector, settings, r, shape, NO_SUB_SHAPE, transform);
}

static bool point_hits(const Float3 &point, const Shape *shape, const Transformf &transform) {
    CollidePointCollector collector = collide_point::create_collector(CollectorKind::ALL);
    def_of(shape).collide_point(collector, collide_point::default_settings(), vec3f::load(point), shape, NO_SUB_SHAPE, transform);
    const bool hit = collector.hits.length == 1;
    return hit;
}

/* the tables */

void test_dispatch_convex_pairs_wired_both_ways(TestCtx &ctx) {
    dispatch::init();
    for (ShapeType a : CONVEX_TYPES) {
        for (ShapeType b : CONVEX_TYPES) {
            CHECK(ctx, collide_fn(a, b) != nullptr && collide_fn(b, a) != nullptr);
            CHECK(ctx, cast_fn(a, b) != nullptr && cast_fn(b, a) != nullptr);
            // no convex pair casts by anything but the generic sweep
            CHECK(ctx, cast_fn(a, b) == convex::cast_convex_vs_convex);
        }
    }
}

void test_dispatch_specialised_handlers_win(TestCtx &ctx) {
    dispatch::init();
    // the closed-form pairs: if registration order regresses, the generic handler overwrites them
    CHECK(ctx, collide_fn(ShapeType::SPHERE, ShapeType::SPHERE) == sphere::collide_sphere_vs_sphere);
    CHECK(ctx, collide_fn(ShapeType::SPHERE, ShapeType::BOX) == box::collide_sphere_vs_box);
    CHECK(ctx, collide_fn(ShapeType::BOX, ShapeType::SPHERE) == collide_shape::reversed_collide_shape_vs_shape<box::collide_sphere_vs_box>);
    // the pairs without a specialisation resolve to the generic handler
    CHECK(ctx, collide_fn(ShapeType::BOX, ShapeType::BOX) == convex::collide_convex_vs_convex);
    CHECK(ctx, collide_fn(ShapeType::CAPSULE, ShapeType::CYLINDER) == convex::collide_convex_vs_convex);
    CHECK(ctx, collide_fn(ShapeType::SPHERE, ShapeType::CAPSULE) == convex::collide_convex_vs_convex);
    CHECK(ctx, collide_fn(ShapeType::CYLINDER, ShapeType::SPHERE) == convex::collide_convex_vs_convex);
}

void test_dispatch_empty_has_no_pairs(TestCtx &ctx) {
    dispatch::init();
    for (ShapeType t : ALL_TYPES) {
        CHECK(ctx, collide_fn(ShapeType::EMPTY, t) == nullptr && collide_fn(t, ShapeType::EMPTY) == nullptr);
        CHECK(ctx, cast_fn(ShapeType::EMPTY, t) == nullptr && cast_fn(t, ShapeType::EMPTY) == nullptr);
    }
}

void test_dispatch_defs(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 1.0f});
    Ref<Shape> b = box::create({.half_extents = {1, 2, 3}});
    Ref<Shape> c = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f});
    Ref<Shape> y = cylinder::create({.half_height = 1.0f, .radius = 0.5f});
    Ref<Shape> e = empty::create();
    CHECK(ctx, is_convex(s) && is_convex(b) && is_convex(c) && is_convex(y));
    CHECK(ctx, !is_convex(e));

    // ray and point: box and sphere have their own, capsule and cylinder the convex ones
    CHECK(ctx, def_of(s).cast_ray == convex::cast_ray_vs_convex && def_of(s).collide_point == sphere::collide_point_vs_sphere);
    CHECK(ctx, def_of(b).cast_ray == box::cast_ray_vs_box && def_of(b).collide_point == box::collide_point_vs_box);
    CHECK(ctx, def_of(c).cast_ray == convex::cast_ray_vs_convex && def_of(c).collide_point == convex::collide_point_vs_convex);
    CHECK(ctx, def_of(y).cast_ray == convex::cast_ray_vs_convex && def_of(y).collide_point == convex::collide_point_vs_convex);

    // the defs' own functions reach the shapes, the defaults fill the rest
    CHECK(ctx, get_shape_inner_radius(b) == 1.0f && get_shape_inner_radius(c) == 0.5f);
    CHECK(ctx, get_shape_inner_radius(e) == 0.0f);
    MassProperties mp = mass_properties::create();
    compute_mass_properties(mp, s);
    CHECK_NEAR(ctx, mp.mass, (4.0f / 3.0f) * scalar::PI * DEFAULT_SHAPE_DENSITY, 1e-1f);
    compute_mass_properties(mp, e);
    CHECK(ctx, mp.mass == 1.0f);
    GetLeafShapeResult leaf;
    get_leaf_shape(leaf, b, 7);
    CHECK(ctx, leaf.shape == b && leaf.remainder == 7);

    // the empty shape's ray and point are no-ops
    CastRayCollector rays = cast_ray::create_collector(CollectorKind::ALL);
    ray(rays, cast_ray::default_settings(), {0, 0, 0}, {1, 0, 0}, 10.0f, e, pose({0, 0, 0}));
    CHECK(ctx, rays.hits.length == 0);
    CHECK(ctx, !point_hits({0, 0, 0}, e, pose({0, 0, 0})));

    s = nullptr;
    b = nullptr;
    c = nullptr;
    y = nullptr;
    e = nullptr;
}

static void marker(CollideShapeCollector &, const CollideShapeSettings &, const Shape *,
                   const SubShapeIdBuilder &, const Transformf &, const Shape *, const SubShapeIdBuilder &,
                   const Transformf &) {}

void test_dispatch_init_is_idempotent(TestCtx &ctx) {
    dispatch::init();
    // a stray entry is cleared, and a second init leaves the tables as the first did
    set_collide_shape_fn(ShapeType::EMPTY, ShapeType::PLANE, marker);
    set_collide_shape_fn(ShapeType::SPHERE, ShapeType::SPHERE, marker);
    dispatch::init();
    CHECK(ctx, collide_fn(ShapeType::EMPTY, ShapeType::PLANE) == nullptr);
    CHECK(ctx, collide_fn(ShapeType::SPHERE, ShapeType::SPHERE) == sphere::collide_sphere_vs_sphere);
    CHECK(ctx, collide_fn(ShapeType::SPHERE, ShapeType::BOX) == box::collide_sphere_vs_box);
    CHECK(ctx, shape_defs[shape_type_index(ShapeType::PLANE)].set_support == nullptr);
}

/* collisions through the tables */

void test_dispatch_sphere_vs_sphere(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> a = sphere::create({.radius = 1.0f, .material_id = 3});
    Ref<Shape> b = sphere::create({.radius = 0.5f, .material_id = 7});

    // b along an oblique centre line, 1.3 from a: 0.2 overlap
    const Float3 centre_a = {2, 3, 4};
    const Vec3f line = unit(vec3f::make(1, -2, 0.5f));
    const Float3 centre_b = {centre_a.x + vec3f::x(line) * 1.3f, centre_a.y + vec3f::y(line) * 1.3f, centre_a.z + vec3f::z(line) * 1.3f};

    CollideShapeCollector collector = collide_shape::create_collector(CollectorKind::ALL);
    collide(collector, collide_shape::default_settings(), a, pose(centre_a, stored_axis_angle(vec3f::make(1, 1, 0), 0.7f)), b, pose(centre_b));
    CHECK(ctx, collector.hits.length == 1);
    const CollideShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.penetration, 0.2f, 1e-5f);
    CHECK(ctx, near3(hit.penetration_axis, line, 1e-5f));
    // each contact on its own sphere's surface, along the line
    CHECK(ctx, near3(hit.point_a, centre_a.x + vec3f::x(line), centre_a.y + vec3f::y(line), centre_a.z + vec3f::z(line), 1e-5f));
    CHECK(ctx, near3(hit.point_b, centre_b.x - vec3f::x(line) * 0.5f, centre_b.y - vec3f::y(line) * 0.5f, centre_b.z - vec3f::z(line) * 0.5f, 1e-5f));
    CHECK(ctx, shapes::get_shape_material(a, hit.sub_shape_id_a) == 3 && shapes::get_shape_material(b, hit.sub_shape_id_b) == 7);

    // a uniform scale of 2 on b: radius 1, 0.7 overlap
    collide_shape::reset(collector);
    collide(collector, collide_shape::default_settings(), a, pose(centre_a), b, pose(centre_b, {0, 0, 0, 1}, {2, 2, 2}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.7f, 1e-5f);

    // 0.1 apart: nothing, unless the separation allowed reaches it, then a negative penetration
    const Float3 apart = {centre_a.x + vec3f::x(line) * 1.6f, centre_a.y + vec3f::y(line) * 1.6f, centre_a.z + vec3f::z(line) * 1.6f};
    collide_shape::reset(collector);
    collide(collector, collide_shape::default_settings(), a, pose(centre_a), b, pose(apart));
    CHECK(ctx, collector.hits.length == 0);
    CollideShapeSettings settings = collide_shape::default_settings();
    settings.max_separation_distance = 0.2f;
    collide(collector, settings, a, pose(centre_a), b, pose(apart));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, -0.1f, 1e-5f);

    a = nullptr;
    b = nullptr;
}

void test_dispatch_sphere_vs_box(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 0.5f, .material_id = 2});
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}, .material_id = 5});
    CollideShapeSettings settings = collide_shape::default_settings();
    settings.collect_faces = true;
    CollideShapeCollector collector = collide_shape::create_collector(CollectorKind::ALL);

    // above the +y face, 0.1 into it: the face normal moves the sphere out, so the axis (moving the box out) is -y
    collide(collector, settings, s, pose({0.3f, 1.4f, -0.2f}), b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const CollideShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.penetration, 0.1f, 1e-5f);
    CHECK(ctx, near3(hit.penetration_axis, 0, -1, 0, 1e-6f));
    CHECK(ctx, near3(hit.point_a, 0.3f, 0.9f, -0.2f, 1e-5f));
    CHECK(ctx, near3(hit.point_b, 0.3f, 1.0f, -0.2f, 1e-5f));
    CHECK(ctx, shapes::get_shape_material(s, hit.sub_shape_id_a) == 2 && shapes::get_shape_material(b, hit.sub_shape_id_b) == 5);
    // the box's face is its +y face, the sphere has none
    CHECK(ctx, hit.face_a.num_vertices == 0 && hit.face_b.num_vertices == 4);
    for (uint32_t i = 0; i < 4; i++) CHECK_NEAR(ctx, vec3f::y(hit.face_b.vertices[i]), 1.0f, 1e-6f);

    // the box turned a quarter about z and moved: its +y face now faces -x
    collide_shape::reset(collector);
    const Float4 quarter_z = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f);
    collide(collector, settings, s, pose({10 - 1.4f, 0, 0}), b, pose({10, 0, 0}, quarter_z));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-5f);
    CHECK(ctx, near3(collector.hits[0].penetration_axis, 1, 0, 0, 1e-6f));
    CHECK(ctx, near3(collector.hits[0].point_b, 9, 0, 0, 1e-5f));

    // the centre inside the box's core: out through the nearest face, the top, 1 - (0.7 - 0.5) = 0.8 deep
    collide_shape::reset(collector);
    collide(collector, settings, s, pose({0, 0.7f, 0}), b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.8f, 1e-5f);
    CHECK(ctx, near3(collector.hits[0].penetration_axis, 0, -1, 0, 1e-6f));
    CHECK(ctx, near3(collector.hits[0].point_b, 0, 1, 0, 1e-5f));

    // the reverse pair: the same contact with a and b swapped back
    collide_shape::reset(collector);
    collide(collector, settings, b, pose({0, 0, 0}), s, pose({0.3f, 1.4f, -0.2f}));
    CHECK(ctx, collector.hits.length == 1);
    const CollideShapeHit &reversed = collector.hits[0];
    CHECK_NEAR(ctx, reversed.penetration, 0.1f, 1e-5f);
    CHECK(ctx, near3(reversed.penetration_axis, 0, 1, 0, 1e-6f));
    CHECK(ctx, near3(reversed.point_a, 0.3f, 1.0f, -0.2f, 1e-5f));
    CHECK(ctx, near3(reversed.point_b, 0.3f, 0.9f, -0.2f, 1e-5f));
    CHECK(ctx, shapes::get_shape_material(b, reversed.sub_shape_id_a) == 5 && shapes::get_shape_material(s, reversed.sub_shape_id_b) == 2);
    CHECK(ctx, reversed.face_a.num_vertices == 4 && reversed.face_b.num_vertices == 0);

    // the analytic handler agrees with the generic path on a shallow contact
    CollideShapeCollector generic = collide_shape::create_collector(CollectorKind::ALL);
    convex::collide_convex_vs_convex(generic, settings, s, NO_SUB_SHAPE, pose({0.3f, 1.4f, -0.2f}), b, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, generic.hits.length == 1);
    CHECK_NEAR(ctx, generic.hits[0].penetration, 0.1f, 1e-4f);
    CHECK(ctx, vec3f::dot(unit(generic.hits[0].penetration_axis), vec3f::make(0, -1, 0)) > 0.9999f);

    // clear of the box: nothing
    collide_shape::reset(collector);
    collide(collector, settings, s, pose({0, 1.6f, 0}), b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    s = nullptr;
    b = nullptr;
}

void test_dispatch_box_vs_box(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> a = box::create({.half_extents = {1, 1, 1}});
    Ref<Shape> b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    // b 0.25 into a's +y face, past the convex radii so the cores overlap and epa runs, through the generic convex handler
    CollideShapeCollector collector = collide_shape::create_collector(CollectorKind::ALL);
    collide(collector, collide_shape::default_settings(), a, pose({0, 0, 0}), b, pose({0.25f, 1.25f, 0.125f}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.25f, 1e-5f);
    CHECK(ctx, vec3f::dot(unit(collector.hits[0].penetration_axis), vec3f::make(0, 1, 0)) > 0.99999f);
    CHECK_NEAR(ctx, vec3f::y(collector.hits[0].point_a), 1.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::y(collector.hits[0].point_b), 0.75f, 1e-5f);

    a = nullptr;
    b = nullptr;
}

void test_dispatch_cast_sphere_onto_box(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 0.5f});
    Ref<Shape> b = box::create({.half_extents = {1, 1, 1}});

    // down from y = 5 by 10: the sphere meets the +y face with its centre at 1.5, fraction 3.5 / 10
    CastShapeCollector collector = cast_shape::create_collector(CollectorKind::CLOSEST);
    const Transformf from = pose({0.2f, 5, -0.3f});
    cast_fn(ShapeType::SPHERE, ShapeType::BOX)(collector, cast_shape::default_settings(), s, NO_SUB_SHAPE, from, vec3f::make(0, -10, 0), b, NO_SUB_SHAPE,
                                     pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit);
    CHECK(ctx, collector.hits[0].status == CastShapeStatus::COLLIDING);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.35f, 1e-4f);
    CHECK(ctx, near3(collector.hits[0].point_b, 0.2f, 1, -0.3f, 1e-3f));

    s = nullptr;
    b = nullptr;
}

void test_dispatch_ray_vs_box(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> b = box::create({.half_extents = {1, 2, 3}, .material_id = 4});
    const CastRaySettings settings = cast_ray::default_settings();
    CastRayCollector collector = cast_ray::create_collector(CollectorKind::CLOSEST);

    // along +x from x = -5: the face x = -1 at 4 of 10
    ray(collector, settings, {-5, 0.3f, 0.2f}, {1, 0, 0}, 10.0f, b, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.4f, 1e-6f);
    CHECK(ctx, shapes::get_shape_material(b, collector.hits[0].sub_shape_id) == 4);

    // turned a quarter about y the box reaches 3 along x: entry at x = -3, 2 of 10
    cast_ray::reset(collector);
    ray(collector, settings, {-5, 0.3f, 0.2f}, {1, 0, 0}, 10.0f, b, pose({0, 0, 0}, stored_axis_angle(vec3f::make(0, 1, 0), scalar::PI * 0.5f)));
    CHECK(ctx, collector.has_hit);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.2f, 1e-5f);

    // scaled by 2 along x and moved: x from 8 to 12, entry 3 of 10 from x = 5
    cast_ray::reset(collector);
    ray(collector, settings, {5, 0, 0}, {1, 0, 0}, 10.0f, b, pose({10, 0, 0}, {0, 0, 0, 1}, {2, 1, 1}));
    CHECK(ctx, collector.has_hit);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.3f, 1e-6f);

    // an oblique ray: from (-1.5, -4, 0) along (3, 4, 0), 5 long, meets the face y = -2 at (0, -2, 0), half way
    cast_ray::reset(collector);
    ray(collector, settings, {-1.5f, -4, 0}, {3, 4, 0}, 5.0f, b, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.5f, 1e-6f);

    // short of the box, past it, and beside it: misses
    cast_ray::reset(collector);
    ray(collector, settings, {-5, 0, 0}, {1, 0, 0}, 3.0f, b, pose({0, 0, 0}));
    ray(collector, settings, {5, 0, 0}, {1, 0, 0}, 10.0f, b, pose({0, 0, 0}));
    ray(collector, settings, {-5, 2.5f, 0}, {1, 0, 0}, 10.0f, b, pose({0, 0, 0}));
    CHECK(ctx, !collector.has_hit);

    // from inside: fraction 0 when solid, nothing otherwise
    ray(collector, settings, {0, 0, 0}, {1, 0, 0}, 10.0f, b, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit && collector.hits[0].fraction == 0.0f);
    cast_ray::reset(collector);
    CastRaySettings hollow = settings;
    hollow.treat_convex_as_solid = false;
    ray(collector, hollow, {0, 0, 0}, {1, 0, 0}, 10.0f, b, pose({0, 0, 0}));
    CHECK(ctx, !collector.has_hit);

    // the analytic ray agrees with the gjk one
    CastRayCollector gjk = cast_ray::create_collector(CollectorKind::CLOSEST);
    const Rayf oblique = {vec3f::make(-1.5f, -4, 0), unit(vec3f::make(3, 4, 0)), 5.0f};
    convex::cast_ray_vs_convex(gjk, settings, oblique, b, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, gjk.has_hit);
    CHECK_NEAR(ctx, gjk.hits[0].fraction, 0.5f, 1e-3f);

    b = nullptr;
}

void test_dispatch_ray_vs_box_back_faces(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> b = box::create({.half_extents = {1, 2, 3}});
    // scaled by 2 along x and moved: x from 8 to 12
    const Transformf transform = pose({10, 0, 0}, {0, 0, 0, 1}, {2, 1, 1});
    CastRayCollector all = cast_ray::create_collector(CollectorKind::ALL);

    // solid with convex back faces: the entry at 3 of 10 from x = 5 and the exit, the far face x = 12, at 7 of 10
    CastRaySettings convex_back = cast_ray::default_settings();
    convex_back.collide_with_backfaces_convex = true;
    ray(all, convex_back, {5, 0, 0}, {1, 0, 0}, 10.0f, b, transform);
    CHECK(ctx, all.hits.length == 2);
    CHECK_NEAR(ctx, all.hits[0].fraction, 0.3f, 1e-6f);
    CHECK_NEAR(ctx, all.hits[1].fraction, 0.7f, 1e-6f);

    // the exit past the ray's end is not reported
    cast_ray::reset(all);
    ray(all, convex_back, {5, 0, 0}, {1, 0, 0}, 6.0f, b, transform);
    CHECK(ctx, all.hits.length == 1);
    CHECK_NEAR(ctx, all.hits[0].fraction, 0.5f, 1e-6f);

    // hollow, from the centre: the far face at 2 of 10 with convex back faces on
    CastRaySettings hollow = convex_back;
    hollow.treat_convex_as_solid = false;
    cast_ray::reset(all);
    ray(all, hollow, {10, 0, 0}, {1, 0, 0}, 10.0f, b, transform);
    CHECK(ctx, all.hits.length == 1);
    CHECK_NEAR(ctx, all.hits[0].fraction, 0.2f, 1e-6f);

    // off, or triangles only: nothing
    hollow.collide_with_backfaces_convex = false;
    cast_ray::reset(all);
    ray(all, hollow, {10, 0, 0}, {1, 0, 0}, 10.0f, b, transform);
    CHECK(ctx, !all.has_hit);
    hollow.collide_with_backfaces_triangles = true;
    ray(all, hollow, {10, 0, 0}, {1, 0, 0}, 10.0f, b, transform);
    CHECK(ctx, !all.has_hit);

    // a closest collector bounds the exit by the entry, so only the entry is kept
    CastRayCollector closest = cast_ray::create_collector(CollectorKind::CLOSEST);
    ray(closest, convex_back, {5, 0, 0}, {1, 0, 0}, 10.0f, b, transform);
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 0.3f, 1e-6f);

    b = nullptr;
}

void test_dispatch_point_vs_sphere_and_box(TestCtx &ctx) {
    dispatch::init();
    Ref<Shape> s = sphere::create({.radius = 1.0f, .material_id = 6});
    Ref<Shape> b = box::create({.half_extents = {1, 2, 3}});

    // radius 1 scaled by 2 about (1, 2, 3)
    const Transformf transform_s = pose({1, 2, 3}, stored_axis_angle(vec3f::make(1, 0, 0), 0.4f), {2, 2, 2});
    const Vec3f out = unit(vec3f::make(1, 1, -1));
    CHECK(ctx, point_hits({1 + vec3f::x(out) * 1.9f, 2 + vec3f::y(out) * 1.9f, 3 + vec3f::z(out) * 1.9f}, s, transform_s));
    CHECK(ctx, !point_hits({1 + vec3f::x(out) * 2.1f, 2 + vec3f::y(out) * 2.1f, 3 + vec3f::z(out) * 2.1f}, s, transform_s));
    CollidePointCollector collector = collide_point::create_collector(CollectorKind::ALL);
    def_of(s).collide_point(collector, collide_point::default_settings(), vec3f::make(1, 2, 3), s, NO_SUB_SHAPE, transform_s);
    CHECK(ctx, collector.hits.length == 1 && shapes::get_shape_material(s, collector.hits[0].sub_shape_id_b) == 6);

    // the box turned a quarter about y: its z extent 3 lies along x
    const Transformf transform_b = pose({0, 0, 0}, stored_axis_angle(vec3f::make(0, 1, 0), scalar::PI * 0.5f));
    CHECK(ctx, point_hits({2.9f, 1.9f, 0.9f}, b, transform_b));
    CHECK(ctx, !point_hits({0, 0, 1.1f}, b, transform_b));
    CHECK(ctx, !point_hits({0, 2.1f, 0}, b, transform_b));

    s = nullptr;
    b = nullptr;
}
