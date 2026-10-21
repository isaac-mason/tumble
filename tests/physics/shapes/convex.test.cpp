#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

/** a world point back into a frame's local space */
static Vec3f to_local(const Transformf &frame, Vec3f world) {
    return vec3f::transform_quat(vec3f::subtract(world, vec3f::load(frame.position)), quatf::conjugate(quatf::load(frame.quaternion)));
}

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};

static void collide(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                    const shapes::Shape *a, const Transformf &transform_a, const shapes::Shape *b, const Transformf &transform_b) {
    convex::collide_convex_vs_convex(collector, settings, a, NO_SUB_SHAPE, transform_a, b, NO_SUB_SHAPE, transform_b);
}

static void cast(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const shapes::Shape *a,
                 const Transformf &transform_a, const Float3 &displacement, const shapes::Shape *b, const Transformf &transform_b) {
    convex::cast_convex_vs_convex(collector, settings, a, NO_SUB_SHAPE, transform_a, vec3f::load(displacement), b, NO_SUB_SHAPE, transform_b);
}

static void ray(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, Vec3f origin, Vec3f direction,
                float length, const shapes::Shape *shape, const Transformf &transform) {
    const cast_ray::Rayf r = {origin, unit(direction), length};
    convex::cast_ray_vs_convex(collector, settings, r, shape, NO_SUB_SHAPE, transform);
}

static bool point_hits(Vec3f point, const shapes::Shape *shape, const Transformf &transform) {
    collide_point::CollidePointCollector collector = collide_point::create_collector(collector::COLLECT_ALL);
    convex::collide_point_vs_convex(collector, collide_point::default_settings(), point, shape, NO_SUB_SHAPE, transform);
    const bool hit = collector.hits.length == 1;
    collide_point::release(collector);
    return hit;
}

/* collide */

void test_convex_collide_spheres_overlapping(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *a = sphere::create({.radius = 1.0f, .material_id = 3});
    shapes::Shape *b = sphere::create({.radius = 0.5f, .material_id = 7});

    // a rotated and away from the origin; b along an oblique centre line, 1.3 from a: 0.2 overlap
    const Float4 rotation_a = stored_axis_angle(vec3f::make(1, 1, 0), 0.7f);
    const Float3 centre_a = {2, 3, 4};
    const Vec3f line = unit(vec3f::make(1, -2, 0.5f));
    const Transformf transform_a = pose(centre_a, rotation_a);
    const Transformf transform_b = pose(vec3f::scale_and_add(vec3f::load(centre_a), line, 1.3f));

    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collide(collector, collide_shape::default_settings(), a, transform_a, b, transform_b);
    CHECK(ctx, collector.hits.length == 1);
    const collide_shape::CollideShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.penetration, 0.2f, 1e-5f);
    // the axis moves b out of a: along the centre line
    CHECK(ctx, vec3f::dot(unit(hit.penetration_axis), line) > 0.99999f);
    // a's point on a's surface, b's on b's
    CHECK(ctx, near3(hit.point_a, vec3f::add(vec3f::load(centre_a), line), 1e-5f));
    CHECK(ctx, near3(hit.point_b, vec3f::scale_and_add(vec3f::load(centre_a), line, 0.8f), 1e-5f));
    CHECK(ctx, shapes::get_shape_material(a, hit.sub_shape_id_a) == 3 && shapes::get_shape_material(b, hit.sub_shape_id_b) == 7);
    CHECK(ctx, hit.sub_shape_id_a == sub_shape::EMPTY_SUB_SHAPE_ID && hit.sub_shape_id_b == sub_shape::EMPTY_SUB_SHAPE_ID);
    // faces not asked for
    CHECK(ctx, hit.face_a.num_vertices == 0 && hit.face_b.num_vertices == 0);

    // swapped: the same depth, the axis reversed
    collide_shape::reset(collector);
    collide(collector, collide_shape::default_settings(), b, transform_b, a, transform_a);
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.2f, 1e-5f);
    CHECK(ctx, vec3f::dot(unit(collector.hits[0].penetration_axis), line) < -0.99999f);

    collide_shape::release(collector);
    sphere::destroy(a);
    sphere::destroy(b);
}

void test_convex_collide_spheres_within_max_separation(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *a = sphere::create({.radius = 1.0f});
    shapes::Shape *b = sphere::create({.radius = 0.5f});
    // 0.1 apart along x
    const Transformf transform_a = pose({0, 0, 0});
    const Transformf transform_b = pose({1.6f, 0, 0});

    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // touching only: nothing
    collide(collector, settings, a, transform_a, b, transform_b);
    CHECK(ctx, collector.hits.length == 0);

    // within the max separation: a negative penetration, the points on the surfaces
    settings.max_separation_distance = 0.25f;
    collide(collector, settings, a, transform_a, b, transform_b);
    CHECK(ctx, collector.hits.length == 1);
    const collide_shape::CollideShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.penetration, -0.1f, 1e-5f);
    CHECK(ctx, vec3f::x(unit(hit.penetration_axis)) > 0.99999f);
    CHECK(ctx, near3(hit.point_a, 1, 0, 0, 1e-5f));
    CHECK(ctx, near3(hit.point_b, 1.1f, 0, 0, 1e-5f));

    // beyond it: nothing
    collide_shape::reset(collector);
    settings.max_separation_distance = 0.05f;
    collide(collector, settings, a, transform_a, b, transform_b);
    CHECK(ctx, collector.hits.length == 0);

    collide_shape::release(collector);
    sphere::destroy(a);
    sphere::destroy(b);
}

void test_convex_collide_box_resting_on_box_face(TestCtx &ctx) {
    dispatch::init();
    // sharp boxes: the cores are the boxes, so the overlap goes to epa
    shapes::Shape *a = box::create({.half_extents = {1, 1, 1}, .convex_radius = 0.0f});
    shapes::Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.0f});
    // b's bottom at 0.9, 0.1 into a's top face, its footprint inside a's
    const Float3 centre_b = {0.25f, 1.4f, -0.125f};

    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.collect_faces = true;
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collide(collector, settings, a, pose({0, 0, 0}), b, pose(centre_b));
    CHECK(ctx, collector.hits.length == 1);
    const collide_shape::CollideShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.penetration, 0.1f, 1e-5f);
    CHECK(ctx, vec3f::y(unit(hit.penetration_axis)) > 0.99999f);
    CHECK_NEAR(ctx, vec3f::y(hit.point_a), 1.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::y(hit.point_b), 0.9f, 1e-5f);

    // a's top face and b's bottom face, four corners each
    CHECK(ctx, hit.face_a.num_vertices == 4 && hit.face_b.num_vertices == 4);
    for (uint32_t i = 0; i < hit.face_a.num_vertices; i++) {
        const Vec3f v = hit.face_a.vertices[i];
        CHECK_NEAR(ctx, vec3f::y(v), 1.0f, 1e-6f);
        CHECK(ctx, scalar::abs(scalar::abs(vec3f::x(v)) - 1.0f) < 1e-6f && scalar::abs(scalar::abs(vec3f::z(v)) - 1.0f) < 1e-6f);
    }
    for (uint32_t i = 0; i < hit.face_b.num_vertices; i++) {
        const Vec3f v = hit.face_b.vertices[i];
        CHECK_NEAR(ctx, vec3f::y(v), 0.9f, 1e-6f);
        CHECK(ctx, scalar::abs(scalar::abs(vec3f::x(v) - centre_b.x) - 0.5f) < 1e-6f && scalar::abs(scalar::abs(vec3f::z(v) - centre_b.z) - 0.5f) < 1e-6f);
    }

    collide_shape::release(collector);
    box::destroy(a);
    box::destroy(b);
}

void test_convex_collide_rounded_boxes_posed(TestCtx &ctx) {
    dispatch::init();
    // rounded boxes 0.0625 into each other: the cores stay apart, gjk's shallow path
    shapes::Shape *a = box::create({.half_extents = {1, 1, 1}});
    shapes::Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    const Float3 local_b = {0.25f, 1.4375f, -0.125f};

    // the whole stack turned and moved: the answer turns with it
    const Float4 rotation = stored_axis_angle(vec3f::make(0.3f, 0.2f, 1), 0.6f);
    const Transformf transform_a = pose({-3, 2, 5}, rotation);
    const Transformf transform_b = pose(vec3f::add(vec3f::load(transform_a.position), vec3f::transform_quat(vec3f::load(local_b), quatf::load(rotation))), rotation);

    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.collect_faces = true;
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collide(collector, settings, a, transform_a, b, transform_b);
    CHECK(ctx, collector.hits.length == 1);
    const collide_shape::CollideShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.penetration, 0.0625f, 1e-5f);
    CHECK(ctx, vec3f::dot(unit(hit.penetration_axis), vec3f::transform_quat(vec3f::axis_y(), quatf::load(rotation))) > 0.99999f);
    CHECK_NEAR(ctx, vec3f::y(to_local(transform_a, hit.point_a)), 1.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::y(to_local(transform_a, hit.point_b)), 0.9375f, 1e-5f);

    CHECK(ctx, hit.face_a.num_vertices == 4 && hit.face_b.num_vertices == 4);
    for (uint32_t i = 0; i < 4; i++) {
        CHECK_NEAR(ctx, vec3f::y(to_local(transform_a, hit.face_a.vertices[i])), 1.0f, 1e-5f);
        CHECK_NEAR(ctx, vec3f::y(to_local(transform_b, hit.face_b.vertices[i])), -0.5f, 1e-5f);
    }

    collide_shape::release(collector);
    box::destroy(a);
    box::destroy(b);
}

void test_convex_collide_scaled(TestCtx &ctx) {
    dispatch::init();
    // a 0.5 sphere at scale 2 is a unit sphere; a unit box mirrored and stretched in x is 1.5 half wide
    shapes::Shape *s = sphere::create({.radius = 0.5f});
    shapes::Shape *b = box::create({.half_extents = {1, 1, 1}, .convex_radius = 0.0f});
    const Transformf transform_s = pose({0, 0, 0}, {0, 0, 0, 1}, {2, 2, 2});
    const Transformf transform_b = pose({2.4f, 0.25f, 0}, {0, 0, 0, 1}, {-1.5f, 1, 1});

    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.collect_faces = true;
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collide(collector, settings, s, transform_s, b, transform_b);
    CHECK(ctx, collector.hits.length == 1);
    const collide_shape::CollideShapeHit &hit = collector.hits[0];
    // the box's near face at x = 0.9
    CHECK_NEAR(ctx, hit.penetration, 0.1f, 1e-5f);
    CHECK(ctx, vec3f::x(unit(hit.penetration_axis)) > 0.99999f);
    CHECK(ctx, near3(hit.point_a, 1, 0, 0, 1e-5f));
    CHECK(ctx, near3(hit.point_b, 0.9f, 0, 0, 1e-5f));

    // the box's facing side at the same stretch unmirrored. mirrored, box::get_supporting_face picks the far side (its face
    // choice ignores the scale's sign), so the faces are checked here
    collide_shape::reset(collector);
    collide(collector, settings, s, transform_s, b, pose({2.4f, 0.25f, 0}, {0, 0, 0, 1}, {1.5f, 1, 1}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-5f);
    const face::Face &face_a = collector.hits[0].face_a;
    const face::Face &face_b = collector.hits[0].face_b;
    CHECK(ctx, face_a.num_vertices == 0);
    CHECK(ctx, face_b.num_vertices == 4);
    for (uint32_t i = 0; i < face_b.num_vertices; i++) {
        const Vec3f v = face_b.vertices[i];
        CHECK_NEAR(ctx, vec3f::x(v), 0.9f, 1e-5f);
        CHECK(ctx, scalar::abs(scalar::abs(vec3f::y(v) - 0.25f) - 1.0f) < 1e-5f && scalar::abs(scalar::abs(vec3f::z(v)) - 1.0f) < 1e-5f);
    }

    collide_shape::release(collector);
    sphere::destroy(s);
    box::destroy(b);
}

void test_convex_collide_early_out(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *a = sphere::create({.radius = 1.0f});
    shapes::Shape *b = sphere::create({.radius = 0.5f});
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // the early-out bound is minus the penetration to beat: shallower contacts are skipped, deeper ones kept
    collector.early_out_fraction = -0.15f;
    collide(collector, collide_shape::default_settings(), a, pose({0, 0, 0}), b, pose({1.4f, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);
    collide(collector, collide_shape::default_settings(), a, pose({0, 0, 0}), b, pose({1.3f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.2f, 1e-5f);

    collide_shape::release(collector);
    sphere::destroy(a);
    sphere::destroy(b);
}

/* cast */

void test_convex_cast_sphere_onto_box(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *s = sphere::create({.radius = 0.5f, .material_id = 2});
    shapes::Shape *b = box::create({.half_extents = {1, 1, 1}, .material_id = 5});

    cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    settings.collect_faces = true;
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // from x = -3 by 4: the sphere meets the face x = -1 with its centre at -1.5, fraction 1.5 / 4
    cast(collector, settings, s, pose({-3, 0.25f, 0.5f}), {4, 0, 0}, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const cast_shape::CastShapeHit &hit = collector.hits[0];
    CHECK(ctx, hit.status == cast_shape::COLLIDING);
    CHECK_NEAR(ctx, hit.fraction, 0.375f, 1e-4f);
    CHECK(ctx, near3(hit.point_a, -1, 0.25f, 0.5f, 1e-3f));
    CHECK(ctx, near3(hit.point_b, -1, 0.25f, 0.5f, 1e-3f));
    CHECK(ctx, near3(hit.normal, -1, 0, 0, 1e-4f));
    CHECK(ctx, vec3f::x(unit(hit.penetration_axis)) > 0.9999f);
    CHECK(ctx, shapes::get_shape_material(s, hit.sub_shape_id_a) == 2 && shapes::get_shape_material(b, hit.sub_shape_id_b) == 5);
    // the sphere has no face; the box's is its -x face
    CHECK(ctx, hit.face_a.num_vertices == 0 && hit.face_b.num_vertices == 4);
    for (uint32_t i = 0; i < 4; i++) CHECK_NEAR(ctx, vec3f::x(hit.face_b.vertices[i]), -1.0f, 1e-5f);

    // the box turned and moved: half extents (1, 2, 3) turned a quarter about y reach 3 along x
    shapes::Shape *long_box = box::create({.half_extents = {1, 2, 3}});
    cast_shape::reset(collector);
    const Transformf transform_long = pose({10, 0, 0}, stored_axis_angle(vec3f::make(0, 1, 0), scalar::PI * 0.5f));
    cast(collector, settings, s, pose({5, 0, 0}), {4, 0, 0}, long_box, transform_long);
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.375f, 1e-4f);
    CHECK(ctx, near3(collector.hits[0].point_b, 7, 0, 0, 1e-3f));
    CHECK(ctx, near3(collector.hits[0].normal, -1, 0, 0, 1e-4f));
    CHECK(ctx, collector.hits[0].face_b.num_vertices == 4);
    for (uint32_t i = 0; i < 4; i++) CHECK_NEAR(ctx, vec3f::x(collector.hits[0].face_b.vertices[i]), 7.0f, 1e-5f);

    // short of the box: a miss
    cast_shape::reset(collector);
    cast(collector, settings, s, pose({-3, 0, 0}), {1, 0, 0}, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    cast_shape::release(collector);
    sphere::destroy(s);
    box::destroy(b);
    box::destroy(long_box);
}

void test_convex_cast_starting_inside(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *s = sphere::create({.radius = 0.5f});
    shapes::Shape *b = box::create({.half_extents = {1, 1, 1}});
    cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // the centre inside the box: fraction 0, and with no deepest point the normal is the cast direction's
    cast(collector, settings, s, pose({-0.75f, 0, 0}), {4, 0, 0}, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].fraction == 0.0f);
    CHECK(ctx, near3(collector.hits[0].normal, -1, 0, 0, 1e-5f));

    // with the deepest point: epa finds the shallowest way out, through x = -1, 0.75 deep
    cast_shape::reset(collector);
    settings.return_deepest_point = true;
    settings.penetration_tolerance = 1e-4f;
    cast(collector, settings, s, pose({-0.75f, 0, 0}), {4, 0, 0}, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const cast_shape::CastShapeHit &hit = collector.hits[0];
    CHECK(ctx, hit.fraction == 0.0f);
    CHECK(ctx, near3(hit.normal, -1, 0, 0, 1e-4f));
    CHECK(ctx, near3(hit.point_a, -0.25f, 0, 0, 1e-4f));
    CHECK(ctx, near3(hit.point_b, -1, 0, 0, 1e-4f));

    // overlapping but moving away: a back face, skipped unless asked for
    cast_shape::reset(collector);
    settings.return_deepest_point = false;
    cast(collector, settings, s, pose({-1.25f, 0, 0}), {-4, 0, 0}, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);
    settings.collide_with_backfaces_triangles = true;
    cast(collector, settings, s, pose({-1.25f, 0, 0}), {-4, 0, 0}, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);
    settings.collide_with_backfaces_convex = true;
    cast(collector, settings, s, pose({-1.25f, 0, 0}), {-4, 0, 0}, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].fraction == 0.0f);

    // a closest collector keeps the deeper of two starting overlaps whichever comes first: the sphere 0.75 deep
    // into the box beats one 0.25 deep (depth is the distance between the hit points, with the deepest point)
    cast_shape::CastShapeCollector closest = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    settings.return_deepest_point = true;
    settings.collide_with_backfaces_convex = false;
    cast(closest, settings, s, pose({-1.25f, 0, 0}), {4, 0, 0}, b, pose({0, 0, 0}));
    CHECK(ctx, closest.hits.length == 1 && closest.hits[0].fraction == 0.0f);
    CHECK_NEAR(ctx, closest.hits[0].penetration_depth, 0.25f, 1e-3f);
    cast(closest, settings, s, pose({-0.75f, 0, 0}), {4, 0, 0}, b, pose({0, 0, 0}));
    CHECK_NEAR(ctx, closest.hits[0].penetration_depth, 0.75f, 1e-3f);
    CHECK_NEAR(ctx, closest.early_out_fraction, -0.75f, 1e-3f);
    cast_shape::reset(closest);
    cast(closest, settings, s, pose({-0.75f, 0, 0}), {4, 0, 0}, b, pose({0, 0, 0}));
    cast(closest, settings, s, pose({-1.25f, 0, 0}), {4, 0, 0}, b, pose({0, 0, 0}));
    CHECK_NEAR(ctx, closest.hits[0].penetration_depth, 0.75f, 1e-3f);
    cast_shape::release(closest);

    cast_shape::release(collector);
    sphere::destroy(s);
    box::destroy(b);
}

void test_convex_cast_perpendicular_normal(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.05f});
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // sliding along a face it overlaps hits at the start. the full shapes overlap, so without the deepest point the
    // normal is only the search direction gjk ended on, as jolt's
    cast(collector, cast_shape::default_settings(), b, pose({0, 0.99f, 0}), {2, 0, 0}, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].fraction == 0.0f);

    // shrunk by their convex radius they are apart, so the normal is the face's: perpendicular to the motion, which is
    // not a back face
    cast_shape::reset(collector);
    cast_shape::CastShapeSettings shrunk = cast_shape::default_settings();
    shrunk.use_shrunken_shape_and_convex_radius = true;
    cast(collector, shrunk, b, pose({0, 0.99f, 0}), {2, 0, 0}, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].fraction == 0.0f);
    CHECK(ctx, vec3f::x(collector.hits[0].normal) == 0.0f);
    CHECK_NEAR(ctx, scalar::abs(vec3f::y(collector.hits[0].normal)), 1.0f, 1e-5f);

    cast_shape::release(collector);
    box::destroy(b);
}

void test_convex_cast_scaled(TestCtx &ctx) {
    dispatch::init();
    // a unit sphere through scale 2 against a unit box mirrored and doubled in x: the face at x = -2
    shapes::Shape *s = sphere::create({.radius = 0.5f});
    shapes::Shape *b = box::create({.half_extents = {1, 1, 1}, .convex_radius = 0.0f});
    cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    settings.collect_faces = true;
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);
    cast(collector, settings, s, pose({-5, 0, 0}, {0, 0, 0, 1}, {2, 2, 2}), {4, 0, 0}, b, pose({0, 0, 0}, {0, 0, 0, 1}, {-2, 1, 1}));
    CHECK(ctx, collector.hits.length == 1);
    const cast_shape::CastShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.fraction, 0.5f, 1e-4f);
    CHECK(ctx, near3(hit.point_b, -2, 0, 0, 1e-3f));
    CHECK(ctx, near3(hit.normal, -1, 0, 0, 1e-4f));

    // the face at the same stretch unmirrored (see collide_scaled)
    cast_shape::reset(collector);
    cast(collector, settings, s, pose({-5, 0, 0}, {0, 0, 0, 1}, {2, 2, 2}), {4, 0, 0}, b, pose({0, 0, 0}, {0, 0, 0, 1}, {2, 1, 1}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.5f, 1e-4f);
    const face::Face &face_b = collector.hits[0].face_b;
    CHECK(ctx, face_b.num_vertices == 4);
    for (uint32_t i = 0; i < face_b.num_vertices; i++) {
        const Vec3f v = face_b.vertices[i];
        CHECK_NEAR(ctx, vec3f::x(v), -2.0f, 1e-5f);
        CHECK(ctx, scalar::abs(scalar::abs(vec3f::y(v)) - 1.0f) < 1e-5f && scalar::abs(scalar::abs(vec3f::z(v)) - 1.0f) < 1e-5f);
    }
    cast_shape::release(collector);
    sphere::destroy(s);
    box::destroy(b);
}

void test_convex_cast_early_out_prunes_farther(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *s = sphere::create({.radius = 0.5f});
    shapes::Shape *b = box::create({.half_extents = {1, 1, 1}});
    const cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    // the near box is met at 0.375, the far one (face at x = 0) at 0.625
    const Transformf near_box = pose({0, 0, 0});
    const Transformf far_box = pose({1, 0, 0});

    cast_shape::CastShapeCollector closest = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    cast(closest, settings, s, pose({-3, 0, 0}), {4, 0, 0}, b, near_box);
    CHECK(ctx, closest.has_hit && closest.hits.length == 1);
    CHECK_NEAR(ctx, closest.early_out_fraction, 0.375f, 1e-4f);
    cast(closest, settings, s, pose({-3, 0, 0}), {4, 0, 0}, b, far_box);
    CHECK(ctx, closest.hits.length == 1);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 0.375f, 1e-4f);

    // the shape itself prunes: a collector keeping everything, bounded at 0.5, takes only the near hit
    cast_shape::CastShapeCollector all = cast_shape::create_collector(collector::COLLECT_ALL);
    all.early_out_fraction = 0.5f;
    cast(all, settings, s, pose({-3, 0, 0}), {4, 0, 0}, b, far_box);
    CHECK(ctx, all.hits.length == 0);
    cast(all, settings, s, pose({-3, 0, 0}), {4, 0, 0}, b, near_box);
    CHECK(ctx, all.hits.length == 1);
    CHECK_NEAR(ctx, all.hits[0].fraction, 0.375f, 1e-4f);

    cast_shape::release(closest);
    cast_shape::release(all);
    sphere::destroy(s);
    box::destroy(b);
}

/* cast ray */

void test_convex_cast_ray_sphere_and_box(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *s = sphere::create({.radius = 1.0f, .material_id = 4});
    shapes::Shape *b = box::create({.half_extents = {1, 2, 3}, .material_id = 9});
    const cast_ray::CastRaySettings settings = cast_ray::default_settings();
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);

    // sphere at x = 5: entered at x = 4 of a 10 long ray
    ray(collector, settings, vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), 10.0f, s, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].status == cast_ray::COLLIDING);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.4f, 1e-4f);
    CHECK(ctx, shapes::get_shape_material(s, collector.hits[0].sub_shape_id) == 4);
    CHECK(ctx, collector.hits[0].sub_shape_id == sub_shape::EMPTY_SUB_SHAPE_ID);

    // an oblique ray through the sphere's centre still enters one radius early
    cast_ray::reset(collector);
    const Float3 centre = {2, -1, 3};
    const Vec3f direction = unit(vec3f::make(1, 2, -2));
    const Vec3f origin = vec3f::scale_and_add(vec3f::load(centre), direction, -4.0f);
    ray(collector, settings, origin, direction, 8.0f, s, pose(centre, stored_axis_angle(vec3f::make(0, 0, 1), 1.0f)));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 3.0f / 8.0f, 1e-4f);

    // the box turned a quarter about x: its y half extent of 2 now lies along z
    cast_ray::reset(collector);
    ray(collector, settings, vec3f::make(0, 0, 0), vec3f::make(0, 0, 1), 10.0f, b, pose({0, 0, 5}, stored_axis_angle(vec3f::make(1, 0, 0), scalar::PI * 0.5f)));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.3f, 1e-4f);
    CHECK(ctx, shapes::get_shape_material(b, collector.hits[0].sub_shape_id) == 9);

    // too short, or off to the side: nothing
    cast_ray::reset(collector);
    ray(collector, settings, vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), 3.0f, s, pose({5, 0, 0}));
    ray(collector, settings, vec3f::make(0, 1.5f, 0), vec3f::make(1, 0, 0), 10.0f, s, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // a closest collector's early out stops a farther sphere from being cast against at all
    cast_ray::CastRayCollector closest = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    ray(closest, settings, vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), 10.0f, s, pose({5, 0, 0}));
    CHECK_NEAR(ctx, closest.early_out_fraction, 0.4f, 1e-4f);
    cast_ray::CastRayCollector bounded = cast_ray::create_collector(collector::COLLECT_ALL);
    bounded.early_out_fraction = closest.early_out_fraction;
    ray(bounded, settings, vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), 10.0f, s, pose({7, 0, 0}));
    CHECK(ctx, bounded.hits.length == 0);

    cast_ray::release(collector);
    cast_ray::release(closest);
    cast_ray::release(bounded);
    sphere::destroy(s);
    box::destroy(b);
}

void test_convex_cast_ray_starting_inside(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *s = sphere::create({.radius = 1.0f});
    shapes::Shape *b = box::create({.half_extents = {1, 1, 1}});
    cast_ray::CastRaySettings settings = cast_ray::default_settings();
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);

    // solid: a ray from inside hits where it starts
    settings.treat_convex_as_solid = true;
    ray(collector, settings, vec3f::make(5.25f, 0, 0), vec3f::make(1, 0, 0), 10.0f, s, pose({5, 0, 0}));
    ray(collector, settings, vec3f::make(0.5f, 0.5f, 0), vec3f::make(0, 1, 0), 10.0f, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 2);
    CHECK(ctx, collector.hits[0].fraction == 0.0f && collector.hits[1].fraction == 0.0f);

    // hollow: no hit from inside without back faces
    cast_ray::reset(collector);
    settings.treat_convex_as_solid = false;
    ray(collector, settings, vec3f::make(5.25f, 0, 0), vec3f::make(1, 0, 0), 10.0f, s, pose({5, 0, 0}));
    ray(collector, settings, vec3f::make(0.5f, 0.5f, 0), vec3f::make(0, 1, 0), 10.0f, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // hollow with back faces: the exit, found by the inverted ray as a second pass (sphere surface at
    // x = 6, from 5.25 over a length of 10: 0.075; the box top at y = 1 from 0.5: 0.05)
    settings.collide_with_backfaces_convex = true;
    ray(collector, settings, vec3f::make(5.25f, 0, 0), vec3f::make(1, 0, 0), 10.0f, s, pose({5, 0, 0}));
    ray(collector, settings, vec3f::make(0.5f, 0.5f, 0), vec3f::make(0, 1, 0), 10.0f, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 2);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.075f, 1e-3f);
    CHECK_NEAR(ctx, collector.hits[1].fraction, 0.05f, 1e-3f);

    // from outside with back faces: the entry and the exit (x = 4 and x = 6 over a length of 10)
    cast_ray::reset(collector);
    ray(collector, settings, vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), 10.0f, s, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 2);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.4f, 1e-4f);
    CHECK_NEAR(ctx, collector.hits[1].fraction, 0.6f, 1e-3f);

    // a closest collector from outside keeps the entry: its bound stops the exit pass short
    cast_ray::CastRayCollector closest = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    ray(closest, settings, vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), 10.0f, s, pose({5, 0, 0}));
    CHECK(ctx, closest.hits.length == 1);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 0.4f, 1e-4f);
    cast_ray::release(closest);

    // from outside without back faces: the entry only
    cast_ray::reset(collector);
    settings.collide_with_backfaces_convex = false;
    ray(collector, settings, vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), 10.0f, s, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.4f, 1e-4f);

    cast_ray::release(collector);
    sphere::destroy(s);
    box::destroy(b);
}

void test_convex_cast_ray_scaled(TestCtx &ctx) {
    dispatch::init();
    // a unit box mirrored and doubled in x, halved in y: half extents (2, 0.5, 1)
    shapes::Shape *b = box::create({.half_extents = {1, 1, 1}});
    const Transformf transform = pose({0, 0, 0}, {0, 0, 0, 1}, {-2, 0.5f, 1});
    const cast_ray::CastRaySettings settings = cast_ray::default_settings();
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);

    ray(collector, settings, vec3f::make(-10, 0.25f, 0), vec3f::make(1, 0, 0), 20.0f, b, transform);
    ray(collector, settings, vec3f::make(0, -10, 0), vec3f::make(0, 1, 0), 20.0f, b, transform);
    CHECK(ctx, collector.hits.length == 2);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 8.0f / 20.0f, 1e-4f);
    CHECK_NEAR(ctx, collector.hits[1].fraction, 9.5f / 20.0f, 1e-4f);

    // above the halved height: a miss
    ray(collector, settings, vec3f::make(-10, 0.75f, 0), vec3f::make(1, 0, 0), 20.0f, b, transform);
    CHECK(ctx, collector.hits.length == 2);

    // a 0.5 sphere at scale 3 is 1.5 in radius
    shapes::Shape *s = sphere::create({.radius = 0.5f});
    cast_ray::reset(collector);
    ray(collector, settings, vec3f::make(0, 0, -10), vec3f::make(0, 0, 1), 20.0f, s, pose({0, 0, 0}, {0, 0, 0, 1}, {3, 3, 3}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 8.5f / 20.0f, 1e-4f);

    cast_ray::release(collector);
    box::destroy(b);
    sphere::destroy(s);
}

/* collide point */

void test_convex_collide_point(TestCtx &ctx) {
    dispatch::init();
    // half extents (1, 2, 3) turned a quarter about z: 2 along x, 1 along y, 3 along z, about (1, 2, 3)
    shapes::Shape *b = box::create({.half_extents = {1, 2, 3}, .material_id = 6});
    const Transformf transform = pose({1, 2, 3}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f));

    CHECK(ctx, point_hits(vec3f::make(1, 2, 3), b, transform));
    CHECK(ctx, point_hits(vec3f::make(2.9f, 2.9f, 5.9f), b, transform));
    CHECK(ctx, !point_hits(vec3f::make(3.1f, 2, 3), b, transform));
    CHECK(ctx, !point_hits(vec3f::make(1, 3.1f, 3), b, transform));
    // past y = 3 in the box's own frame, but inside once turned: the turn is applied
    CHECK(ctx, point_hits(vec3f::make(-0.5f, 2, 3), b, transform));
    CHECK(ctx, !point_hits(vec3f::make(1, 0.5f, 3), b, transform));
    // on the surface
    CHECK(ctx, point_hits(vec3f::make(1, 2, 6), b, transform));

    collide_point::CollidePointCollector collector = collide_point::create_collector(collector::COLLECT_ALL);
    collector.body_id_b = 42;
    convex::collide_point_vs_convex(collector, collide_point::default_settings(), vec3f::make(1, 2, 3), b, NO_SUB_SHAPE, transform);
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, shapes::get_shape_material(b, collector.hits[0].sub_shape_id_b) == 6 && collector.hits[0].body_id_b == 42);
    CHECK(ctx, collector.hits[0].sub_shape_id_b == sub_shape::EMPTY_SUB_SHAPE_ID);
    collide_point::release(collector);

    // a 0.5 sphere at scale 2, about (1, 1, 1): radius 1, on the diagonal
    shapes::Shape *s = sphere::create({.radius = 0.5f});
    const Transformf transform_s = pose({1, 1, 1}, {0, 0, 0, 1}, {2, 2, 2});
    const Vec3f diagonal = unit(vec3f::make(1, 1, 1));
    const auto along = [&](float distance) { return vec3f::scale_and_add(vec3f::make(1, 1, 1), diagonal, distance); };
    CHECK(ctx, point_hits(along(0.0f), s, transform_s));
    CHECK(ctx, point_hits(along(0.99f), s, transform_s));
    CHECK(ctx, !point_hits(along(1.01f), s, transform_s));
    // the surface counts: the point lands off it by rounding, which the collision tolerance absorbs; it does not grow the shape
    CHECK(ctx, point_hits(along(1.0f), s, transform_s));
    CHECK(ctx, !point_hits(along(1.0f + 1e-3f), s, transform_s));

    box::destroy(b);
    sphere::destroy(s);
}
