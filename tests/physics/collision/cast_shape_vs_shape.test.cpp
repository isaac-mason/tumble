#include "instance.h"
#include "math/quatf.h"
#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/collision/cast_shape_vs_shape.h"
#include "physics/shapes/box.h"
#include "physics/shapes/capsule.h"
#include "physics/shapes/convex.h"
#include "physics/shapes/cylinder.h"
#include "physics/shapes/dispatch.h"
#include "physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

using cast_shape::CastShapeCollector;
using cast_shape::CastShapeHit;
using cast_shape::CastShapeSettings;

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};

/** sweeps a against b through the cast table, as the narrowphase dispatches it */
static void cast(CastShapeCollector &collector, const CastShapeSettings &settings, const shapes::Shape *a, const Transformf &transform_a,
                 const Float3 &displacement, const shapes::Shape *b, const Transformf &transform_b) {
    const shapes::CastShapeVsShapeFn fn = shapes::cast_fn(a->type, b->type);
    if (fn != nullptr) fn(collector, settings, a, NO_SUB_SHAPE, transform_a, vec3f::load(displacement), b, NO_SUB_SHAPE, transform_b);
}

void test_cast_shape_vs_shape_sphere_vs_sphere(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *a = sphere::create({.radius = 1.0f});
    shapes::Shape *b = sphere::create({.radius = 1.0f});
    const CastShapeSettings settings = cast_shape::default_settings();
    CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // b at x = 5; a moves right by 6 and touches after 3: half way. the contact at x = 4, the normal from b to a
    cast(collector, settings, a, pose({0, 0, 0}), {6, 0, 0}, b, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const CastShapeHit &hit = collector.hits[0];
    CHECK(ctx, hit.status == cast_shape::COLLIDING);
    CHECK_NEAR(ctx, hit.fraction, 0.5f, 2e-3f);
    CHECK(ctx, near3(hit.point_a, 4, 0, 0, 2e-2f) && near3(hit.point_b, 4, 0, 0, 2e-2f));
    CHECK(ctx, near3(hit.normal, -1, 0, 0, 1e-4f));

    // a moves right by 3, b at x = 5: the gap of 3 closes exactly at the end
    cast_shape::reset(collector);
    cast(collector, settings, a, pose({0, 0, 0}), {3, 0, 0}, b, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 1.0f, 1e-2f);

    // b at x = 10: a ends 7 away
    cast_shape::reset(collector);
    cast(collector, settings, a, pose({0, 0, 0}), {3, 0, 0}, b, pose({10, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // already overlapping: a hit at the start
    cast_shape::reset(collector);
    cast(collector, settings, a, pose({0, 0, 0}), {1, 0, 0}, b, pose({1, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].status == cast_shape::COLLIDING && collector.hits[0].fraction == 0.0f);

    // b at x = 4, a moves by 2: touches at the end
    cast_shape::reset(collector);
    cast(collector, settings, a, pose({0, 0, 0}), {2, 0, 0}, b, pose({4, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 1.0f, 1e-2f);

    cast_shape::release(collector);
    sphere::destroy(a);
    sphere::destroy(b);
}

void test_cast_shape_vs_shape_box_vs_box(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *a = box::create({.half_extents = {1, 1, 1}, .convex_radius = 0.0f});
    shapes::Shape *b = box::create({.half_extents = {1, 1, 1}, .convex_radius = 0.0f});
    const CastShapeSettings settings = cast_shape::default_settings();
    CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // b at 5 along each axis; a moves 4 along it and closes the gap of 3 at three quarters
    const Float3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (const Float3 &axis : axes) {
        cast_shape::reset(collector);
        Float3 displacement;
        Float3 target;
        vec3f::store(displacement, vec3f::scale(vec3f::load(axis), 4));
        vec3f::store(target, vec3f::scale(vec3f::load(axis), 5));
        cast(collector, settings, a, pose({0, 0, 0}), displacement, b, pose(target));
        CHECK(ctx, collector.hits.length == 1);
        CHECK(ctx, collector.hits[0].status == cast_shape::COLLIDING);
        CHECK_NEAR(ctx, collector.hits[0].fraction, 0.75f, 3e-3f);
        CHECK(ctx, near3(collector.hits[0].normal, vec3f::negate(vec3f::load(axis)), 1e-4f));
    }

    // b at x = 10: no contact within 3
    cast_shape::reset(collector);
    cast(collector, settings, a, pose({0, 0, 0}), {3, 0, 0}, b, pose({10, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // overlapping at the start
    cast_shape::reset(collector);
    cast(collector, settings, a, pose({0, 0, 0}), {1, 0, 0}, b, pose({0.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1 && collector.hits[0].fraction == 0.0f);

    cast_shape::release(collector);
    box::destroy(a);
    box::destroy(b);
}

void test_cast_shape_vs_shape_sphere_vs_box(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *a = sphere::create({.radius = 1.0f});
    shapes::Shape *b = box::create({.half_extents = {1, 1, 1}, .convex_radius = 0.0f});
    const CastShapeSettings settings = cast_shape::default_settings();
    CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // the face at x = 4: the sphere travels 3 of 4
    cast(collector, settings, a, pose({0, 0, 0}), {4, 0, 0}, b, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.75f, 3e-3f);
    CHECK(ctx, near3(collector.hits[0].point_b, 4, 0, 0, 1e-2f));

    // the box at (10, 10) is never near
    cast_shape::reset(collector);
    cast(collector, settings, a, pose({0, 0, 0}), {3, 0, 0}, b, pose({10, 10, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // diagonally at the edge through (2, 2): the centre stops at (s, s), s = 2 - 1 / sqrt 2, of 2.5
    cast_shape::reset(collector);
    cast(collector, settings, a, pose({0, 0, 0}), {2.5f, 2.5f, 0}, b, pose({3, 3, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, (2.0f - 1.0f / scalar::sqrt(2.0f)) / 2.5f, 3e-3f);
    CHECK(ctx, near3(collector.hits[0].point_b, 2, 2, 0, 1e-2f));

    // overlapping at the start
    cast_shape::reset(collector);
    cast(collector, settings, a, pose({0, 0, 0}), {1, 0, 0}, b, pose({0.5f, 0, 0}));
    CHECK(ctx, collector.hits.length == 1 && collector.hits[0].fraction == 0.0f);

    cast_shape::release(collector);
    sphere::destroy(a);
    box::destroy(b);
}

void test_cast_shape_vs_shape_supporting_faces(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *unit_box_a = box::create({.half_extents = {1, 1, 1}, .convex_radius = 0.0f});
    shapes::Shape *unit_box_b = box::create({.half_extents = {1, 1, 1}, .convex_radius = 0.0f});
    shapes::Shape *ball = sphere::create({.radius = 1.0f});
    shapes::Shape *small_ball = sphere::create({.radius = 0.5f});
    CastShapeSettings with_faces = cast_shape::default_settings();
    with_faces.collect_faces = true;
    const CastShapeSettings without_faces = cast_shape::default_settings();
    CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // faces only when asked for
    cast(collector, without_faces, unit_box_a, pose({0, 0, -2.5f}), {0, 0, 2}, unit_box_b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].face_a.num_vertices == 0 && collector.hits[0].face_b.num_vertices == 0);

    // box onto box along +z: a's +z face and b's -z face, both at z = -1 in world space at the contact
    cast_shape::reset(collector);
    cast(collector, with_faces, unit_box_a, pose({0, 0, -2.5f}), {0, 0, 2}, unit_box_b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const CastShapeHit &boxes = collector.hits[0];
    CHECK_NEAR(ctx, boxes.fraction, 0.25f, 3e-3f);
    CHECK(ctx, boxes.face_a.num_vertices == 4 && boxes.face_b.num_vertices == 4);
    bool on_plane = true;
    for (uint32_t i = 0; i < 4; i++) {
        on_plane = on_plane && scalar::abs(vec3f::z(boxes.face_b.vertices[i]) + 1.0f) < 1e-5f;
        on_plane = on_plane && scalar::abs(scalar::abs(vec3f::x(boxes.face_b.vertices[i])) - 1.0f) < 1e-5f;
    }
    CHECK(ctx, on_plane);

    // a sphere has no polygonal face, the box's -z face is the one it meets
    cast_shape::reset(collector);
    cast(collector, with_faces, ball, pose({0, 0, -3}), {0, 0, 2}, unit_box_b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.5f, 3e-3f);
    CHECK(ctx, collector.hits[0].face_a.num_vertices == 0 && collector.hits[0].face_b.num_vertices == 4);
    bool at_minus_one = true;
    for (uint32_t i = 0; i < 4; i++) at_minus_one = at_minus_one && scalar::abs(vec3f::z(collector.hits[0].face_b.vertices[i]) + 1.0f) < 1e-5f;
    CHECK(ctx, at_minus_one);

    // b turned 90 degrees about y and approached along +x: the face b presents sits at x = -1 in world space
    cast_shape::reset(collector);
    cast(collector, with_faces, unit_box_a, pose({-3, 0, 0}), {2, 0, 0}, unit_box_b, pose({0, 0, 0}, stored_axis_angle(vec3f::make(0, 1, 0), scalar::PI / 2.0f)));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].face_a.num_vertices == 4 && collector.hits[0].face_b.num_vertices == 4);
    bool at_x = true;
    for (uint32_t i = 0; i < 4; i++) {
        at_x = at_x && scalar::abs(vec3f::x(collector.hits[0].face_b.vertices[i]) + 1.0f) < 1e-5f;
        at_x = at_x && scalar::abs(vec3f::x(collector.hits[0].face_a.vertices[i]) + 1.0f) < 1e-5f;
    }
    CHECK(ctx, at_x);

    // b turned 45 degrees about z, a small sphere from below: b's -z face, still at z = -1
    cast_shape::reset(collector);
    cast(collector, with_faces, small_ball, pose({0, 0, -3}), {0, 0, 2}, unit_box_b, pose({0, 0, 0}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI / 4.0f)));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].face_a.num_vertices == 0 && collector.hits[0].face_b.num_vertices == 4);
    bool turned = true;
    for (uint32_t i = 0; i < 4; i++) {
        const Vec3f v = collector.hits[0].face_b.vertices[i];
        turned = turned && scalar::abs(vec3f::z(v) + 1.0f) < 1e-5f && scalar::abs(vec3f::x(v) * vec3f::x(v) + vec3f::y(v) * vec3f::y(v) - 2.0f) < 1e-4f;
    }
    CHECK(ctx, turned);

    cast_shape::release(collector);
    box::destroy(unit_box_a);
    box::destroy(unit_box_b);
    sphere::destroy(ball);
    sphere::destroy(small_ball);
}

void test_cast_shape_vs_shape_epa_faces(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *a = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.0f});
    shapes::Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.0f});
    CastShapeSettings settings = cast_shape::default_settings();
    settings.collect_faces = true;
    CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // a moving +z onto b: touches after 1 of 2
    cast(collector, settings, a, pose({0, 0, -2}), {0, 0, 2}, b, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const CastShapeHit &hit = collector.hits[0];
    CHECK_NEAR(ctx, hit.fraction, 0.5f, 3e-3f);
    CHECK(ctx, hit.face_a.num_vertices == 4 && hit.face_b.num_vertices == 4);
    // the axis points from a to b; the normal is its negated unit
    CHECK(ctx, vec3f::z(hit.penetration_axis) > 0.0f);
    CHECK(ctx, near3(hit.normal, 0, 0, -1, 1e-4f));
    CHECK_NEAR(ctx, vec3f::length(hit.normal), 1.0f, 1e-5f);

    cast_shape::release(collector);
    box::destroy(a);
    box::destroy(b);
}

void test_cast_shape_vs_shape_gjk_restart(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // this configuration takes the branch where gjk fails to find a closer simplex and has already used its one
    // restart. the contact points must come out of the previous valid simplex, not the rejected support point.
    shapes::Shape *a = cylinder::create({.half_height = 0.3f, .radius = 0.4975f});
    shapes::Shape *b = capsule::create({.half_height_of_cylinder = 2.03114343f, .radius = 7.19732189f});
    CastShapeSettings settings = cast_shape::default_settings();
    settings.use_shrunken_shape_and_convex_radius = true;
    CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    cast(collector, settings, a, pose({-0.951660156f, -2.09155273f, -7.63574218f}), {0.00244140625f, -0.0068359375f, 0.0029296875f}, b,
         pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].fraction == 0.0f);
    CHECK(ctx, vec3f::distance(collector.hits[0].point_a, collector.hits[0].point_b) < 1e-4f);

    cast_shape::release(collector);
    cylinder::destroy(a);
    capsule::destroy(b);
}

struct CountingCollector {
    CastShapeCollector collector;
    CastShapeCollector inner;
    uint32_t hit_count;
};

static void counting_add_hit(CastShapeCollector &c, const CastShapeHit &h) {
    CountingCollector &counting = *static_cast<CountingCollector *>(c.context);
    counting.hit_count++;
    cast_shape::add_hit(counting.inner, h);
    c.early_out_fraction = counting.inner.early_out_fraction;
}

static bool counting_should_early_out(const CastShapeCollector &c) {
    return cast_shape::should_early_out(static_cast<const CountingCollector *>(c.context)->inner);
}

/** counts the hits that get past a shape's early-out check and reach `kind` */
static void init_counting(CountingCollector &counting, collector::CollectorKind kind) {
    counting.inner = cast_shape::create_collector(kind);
    counting.collector = cast_shape::create_collector(collector::COLLECT_CUSTOM);
    counting.collector.custom_add_hit = counting_add_hit;
    counting.collector.custom_should_early_out = counting_should_early_out;
    counting.collector.context = &counting;
    counting.hit_count = 0;
}

void test_cast_shape_vs_shape_any_collector_early_out(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *ball = sphere::create({.radius = 1.0f});
    // its top face at y = 0
    shapes::Shape *slab = box::create({.half_extents = {4, 1, 4}});
    const CastShapeSettings settings = cast_shape::default_settings();

    CountingCollector counting;
    init_counting(counting, collector::COLLECT_ANY);

    // a sphere starting at height y with radius 1 penetrates by 1 - y, and moves down into it
    cast(counting.collector, settings, ball, pose({0, 0.9f, 0}), {0, -1, 0}, slab, pose({0, -1, 0}));
    CHECK(ctx, counting.hit_count == 1);
    const CastShapeHit *first = cast_shape::hit(counting.inner);
    CHECK(ctx, first != nullptr && first->fraction == 0.0f);
    CHECK_NEAR(ctx, first->penetration_depth, 0.1f, 1e-4f);
    CHECK(ctx, cast_shape::should_early_out(counting.collector));

    // after the first hit, a deeper start in collision never reaches the collector
    cast(counting.collector, settings, ball, pose({0, 0.5f, 0}), {0, -1, 0}, slab, pose({0, -1, 0}));
    CHECK(ctx, counting.hit_count == 1);
    CHECK_NEAR(ctx, cast_shape::hit(counting.inner)->penetration_depth, 0.1f, 1e-4f);

    cast_shape::release(counting.inner);
    sphere::destroy(ball);
    box::destroy(slab);
}

void test_cast_shape_vs_shape_closest_orders_starting_overlaps_by_depth(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *ball = sphere::create({.radius = 0.5f});
    // its top face at y = 0, so a radius 0.5 sphere at height y starts 0.5 - y deep
    shapes::Shape *slab = box::create({.half_extents = {2, 1, 2}});
    CastShapeSettings settings = cast_shape::default_settings();
    settings.return_deepest_point = true;

    const float orders[2][2] = {{0.4f, 0.2f}, {0.2f, 0.4f}};
    for (const auto &heights : orders) {
        CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CLOSEST);
        for (float y : heights) cast(collector, settings, ball, pose({0, y, 0}), {0, -1, 0}, slab, pose({0, -1, 0}));
        const CastShapeHit *hit = cast_shape::hit(collector);
        CHECK(ctx, hit != nullptr && hit->status == cast_shape::COLLIDING);
        CHECK(ctx, hit->fraction == 0.0f);
        CHECK_NEAR(ctx, hit->penetration_depth, 0.3f, 1e-4f);
        CHECK_NEAR(ctx, collector.early_out_fraction, -0.3f, 1e-4f);
        CHECK(ctx, !cast_shape::should_early_out(collector));
        cast_shape::release(collector);
    }

    sphere::destroy(ball);
    box::destroy(slab);
}

void test_cast_shape_vs_shape_reversed(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *ball = sphere::create({.radius = 0.5f, .material_id = 3});
    shapes::Shape *block = box::create({.half_extents = {1, 1, 1}, .material_id = 7});
    CastShapeSettings settings = cast_shape::default_settings();
    settings.collect_faces = true;

    // sphere at (-3, 0.3, 0) sweeping +6 in x hits the box at origin at fraction (3 - 1.5) / 6 = 0.25
    CastShapeCollector forward = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    convex::cast_convex_vs_convex(forward, settings, ball, NO_SUB_SHAPE, pose({-3, 0.3f, 0}), vec3f::make(6, 0, 0), block, NO_SUB_SHAPE,
                                  pose({0, 0, 0}));
    // the same relative motion seen from the box: the box sweeps -6 in x into the resting sphere
    CastShapeCollector reversed = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    cast_shape::reversed_cast_shape_vs_shape<convex::cast_convex_vs_convex>(reversed, settings, block, NO_SUB_SHAPE, pose({0, 0, 0}),
                                                                            vec3f::make(-6, 0, 0), ball, NO_SUB_SHAPE, pose({-3, 0.3f, 0}));
    const CastShapeHit *f = cast_shape::hit(forward);
    const CastShapeHit *r = cast_shape::hit(reversed);
    CHECK(ctx, f != nullptr && r != nullptr);
    CHECK(ctx, f->status == cast_shape::COLLIDING && r->status == cast_shape::COLLIDING);
    CHECK_NEAR(ctx, f->fraction, 0.25f, 1e-3f);
    CHECK(ctx, r->fraction == f->fraction);
    CHECK(ctx, r->penetration_depth == f->penetration_depth);

    // axis flips, normal stays -normalize(axis), so it mirrors the forward normal
    CHECK(ctx, vec3f::x(r->penetration_axis) == -vec3f::x(f->penetration_axis) && vec3f::y(r->penetration_axis) == -vec3f::y(f->penetration_axis) &&
                   vec3f::z(r->penetration_axis) == -vec3f::z(f->penetration_axis));
    const float axis_length = vec3f::length(r->penetration_axis);
    CHECK(ctx, near3(r->normal, -vec3f::x(r->penetration_axis) / axis_length, -vec3f::y(r->penetration_axis) / axis_length,
                     -vec3f::z(r->penetration_axis) / axis_length, 1e-6f));
    CHECK(ctx, near3(r->normal, -vec3f::x(f->normal), -vec3f::y(f->normal), -vec3f::z(f->normal), 1e-6f));
    CHECK_NEAR(ctx, vec3f::x(r->normal), 1.0f, 1e-3f);

    CHECK(ctx, r->material_id_a == 7 && r->material_id_b == 3);

    // shape a (the box) is reported in its swept frame: centre at x = -1.5, face at x = -2.5
    CHECK_NEAR(ctx, vec3f::x(r->point_a), -2.5f, 1e-3f);
    CHECK_NEAR(ctx, vec3f::y(r->point_a), 0.3f, 1e-3f);
    // shape b (the sphere) at rest: on its surface, radius from (-3, 0.3, 0)
    CHECK_NEAR(ctx, vec3f::x(r->point_b), -2.5f, 1e-3f);
    CHECK_NEAR(ctx, vec3f::distance(r->point_b, vec3f::make(-3, 0.3f, 0)), 0.5f, 1e-3f);

    // the box's supporting face is its -x face at the hit; the sphere's is the forward one shifted back to rest
    CHECK(ctx, r->face_a.num_vertices == 4);
    for (uint32_t i = 0; i < r->face_a.num_vertices; i++) CHECK_NEAR(ctx, vec3f::x(r->face_a.vertices[i]), -2.5f, 1e-3f);
    CHECK(ctx, r->face_b.num_vertices == f->face_a.num_vertices);
    for (uint32_t i = 0; i < f->face_a.num_vertices; i++) {
        const Vec3f rv = r->face_b.vertices[i], fv = f->face_a.vertices[i];
        CHECK_NEAR(ctx, vec3f::x(rv), vec3f::x(fv) - f->fraction * 6.0f, 1e-5f);
        CHECK_NEAR(ctx, vec3f::y(rv), vec3f::y(fv), 1e-5f);
        CHECK_NEAR(ctx, vec3f::z(rv), vec3f::z(fv), 1e-5f);
    }

    cast_shape::release(forward);
    cast_shape::release(reversed);
    sphere::destroy(ball);
    box::destroy(block);
}

void test_cast_shape_vs_shape_sort(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *shape = sphere::create({.radius = 1.0f});
    const CastShapeSettings settings = cast_shape::default_settings();
    CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // a sweep along +x by 10: b at 8 is hit at fraction 0.6, b at 5 at 0.3, b at 1.5 and 1.0 overlap at the start by 0.5 and 1.0
    const float centers[4] = {8.0f, 1.5f, 5.0f, 1.0f};
    for (float x : centers) cast(collector, settings, shape, pose({0, 0, 0}), {10, 0, 0}, shape, pose({x, 0, 0}));
    CHECK(ctx, collector.hits.length == 4);

    cast_shape::sort(collector);
    CHECK(ctx, collector.hits[0].fraction == 0.0f);
    CHECK_NEAR(ctx, collector.hits[0].penetration_depth, 1.0f, 1e-4f);
    CHECK(ctx, collector.hits[1].fraction == 0.0f);
    CHECK_NEAR(ctx, collector.hits[1].penetration_depth, 0.5f, 1e-4f);
    CHECK_NEAR(ctx, collector.hits[2].fraction, 0.3f, 1e-4f);
    CHECK_NEAR(ctx, collector.hits[3].fraction, 0.6f, 1e-4f);

    cast_shape::release(collector);
    sphere::destroy(shape);
}
