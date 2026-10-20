#include "tumble/instance.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

using collide_shape::CollideShapeCollector;
using collide_shape::CollideShapeHit;
using collide_shape::CollideShapeSettings;

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};

/** collides a against b through the collide table, as the narrowphase dispatches it */
static void collide(CollideShapeCollector &collector, const CollideShapeSettings &settings, const shapes::Shape *a, const Transformf &transform_a,
                    const shapes::Shape *b, const Transformf &transform_b) {
    const shapes::CollideShapeVsShapeFn fn = shapes::collide_fn(a->type, b->type);
    if (fn != nullptr) fn(collector, settings, a, NO_SUB_SHAPE, transform_a, b, NO_SUB_SHAPE, transform_b);
}

/** the hits of a at the origin against b at `position`, into a fresh all collector the caller releases */
static CollideShapeCollector hits_at(const shapes::Shape *a, const shapes::Shape *b, const Float3 &position,
                                     const CollideShapeSettings &settings = collide_shape::default_settings(),
                                     const Transformf &transform_a = pose({0, 0, 0})) {
    CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collide(collector, settings, a, transform_a, b, pose(position));
    return collector;
}

/** the single penetration of a at the origin against b at `position`, or -1 when there is not exactly one hit */
static float penetration_at(const shapes::Shape *a, const shapes::Shape *b, const Float3 &position, const Transformf &transform_a = pose({0, 0, 0}),
                            const Float4 &rotation_b = {0, 0, 0, 1}) {
    CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collide(collector, collide_shape::default_settings(), a, transform_a, b, pose(position, rotation_b));
    const float penetration = collector.hits.length == 1 ? collector.hits[0].penetration : -1.0f;
    collide_shape::release(collector);
    return penetration;
}

static uint32_t hit_count_at(const shapes::Shape *a, const shapes::Shape *b, const Float3 &position) {
    CollideShapeCollector collector = hits_at(a, b, position);
    const uint32_t count = collector.hits.length;
    collide_shape::release(collector);
    return count;
}

static shapes::Shape *hull(convex_hull::ConvexHullData **out_data, const float *positions, uint32_t count, float convex_radius) {
    *out_data = convex_hull::build({.positions = positions, .num_positions = count, .convex_radius = convex_radius});
    return convex_hull::create({.data = *out_data});
}

void test_collide_shape_vs_shape_default_settings(TestCtx &ctx) {
    // back faces off by default, as jolt
    const CollideShapeSettings settings = collide_shape::default_settings();
    CHECK(ctx, !settings.collide_with_backfaces);
    CHECK(ctx, settings.collide_only_with_active_edges && !settings.collect_faces);
    CHECK(ctx, settings.max_separation_distance == 0.0f);
}

void test_collide_shape_vs_shape_sphere_vs_sphere(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *one = sphere::create({.radius = 1.0f});
    shapes::Shape *two = sphere::create({.radius = 2.0f});
    shapes::Shape *small = sphere::create({.radius = 1.8f});

    CHECK(ctx, hit_count_at(one, one, {10, 0, 0}) == 0);
    CHECK_NEAR(ctx, penetration_at(one, one, {1.5f, 0, 0}), 0.5f, 1e-5f);
    CHECK_NEAR(ctx, penetration_at(one, one, {1.99f, 0, 0}), 0.01f, 1e-5f);
    // a scaled by 2 is radius 2
    CHECK_NEAR(ctx, penetration_at(one, one, {2.5f, 0, 0}, pose({0, 0, 0}, {0, 0, 0, 1}, {2, 2, 2})), 0.5f, 1e-5f);
    // coincident: the worst case, the full diameter
    CHECK_NEAR(ctx, penetration_at(two, two, {1, 2, 3}, pose({1, 2, 3})), 4.0f, 1e-4f);
    // nearly coincident: r1 + r2 - distance
    CHECK_NEAR(ctx, penetration_at(two, small, {1.1f, 2, 3}, pose({1, 2, 3})), 2.0f + 1.8f - 0.1f, 1e-4f);

    sphere::destroy(one);
    sphere::destroy(two);
    sphere::destroy(small);
}

void test_collide_shape_vs_shape_box_vs_box(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *unit = box::create({.half_extents = {1, 1, 1}});
    shapes::Shape *big = box::create({.half_extents = {2, 2, 2}});
    shapes::Shape *small = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    CHECK(ctx, hit_count_at(unit, unit, {10, 0, 0}) == 0);
    CHECK_NEAR(ctx, penetration_at(unit, unit, {1.5f, 0, 0}), 0.5f, 1e-4f);
    CHECK_NEAR(ctx, penetration_at(unit, unit, {1.99f, 0, 0}), 0.01f, 1e-4f);
    {
        CollideShapeCollector collector = hits_at(unit, unit, {1.5f, 0, 0});
        // the axis points from a to b
        CHECK(ctx, vec3f::x(vec3f::normalize(collector.hits[0].penetration_axis)) > 0.9999f);
        collide_shape::release(collector);
    }
    // b turned 45 degrees about z at x = 0.8: deep enough for epa, which runs on the full boxes, so b's corner reaches
    // sqrt 2 back along x, the shallowest way out
    Float4 rotation = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI / 4.0f);
    CHECK_NEAR(ctx, penetration_at(unit, unit, {0.8f, 0, 0}, pose({0, 0, 0}), rotation), 1.0f + scalar::sqrt(2.0f) - 0.8f, 1e-3f);
    // a stretched to x in [-2, 2]
    CHECK_NEAR(ctx, penetration_at(unit, unit, {2.5f, 0, 0}, pose({0, 0, 0}, {0, 0, 0, 1}, {2, 1, 1})), 0.5f, 1e-4f);
    // one inside the other: the small box spans [0, 1] on each axis, 2 from the big one's faces the near way
    CHECK_NEAR(ctx, penetration_at(big, small, {0.5f, 0.5f, 0.5f}), 2.0f, 1e-3f);

    box::destroy(unit);
    box::destroy(big);
    box::destroy(small);
}

void test_collide_shape_vs_shape_sphere_vs_box(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *ball = sphere::create({.radius = 1.0f});
    shapes::Shape *unit = box::create({.half_extents = {1, 1, 1}});

    CHECK(ctx, hit_count_at(ball, unit, {10, 0, 0}) == 0);
    CHECK_NEAR(ctx, penetration_at(ball, unit, {1.5f, 0, 0}), 0.5f, 1e-5f);
    // the centre inside the box, 0.7 from its nearest faces: radius + 0.7
    CHECK_NEAR(ctx, penetration_at(ball, unit, {0.3f, 0.3f, 0.3f}), 1.7f, 1e-5f);

    sphere::destroy(ball);
    box::destroy(unit);
}

void test_collide_shape_vs_shape_supporting_faces(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *ball = sphere::create({.radius = 1.0f});
    shapes::Shape *unit = box::create({.half_extents = {1, 1, 1}});
    CollideShapeSettings with_faces = collide_shape::default_settings();
    with_faces.collect_faces = true;

    // spheres have no polygonal faces
    CollideShapeCollector spheres = hits_at(ball, ball, {1.5f, 0, 0}, with_faces);
    CHECK(ctx, spheres.hits.length == 1);
    CHECK(ctx, spheres.hits[0].face_a.num_vertices == 0 && spheres.hits[0].face_b.num_vertices == 0);
    collide_shape::release(spheres);

    // boxes without faces asked for
    CollideShapeCollector bare = hits_at(unit, unit, {1.5f, 0, 0});
    CHECK(ctx, bare.hits.length == 1);
    CHECK(ctx, bare.hits[0].face_a.num_vertices == 0 && bare.hits[0].face_b.num_vertices == 0);
    collide_shape::release(bare);

    // with faces: a's +x face at x = 1 and b's -x face at x = 0.5, world space
    CollideShapeCollector boxes = hits_at(unit, unit, {1.5f, 0, 0}, with_faces);
    CHECK(ctx, boxes.hits.length == 1);
    const CollideShapeHit &hit = boxes.hits[0];
    CHECK(ctx, hit.face_a.num_vertices == 4 && hit.face_b.num_vertices == 4);
    CHECK(ctx, vec3f::x(hit.penetration_axis) > 0.0f);
    bool faces_placed = true;
    for (uint32_t i = 0; i < 4; i++) {
        faces_placed = faces_placed && scalar::abs(vec3f::x(hit.face_a.vertices[i]) - 1.0f) < 1e-5f;
        faces_placed = faces_placed && scalar::abs(vec3f::x(hit.face_b.vertices[i]) - 0.5f) < 1e-5f;
        faces_placed = faces_placed && scalar::abs(scalar::abs(vec3f::y(hit.face_a.vertices[i])) - 1.0f) < 1e-5f;
        faces_placed = faces_placed && scalar::abs(scalar::abs(vec3f::z(hit.face_b.vertices[i])) - 1.0f) < 1e-5f;
    }
    CHECK(ctx, faces_placed);
    collide_shape::release(boxes);

    // a stretched along x: its face at x = 2, y and z still within 1
    CollideShapeCollector scaled = hits_at(unit, unit, {2.5f, 0, 0}, with_faces, pose({0, 0, 0}, {0, 0, 0, 1}, {2, 1, 1}));
    CHECK(ctx, scaled.hits.length == 1 && scaled.hits[0].face_a.num_vertices == 4 && scaled.hits[0].face_b.num_vertices == 4);
    bool stretched = true;
    for (uint32_t i = 0; i < 4; i++) {
        const Vec3f v = scaled.hits[0].face_a.vertices[i];
        stretched = stretched && scalar::abs(vec3f::x(v) - 2.0f) < 1e-5f && scalar::abs(vec3f::y(v)) <= 1.0f && scalar::abs(vec3f::z(v)) <= 1.0f;
    }
    CHECK(ctx, stretched);
    collide_shape::release(scaled);

    sphere::destroy(ball);
    box::destroy(unit);
}

void test_collide_shape_vs_shape_convex_hulls(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    const float box_points[] = {-1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1};
    const float tetrahedron_points[] = {-1, 0, -1, 0, 0, 1, 1, 0, -1, 0, -1, 0};
    convex_hull::ConvexHullData *sharp_data;
    convex_hull::ConvexHullData *rounded_data;
    convex_hull::ConvexHullData *tetrahedron_data;
    shapes::Shape *sharp = hull(&sharp_data, box_points, 8, 0.0f);
    shapes::Shape *rounded = hull(&rounded_data, box_points, 8, 0.1f);
    shapes::Shape *tetrahedron = hull(&tetrahedron_data, tetrahedron_points, 4, 0.0f);
    CHECK(ctx, sharp != nullptr && rounded != nullptr && tetrahedron != nullptr);

    // vs spheres: just into a face, deep into a face, just into an edge, just into a vertex
    shapes::Shape *face_ball = sphere::create({.radius = 1.51f});
    shapes::Shape *ball = sphere::create({.radius = 1.0f});
    shapes::Shape *edge_ball = sphere::create({.radius = scalar::sqrt(2.0f) + 0.01f});
    shapes::Shape *vertex_ball = sphere::create({.radius = scalar::sqrt(3.0f) + 0.01f});
    CHECK_NEAR(ctx, penetration_at(sharp, face_ball, {2.5f, 0, 0}), 0.01f, 1e-4f);
    CHECK_NEAR(ctx, penetration_at(sharp, ball, {1.5f, 0, 0}), 0.5f, 1e-3f);
    CHECK_NEAR(ctx, penetration_at(sharp, edge_ball, {2, 2, 0}), 0.01f, 1e-4f);
    CHECK_NEAR(ctx, penetration_at(sharp, vertex_ball, {2, 2, 2}), 0.01f, 1e-4f);

    // vs boxes: overlapping by 0.5; a small box at [0, 0.6] inside, out the near way by 1
    shapes::Shape *unit = box::create({.half_extents = {1, 1, 1}});
    shapes::Shape *small = box::create({.half_extents = {0.3f, 0.3f, 0.3f}});
    CHECK_NEAR(ctx, penetration_at(sharp, unit, {1.5f, 0, 0}), 0.5f, 1e-3f);
    CHECK_NEAR(ctx, penetration_at(sharp, small, {0.3f, 0.3f, 0.3f}), 1.0f, 1e-3f);

    // vs hulls: box by box, box by tetrahedron, two rounded boxes 0.2 into each other
    CHECK_NEAR(ctx, penetration_at(sharp, sharp, {1.5f, 0, 0}), 0.5f, 1e-3f);
    CHECK(ctx, penetration_at(sharp, tetrahedron, {0.5f, 0, 0}) > 0.0f);
    CHECK_NEAR(ctx, penetration_at(rounded, rounded, {1.8f, 0, 0}), 0.2f, 1e-3f);

    sphere::destroy(face_ball);
    sphere::destroy(ball);
    sphere::destroy(edge_ball);
    sphere::destroy(vertex_ball);
    box::destroy(unit);
    box::destroy(small);
    convex_hull::destroy(sharp);
    convex_hull::destroy(rounded);
    convex_hull::destroy(tetrahedron);
    convex_hull::release(sharp_data);
    convex_hull::release(rounded_data);
    convex_hull::release(tetrahedron_data);
}

void test_collide_shape_vs_shape_cylinders(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *ball = sphere::create({.radius = 1.0f});
    shapes::Shape *unit = box::create({.half_extents = {1, 1, 1}});
    shapes::Shape *tall = cylinder::create({.half_height = 2.0f, .radius = 1.0f});
    shapes::Shape *thin = cylinder::create({.half_height = 1.0f, .radius = 0.5f});

    CHECK_NEAR(ctx, penetration_at(ball, tall, {1.5f, 0, 0}), 0.5f, 1e-3f);
    CHECK_NEAR(ctx, penetration_at(unit, tall, {1.5f, 0, 0}), 0.5f, 1e-3f);
    CHECK_NEAR(ctx, penetration_at(thin, thin, {0.8f, 0, 0}), 0.2f, 1e-3f);
    CHECK(ctx, hit_count_at(thin, thin, {5, 0, 0}) == 0);

    sphere::destroy(ball);
    box::destroy(unit);
    cylinder::destroy(tall);
    cylinder::destroy(thin);
}

void test_collide_shape_vs_shape_reversed_pair_materials(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // sphere vs box runs the specialised handler; box vs sphere runs it reversed, and the hit's materials must follow
    // the shape order like its points and sub shape ids do
    shapes::Shape *ball = sphere::create({.radius = 1.0f, .material_id = 3});
    shapes::Shape *unit = box::create({.half_extents = {1, 1, 1}, .material_id = 5});
    CollideShapeSettings with_faces = collide_shape::default_settings();
    with_faces.collect_faces = true;

    CollideShapeCollector forward = hits_at(ball, unit, {1.5f, 0, 0}, with_faces);
    CHECK(ctx, forward.hits.length == 1);
    CHECK(ctx, forward.hits[0].material_id_a == 3 && forward.hits[0].material_id_b == 5);
    CHECK(ctx, vec3f::x(forward.hits[0].penetration_axis) > 0.0f);

    CollideShapeCollector reversed = hits_at(unit, ball, {1.5f, 0, 0}, with_faces);
    CHECK(ctx, reversed.hits.length == 1);
    const CollideShapeHit &hit = reversed.hits[0];
    CHECK(ctx, hit.material_id_a == 5 && hit.material_id_b == 3);
    CHECK(ctx, vec3f::x(hit.penetration_axis) > 0.0f);
    CHECK_NEAR(ctx, hit.penetration, 0.5f, 1e-5f);
    // a's point on the box face at x = 1, b's on the sphere at x = 0.5; the box face moves to face_a
    CHECK_NEAR(ctx, vec3f::x(hit.point_a), 1.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::x(hit.point_b), 0.5f, 1e-5f);
    CHECK(ctx, hit.face_a.num_vertices == 4 && hit.face_b.num_vertices == 0);

    collide_shape::release(forward);
    collide_shape::release(reversed);
    sphere::destroy(ball);
    box::destroy(unit);
}

struct CountingCollector {
    CollideShapeCollector collector;
    CollideShapeCollector inner;
    uint32_t hit_count;
};

static void counting_add_hit(CollideShapeCollector &c, const CollideShapeHit &h) {
    CountingCollector &counting = *static_cast<CountingCollector *>(c.context);
    counting.hit_count++;
    collide_shape::add_hit(counting.inner, h);
    c.early_out_fraction = counting.inner.early_out_fraction;
}

static bool counting_should_early_out(const CollideShapeCollector &c) {
    return collide_shape::should_early_out(static_cast<const CountingCollector *>(c.context)->inner);
}

/** counts the hits that get past a shape's early-out check and reach `kind` */
static void init_counting(CountingCollector &counting, collector::CollectorKind kind) {
    counting.inner = collide_shape::create_collector(kind);
    counting.collector = collide_shape::create_collector(collector::COLLECT_CUSTOM);
    counting.collector.custom_add_hit = counting_add_hit;
    counting.collector.custom_should_early_out = counting_should_early_out;
    counting.collector.context = &counting;
    counting.hit_count = 0;
}

void test_collide_shape_vs_shape_closest_collector_early_out(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *unit = box::create({.half_extents = {1, 1, 1}});
    CountingCollector counting;
    init_counting(counting, collector::COLLECT_CLOSEST);
    // unit boxes overlapping along x by the given penetration depth
    const auto collide_boxes = [&](float penetration) {
        collide(counting.collector, collide_shape::default_settings(), unit, pose({0, 0, 0}), unit, pose({2 - penetration, 0, 0}));
    };

    collide_boxes(0.2f);
    CHECK(ctx, counting.hit_count == 1);
    CHECK_NEAR(ctx, collide_shape::hit(counting.inner)->penetration, 0.2f, 1e-4f);
    CHECK_NEAR(ctx, counting.collector.early_out_fraction, -0.2f, 1e-4f);

    // shallower: rejected by the convex early-out check before reaching the collector
    collide_boxes(0.1f);
    CHECK(ctx, counting.hit_count == 1);
    CHECK_NEAR(ctx, counting.collector.early_out_fraction, -0.2f, 1e-4f);

    // deeper: passes the check and becomes the closest hit
    collide_boxes(0.3f);
    CHECK(ctx, counting.hit_count == 2);
    CHECK_NEAR(ctx, collide_shape::hit(counting.inner)->penetration, 0.3f, 1e-4f);
    CHECK_NEAR(ctx, counting.collector.early_out_fraction, -0.3f, 1e-4f);

    collide_shape::release(counting.inner);
    box::destroy(unit);
}

void test_collide_shape_vs_shape_any_collector_early_out(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *thin = cylinder::create({.half_height = 1.0f, .radius = 0.5f});
    CountingCollector counting;
    init_counting(counting, collector::COLLECT_ANY);
    // two upright cylinders of radius 0.5 whose axes are `distance` apart overlap by 1 - distance
    const auto collide_at = [&](float distance) {
        collide(counting.collector, collide_shape::default_settings(), thin, pose({0, 0, 0}), thin, pose({distance, 0, 0}));
    };

    collide_at(0.9f);
    CHECK(ctx, counting.hit_count == 1);
    CHECK_NEAR(ctx, collide_shape::hit(counting.inner)->penetration, 0.1f, 1e-3f);
    CHECK(ctx, collide_shape::should_early_out(counting.collector));

    // after the first hit, a deeper candidate is pruned before it reaches the collector
    collide_at(0.5f);
    CHECK(ctx, counting.hit_count == 1);
    CHECK_NEAR(ctx, collide_shape::hit(counting.inner)->penetration, 0.1f, 1e-3f);

    collide_shape::release(counting.inner);
    cylinder::destroy(thin);
}

void test_collide_shape_vs_shape_sort(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    shapes::Shape *shape = sphere::create({.radius = 1.0f});
    CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // spheres overlapping the one at the origin by 0.2, 0.6 and 0.4
    const float centers[3] = {1.8f, 1.4f, 1.6f};
    for (float x : centers) collide(collector, collide_shape::default_settings(), shape, pose({0, 0, 0}), shape, pose({x, 0, 0}));
    CHECK(ctx, collector.hits.length == 3);

    collide_shape::sort(collector);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.6f, 1e-5f);
    CHECK_NEAR(ctx, collector.hits[1].penetration, 0.4f, 1e-5f);
    CHECK_NEAR(ctx, collector.hits[2].penetration, 0.2f, 1e-5f);

    collide_shape::release(collector);
    sphere::destroy(shape);
}
