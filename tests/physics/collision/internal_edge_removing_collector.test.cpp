#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/internal_edge_removing_collector.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

using collide_shape::CollideShapeCollector;
using collide_shape::CollideShapeHit;
using internal_edge_removing::InternalEdgeRemovingCollector;

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};

static void set_face(face::Face &face, uint32_t count, const float *vertices) {
    face.num_vertices = count;
    for (uint32_t i = 0; i < count; i++) face.vertices[i] = vec3f::load(vertices, i * 3);
}

static CollideShapeHit make_hit(float penetration, const Float3 &axis, const Float3 &point_b, uint32_t face_count, const float *face_vertices,
                                sub_shape::SubShapeId sub_shape_id_a = sub_shape::EMPTY_SUB_SHAPE_ID) {
    CollideShapeHit hit = collide_shape::create_hit();
    hit.body_id_b = 1;
    hit.penetration = penetration;
    hit.sub_shape_id_a = sub_shape_id_a;
    hit.penetration_axis = vec3f::load(axis);
    hit.point_b = vec3f::load(point_b);
    set_face(hit.face_b, face_count, face_vertices);
    return hit;
}

// a triangle in the xy plane, counter-clockwise seen from +z
static const float TRIANGLE[9] = {0, 0, 0, 1, 0, 0, 0.5f, 0.86602540378f, 0};

void test_internal_edge_removing_face_contact_detection(TestCtx &ctx) {
    CollideShapeCollector chained = collide_shape::create_collector(collector::COLLECT_ALL);
    InternalEdgeRemovingCollector wrapper = internal_edge_removing::create();
    internal_edge_removing::set(wrapper, chained, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);

    // the contact normal (opposite the penetration axis) along the face normal: forwarded at once
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, 0, -1}, {0.5f, 0.3f, 0}, 3, TRIANGLE));
    CHECK(ctx, chained.hits.length == 1 && chained.hits[0].penetration == 1.0f);
    CHECK(ctx, chained.body_id_b == 1);

    // an edge contact: held until the flush
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, -1, 0}, {0.5f, 0, 0}, 3, TRIANGLE));
    CHECK(ctx, chained.hits.length == 1);
    internal_edge_removing::flush(wrapper);
    // its edge (0,0)-(1,0) was voided by the face contact on the same triangle, so it is dropped
    CHECK(ctx, chained.hits.length == 1);

    // on its own, the edge contact goes through at the flush
    internal_edge_removing::reset(wrapper);
    collide_shape::reset(chained);
    internal_edge_removing::set(wrapper, chained, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, -1, 0}, {0.5f, 0, 0}, 3, TRIANGLE));
    CHECK(ctx, chained.hits.length == 0);
    internal_edge_removing::flush(wrapper);
    CHECK(ctx, chained.hits.length == 1);

    // a face with fewer than 3 vertices has no normal: forwarded at once
    const float segment[6] = {0, 0, 0, 1, 0, 0};
    collide_shape::reset(chained);
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, 0, -1}, {0.5f, 0, 0}, 2, segment));
    CHECK(ctx, chained.hits.length == 1);
    // so is a degenerate (collinear) triangle
    const float collinear[9] = {0, 0, 0, 1, 0, 0, 2, 0, 0};
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, -1, 0}, {0.5f, 0, 0}, 3, collinear));
    CHECK(ctx, chained.hits.length == 2);

    internal_edge_removing::release(wrapper);
    collide_shape::release(chained);
}

void test_internal_edge_removing_voiding(TestCtx &ctx) {
    CollideShapeCollector chained = collide_shape::create_collector(collector::COLLECT_ALL);
    InternalEdgeRemovingCollector wrapper = internal_edge_removing::create();

    // a face contact on one square and an edge contact on the shared edge x = 1 of the next: the edge is voided
    {
        internal_edge_removing::set(wrapper, chained, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);
        const float square_1[12] = {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0};
        const float square_2[12] = {1, 0, 0, 2, 0, 0, 2, 1, 0, 1, 1, 0};
        collide_shape::add_hit(wrapper.collector, make_hit(2.0f, {0, 0, -1}, {0.5f, 0.5f, 0}, 4, square_1));
        collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {1, 0, 0}, {1, 0.5f, 0}, 4, square_2));
        internal_edge_removing::flush(wrapper);
        CHECK(ctx, chained.hits.length == 1 && chained.hits[0].penetration == 2.0f);
    }

    // three edge contacts on triangles sharing no vertices go through deepest first, whatever the order they came in
    {
        collide_shape::reset(chained);
        internal_edge_removing::set(wrapper, chained, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);
        const float depths[3] = {1.0f, 3.0f, 2.0f};
        const float edge_x[3] = {0.0f, 4.0f, 2.0f};
        for (int i = 0; i < 3; i++) {
            const float x = edge_x[i];
            const float triangle[9] = {x, 0, 0, x + 1, 0, 0, x + 0.5f, 1, 0};
            collide_shape::add_hit(wrapper.collector, make_hit(depths[i], {0, -1, 0}, {x + 0.5f, 0, 0}, 3, triangle));
        }
        internal_edge_removing::flush(wrapper);
        CHECK(ctx, chained.hits.length == 3);
        CHECK(ctx, chained.hits[0].penetration == 3.0f && chained.hits[1].penetration == 2.0f && chained.hits[2].penetration == 1.0f);
        // the flush leaves nothing behind for the next body
        CHECK(ctx, wrapper.delayed_results.length == 0 && wrapper.voided_features.length == 0);
    }

    // two triangles touching at the shared vertex (1, 0, 0): the deeper goes through and voids it for the other
    {
        collide_shape::reset(chained);
        internal_edge_removing::set(wrapper, chained, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);
        const float triangle_1[9] = {0, 0, 0, 1, 0, 0, 0.5f, 1, 0};
        const float triangle_2[9] = {1, 0, 0, 2, 0, 0, 1.5f, 1, 0};
        collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, -1, 0}, {1, 0, 0}, 3, triangle_2));
        collide_shape::add_hit(wrapper.collector, make_hit(2.0f, {0, -1, 0}, {1, 0, 0}, 3, triangle_1));
        internal_edge_removing::flush(wrapper);
        CHECK(ctx, chained.hits.length == 1 && chained.hits[0].penetration == 2.0f);
    }

    // voiding is per sub shape of the convex side: the same edge for two sub shapes goes through twice
    {
        collide_shape::reset(chained);
        internal_edge_removing::set(wrapper, chained, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);
        const float triangle[9] = {0, 0, 0, 1, 0, 0, 0.5f, 1, 0};
        collide_shape::add_hit(wrapper.collector, make_hit(2.0f, {0, -1, 0}, {0.5f, 0, 0}, 3, triangle, 1));
        collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, -1, 0}, {0.5f, 0, 0}, 3, triangle, 2));
        internal_edge_removing::flush(wrapper);
        CHECK(ctx, chained.hits.length == 2);
    }

    internal_edge_removing::release(wrapper);
    collide_shape::release(chained);
}

void test_internal_edge_removing_reset_and_early_out(TestCtx &ctx) {
    CollideShapeCollector chained = collide_shape::create_collector(collector::COLLECT_ALL);
    InternalEdgeRemovingCollector wrapper = internal_edge_removing::create();
    internal_edge_removing::set(wrapper, chained, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);

    // reset drops the delayed hits
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, -1, 0}, {0.5f, 0, 0}, 3, TRIANGLE));
    CHECK(ctx, wrapper.delayed_results.length == 1);
    internal_edge_removing::reset(wrapper);
    CHECK(ctx, wrapper.collector.early_out_fraction == collector::UNBOUNDED);
    internal_edge_removing::flush(wrapper);
    CHECK(ctx, chained.hits.length == 0);

    // the early out follows the chained collector once a hit goes through
    internal_edge_removing::set(wrapper, chained, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);
    CHECK(ctx, !collide_shape::should_early_out(wrapper.collector));
    chained.early_out_fraction = 0.0f;
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, 0, -1}, {0.5f, 0.3f, 0}, 3, TRIANGLE));
    CHECK(ctx, wrapper.collector.early_out_fraction == 0.0f);

    // and should_early_out is the chained collector's
    CollideShapeCollector any = collide_shape::create_collector(collector::COLLECT_ANY);
    internal_edge_removing::set(wrapper, any, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);
    CHECK(ctx, !collide_shape::should_early_out(wrapper.collector));
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, 0, -1}, {0.5f, 0.3f, 0}, 3, TRIANGLE));
    CHECK(ctx, collide_shape::should_early_out(wrapper.collector));
    CHECK(ctx, wrapper.collector.early_out_fraction == -collector::UNBOUNDED);

    internal_edge_removing::release(wrapper);
    collide_shape::release(chained);
    collide_shape::release(any);
}

/** collides the ball against both slabs into `collector` (the wrapper's or a plain one), one query over two parts */
static void collide_ball_with_slabs(CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings, const shapes::Shape *ball,
                                    const Float3 &centre, const shapes::Shape *slab) {
    const Transformf ball_pose = {centre, {0, 0, 0, 1}, {1, 1, 1}};
    // two slabs with tops at y = 0 meeting at x = 0: x in [-2, 0] and x in [0, 2]
    const Transformf left = {{-1, -0.5f, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    const Transformf right = {{1, -0.5f, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    const shapes::CollideShapeVsShapeFn fn = shapes::collide_fn(ball->type, slab->type);
    fn(collector, settings, ball, NO_SUB_SHAPE, ball_pose, slab, {0, 0}, left);
    fn(collector, settings, ball, NO_SUB_SHAPE, ball_pose, slab, {1, 0}, right);
}

static float normal_y(const CollideShapeHit &hit) {
    return -vec3f::y(unit(hit.penetration_axis));
}

void test_internal_edge_removing_sphere_across_coplanar_boxes(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *ball = sphere::create({.radius = 0.5f});
    shapes::Shape *slab = box::create({.half_extents = {1, 0.5f, 1}, .convex_radius = 0.0f});
    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.collect_faces = true;
    settings.collide_only_with_active_edges = false;

    CollideShapeCollector chained = collide_shape::create_collector(collector::COLLECT_ALL);
    CollideShapeCollector plain = collide_shape::create_collector(collector::COLLECT_ALL);
    InternalEdgeRemovingCollector wrapper = internal_edge_removing::create();

    // a ball of radius 0.5 sunk 0.1 into the floor, sliding across the seam: within 0.3 of it, the slab it is not over
    // reaches it only at the seam's edge, a ghost contact whose normal tilts towards the ball
    uint32_t ghosts_without = 0;
    bool ghosts_with = false;
    for (int step = -5; step <= 5; step++) {
        const Float3 centre = {(float)step * 0.05f, 0.4f, 0.25f};

        collide_shape::reset(plain);
        collide_ball_with_slabs(plain, settings, ball, centre, slab);
        for (uint32_t i = 0; i < plain.hits.length; i++) {
            if (normal_y(plain.hits[i]) < 0.9999f) ghosts_without++;
        }

        collide_shape::reset(chained);
        internal_edge_removing::set(wrapper, chained, settings.internal_edge_removal_vertex_tolerance_sq);
        collide_ball_with_slabs(wrapper.collector, settings, ball, centre, slab);
        internal_edge_removing::flush(wrapper);
        // the floor contact is kept, the ghost is dropped: every kept normal points straight up, 0.1 deep
        CHECK(ctx, chained.hits.length >= 1);
        for (uint32_t i = 0; i < chained.hits.length; i++) {
            if (normal_y(chained.hits[i]) < 0.9999f) ghosts_with = true;
            CHECK_NEAR(ctx, chained.hits[i].penetration, 0.1f, 1e-5f);
        }
    }
    // off the seam itself each position has a ghost without the wrapper
    CHECK(ctx, ghosts_without == 10);
    CHECK(ctx, !ghosts_with);

    // at the slabs' outer edge there is no neighbour: the edge contact is real and kept
    collide_shape::reset(chained);
    internal_edge_removing::set(wrapper, chained, settings.internal_edge_removal_vertex_tolerance_sq);
    collide_ball_with_slabs(wrapper.collector, settings, ball, {2.1f, 0.4f, 0}, slab);
    CHECK(ctx, chained.hits.length == 0);
    internal_edge_removing::flush(wrapper);
    CHECK(ctx, chained.hits.length == 1);
    // from the edge (2, 0) to the centre (2.1, 0.4)
    const float distance = scalar::sqrt(0.1f * 0.1f + 0.4f * 0.4f);
    CHECK_NEAR(ctx, chained.hits[0].penetration, 0.5f - distance, 1e-5f);
    CHECK_NEAR(ctx, normal_y(chained.hits[0]), 0.4f / distance, 1e-5f);
    CHECK(ctx, chained.hits[0].sub_shape_id_b == 1);

    internal_edge_removing::release(wrapper);
    collide_shape::release(chained);
    collide_shape::release(plain);
    sphere::destroy(ball);
    box::destroy(slab);
}

void test_internal_edge_removing_vertex_tolerance(TestCtx &ctx) {
    CollideShapeCollector chained = collide_shape::create_collector(collector::COLLECT_ALL);
    InternalEdgeRemovingCollector wrapper = internal_edge_removing::create();

    // two vertex contacts on triangles whose shared vertex is welded 5e-5 apart (squared 2.5e-9 <= 1e-8): one feature
    const float triangle_1[9] = {0, 0, 0, 1, 0, 0, 0.5f, 1, 0};
    const float triangle_2[9] = {1 + 5e-5f, 0, 0, 2, 0, 0, 1.5f, 1, 0};
    internal_edge_removing::set(wrapper, chained, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);
    collide_shape::add_hit(wrapper.collector, make_hit(2.0f, {0, -1, 0}, {1, 0, 0}, 3, triangle_1));
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, -1, 0}, {1 + 5e-5f, 0, 0}, 3, triangle_2));
    internal_edge_removing::flush(wrapper);
    CHECK(ctx, chained.hits.length == 1 && chained.hits[0].penetration == 2.0f);

    // a tolerance below their distance keeps them apart
    collide_shape::reset(chained);
    internal_edge_removing::set(wrapper, chained, 1e-10f);
    collide_shape::add_hit(wrapper.collector, make_hit(2.0f, {0, -1, 0}, {1, 0, 0}, 3, triangle_1));
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, -1, 0}, {1 + 5e-5f, 0, 0}, 3, triangle_2));
    internal_edge_removing::flush(wrapper);
    CHECK(ctx, chained.hits.length == 2);

    internal_edge_removing::release(wrapper);
    collide_shape::release(chained);
}

void test_internal_edge_removing_short_edge_is_not_degenerate(TestCtx &ctx) {
    CollideShapeCollector chained = collide_shape::create_collector(collector::COLLECT_ALL);
    InternalEdgeRemovingCollector wrapper = internal_edge_removing::create();
    internal_edge_removing::set(wrapper, chained, 0.0f);

    // the deeper contact voids (1, 0, 0) but not (1, 5e-7, 0)
    const float triangle[9] = {1, 0, 0, 2, 0, 0, 2, -1, 0};
    // the shallower contact is closest to the short edge (1, 0, 0) - (1, 5e-7, 0), nearer its voided end; above f32
    // epsilon squared it is an edge with only one end voided, so it goes through
    const float quad[12] = {0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 5e-7f, 0};
    collide_shape::add_hit(wrapper.collector, make_hit(2.0f, {0, -1, 0}, {1, 0, 0}, 3, triangle));
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {-1, 0, 0}, {1 + 1e-3f, 1e-7f, 0}, 4, quad));
    internal_edge_removing::flush(wrapper);
    CHECK(ctx, chained.hits.length == 2);

    internal_edge_removing::release(wrapper);
    collide_shape::release(chained);
}

void test_internal_edge_removing_reset_resets_chained(TestCtx &ctx) {
    CollideShapeCollector chained = collide_shape::create_collector(collector::COLLECT_ALL);
    InternalEdgeRemovingCollector wrapper = internal_edge_removing::create();
    internal_edge_removing::set(wrapper, chained, collide_shape::DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ);

    // a face contact is forwarded immediately
    collide_shape::add_hit(wrapper.collector, make_hit(1.0f, {0, 0, -1}, {0.5f, 0.3f, 0}, 3, TRIANGLE));
    CHECK(ctx, chained.hits.length == 1 && chained.has_hit);

    chained.early_out_fraction = 0.0f;
    internal_edge_removing::reset(wrapper);
    CHECK(ctx, chained.hits.length == 0 && !chained.has_hit);
    CHECK(ctx, chained.early_out_fraction == collector::UNBOUNDED && chained.body_id_b == body_id::INVALID_BODY_ID);
    CHECK(ctx, wrapper.collector.early_out_fraction == collector::UNBOUNDED);

    internal_edge_removing::release(wrapper);
    collide_shape::release(chained);
}
