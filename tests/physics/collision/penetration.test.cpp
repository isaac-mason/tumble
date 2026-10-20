#include "tumble/instance.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/gjk.h"
#include "tumble/physics/collision/penetration.h"
#include "tumble/physics/collision/support.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/dispatch.h"
#include "test.h"
#include "test_math.h"

static constexpr float COLLISION_TOLERANCE = 1e-4f;
static constexpr float PENETRATION_TOLERANCE = 1e-4f;

static float depth_of(const penetration::PenetrationDepth &depth) {
    const Vec3f d = vec3f::subtract(depth.point_a, depth.point_b);
    return vec3f::length(d);
}

/** a sphere support: include mode rounds the origin by the radius, exclude mode reports it instead. */
static support::Support sphere_support(float radius, bool include_radius) {
    support::Support s = support::create();
    s.get_support = support::sphere_support;
    s.sphere.radius = include_radius ? radius : 0.0f;
    s.convex_radius = include_radius ? 0.0f : radius;
    return s;
}

/** a box support: include mode is the whole box, exclude mode the core shrunk by the convex radius. */
static support::Support box_support(const Float3 &half_extents, float convex_radius, bool include_radius) {
    support::Support s = support::create();
    s.get_support = support::box_support;
    if (include_radius) {
        s.box.half_extents = vec3f::load(half_extents);
        s.convex_radius = 0.0f;
    } else {
        s.box.half_extents = vec3f::subtract(vec3f::load(half_extents), vec3f::splat(convex_radius));
        s.convex_radius = convex_radius;
    }
    return s;
}

static void place(support::Support &s, const Float3 &position) {
    s.has_transform = true;
    s.transform = translation(vec3f::load(position));
}

/** the narrowphase's two steps as convex.ts runs them: gjk on the cores, epa on the whole shapes when the cores overlap. */
struct TwoStep {
    penetration::PenetrationDepthStatus gjk_status;
    bool epa_ran;
};

static TwoStep run_two_step(penetration::PenetrationDepth &depth, support::Support &core_a, support::Support &core_b,
                            support::Support &whole_a, support::Support &whole_b, const Float3 &direction) {
    simplex::Simplex simplex;
    simplex.size = 0;
    penetration::penetration_depth_step_gjk(depth, simplex, core_a, core_b, core_a.convex_radius, core_b.convex_radius, vec3f::load(direction),
                                            COLLISION_TOLERANCE);
    TwoStep result = {depth.status, false};
    if (depth.status == penetration::INDETERMINATE) {
        result.epa_ran = true;
        penetration::penetration_depth_step_epa(depth, whole_a, whole_b, PENETRATION_TOLERANCE, simplex);
    }
    return result;
}

/** epa forced on the whole shapes: gjk with no radius finds them overlapping and hands epa its simplex. */
static bool run_epa(penetration::PenetrationDepth &depth, support::Support &whole_a, support::Support &whole_b, const Float3 &direction) {
    simplex::Simplex simplex;
    simplex.size = 0;
    penetration::penetration_depth_step_gjk(depth, simplex, whole_a, whole_b, 0.0f, 0.0f, vec3f::load(direction), COLLISION_TOLERANCE);
    if (depth.status != penetration::INDETERMINATE) return false;
    return penetration::penetration_depth_step_epa(depth, whole_a, whole_b, PENETRATION_TOLERANCE, simplex);
}

void test_penetration_zeroed_and_reset(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    penetration::PenetrationDepth depth = {};
    CHECK(ctx, depth.status == penetration::NOT_COLLIDING);
    CHECK(ctx, vec3f::exact_equals(depth.penetration_axis, vec3f::zero()) && vec3f::exact_equals(depth.point_a, vec3f::zero()) && vec3f::exact_equals(depth.point_b, vec3f::zero()));

    depth.status = penetration::COLLIDING;
    depth.point_a = vec3f::make(1, 2, 3);
    penetration::reset(depth);
    CHECK(ctx, depth.status == penetration::NOT_COLLIDING && vec3f::exact_equals(depth.point_a, vec3f::zero()));
}

void test_penetration_spheres_overlapping(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    penetration::PenetrationDepth depth = {};

    const float radius_a = 1.0f, radius_b = 0.5f;
    const Float3 centre_b = {1.0f, 0.4f, -0.2f};
    const float distance = vec3f::length(vec3f::load(centre_b));
    const float expected_depth = radius_a + radius_b - distance;
    const Vec3f axis = unit(vec3f::load(centre_b));

    // the gjk step alone: the cores (the centres) are apart, the overlap is within the radii
    support::Support core_a = sphere_support(radius_a, false);
    support::Support core_b = sphere_support(radius_b, false);
    place(core_b, centre_b);
    support::Support whole_a = sphere_support(radius_a, true);
    support::Support whole_b = sphere_support(radius_b, true);
    place(whole_b, centre_b);

    const TwoStep steps = run_two_step(depth, core_a, core_b, whole_a, whole_b, centre_b);
    CHECK(ctx, steps.gjk_status == penetration::COLLIDING && !steps.epa_ran);
    CHECK_NEAR(ctx, depth_of(depth), expected_depth, 1e-5f);
    // the axis runs along the centre line, from a towards b
    CHECK(ctx, near3(unit(depth.penetration_axis), axis, 1e-5f));
    // a's deepest point is its surface towards b, b's its surface towards a
    const Vec3f surface_a = vec3f::scale(axis, radius_a);
    const Vec3f surface_b = vec3f::scale_and_add(vec3f::load(centre_b), axis, -radius_b);
    CHECK(ctx, near3(depth.point_a, surface_a, 1e-5f));
    CHECK(ctx, near3(depth.point_b, surface_b, 1e-5f));
    const Vec3f gjk_point_a = depth.point_a, gjk_point_b = depth.point_b, gjk_axis = unit(depth.penetration_axis);

    // epa on the whole spheres agrees: its axis is the penetration vector itself, pointing from a towards b
    penetration::PenetrationDepth epa = {};
    CHECK(ctx, run_epa(epa, whole_a, whole_b, centre_b));
    CHECK(ctx, epa.status == penetration::COLLIDING);
    CHECK_NEAR(ctx, depth_of(epa), expected_depth, 2e-3f);
    CHECK_NEAR(ctx, vec3f::length(epa.penetration_axis), expected_depth, 2e-3f);
    CHECK(ctx, near3(unit(epa.penetration_axis), gjk_axis, 5e-3f));
    CHECK(ctx, near3(epa.point_a, gjk_point_a, 5e-3f));
    CHECK(ctx, near3(epa.point_b, gjk_point_b, 5e-3f));

    // centres coincide: the cores overlap and epa takes over; any axis, the depth is the radii sum
    place(core_b, {0, 0, 0});
    place(whole_b, {0, 0, 0});
    const TwoStep concentric = run_two_step(depth, core_a, core_b, whole_a, whole_b, {1, 0, 0});
    CHECK(ctx, concentric.gjk_status == penetration::INDETERMINATE && concentric.epa_ran);
    CHECK(ctx, depth.status == penetration::COLLIDING);
    // epa's polytope is inscribed in the round minkowski sum, so it can only fall short: crashcat reports 1.4454 here
    CHECK(ctx, depth_of(depth) <= radius_a + radius_b + 1e-5f && depth_of(depth) > 1.4f);

    // apart by more than the radii: not colliding, the output cleared
    place(core_b, {3.0f, 0.0f, 0.0f});
    place(whole_b, {3.0f, 0.0f, 0.0f});
    const TwoStep apart = run_two_step(depth, core_a, core_b, whole_a, whole_b, {1, 0, 0});
    CHECK(ctx, apart.gjk_status == penetration::NOT_COLLIDING && depth.status == penetration::NOT_COLLIDING);
    CHECK(ctx, vec3f::exact_equals(depth.point_a, vec3f::zero()) && vec3f::exact_equals(depth.point_b, vec3f::zero()) && vec3f::exact_equals(depth.penetration_axis, vec3f::zero()));

}

void test_penetration_box_resting_in_box(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    penetration::PenetrationDepth depth = {};
    penetration::PenetrationDepth epa = {};

    // dyadic sizes and offsets, so f32 holds them exactly (see test_penetration_epa_coplanar_box_faces for why that matters)
    const Float3 half = {1.0f, 1.0f, 1.0f};
    const float convex_radius = 0.0625f;
    support::Support core_a = box_support(half, convex_radius, false);
    support::Support whole_a = box_support(half, convex_radius, true);

    // b sits on a's top face, offset sideways, sunk by `sinks[i]`: the first within the convex radii, the rest deeper
    const float sinks[3] = {0.0625f, 0.25f, 0.5f};
    for (uint32_t i = 0; i < 3; i++) {
        const Float3 centre_b = {0.25f, 2.0f - sinks[i], -0.125f};
        support::Support core_b = box_support(half, convex_radius, false);
        support::Support whole_b = box_support(half, convex_radius, true);
        place(core_b, centre_b);
        place(whole_b, centre_b);

        const TwoStep steps = run_two_step(depth, core_a, core_b, whole_a, whole_b, centre_b);
        // the gjk step handles the shallow case, epa takes over once the cores overlap
        if (i == 0) {
            CHECK(ctx, steps.gjk_status == penetration::COLLIDING && !steps.epa_ran);
        } else {
            CHECK(ctx, steps.gjk_status == penetration::INDETERMINATE && steps.epa_ran);
        }
        CHECK(ctx, depth.status == penetration::COLLIDING);
        CHECK_NEAR(ctx, depth_of(depth), sinks[i], 1e-5f);
        CHECK(ctx, near3(unit(depth.penetration_axis), 0, 1, 0, 1e-5f));
        // a's point on its top face, b's on its bottom face, one above the other inside the overlap
        CHECK_NEAR(ctx, vec3f::y(depth.point_a), 1.0f, 1e-5f);
        CHECK_NEAR(ctx, vec3f::y(depth.point_b), 1.0f - sinks[i], 1e-5f);
        CHECK_NEAR(ctx, vec3f::x(depth.point_a), vec3f::x(depth.point_b), 1e-5f);
        CHECK_NEAR(ctx, vec3f::z(depth.point_a), vec3f::z(depth.point_b), 1e-5f);
        CHECK(ctx, vec3f::x(depth.point_a) >= -0.75f - 1e-5f && vec3f::x(depth.point_a) <= 1.0f + 1e-5f);
        CHECK(ctx, vec3f::z(depth.point_a) >= -1.0f - 1e-5f && vec3f::z(depth.point_a) <= 0.875f + 1e-5f);

        // epa on the whole boxes agrees with whichever step answered
        CHECK(ctx, run_epa(epa, whole_a, whole_b, centre_b));
        CHECK_NEAR(ctx, depth_of(epa), sinks[i], 1e-5f);
        CHECK(ctx, near3(unit(epa.penetration_axis), 0, 1, 0, 1e-5f));
        CHECK_NEAR(ctx, vec3f::y(epa.point_a), 1.0f, 1e-5f);
        CHECK_NEAR(ctx, vec3f::y(epa.point_b), 1.0f - sinks[i], 1e-5f);
    }

}

void test_penetration_epa_coplanar_box_faces(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    penetration::PenetrationDepth depth = {};

    // unit boxes, b sunk 0.3 into a's top: the minkowski difference has many coplanar support points,
    // and epa's answer turns on the last bit of the simplex it starts from
    const Float3 centre_b = {0.3f, 1.7f, -0.2f};
    support::Support core_a = box_support({1, 1, 1}, 0.05f, false);
    support::Support core_b = box_support({1, 1, 1}, 0.05f, false);
    support::Support whole_a = box_support({1, 1, 1}, 0.05f, true);
    support::Support whole_b = box_support({1, 1, 1}, 0.05f, true);
    place(core_b, centre_b);
    place(whole_b, centre_b);

    // the simplex crashcat's gjk ends on, as decimal literals: epa converges on the top face
    simplex::Simplex literal;
    literal.size = 4;
    const float p[12] = {0.95f, 0.95f, -0.95f, -0.95f, -0.95f, 0.95f, -0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f};
    const float q[12] = {-0.65f, 0.75f, 0.75f, 1.25f, 2.65f, -1.15f, 1.25f, 0.75f, -1.15f, -0.65f, 2.65f, -1.15f};
    for (uint32_t i = 0; i < 4; i++) {
        literal.p[i] = vec3f::load(p, i * 3);
        literal.q[i] = vec3f::load(q, i * 3);
        literal.y[i] = vec3f::subtract(literal.p[i], literal.q[i]);
    }
    CHECK(ctx, penetration::penetration_depth_step_epa(depth, whole_a, whole_b, PENETRATION_TOLERANCE, literal));
    CHECK_NEAR(ctx, depth_of(depth), 0.3f, 1e-6f);
    CHECK(ctx, near3(unit(depth.penetration_axis), 0, 1, 0, 1e-6f));

    // the same simplex as f32 computes it (two q.y are 0.75 + 1 ulp): the fourth new face is a sliver whose
    // normal faces the origin, so epa stops on the hull defect and reports its last good face, tilted.
    // the expected values are crashcat's algorithm run in emulated f32 on this input
    simplex::Simplex computed;
    computed.size = 0;
    penetration::penetration_depth_step_gjk(depth, computed, core_a, core_b, core_a.convex_radius, core_b.convex_radius, vec3f::load(centre_b),
                                            COLLISION_TOLERANCE);
    CHECK(ctx, depth.status == penetration::INDETERMINATE && computed.size == 4);
    bool one_ulp_off = true;
    float computed_p[12], computed_q[12], literal_p[12], literal_q[12];
    flatten3(computed_p, computed.p, 4);
    flatten3(computed_q, computed.q, 4);
    flatten3(literal_p, literal.p, 4);
    flatten3(literal_q, literal.q, 4);
    for (uint32_t i = 0; i < 12; i++) {
        const bool differs = computed_q[i] != literal_q[i];
        one_ulp_off = one_ulp_off && computed_p[i] == literal_p[i] && differs == (i == 1 || i == 7);
    }
    CHECK(ctx, one_ulp_off);
    CHECK(ctx, penetration::penetration_depth_step_epa(depth, whole_a, whole_b, PENETRATION_TOLERANCE, computed));
    CHECK_NEAR(ctx, depth_of(depth), 0.2866436f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::x(depth.penetration_axis), 0.0075380f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(depth.penetration_axis), 0.2864453f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(depth.penetration_axis), -0.0075380f, 1e-6f);

}

void test_penetration_epa_fills_small_simplices(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    penetration::PenetrationDepth depth = {};

    // a small box sunk 0.3 into the top of a big one: whatever simplex epa starts from, the answer is the same
    support::Support a = box_support({2.0f, 1.0f, 2.0f}, 0.0f, true);
    support::Support b = box_support({0.5f, 0.5f, 0.5f}, 0.0f, true);
    place(b, {0.2f, 1.2f, 0.1f});

    // one point: the origin as p - q, from a point both boxes hold
    simplex::Simplex one;
    one.size = 1;
    one.y[0] = vec3f::zero();
    one.p[0] = vec3f::make(0.2f, 0.9f, 0.1f);
    one.q[0] = vec3f::make(0.2f, 0.9f, 0.1f);
    CHECK(ctx, penetration::penetration_depth_step_epa(depth, a, b, PENETRATION_TOLERANCE, one));
    CHECK_NEAR(ctx, depth_of(depth), 0.3f, 1e-4f);
    CHECK(ctx, near3(unit(depth.penetration_axis), 0, 1, 0, 1e-4f));
    CHECK_NEAR(ctx, vec3f::y(depth.point_a), 1.0f, 1e-4f);
    CHECK_NEAR(ctx, vec3f::y(depth.point_b), 0.7f, 1e-4f);

    // two points: support pairs along +-x, a segment through the minkowski difference
    simplex::Simplex two;
    two.size = 2;
    const Float3 directions[2] = {{1.0f, 0.1f, 0.05f}, {-1.0f, -0.1f, -0.05f}};
    for (uint32_t i = 0; i < 2; i++) {
        const Vec3f direction = vec3f::load(directions[i]);
        two.p[i] = a.get_support(a, direction);
        two.q[i] = b.get_support(b, vec3f::negate(direction));
        two.y[i] = vec3f::subtract(two.p[i], two.q[i]);
    }
    CHECK(ctx, penetration::penetration_depth_step_epa(depth, a, b, PENETRATION_TOLERANCE, two));
    CHECK_NEAR(ctx, depth_of(depth), 0.3f, 1e-4f);
    CHECK(ctx, near3(unit(depth.penetration_axis), 0, 1, 0, 1e-4f));

    // the add radius epa is handed inflates both: 0.1 on a, 0.05 on b, all along the axis
    a.add_radius = 0.1f;
    b.add_radius = 0.05f;
    CHECK(ctx, penetration::penetration_depth_step_epa(depth, a, b, PENETRATION_TOLERANCE, one));
    CHECK_NEAR(ctx, depth_of(depth), 0.45f, 1e-4f);
    CHECK(ctx, near3(unit(depth.penetration_axis), 0, 1, 0, 1e-4f));

}

void test_penetration_cast_shape_starting_inside(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    // a sphere of radius 0.5 (exclude mode: a point core) whose centre is already inside a unit box rounded by 0.05
    support::Support sphere = sphere_support(0.5f, false);
    support::Support box = box_support({1.0f, 1.0f, 1.0f}, 0.05f, false);
    Mat4f a_to_b = translation(vec3f::make(0.7f, 0.0f, 0.0f));

    // without asking for the deepest point: a hit at fraction 0, and since the cores overlap there is no
    // usable contact normal, so the cast direction stands in for it
    gjk::GjkCastShapeResult result = gjk::create_gjk_cast_shape_result();
    penetration::cast_shape(result, a_to_b, sphere, box, vec3f::make(0, 5, 0), COLLISION_TOLERANCE, PENETRATION_TOLERANCE, sphere.convex_radius,
                            box.convex_radius, 1.0f, false);
    CHECK(ctx, result.hit);
    CHECK(ctx, result.lambda == 0.0f);
    CHECK(ctx, vec3f::exact_equals(result.separating_axis, vec3f::make(0, 5, 0)));
    // epa did not run: the radii were not folded onto the supports
    CHECK(ctx, sphere.add_radius == 0.0f && box.add_radius == 0.0f);

    // touching within the convex radii only (the sphere's centre outside the box's core): a hit at 0 with
    // gjk's own normal, no epa needed even when the deepest point is asked for
    a_to_b = translation(vec3f::make(1.3f, 0.2f, 0.0f));
    penetration::cast_shape(result, a_to_b, sphere, box, vec3f::make(0, 5, 0), COLLISION_TOLERANCE, PENETRATION_TOLERANCE, sphere.convex_radius,
                            box.convex_radius, 1.0f, true);
    CHECK(ctx, result.hit);
    CHECK(ctx, result.lambda == 0.0f);
    CHECK(ctx, near3(unit(result.separating_axis), -1, 0, 0, 1e-3f));
    CHECK(ctx, sphere.add_radius == 0.0f && box.add_radius == 0.0f);

    // the deepest point: a whole sphere (include mode) of radius 0.5 centred 0.7 inside a unit box's +x
    // face. epa continues from the cast's simplex and finds the shortest way out: 0.3 to the face plus
    // the radius, 0.8 along x, with the two contact points 0.8 apart on that line
    support::Support whole_sphere = sphere_support(0.5f, true);
    support::Support whole_box = box_support({1.0f, 1.0f, 1.0f}, 0.05f, true);
    a_to_b = translation(vec3f::make(0.7f, 0.0f, 0.0f));
    penetration::cast_shape(result, a_to_b, whole_sphere, whole_box, vec3f::make(0, 5, 0), COLLISION_TOLERANCE, PENETRATION_TOLERANCE,
                            whole_sphere.convex_radius, whole_box.convex_radius, 1.0f, true);
    CHECK(ctx, result.hit);
    CHECK(ctx, result.lambda == 0.0f);
    CHECK(ctx, result.simplex.size >= 1 && result.simplex.size <= 4);
    CHECK(ctx, scalar::abs(vec3f::x(unit(result.separating_axis))) > 0.999f);
    CHECK_NEAR(ctx, scalar::abs(vec3f::x(result.point_a) - vec3f::x(result.point_b)), 0.8f, 2e-2f);
    CHECK_NEAR(ctx, vec3f::y(result.point_a), 0.0f, 2e-2f);
    CHECK_NEAR(ctx, vec3f::z(result.point_a), 0.0f, 2e-2f);
    CHECK_NEAR(ctx, vec3f::y(result.point_b), 0.0f, 2e-2f);
    CHECK_NEAR(ctx, vec3f::z(result.point_b), 0.0f, 2e-2f);
}

void test_penetration_cast_shape_sweep(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    // a sphere of radius 0.5 sweeping along +x into a unit box rounded by 0.05, as in the gjk cast test
    const Mat4f a_to_b = translation(vec3f::make(-5.0f, 0.2f, -0.1f));
    const Float3 displacements[3] = {{10, 0, 0}, {0, 10, 0}, {3, 0, 0}};
    for (uint32_t i = 0; i < 3; i++) {
        support::Support sphere = sphere_support(0.5f, false);
        support::Support box = box_support({1.0f, 1.0f, 1.0f}, 0.05f, false);
        gjk::GjkCastShapeResult with_penetration = gjk::create_gjk_cast_shape_result();
        penetration::cast_shape(with_penetration, a_to_b, sphere, box, vec3f::load(displacements[i]), COLLISION_TOLERANCE, PENETRATION_TOLERANCE,
                                sphere.convex_radius, box.convex_radius, 1.0f, true);

        support::Support sphere_plain = sphere_support(0.5f, false);
        support::Support box_plain = box_support({1.0f, 1.0f, 1.0f}, 0.05f, false);
        gjk::GjkCastShapeResult plain = gjk::create_gjk_cast_shape_result();
        gjk::cast_shape(plain, a_to_b, sphere_plain, box_plain, vec3f::load(displacements[i]), COLLISION_TOLERANCE, sphere_plain.convex_radius,
                        box_plain.convex_radius, 1.0f);

        // no penetration involved: exactly the plain cast's answer
        CHECK(ctx, with_penetration.hit == plain.hit);
        if (!plain.hit) continue;
        CHECK(ctx, with_penetration.lambda == plain.lambda);
        CHECK(ctx, vec3f::exact_equals(with_penetration.point_a, plain.point_a));
        CHECK(ctx, vec3f::exact_equals(with_penetration.point_b, plain.point_b));
        CHECK(ctx, vec3f::exact_equals(with_penetration.separating_axis, plain.separating_axis));

        // first touch with the sphere's centre at x = -1.5: 3.5 of the 10
        CHECK(ctx, i == 0);
        CHECK_NEAR(ctx, with_penetration.lambda, 0.35f, 1e-4f);
        CHECK(ctx, near3(with_penetration.point_b, -1.0f, 0.2f, -0.1f, 1e-3f));
        CHECK(ctx, near3(unit(with_penetration.separating_axis), 1, 0, 0, 1e-3f));
    }
}


/** box half extents 1 against box half extents 1 with centres 0.3 apart along x: 1.7 deep, through epa. */
static bool deep_boxes_collide(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *a = box::create({.half_extents = {1, 1, 1}});
    shapes::Shape *b = box::create({.half_extents = {1, 1, 1}});
    const sub_shape::SubShapeIdBuilder no_sub_shape = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};
    const Transformf transform_a = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    const Transformf transform_b = {{0.3f, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};

    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    convex::collide_convex_vs_convex(collector, collide_shape::default_settings(), a, no_sub_shape, transform_a, b, no_sub_shape,
                                     transform_b);
    const bool hit = collector.hits.length == 1;
    CHECK(ctx, hit);
    if (hit) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 1.7f, 1e-4f);
        CHECK(ctx, vec3f::dot(unit(collector.hits[0].penetration_axis), vec3f::make(1, 0, 0)) > 0.9999f);
    }

    collide_shape::release(collector);
    box::destroy(a);
    box::destroy(b);
    return hit;
}

void test_penetration_deep_boxes_across_heap_resets(TestCtx &ctx) {
    // convex's epa scratch is a static that outlives the heap; it must hold no heap memory
    CHECK(ctx, util::init(test_heap_begin()));
    CHECK(ctx, deep_boxes_collide(ctx));
    CHECK(ctx, util::init(test_heap_begin()));
    CHECK(ctx, deep_boxes_collide(ctx));
}
