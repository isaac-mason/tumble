#pragma once

// crashcat's penetration.ts: penetration depth between convex supports, gjk for the shallow case
// and epa past the convex radius, plus the penetration-aware shape cast. after jolt's
// EPAPenetrationDepth.

#include <stdint.h>

#include "math/types.h"
#include "physics/collision/epa_convex_hull_builder.h"
#include "physics/collision/gjk.h"
#include "physics/collision/simplex.h"
#include "physics/collision/support.h"

namespace penetration {

constexpr uint32_t EPA_MAX_POINTS_TO_INCLUDE_ORIGIN_IN_HULL = 32;
constexpr uint32_t EPA_MAX_POINTS = 128;

enum PenetrationDepthStatus : uint32_t {
    NOT_COLLIDING = 0,
    COLLIDING = 1,
    /** penetrating further than the convex radii: run the epa step */
    INDETERMINATE = 2,
};

/** epa's support points, as jolt's EPAPenetrationDepth::SupportPoints: index i is one pair in all three. */
struct EpaSupportPoints {
    /** minkowski difference points p - q */
    Vec3f y[EPA_MAX_POINTS];
    /** shape a support points */
    Vec3f p[EPA_MAX_POINTS];
    /** shape b support points */
    Vec3f q[EPA_MAX_POINTS];
    uint32_t size;
};

/** zeroed is not colliding with zero vectors and empty epa scratch. */
struct PenetrationDepth {
    PenetrationDepthStatus status;
    Vec3f penetration_axis;
    Vec3f point_a;
    Vec3f point_b;

    /** epa scratch */
    EpaSupportPoints support_points;
    epa_convex_hull_builder::EpaConvexHullBuilderState hull;
    epa_convex_hull_builder::NewTriangles new_triangles;
};

/** back to not colliding with zero vectors. */
void reset(PenetrationDepth &depth);

/**
 * the gjk step: closest points between the cores of a and b (supports that exclude their convex
 * radius). NOT_COLLIDING when further apart than the radii sum; COLLIDING with the points moved
 * out by the radii when within it; INDETERMINATE (epa needed, `out_simplex` filled) when the cores
 * overlap.
 */
void penetration_depth_step_gjk(PenetrationDepth &out_penetration_depth, simplex::Simplex &out_simplex, support::Support &support_a,
                                support::Support &support_b, float convex_radius_a, float convex_radius_b, Vec3f direction,
                                float tolerance);

/**
 * the epa step from the gjk simplex. expects supports that INCLUDE their convex radius; the caller
 * fills them in include mode and folds any extra separation onto `add_radius`. false (and
 * NOT_COLLIDING) when no penetration could be established.
 */
bool penetration_depth_step_epa(PenetrationDepth &out, support::Support &support_a_including_radius,
                                support::Support &support_b_including_radius, float tolerance, const simplex::Simplex &simplex);

/**
 * gjk::cast_shape, then, when a and b already overlap at lambda 0 and `return_deepest_point` is
 * set, epa for the deepest point and the penetration axis. an invalid contact normal falls back to
 * the displacement.
 */
void cast_shape(gjk::GjkCastShapeResult &out, const Mat4f &transform_a_to_b, support::Support &shape_a_support,
                support::Support &shape_b_support, Vec3f displacement, float collision_tolerance, float penetration_tolerance,
                float convex_radius_a, float convex_radius_b, float max_lambda, bool return_deepest_point);

}  // namespace penetration
