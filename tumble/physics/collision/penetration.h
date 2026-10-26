#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/collision/epa_convex_hull_builder.h"
#include "tumble/physics/collision/gjk.h"
#include "tumble/physics/collision/simplex.h"
#include "tumble/physics/collision/support.h"

TUMBLE_NAMESPACE_BEGIN

// penetration depth between convex supports, gjk for the shallow case and epa past the convex radius, and the
// penetration-aware shape cast

namespace penetration {

constexpr uint32_t EPA_MAX_POINTS_TO_INCLUDE_ORIGIN_IN_HULL = 32;
constexpr uint32_t EPA_MAX_POINTS = 128;

}  // namespace penetration

enum class PenetrationDepthStatus : uint32_t {
    NOT_COLLIDING = 0,
    COLLIDING = 1,
    /** penetrating further than the convex radii: run the epa step */
    INDETERMINATE = 2,
};

/** epa's support points: index i is one pair in all three */
struct EpaSupportPoints {
    /** minkowski difference points p - q */
    Vec3f y[penetration::EPA_MAX_POINTS];
    /** shape a support points */
    Vec3f p[penetration::EPA_MAX_POINTS];
    /** shape b support points */
    Vec3f q[penetration::EPA_MAX_POINTS];
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
    EpaConvexHullBuilderState hull;
    EpaConvexHullBuilderState::NewTriangles new_triangles;
};

namespace penetration {

/** back to not colliding with zero vectors. */
void reset(PenetrationDepth &depth);

/**
 * the gjk step: closest points between the cores of a and b (supports that exclude their convex
 * radius). NOT_COLLIDING when further apart than the radii sum; COLLIDING with the points moved
 * out by the radii when within it; INDETERMINATE (epa needed, `out_simplex` filled) when the cores
 * overlap.
 */
void penetration_depth_step_gjk(PenetrationDepth &out_penetration_depth, Simplex &out_simplex, Support &support_a,
                                Support &support_b, float convex_radius_a, float convex_radius_b, Vec3f direction,
                                float tolerance);

/**
 * the epa step from the gjk simplex. expects supports that INCLUDE their convex radius; the caller
 * fills them in include mode and folds any extra separation onto `add_radius`. false (and
 * NOT_COLLIDING) when no penetration could be established.
 */
bool penetration_depth_step_epa(PenetrationDepth &out, Support &support_a_including_radius,
                                Support &support_b_including_radius, float tolerance, const Simplex &simplex);

/**
 * gjk::cast_shape, then, when a and b already overlap at lambda 0 and `return_deepest_point` is
 * set, epa for the deepest point and the penetration axis. an invalid contact normal falls back to
 * the displacement.
 */
void cast_shape(GjkCastShapeResult &out, const Mat4f &transform_a_to_b, Support &shape_a_support,
                Support &shape_b_support, Vec3f displacement, float collision_tolerance, float penetration_tolerance,
                float convex_radius_a, float convex_radius_b, float max_lambda, bool return_deepest_point);

}  // namespace penetration

TUMBLE_NAMESPACE_END
