#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// an axis-aligned box about its centre, rounded by a convex radius

struct BoxShape : Shape {
    static constexpr ShapeType TYPE = ShapeType::BOX;
    Float3 half_extents;
    float convex_radius;
    float density;
};

struct BoxShapeSettings {
    Float3 half_extents;
    float convex_radius = support::DEFAULT_CONVEX_RADIUS;
    float density = shapes::DEFAULT_SHAPE_DENSITY;
    int32_t material_id = -1;
};

namespace box {

/** the convex radius is clamped to the smallest half extent */
Shape *create(const BoxShapeSettings &settings);
/** after the fields changed: the convex radius clamped to fit, the bounds and volume recomputed */
void update(Shape *shape);

void compute_mass_properties(MassProperties &out, const Shape *shape);
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
/** the face whose outward normal is most opposed to `direction` */
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);
void set_support(Support &out, const Shape *shape, SupportFunctionMode mode, Vec3f scale);

/**
 * analytical sphere vs box, a the sphere and b the box: the sphere centre clamped to the box's shrunk core (as set_support
 * excludes the convex radius), the combined radius covering the rounded shell. skips gjk/epa; the deep case
 * (centre inside the core) is a per-axis scan for the nearest face. agrees with the convex path on shallow contacts
 * to within floating point.
 */
void collide_sphere_vs_box(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                           const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                           const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/**
 * analytic ray vs box (slab test) against the sharp box of |scale| * half extents, the geometry the gjk ray casts against;
 * reports the entry behind treat_convex_as_solid, the exit when collide_with_backfaces_convex
 */
void cast_ray_vs_box(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                     const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
void collide_point_vs_box(CollidePointCollector &collector, const CollidePointSettings &settings,
                          Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                          const Transformf &transform_b);

ShapeDef def();

}  // namespace box

TUMBLE_NAMESPACE_END
