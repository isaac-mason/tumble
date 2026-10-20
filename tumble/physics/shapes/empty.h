#pragma once

#include "tumble/physics/shapes/shapes.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// no volume and no collision geometry: an anchor for constraints, or a placeholder. it is in the broadphase but its
// dispatch entries do nothing
namespace empty {

using shapes::Shape;

/** a zero-size bounds and centre of mass at the origin, zero volume */
Shape *create();
void destroy(Shape *shape);

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** -1: no surface */
int32_t get_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** no-op */
void cast_ray(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray, const Shape *shape,
              const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
/** no-op */
void collide_point(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings, Vec3f point,
                   const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** no support: not convex */
shapes::ShapeDef def();

}  // namespace empty

TUMBLE_NAMESPACE_END
