#pragma once

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// no volume and no collision geometry: an anchor for constraints, or a placeholder. it is in the broadphase but its
// dispatch entries do nothing

struct EmptyShape : Shape {
    static constexpr ShapeType TYPE = ShapeType::EMPTY;
};

namespace empty {

/** a zero-size bounds and centre of mass at the origin, zero volume */
Shape *create();

void compute_mass_properties(MassProperties &out, const Shape *shape);
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
/** -1: no surface */
int32_t get_material(const Shape *shape, SubShapeId sub_shape_id);
/** no-op */
void cast_ray(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray, const Shape *shape,
              const SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
/** no-op */
void collide_point(CollidePointCollector &collector, const CollidePointSettings &settings, Vec3f point,
                   const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** no support: not convex */
ShapeDef def();

}  // namespace empty

TUMBLE_NAMESPACE_END
