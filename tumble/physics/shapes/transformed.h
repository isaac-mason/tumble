#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// an inner shape placed by a rotation and a translation; borrowed, taking no sub shape id bits

struct TransformedShape : Shape {
    static constexpr ShapeType TYPE = ShapeType::TRANSFORMED;
    /** held */
    Ref<Shape> shape;
    Float3 position;
    Float4 quaternion;
};

struct TransformedShapeSettings {
    /** the caller's; outlives every transformed shape over it */
    Shape *shape = nullptr;
    Float3 position = {0.0f, 0.0f, 0.0f};
    Float4 quaternion = {0.0f, 0.0f, 0.0f, 1.0f};
};

namespace transformed {

Shape *create(const TransformedShapeSettings &settings);
/** bounds, centre of mass and volume from the inner shape; call after changing the fields or the inner shape */
void update(Shape *shape);

void compute_mass_properties(MassProperties &out, const Shape *shape);
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
/** the inner shape's */
int32_t get_material(const Shape *shape, SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);
void get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);
void get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);

void cast_ray_vs_transformed(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                             const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
void collide_point_vs_transformed(CollidePointCollector &collector, const CollidePointSettings &settings,
                                  Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                  const Transformf &transform_b);

/** a's inner shape at a's accumulated transform against b, through the collide table */
void collide_transformed_vs_shape(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                  const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a against b's inner shape at b's accumulated transform, through the collide table */
void collide_shape_vs_transformed(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                  const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_transformed_vs_shape(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                               const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                               const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_shape_vs_transformed(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                               const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                               const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

ShapeDef def();

}  // namespace transformed

TUMBLE_NAMESPACE_END
