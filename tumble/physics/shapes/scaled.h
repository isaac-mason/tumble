#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// an inner shape scaled per axis, borrowed, taking no sub shape id bits. a sphere supports only uniform scaling; a
// compound scales its children's offsets too and under a non-uniform scale its children must not be rotated; a
// transformed shape passes a non-uniform scale into its inner shape's frame, exact for rotations that keep the axes
// aligned; a convex hull's non-uniform scale is slower than a uniform one

struct ScaledShape : Shape {
    static constexpr ShapeType TYPE = ShapeType::SCALED;
    /** held */
    Ref<Shape> shape;
    Float3 scale;
};

struct ScaledShapeSettings {
    /** the caller's; outlives every scaled shape over it */
    Shape *shape = nullptr;
    Float3 scale = {1.0f, 1.0f, 1.0f};
};

namespace scaled {

Shape *create(const ScaledShapeSettings &settings);
/** bounds, centre of mass and volume from the inner shape; call after changing the scale or the inner shape */
void update(Shape *shape);

void compute_mass_properties(MassProperties &out, const Shape *shape);
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
/** the inner shape's */
int32_t get_material(const Shape *shape, SubShapeId sub_shape_id);
/** the inner shape's, times the smallest absolute scale */
float get_inner_radius(const Shape *shape);
void get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);
void get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);

void cast_ray_vs_scaled(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                        const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
void collide_point_vs_scaled(CollidePointCollector &collector, const CollidePointSettings &settings,
                             Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                             const Transformf &transform_b);

/** a's inner shape at a's accumulated scale against b, through the collide table */
void collide_scaled_vs_shape(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                             const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                             const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a against b's inner shape at b's accumulated scale, through the collide table */
void collide_shape_vs_scaled(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                             const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                             const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_scaled_vs_shape(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                          const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                          const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_shape_vs_scaled(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                          const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                          const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

ShapeDef def();

}  // namespace scaled

TUMBLE_NAMESPACE_END
