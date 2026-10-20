#pragma once

#include <stdint.h>

#include "tumble/physics/shapes/shapes.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// an inner shape placed by a rotation and a translation; borrowed, taking no sub shape id bits
namespace transformed {

using shapes::Shape;

struct TransformedShapeSettings {
    /** the caller's; outlives every transformed shape over it */
    Shape *shape = nullptr;
    Float3 position = {0.0f, 0.0f, 0.0f};
    Float4 quaternion = {0.0f, 0.0f, 0.0f, 1.0f};
};

/** null when the allocator fails */
Shape *create(const TransformedShapeSettings &settings);
/** frees the shape, not the shape it wraps */
void destroy(Shape *shape);
/** bounds, centre of mass and volume from the inner shape; call after changing the fields or the inner shape */
void update(Shape *shape);

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** the inner shape's */
int32_t get_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);
void get_leaf_shape(shapes::GetLeafShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_sub_shape_transformed_shape(shapes::GetSubShapeTransformedShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);

void cast_ray_vs_transformed(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                             const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
void collide_point_vs_transformed(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                                  Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                  const Transformf &transform_b);

/** a's inner shape at a's accumulated transform against b, through the collide table */
void collide_transformed_vs_shape(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                  const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a against b's inner shape at b's accumulated transform, through the collide table */
void collide_shape_vs_transformed(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                  const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_transformed_vs_shape(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *shape_a,
                               const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                               const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_shape_vs_transformed(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *shape_a,
                               const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                               const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

shapes::ShapeDef def();

}  // namespace transformed

TUMBLE_NAMESPACE_END
