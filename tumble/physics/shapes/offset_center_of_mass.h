#pragma once

// crashcat's offset-center-of-mass.ts: shifts an inner shape's centre of mass without moving its
// collision geometry, to steady a car or a boat by lowering it. shapes here are placed by their origin,
// not their centre of mass, so every query passes straight through; only the centre of mass and the
// inertia change. the inner shape is borrowed; the wrapper consumes no sub shape id bits.

#include <stdint.h>

#include "tumble/physics/shapes/shapes.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace offset_center_of_mass {

using shapes::Shape;

struct OffsetCenterOfMassShapeSettings {
    /** js-owned; outlives every offset shape over it */
    Shape *shape = nullptr;
    Float3 offset = {0.0f, 0.0f, 0.0f};
};

/** null when the memory cannot grow */
Shape *create(const OffsetCenterOfMassShapeSettings &settings);
/** frees the shape, not the shape it wraps */
void destroy(Shape *shape);
/** bounds, centre of mass and volume from the inner shape; call after changing the offset or the inner shape */
void update(Shape *shape);

/** the inner shape's, its inertia moved by the offset (parallel axis theorem) */
void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** the inner shape's */
int32_t get_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);
void get_leaf_shape(shapes::GetLeafShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_sub_shape_transformed_shape(shapes::GetSubShapeTransformedShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);

void cast_ray_vs_offset_center_of_mass(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings,
                                       const cast_ray::Rayf &ray, const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id,
                                       const Transformf &transform);
void collide_point_vs_offset_center_of_mass(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                                            Vec3f point, const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id,
                                            const Transformf &transform);

/** a's inner shape against b, through the collide table */
void collide_offset_center_of_mass_vs_shape(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                            const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a,
                                            const Transformf &transform_a, const Shape *shape_b,
                                            const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a against b's inner shape, through the collide table */
void collide_shape_vs_offset_center_of_mass(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                            const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a,
                                            const Transformf &transform_a, const Shape *shape_b,
                                            const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_offset_center_of_mass_vs_shape(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                         const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                         Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                         const Transformf &transform_b);
void cast_shape_vs_offset_center_of_mass(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                         const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                         Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                         const Transformf &transform_b);

shapes::ShapeDef def();

}  // namespace offset_center_of_mass

TUMBLE_NAMESPACE_END
