#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// an inner shape's centre of mass moved without moving its geometry, to steady a car or a boat by lowering it. shapes
// are placed by their origin, so every query passes straight through and only the centre of mass and inertia change.
// the inner shape is borrowed; the wrapper takes no sub shape id bits

struct OffsetCenterOfMassShape : Shape {
    static constexpr ShapeType TYPE = ShapeType::OFFSET_CENTER_OF_MASS;
    /** held */
    Ref<Shape> shape;
    Float3 offset;
};

struct OffsetCenterOfMassShapeSettings {
    /** the caller's; outlives every offset shape over it */
    Shape *shape = nullptr;
    Float3 offset = {0.0f, 0.0f, 0.0f};
};

namespace offset_center_of_mass {

Shape *create(const OffsetCenterOfMassShapeSettings &settings);
/** bounds, centre of mass and volume from the inner shape; call after changing the offset or the inner shape */
void update(Shape *shape);

/** the inner shape's, its inertia moved by the offset (parallel axis theorem) */
void compute_mass_properties(MassProperties &out, const Shape *shape);
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
/** the inner shape's */
int32_t get_material(const Shape *shape, SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);
void get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);
void get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);

void cast_ray_vs_offset_center_of_mass(CastRayCollector &collector, const CastRaySettings &settings,
                                       const Rayf &ray, const Shape *shape, const SubShapeIdBuilder &sub_shape_id,
                                       const Transformf &transform);
void collide_point_vs_offset_center_of_mass(CollidePointCollector &collector, const CollidePointSettings &settings,
                                            Vec3f point, const Shape *shape, const SubShapeIdBuilder &sub_shape_id,
                                            const Transformf &transform);

/** a's inner shape against b, through the collide table */
void collide_offset_center_of_mass_vs_shape(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                            const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a,
                                            const Transformf &transform_a, const Shape *shape_b,
                                            const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a against b's inner shape, through the collide table */
void collide_shape_vs_offset_center_of_mass(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                            const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a,
                                            const Transformf &transform_a, const Shape *shape_b,
                                            const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_offset_center_of_mass_vs_shape(CastShapeCollector &collector, const CastShapeSettings &settings,
                                         const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                         Vec3f displacement_a, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                         const Transformf &transform_b);
void cast_shape_vs_offset_center_of_mass(CastShapeCollector &collector, const CastShapeSettings &settings,
                                         const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                         Vec3f displacement_a, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                         const Transformf &transform_b);

ShapeDef def();

}  // namespace offset_center_of_mass

TUMBLE_NAMESPACE_END
