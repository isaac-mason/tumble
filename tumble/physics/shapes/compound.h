#pragma once

#include <stdint.h>

#include "tumble/physics/shapes/shapes.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// child shapes, each placed by a position and rotation; a sub shape id's level indexes the children. the static
// compound shares the functions that only walk the child list
namespace compound {

using shapes::Shape;

struct CompoundShapeChild {
    Float3 position;
    Float4 quaternion;
    /** the caller's; outlives every compound it is a child of */
    Shape *shape;
};

/** the child list, owned by the compound shape: copied from the settings by create, freed by destroy */
struct CompoundData {
    collections::Vec<CompoundShapeChild> children;
    /** each child's bounds in the compound's unscaled space, for culling children against a query; filled by update */
    collections::Vec<Box3f> child_bounds;
};

struct CompoundShapeSettings {
    /** read during create only; the shapes they point at are borrowed */
    const CompoundShapeChild *children = nullptr;
    /** at least one */
    uint32_t num_children = 0;
};

/** null when the memory cannot grow */
Shape *create(const CompoundShapeSettings &settings);
/** frees the shape and its child list, not the children */
void destroy(Shape *shape);
/** bounds, centre of mass and volume from the children; call after changing the child list */
void update(Shape *shape);

/** the child list of a compound or a static compound */
const collections::Vec<CompoundShapeChild> &children_of(const Shape *shape);
/** a child's world transform: the parent's scale applies to the child's position and passes on unchanged */
void child_transform(Transformf &out, const Transformf &parent, const CompoundShapeChild &child);

/** the children's mass properties summed about the compound's centre of mass */
void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** the material of the child's leaf */
int32_t get_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** the smallest child's */
float get_inner_radius(const Shape *shape);
void get_leaf_shape(shapes::GetLeafShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_sub_shape_transformed_shape(shapes::GetSubShapeTransformedShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);

void cast_ray_vs_compound(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                          const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
void collide_point_vs_compound(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                               Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                               const Transformf &transform_b);

/** each of a's children against b, through the collide table */
void collide_compound_vs_shape(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                               const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                               const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a against each of b's children, through the collide table */
void collide_shape_vs_compound(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                               const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                               const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_compound_vs_shape(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *shape_a,
                            const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                            const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_shape_vs_compound(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *shape_a,
                            const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                            const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

shapes::ShapeDef def();

}  // namespace compound

TUMBLE_NAMESPACE_END
