#pragma once

#include <stdint.h>

#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/shapes/utils/bvh.h"
#include "tumble/physics/shapes/utils/static_compound_bvh.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// a compound whose children never change after create, walked through a bvh over them; building the bvh reorders the
// children, and a sub shape id indexes them in that order
namespace static_compound {

using compound::CompoundShapeChild;
using shapes::Shape;

/** the child list (bvh order) and its bvh, owned by the static compound shape */
struct StaticCompoundData {
    collections::Vec<CompoundShapeChild> children;
    /** pre-order, the root at 0; empty when there are no children */
    collections::Vec<bvh::Node> bvh;
    static_compound_bvh::BvhBuildSettings bvh_settings;
};

struct StaticCompoundShape : Shape {
    static constexpr shapes::ShapeType TYPE = shapes::SHAPE_STATIC_COMPOUND;
    StaticCompoundData *data;
};

struct StaticCompoundShapeSettings {
    /** read during create only; the shapes they point at are borrowed */
    const CompoundShapeChild *children = nullptr;
    uint32_t num_children = 0;
    /** maximum children per leaf node */
    uint32_t bvh_max_leaf_children = 4;
};

/** null (error set) when the allocator fails */
Shape *create(const StaticCompoundShapeSettings &settings);
/** frees the shape, its child list and its bvh, not the children */
void destroy(Shape *shape);
/**
 * rebuilds the bvh (reordering the children, which invalidates sub shape ids taken before) and
 * recomputes bounds, centre of mass and volume. false (error set) when the allocator fails.
 */
bool update(Shape *shape);

void cast_ray(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray, const Shape *shape,
              const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
void collide_point(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings, Vec3f point,
                   const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** a's children whose bounds overlap b's, against b through the collide table */
void collide_static_compound_vs_shape(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                      const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a,
                                      const Transformf &transform_a, const Shape *shape_b,
                                      const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a against b's children whose bounds overlap a's, through the collide table */
void collide_shape_vs_static_compound(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                      const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a,
                                      const Transformf &transform_a, const Shape *shape_b,
                                      const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a's children whose bounds overlap b's bounds swept back along the displacement, cast against b */
void cast_static_compound_vs_shape(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                   const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                   Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                   const Transformf &transform_b);
/** a cast against b's children, nearest first along the sweep of a's bounds */
void cast_shape_vs_static_compound(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                   const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                   Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                   const Transformf &transform_b);

shapes::ShapeDef def();

}  // namespace static_compound

TUMBLE_NAMESPACE_END
