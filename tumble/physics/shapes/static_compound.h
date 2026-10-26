#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/shapes/utils/bvh.h"
#include "tumble/physics/shapes/utils/static_compound_bvh.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct StaticCompoundShape : Shape {
    // a compound whose children never change after create, walked through a bvh over them; building the bvh reorders the
    // children, and a sub shape id indexes them in that order

    /** the child list (bvh order) and its bvh, owned by the static compound shape */
    struct Data {
        collections::Array<CompoundShape::Child> children;
        /** pre-order, the root at 0; empty when there are no children */
        collections::Array<BvhNode> bvh;
        StaticCompoundBvhBuildSettings bvh_settings;
    };

    static constexpr ShapeType TYPE = ShapeType::STATIC_COMPOUND;
    Data *data;
};

struct StaticCompoundShapeSettings {
    /** read during create only; the compound holds the shapes they point at */
    const CompoundShapeSettings::Child *children = nullptr;
    uint32_t num_children = 0;
    /** maximum children per leaf node */
    uint32_t bvh_max_leaf_children = 4;
};

namespace static_compound {

Shape *create(const StaticCompoundShapeSettings &settings);
/**
 * rebuilds the bvh (reordering the children, which invalidates sub shape ids taken before) and
 * recomputes bounds, centre of mass and volume.
 */
void update(Shape *shape);

void cast_ray(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray, const Shape *shape,
              const SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
void collide_point(CollidePointCollector &collector, const CollidePointSettings &settings, Vec3f point,
                   const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** a's children whose bounds overlap b's, against b through the collide table */
void collide_static_compound_vs_shape(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                      const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a,
                                      const Transformf &transform_a, const Shape *shape_b,
                                      const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a against b's children whose bounds overlap a's, through the collide table */
void collide_shape_vs_static_compound(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                      const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a,
                                      const Transformf &transform_a, const Shape *shape_b,
                                      const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a's children whose bounds overlap b's bounds swept back along the displacement, cast against b */
void cast_static_compound_vs_shape(CastShapeCollector &collector, const CastShapeSettings &settings,
                                   const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                   Vec3f displacement_a, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                   const Transformf &transform_b);
/** a cast against b's children, nearest first along the sweep of a's bounds */
void cast_shape_vs_static_compound(CastShapeCollector &collector, const CastShapeSettings &settings,
                                   const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                   Vec3f displacement_a, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                   const Transformf &transform_b);

ShapeDef def();

}  // namespace static_compound

TUMBLE_NAMESPACE_END
