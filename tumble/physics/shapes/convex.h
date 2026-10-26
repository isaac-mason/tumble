#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// the narrowphase entry points every convex shape uses: gjk and epa over the shapes' supports

namespace convex {

/** a convex shape is a single leaf with a single material */
int32_t get_convex_material(const Shape *shape, SubShapeId sub_shape_id);

/** cast ray implementation for convex shapes */
void cast_ray_vs_convex(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                        const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform);

/** collide point implementation for convex shapes */
void collide_point_vs_convex(CollidePointCollector &collector, const CollidePointSettings &settings,
                             Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                             const Transformf &transform_b);

/** world-space entry point for convex vs convex collision: moves b into a's local space and delegates to the local function. */
void collide_convex_vs_convex(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                              const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                              const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** convex vs convex in a's local space: `transform_b_in_a` places b relative to a, both rotation + translation only. */
void collide_convex_vs_convex_local(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                    const Shape *shape_a, SubShapeId sub_shape_id_a, const Shape *shape_b,
                                    SubShapeId sub_shape_id_b, const Mat4f &transform_b_in_a, const Mat4f &transform_a_in_world,
                                    Vec3f scale_a, Vec3f scale_b);

/** world-space entry point for sweeping convex a by `displacement_a` against convex b: moves a into b's local space. */
void cast_convex_vs_convex(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                           const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                           const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** the sweep in b's local space: `cast_transform` is a's start in b's space, `target_transform` b's world transform. */
void cast_convex_vs_convex_local(CastShapeCollector &collector, const CastShapeSettings &settings,
                                 const Shape *shape_a, SubShapeId sub_shape_id_a, const Shape *shape_b,
                                 SubShapeId sub_shape_id_b, const Mat4f &cast_transform, Vec3f scale_a,
                                 Vec3f displacement_in_b, Vec3f scale_b, const Mat4f &target_transform);

}  // namespace convex

TUMBLE_NAMESPACE_END
