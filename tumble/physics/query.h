#pragma once

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/collision/cast_shape_vs_shape.h"
#include "tumble/physics/collision/collide_point_vs_shape.h"
#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/world.h"

TUMBLE_NAMESPACE_BEGIN

// each a broadphase walk under the filter, the shape query run against every body reached

namespace query {

void cast_ray(World &world, CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
              const Filter &filter);

/** sweeps the shape at the pose along the displacement */
void cast_shape(World &world, CastShapeCollector &collector, const CastShapeSettings &settings,
                const Shape *shape, const Transformf &transform, Vec3f displacement, const Filter &filter);

void collide_shape(World &world, CollideShapeCollector &collector, const CollideShapeSettings &settings,
                   const Shape *shape, const Transformf &transform, const Filter &filter);

/** collide_shape through the internal edge removing wrapper, flushed per body */
void collide_shape_with_internal_edge_removal(World &world, CollideShapeCollector &collector,
                                              const CollideShapeSettings &settings, const Shape *shape,
                                              const Transformf &transform, const Filter &filter);

void collide_point(World &world, CollidePointCollector &collector, const CollidePointSettings &settings,
                   Vec3f point, const Filter &filter);

}  // namespace query

TUMBLE_NAMESPACE_END
