#pragma once

#include "tumble/math/types.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/collision/cast_shape_vs_shape.h"
#include "tumble/physics/collision/collide_point_vs_shape.h"
#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/world.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// each a broadphase walk under the filter, the shape query run against every body reached
namespace query {

void cast_ray(world::World &world, cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
              const filter::Filter &filter);

/** sweeps the shape at the pose along the displacement */
void cast_shape(world::World &world, cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                const shapes::Shape *shape, const Transformf &transform, Vec3f displacement, const filter::Filter &filter);

void collide_shape(world::World &world, collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                   const shapes::Shape *shape, const Transformf &transform, const filter::Filter &filter);

/** collide_shape through the internal edge removing wrapper, flushed per body */
void collide_shape_with_internal_edge_removal(world::World &world, collide_shape::CollideShapeCollector &collector,
                                              const collide_shape::CollideShapeSettings &settings, const shapes::Shape *shape,
                                              const Transformf &transform, const filter::Filter &filter);

void collide_point(world::World &world, collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                   Vec3f point, const filter::Filter &filter);

}  // namespace query

TUMBLE_NAMESPACE_END
