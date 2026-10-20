#pragma once

#include "tumble/math/types.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/collision/cast_shape_vs_shape.h"
#include "tumble/physics/collision/collide_point_vs_shape.h"
#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace shapes {
struct Shape;
}

// the query entry points: a ray or a point goes to the shape's own def, a pair of shapes to the pair's entry in the
// collide or cast table; a pair with no entry is no collision
namespace narrowphase {

void cast_ray_vs_shape(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                       const shapes::Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform);

/** sweeps a by displacement_a (world space) against b */
void cast_shape_vs_shape(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const shapes::Shape *shape_a,
                         const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                         const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

void collide_shape_vs_shape(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                            const shapes::Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                            const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/**
 * collide_shape_vs_shape with enhanced internal edge removal, against the ghost collisions a shape sliding across the
 * internal edges of a mesh or compound gets: the settings are copied with collect_faces on and active edges only off
 * (the algorithm needs every edge and the faces), the collector is wrapped by an InternalEdgeRemovingCollector, and the
 * held back hits are flushed into it after the collision
 */
void collide_shape_vs_shape_with_internal_edge_removal(collide_shape::CollideShapeCollector &collector,
                                                       const collide_shape::CollideShapeSettings &settings, const shapes::Shape *shape_a,
                                                       const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                                       const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                                       const Transformf &transform_b);

void collide_point_vs_shape(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                            Vec3f point, const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                            const Transformf &transform_b);

}  // namespace narrowphase

TUMBLE_NAMESPACE_END
