#include "tumble/physics/collision/narrowphase.h"

#include "tumble/physics/collision/internal_edge_removing_collector.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace narrowphase {

void cast_ray_vs_shape(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                       const shapes::Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    const shapes::ShapeDef &def = shapes::def_of(shape);
    TUMBLE_ASSERT(def.cast_ray != nullptr);
    def.cast_ray(collector, settings, ray, shape, sub_shape_id, transform);
}

void cast_shape_vs_shape(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const shapes::Shape *shape_a,
                         const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                         const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const shapes::CastShapeVsShapeFn fn = shapes::cast_fn(shape_a->type, shape_b->type);
    if (fn == nullptr) return;
    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, displacement_a, shape_b, sub_shape_id_b, transform_b);
}

void collide_shape_vs_shape(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                            const shapes::Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                            const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const shapes::CollideShapeVsShapeFn fn = shapes::collide_fn(shape_a->type, shape_b->type);
    if (fn == nullptr) return;
    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, shape_b, sub_shape_id_b, transform_b);
}

void collide_shape_vs_shape_with_internal_edge_removal(collide_shape::CollideShapeCollector &collector,
                                                       const collide_shape::CollideShapeSettings &settings, const shapes::Shape *shape_a,
                                                       const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                                       const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                                       const Transformf &transform_b) {
    // copy settings and force the flags the algorithm needs
    collide_shape::CollideShapeSettings modified_settings = settings;
    modified_settings.collide_only_with_active_edges = false;
    modified_settings.collect_faces = true;

    // the wrapper lives for this call: a static would keep heap vectors past a util::init
    internal_edge_removing::InternalEdgeRemovingCollector wrapper = internal_edge_removing::create();
    internal_edge_removing::set(wrapper, collector, settings.internal_edge_removal_vertex_tolerance_sq);

    collide_shape_vs_shape(wrapper.collector, modified_settings, shape_a, sub_shape_id_a, transform_a, shape_b, sub_shape_id_b, transform_b);

    // flush the delayed results
    internal_edge_removing::flush(wrapper);
    internal_edge_removing::release(wrapper);
}

void collide_point_vs_shape(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                            Vec3f point, const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                            const Transformf &transform_b) {
    const shapes::ShapeDef &def = shapes::def_of(shape_b);
    TUMBLE_ASSERT(def.collide_point != nullptr);
    def.collide_point(collector, settings, point, shape_b, sub_shape_id_b, transform_b);
}

}  // namespace narrowphase

TUMBLE_NAMESPACE_END
