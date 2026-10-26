#include "tumble/physics/collision/narrowphase.h"

#include "tumble/physics/collision/internal_edge_removing_collector.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace narrowphase {

void cast_ray_vs_shape(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                       const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    const ShapeDef &def = shapes::def_of(shape);
    TUMBLE_ASSERT(def.cast_ray != nullptr);
    def.cast_ray(collector, settings, ray, shape, sub_shape_id, transform);
}

void cast_shape_vs_shape(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                         const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                         const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const CastShapeVsShapeFn fn = shapes::cast_fn(shape_a->type, shape_b->type);
    if (fn == nullptr) return;
    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, displacement_a, shape_b, sub_shape_id_b, transform_b);
}

void collide_shape_vs_shape(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                            const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                            const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const CollideShapeVsShapeFn fn = shapes::collide_fn(shape_a->type, shape_b->type);
    if (fn == nullptr) return;
    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, shape_b, sub_shape_id_b, transform_b);
}

void collide_shape_vs_shape_with_internal_edge_removal(CollideShapeCollector &collector,
                                                       const CollideShapeSettings &settings, const Shape *shape_a,
                                                       const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                                       const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                                       const Transformf &transform_b) {
    // copy settings and force the flags the algorithm needs
    CollideShapeSettings modified_settings = settings;
    modified_settings.collide_only_with_active_edges = false;
    modified_settings.collect_faces = true;

    // the wrapper lives for this call: a static would hold allocations that no world owns
    InternalEdgeRemovingCollector wrapper = internal_edge_removing::create();
    internal_edge_removing::set(wrapper, collector, settings.internal_edge_removal_vertex_tolerance_sq);

    collide_shape_vs_shape(wrapper.collector, modified_settings, shape_a, sub_shape_id_a, transform_a, shape_b, sub_shape_id_b, transform_b);

    internal_edge_removing::flush(wrapper);
}

void collide_point_vs_shape(CollidePointCollector &collector, const CollidePointSettings &settings,
                            Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                            const Transformf &transform_b) {
    const ShapeDef &def = shapes::def_of(shape_b);
    TUMBLE_ASSERT(def.collide_point != nullptr);
    def.collide_point(collector, settings, point, shape_b, sub_shape_id_b, transform_b);
}

}  // namespace narrowphase

TUMBLE_NAMESPACE_END
