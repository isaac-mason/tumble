#include "tumble/physics/collision/collide_shape_vs_shape.h"

#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace collide_shape {

void inverted_add_hit(CollideShapeCollector &c, const CollideShapeHit &h) {
    CollideShapeCollector &base = *static_cast<CollideShapeCollector *>(c.context);
    CollideShapeHit swapped;
    swapped.point_a = h.point_b;
    swapped.point_b = h.point_a;
    swapped.penetration_axis = vec3f::negate(h.penetration_axis);
    swapped.penetration = h.penetration;
    swapped.sub_shape_id_a = h.sub_shape_id_b;
    swapped.sub_shape_id_b = h.sub_shape_id_a;
    swapped.material_id_a = h.material_id_b;
    swapped.material_id_b = h.material_id_a;
    face::copy(swapped.face_a, h.face_b);
    face::copy(swapped.face_b, h.face_a);
    swapped.body_id_b = h.body_id_b;
    add_hit(base, swapped);
    c.early_out_fraction = base.early_out_fraction;
}

void inverted_add_miss(CollideShapeCollector &c) { add_miss(*static_cast<CollideShapeCollector *>(c.context)); }

bool inverted_should_early_out(const CollideShapeCollector &c) {
    return should_early_out(*static_cast<const CollideShapeCollector *>(c.context));
}

}  // namespace collide_shape

TUMBLE_NAMESPACE_END
