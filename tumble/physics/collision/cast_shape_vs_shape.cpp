#include "tumble/physics/collision/cast_shape_vs_shape.h"

#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace cast_shape {

static void shift_face(face::Face &out, const face::Face &source, Vec3f delta) {
    out.num_vertices = source.num_vertices;
    for (uint32_t i = 0; i < source.num_vertices; i++) out.vertices[i] = vec3f::subtract(source.vertices[i], delta);
}

void inverted_add_hit(CastShapeCollector &c, const CastShapeHit &h) {
    const InvertedContext &context = *static_cast<const InvertedContext *>(c.context);
    CastShapeCollector &base = *context.base;
    // the reversed cast reports shape b in its swept frame and shape a at rest, shift both back
    // by fraction x reversed displacement so shape a is in its swept frame and shape b at rest
    const Vec3f delta = vec3f::scale(context.reversed_displacement, h.fraction);
    CastShapeHit swapped;
    swapped.status = h.status;
    swapped.fraction = h.fraction;
    swapped.point_a = vec3f::subtract(h.point_b, delta);
    swapped.point_b = vec3f::subtract(h.point_a, delta);
    swapped.penetration_depth = h.penetration_depth;
    // negate penetration_axis, normal follows as -normalize(penetration_axis)
    swapped.penetration_axis = vec3f::negate(h.penetration_axis);
    swapped.normal = vec3f::normalize(swapped.penetration_axis);
    swapped.normal = vec3f::negate(swapped.normal);
    swapped.sub_shape_id_a = h.sub_shape_id_b;
    swapped.sub_shape_id_b = h.sub_shape_id_a;
    shift_face(swapped.face_a, h.face_b, delta);
    shift_face(swapped.face_b, h.face_a, delta);
    swapped.body_id_b = h.body_id_b;
    add_hit(base, swapped);
    c.early_out_fraction = base.early_out_fraction;
}

void inverted_add_miss(CastShapeCollector &c) { add_miss(*static_cast<InvertedContext *>(c.context)->base); }

bool inverted_should_early_out(const CastShapeCollector &c) { return should_early_out(*static_cast<const InvertedContext *>(c.context)->base); }

}  // namespace cast_shape

TUMBLE_NAMESPACE_END
