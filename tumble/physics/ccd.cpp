#include "tumble/physics/ccd.h"

#include "tumble/math/aabox.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace ccd {

void reset_ccd_body(CCD::Body &ccd_body) {
    ccd_body.body_index = ~0u;
    ccd_body.hit_body_index = ~0u;
    ccd_body.delta_position = vec3f::zero();
    ccd_body.contact_normal = vec3f::zero();
    ccd_body.contact_point = vec3f::zero();
    ccd_body.fraction = 1.0f;
    ccd_body.fraction_plus_slop = 1.0f;
    ccd_body.linear_cast_threshold_sq = 0.0f;
    ccd_body.max_penetration = 0.0f;
    ccd_body.sub_shape_id_1 = 0;
    ccd_body.sub_shape_id_2 = 0;
    ccd_body.contact_settings = ContactSettings();
}

void clear(CCD &ccd, Bodies &bodies) {
    // unlink the bodies with a ccd record. only those ever point at one, so this stays proportional to the ccd body count
    // rather than the body pool. a slot recycled by a body removed and re-created between steps already reads -1, so the
    // write is harmless.
    for (uint32_t i = 0; i < ccd.ccd_bodies.length; i++) {
        bodies.pool[ccd.ccd_bodies[i].body_index]->ccd_body_index = -1;
    }
    ccd.ccd_bodies.length = 0;
    ccd.order.length = 0;
}

AABox compute_swept_aabb(const RigidBody &body, Vec3f displacement) {
    // the body's aabb, and the same translated by the displacement
    const AABox aabb = aabox::load(body.aabb);
    return aabox::union_(aabb, {vec3f::add(aabb.min, displacement), vec3f::add(aabb.max, displacement)});
}

}  // namespace ccd

TUMBLE_NAMESPACE_END
