#pragma once

// crashcat's ccd.ts, the types: the bodies a step sweeps continuously because they move further than a
// fraction of their inner radius, and what each sweep hit. the sweeps and their resolution are update.cpp.

#include <stdint.h>

#include "math/aabox.h"
#include "math/types.h"
#include "physics/body/bodies.h"
#include "physics/body/rigid_body.h"
#include "physics/body/sub_shape.h"
#include "physics/constraints/contact_settings.h"
#include "util/collections.h"

namespace ccd {

struct CCDBody {
    uint32_t body_index;
    /** the body the sweep hit, or ~0u */
    uint32_t hit_body_index;
    /** the step's displacement */
    Vec3f delta_position;
    Vec3f contact_normal;
    Vec3f contact_point;
    /** along the displacement, and the same with the penetration slop added */
    float fraction;
    float fraction_plus_slop;
    float linear_cast_threshold_sq;
    float max_penetration;
    sub_shape::SubShapeId sub_shape_id_1;
    sub_shape::SubShapeId sub_shape_id_2;
    contact_constraints::ContactSettings contact_settings;
};

struct CCD {
    collections::Vec<CCDBody> ccd_bodies;
    /** indices into ccd_bodies in time of impact order, so the records and every body's ccd_body_index stay put */
    collections::Vec<uint32_t> order;
};

inline CCD create() {
    CCD c = {};
    return c;
}

inline void release(CCD &ccd) {
    ccd.ccd_bodies.release();
    ccd.order.release();
}

/** no hit, fraction 1, default contact settings */
void reset_ccd_body(CCDBody &ccd_body);
/** drops this step's entries and clears every body's ccd mark */
void clear(CCD &ccd, bodies::Bodies &bodies);
/** the body's bounds swept along the displacement */
AABox compute_swept_aabb(const rigid_body::RigidBody &body, Vec3f displacement);

}  // namespace ccd
