#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/utils/face.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace manifold {

/** the most points a manifold keeps after pruning */
constexpr uint32_t MAX_CONTACT_POINTS = 4;
/** the most points clipping can produce before pruning */
constexpr uint32_t MAX_CLIPPING_CONTACT_POINTS = 64;

struct ContactManifold {
    /** 0 to MAX_CLIPPING_CONTACT_POINTS while clipping, 0 to MAX_CONTACT_POINTS after pruning */
    uint32_t num_contact_points;
    /** every point is relative to this, world space */
    Vec3f base_offset;
    Vec3f relative_contact_points_on_a[MAX_CLIPPING_CONTACT_POINTS];
    Vec3f relative_contact_points_on_b[MAX_CLIPPING_CONTACT_POINTS];
    /** unit, world space, from a to b */
    Vec3f world_space_normal;
    /** one depth for the whole manifold */
    float penetration_depth;
    sub_shape::SubShapeId sub_shape_id_a;
    sub_shape::SubShapeId sub_shape_id_b;
    int32_t material_id_a;
    int32_t material_id_b;
};

inline void reset(ContactManifold &m) {
    m.num_contact_points = 0;
    m.base_offset = vec3f::zero();
    m.world_space_normal = vec3f::zero();
    m.penetration_depth = 0.0f;
    m.sub_shape_id_a = sub_shape::EMPTY_SUB_SHAPE_ID;
    m.sub_shape_id_b = sub_shape::EMPTY_SUB_SHAPE_ID;
    m.material_id_a = -1;
    m.material_id_b = -1;
}

inline ContactManifold create() {
    ContactManifold m = {};
    reset(m);
    return m;
}

inline Vec3f get_world_space_contact_point_on_a(const ContactManifold &m, uint32_t index) {
    return vec3f::add(m.base_offset, m.relative_contact_points_on_a[index]);
}

inline Vec3f get_world_space_contact_point_on_b(const ContactManifold &m, uint32_t index) {
    return vec3f::add(m.base_offset, m.relative_contact_points_on_b[index]);
}

/** a and b change places: points swapped, normal negated, ids and materials swapped */
void swap_shapes(ContactManifold &m);

/** cuts the manifold to MAX_CONTACT_POINTS, keeping the deepest point and the three that span the most area. the axis is unit */
void prune_contact_points(ContactManifold &m, Vec3f penetration_axis);

/**
 * the manifold between two supporting faces: the face with 3+ vertices clipped against the other along the
 * penetration axis (face 2 against face 1, the faces swapped when face 2 is an edge), keeping points within
 * max_contact_distance (> 0) of each other along the axis; the contact points alone when either face has under 2
 * vertices, both have 2, or nothing survives. reads `out.base_offset`, writes the points and their count.
 */
void manifold_between_two_faces(ContactManifold &out, Vec3f contact_point_1, Vec3f contact_point_2, Vec3f penetration_axis,
                                float max_contact_distance, const face::Face &shape_1_face,
                                const face::Face &shape_2_face);

}  // namespace manifold

TUMBLE_NAMESPACE_END
