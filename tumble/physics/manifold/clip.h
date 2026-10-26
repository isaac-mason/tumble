#pragma once

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/utils/face.h"

TUMBLE_NAMESPACE_BEGIN

// sutherland-hodgman

namespace clip {

/** keeps the part of the polygon in front of the plane (on the normal's side) */
void clip_poly_vs_plane(SupportingFace &out, const SupportingFace &polygon, Vec3f plane_origin, Vec3f plane_normal);

/** clips the polygon against every edge plane of the clipping polygon, whose normal picks the inside */
void clip_poly_vs_poly(SupportingFace &out, const SupportingFace &polygon_to_clip, const SupportingFace &clipping_polygon,
                       Vec3f clipping_polygon_normal);

/** clips the polygon against the two planes through an edge's ends, perpendicular to the edge, then to the edge's plane */
void clip_poly_vs_edge(SupportingFace &out, const SupportingFace &polygon, Vec3f edge_vertex_1, Vec3f edge_vertex_2,
                       Vec3f clipping_edge_normal);

}  // namespace clip

TUMBLE_NAMESPACE_END
