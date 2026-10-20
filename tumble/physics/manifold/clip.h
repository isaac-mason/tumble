#pragma once

#include "tumble/math/types.h"
#include "tumble/physics/utils/face.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// sutherland-hodgman
namespace clip {

/** keeps the part of the polygon in front of the plane (on the normal's side) */
void clip_poly_vs_plane(face::Face &out, const face::Face &polygon, Vec3f plane_origin, Vec3f plane_normal);

/** clips the polygon against every edge plane of the clipping polygon, whose normal picks the inside */
void clip_poly_vs_poly(face::Face &out, const face::Face &polygon_to_clip, const face::Face &clipping_polygon,
                       Vec3f clipping_polygon_normal);

/** clips the polygon against the two planes through an edge's ends, perpendicular to the edge, then to the edge's plane */
void clip_poly_vs_edge(face::Face &out, const face::Face &polygon, Vec3f edge_vertex_1, Vec3f edge_vertex_2,
                       Vec3f clipping_edge_normal);

}  // namespace clip

TUMBLE_NAMESPACE_END
