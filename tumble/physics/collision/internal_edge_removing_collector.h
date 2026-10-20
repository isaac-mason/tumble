#pragma once

// crashcat's internal-edge-removing-collector.ts, the types: a collide collector that wraps another
// and drops the ghost contacts a convex shape gets from the internal edges of a mesh or compound. it
// forwards face contacts at once and holds edge and vertex contacts back until `flush`, where the
// deepest contacts void the features of their faces so a shallower contact on a voided edge is dropped.
// the behaviour is internal_edge_removing_collector.cpp.

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

namespace internal_edge_removing {

/** a vertex a deeper contact has claimed, in world space, for one sub shape of the convex side */
struct VoidedFeature {
    Vec3f feature;
    sub_shape::SubShapeId sub_shape_id;
};

/**
 * the wrapper: `collector` is the CUSTOM collide collector a query hands to the shapes (its context points back
 * here); `chained` is the collector the kept hits go to. one wrapper serves a whole query: `set` before each
 * body, `flush` after it, `reset` between queries.
 */
struct InternalEdgeRemovingCollector {
    collide_shape::CollideShapeCollector collector;
    collide_shape::CollideShapeCollector *chained;
    collections::Vec<VoidedFeature> voided_features;
    collections::Vec<collide_shape::CollideShapeHit> delayed_results;
    /** max squared distance to consider a vertex to be the same as another vertex, used to determine if a feature is voided */
    float vertex_tolerance_sq;
};

/** the wrapper with no chained collector yet */
InternalEdgeRemovingCollector create();
void release(InternalEdgeRemovingCollector &wrapper);

/** points the wrapper at the collector the kept hits go to, taking its bound and body, with the vertex tolerance; drops any held state */
void set(InternalEdgeRemovingCollector &wrapper, collide_shape::CollideShapeCollector &chained, float vertex_tolerance_sq);
/** forwards the delayed hits that survive voiding, deepest first, then clears them for the next body */
void flush(InternalEdgeRemovingCollector &wrapper);
/** drops delayed hits and voided features, unbounds the collector and resets the chained collector */
void reset(InternalEdgeRemovingCollector &wrapper);

}  // namespace internal_edge_removing

TUMBLE_NAMESPACE_END
