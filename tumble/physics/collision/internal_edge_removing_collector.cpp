#include "tumble/physics/collision/internal_edge_removing_collector.h"

#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/assert.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

namespace internal_edge_removing {

using collide_shape::CollideShapeCollector;
using collide_shape::CollideShapeHit;

// cosine of 1 degree (for face contact detection threshold)
static constexpr float COS_1_DEGREE = 0.99984769515639123916f;

// f32 machine epsilon
static constexpr float FLT_EPSILON_SQ = scalar::FLOAT_EPSILON * scalar::FLOAT_EPSILON;

struct ClosestFeatureResult {
    uint32_t vertex_index_1;
    uint32_t vertex_index_2;
    float distance_sq;
};

static InternalEdgeRemovingCollector &wrapper_of(const CollideShapeCollector &c) {
    return *static_cast<InternalEdgeRemovingCollector *>(c.context);
}

/** the face normal from the first 3 vertices */
static void calculate_face_normal(Vec3f &out, const face::Face &face) {
    if (face.num_vertices < 3) {
        out = vec3f::zero();
        return;
    }
    const Vec3f v0 = face.vertices[0];
    const Vec3f v1 = face.vertices[1];
    const Vec3f v2 = face.vertices[2];
    const Vec3f edge_01 = vec3f::subtract(v1, v0);
    const Vec3f edge_02 = vec3f::subtract(v2, v0);
    out = vec3f::cross(edge_01, edge_02);
}

/**
 * the closest feature (vertex or edge) on a face to a point, as the indices of the vertices that form it: equal for
 * a vertex, different for an edge.
 */
static void find_closest_feature(ClosestFeatureResult &out, const face::Face &face, Vec3f point) {
    float best_dist_sq = scalar::FLOAT_MAX;
    uint32_t best_v1 = 0;
    uint32_t best_v2 = 0;

    const uint32_t num_vertices = face.num_vertices;
    uint32_t v1_index = num_vertices - 1;
    Vec3f v1 = vec3f::subtract(face.vertices[v1_index], point);

    for (uint32_t v2_index = 0; v2_index < num_vertices; v2_index++) {
        Vec3f v2 = vec3f::subtract(face.vertices[v2_index], point);
        const Vec3f v1_v2 = vec3f::subtract(v2, v1);

        const float denominator = vec3f::squared_length(v1_v2);

        if (denominator < FLT_EPSILON_SQ) {
            // degenerate edge, test v1 only
            const float v1_len_sq = vec3f::squared_length(v1);
            if (v1_len_sq < best_dist_sq) {
                best_dist_sq = v1_len_sq;
                best_v1 = v1_index;
                best_v2 = v1_index;
            }
        } else {
            // project point onto line segment
            const float fraction = -vec3f::dot(v1, v1_v2) / denominator;

            if (fraction < 1e-6f) {
                // closest to v1
                const float v1_len_sq = vec3f::squared_length(v1);
                if (v1_len_sq < best_dist_sq) {
                    best_dist_sq = v1_len_sq;
                    best_v1 = v1_index;
                    best_v2 = v1_index;
                }
            } else if (fraction < 1.0f - 1e-6f) {
                // closest to edge
                const Vec3f closest = vec3f::scale_and_add(v1, v1_v2, fraction);
                const float closest_len_sq = vec3f::squared_length(closest);
                if (closest_len_sq < best_dist_sq) {
                    best_dist_sq = closest_len_sq;
                    best_v1 = v1_index;
                    best_v2 = v2_index;
                }
            }
            // else closest to v2, will be tested in next iteration
        }

        v1_index = v2_index;
        v1 = v2;
    }

    out.vertex_index_1 = best_v1;
    out.vertex_index_2 = best_v2;
    out.distance_sq = best_dist_sq;
}

/** whether a feature (vertex) is voided */
static bool is_voided(const InternalEdgeRemovingCollector &wrapper, sub_shape::SubShapeId sub_shape_id, Vec3f feature) {
    for (uint32_t i = 0; i < wrapper.voided_features.length; i++) {
        const VoidedFeature &voided = wrapper.voided_features[i];
        if (voided.sub_shape_id == sub_shape_id && vec3f::squared_distance(voided.feature, feature) <= wrapper.vertex_tolerance_sq) return true;
    }
    return false;
}

/** void all vertices of a face */
static void void_features(InternalEdgeRemovingCollector &wrapper, const CollideShapeHit &hit) {
    const face::Face &face = hit.face_b;
    for (uint32_t i = 0; i < face.num_vertices; i++) {
        const Vec3f vertex = face.vertices[i];

        // don't duplicate voided features
        if (!is_voided(wrapper, hit.sub_shape_id_a, vertex)) wrapper.voided_features.push({vertex, hit.sub_shape_id_a});
    }
}

/** forward a hit to the chained collector */
static void chain(InternalEdgeRemovingCollector &wrapper, const CollideShapeHit &hit) {
    wrapper.chained->body_id_b = hit.body_id_b;
    collide_shape::add_hit(*wrapper.chained, hit);
    // sync early out fraction from chained collector
    wrapper.collector.early_out_fraction = wrapper.chained->early_out_fraction;
}

/** forward a hit to the chained collector and void its features */
static void chain_and_void(InternalEdgeRemovingCollector &wrapper, const CollideShapeHit &hit) {
    chain(wrapper, hit);
    void_features(wrapper, hit);
}

static void add_hit(CollideShapeCollector &collector, const CollideShapeHit &hit) {
    InternalEdgeRemovingCollector &wrapper = wrapper_of(collector);

    // a face of under 3 vertices has no internal edges
    if (hit.face_b.num_vertices < 3) {
        chain_and_void(wrapper, hit);
        return;
    }

    Vec3f triangle_normal;
    calculate_face_normal(triangle_normal, hit.face_b);
    const float normal_length_sq = vec3f::squared_length(triangle_normal);

    if (normal_length_sq < 1e-12f) {
        // degenerate face, forward immediately
        chain_and_void(wrapper, hit);
        return;
    }

    // a contact normal within 1 degree of the face normal is a face contact
    // contact normal is opposite of penetration axis
    const Vec3f contact_normal = vec3f::negate(hit.penetration_axis);

    const float dot_product = vec3f::dot(triangle_normal, contact_normal);
    const float contact_normal_length = vec3f::length(contact_normal);
    const float face_normal_length = scalar::sqrt(normal_length_sq);
    const float threshold = COS_1_DEGREE * face_normal_length * contact_normal_length;

    if (dot_product > threshold) {
        // face contact - process immediately
        chain_and_void(wrapper, hit);
        return;
    }

    // an edge or vertex contact waits for flush
    wrapper.delayed_results.push(hit);
}

static void add_miss(CollideShapeCollector &collector) { collide_shape::add_miss(*wrapper_of(collector).chained); }

InternalEdgeRemovingCollector create() {
    InternalEdgeRemovingCollector wrapper = {};
    wrapper.collector = collide_shape::create_collector(collector::COLLECT_CUSTOM);
    wrapper.collector.custom_add_hit = add_hit;
    wrapper.collector.custom_add_miss = add_miss;
    return wrapper;
}

void release(InternalEdgeRemovingCollector &wrapper) {
    wrapper.voided_features.release();
    wrapper.delayed_results.release();
}

void set(InternalEdgeRemovingCollector &wrapper, CollideShapeCollector &chained, float vertex_tolerance_sq) {
    wrapper.chained = &chained;
    wrapper.vertex_tolerance_sq = vertex_tolerance_sq;
    wrapper.collector.context = &wrapper;
    wrapper.collector.early_out_fraction = chained.early_out_fraction;
    wrapper.collector.body_id_b = chained.body_id_b;
    wrapper.voided_features.length = 0;
    wrapper.delayed_results.length = 0;
}

void flush(InternalEdgeRemovingCollector &wrapper) {
    collections::Vec<CollideShapeHit> &delayed = wrapper.delayed_results;

    // deepest first, stable
    sort::insertion_sort(
        delayed.data, delayed.data + delayed.length,
        [](const CollideShapeHit &a, const CollideShapeHit &b) { return a.penetration > b.penetration; }, collide_shape::copy_hit);

    for (uint32_t i = 0; i < delayed.length; i++) {
        const CollideShapeHit &result = delayed[i];

        // the closest feature, a vertex or an edge
        ClosestFeatureResult closest;
        find_closest_feature(closest, result.face_b, result.point_b);

        bool voided = is_voided(wrapper, result.sub_shape_id_a, result.face_b.vertices[closest.vertex_index_1]);

        if (closest.vertex_index_1 != closest.vertex_index_2) {
            // edge contact - both vertices must be voided
            if (!is_voided(wrapper, result.sub_shape_id_a, result.face_b.vertices[closest.vertex_index_2])) voided = false;
        }

        if (!voided) chain(wrapper, result);

        // every feature of this face is voided
        void_features(wrapper, result);
    }

    wrapper.voided_features.length = 0;
    delayed.length = 0;
}

void reset(InternalEdgeRemovingCollector &wrapper) {
    TUMBLE_ASSERT(wrapper.chained != nullptr);
    collide_shape::reset(*wrapper.chained);
    wrapper.voided_features.length = 0;
    wrapper.delayed_results.length = 0;
    wrapper.collector.early_out_fraction = collector::UNBOUNDED;
    wrapper.collector.body_id_b = body_id::INVALID_BODY_ID;
}

}  // namespace internal_edge_removing

TUMBLE_NAMESPACE_END
