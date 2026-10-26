#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/layers.h"

TUMBLE_NAMESPACE_BEGIN

struct WorldSettings {
    struct Contacts {
        /** squared distance between an old and a new contact point under which the warm start impulse carries over, m2 (1 cm) */
        float contact_point_preserve_lambda_max_dist_sq = 0.01f * 0.01f;
    };

    struct Narrowphase {
        /** closer than this is colliding (gjk), m */
        float collision_tolerance = 1e-4f;
        /** epa stops when the squared distance changes by less than this fraction of the depth squared */
        float penetration_tolerance = 1e-4f;
        bool return_deepest_point = false;
        /** only active edges collide, so a mesh's shared edges give no ghost collisions */
        bool collide_only_with_active_edges = true;
        /** speculative contacts are found within this distance, m (2 cm); too big gives ghost collisions */
        float speculative_contact_distance = 0.02f;
        /** points within this distance are on one plane when a manifold is built between two faces, m (1 mm) */
        float manifold_tolerance = 1e-3f;
        /** the simulation's own collides see back facing triangles */
        bool collide_with_backfaces = false;
        /** manifolds of different sub shapes with similar normals merge; a body can turn it off for itself */
        bool use_manifold_reduction = true;
        /** cos of the largest angle between normals that still merge (5 degrees) */
        float normal_cos_max_delta_rotation = 0.996194698f;
        /** a pair that barely moved reuses last step's manifolds without a narrowphase */
        bool use_body_pair_contact_cache = true;
        /** squared change of the relative translation that invalidates the pair cache, m2 (1 mm) */
        float body_pair_cache_max_delta_position_sq = 1e-6f;
        /** cos(max angle / 2) of the relative rotation change that invalidates it (1 degree) */
        float body_pair_cache_cos_max_delta_rotation_div_2 = 0.999847695f;
        /** squared distance under which two vertices are one for the internal edge removal, m2 */
        float internal_edge_removal_vertex_tolerance_sq = 1e-8f;
    };

    struct Solver {
        /** below this relative normal velocity restitution is 0, so things settle, m/s */
        float min_velocity_for_restitution = 1.0f;
        /** at least 2 for friction, which uses the step before's normal impulse */
        uint32_t velocity_iterations = 10;
        uint32_t position_iterations = 2;
        bool warm_starting = true;
        /** allowed sinking, m (2 cm), against jitter in stacks */
        float penetration_slop = 0.02f;
        /** the fraction of the position error corrected per update */
        float baumgarte_factor = 0.2f;
        float warm_start_impulse_ratio = 1.0f;
        /** the most one position iteration corrects, m (20 cm) */
        float max_penetration_distance = 0.2f;
    };

    struct Sleeping {
        bool allow_sleeping = true;
        /** s */
        float time_before_sleep = 0.5f;
        /** the three tracked points (centre of mass and two far bound faces) must all move slower than this, m/s */
        float point_velocity_sleep_threshold = 0.03f;
    };

    struct Ccd {
        /** a body moving less than this fraction of its inner radius takes the discrete update */
        float linear_cast_threshold = 0.05f;
        /** the fraction of the inner radius a cast may penetrate */
        float linear_cast_max_penetration = 0.25f;
        /** m/s */
        float min_velocity_for_restitution = 1.0f;
    };

    Layers layers;
    /** m/s2 */
    Float3 gravity = {0.0f, -9.81f, 0.0f};
    bool gravity_enabled = true;
    Contacts contacts;
    Narrowphase narrowphase;
    Solver solver;
    Sleeping sleeping;
    Ccd ccd;
};

namespace world_settings {

/** the defaults, with empty layers */
inline WorldSettings create() { return {}; }

inline uint32_t add_broadphase_layer(WorldSettings &settings) { return layers::add_broadphase_layer(settings.layers); }
inline uint32_t add_object_layer(WorldSettings &settings, uint32_t broadphase_layer) {
    return layers::add_object_layer(settings.layers, broadphase_layer);
}
inline void enable_collision(WorldSettings &settings, uint32_t object_layer_a, uint32_t object_layer_b) {
    layers::enable_collision(settings.layers, object_layer_a, object_layer_b);
}
inline void disable_collision(WorldSettings &settings, uint32_t object_layer_a, uint32_t object_layer_b) {
    layers::disable_collision(settings.layers, object_layer_a, object_layer_b);
}

}  // namespace world_settings

TUMBLE_NAMESPACE_END
