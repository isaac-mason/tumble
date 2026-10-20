#pragma once

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace contact_constraints {

struct ContactSettings {
    float combined_friction;
    float combined_restitution;
    /** a sensor reports the contact but applies no impulse */
    bool is_sensor;
    /** scale the inverse mass and inertia each body brings to this contact; 0 makes it immovable here */
    float inv_mass_scale_1 = 1.0f;
    float inv_mass_scale_2 = 1.0f;
    float inv_inertia_scale_1 = 1.0f;
    float inv_inertia_scale_2 = 1.0f;
    /** surface velocity of body 2 relative to body 1, world space: a conveyor belt */
    Float3 relative_linear_surface_velocity;
    Float3 relative_angular_surface_velocity;
};

}  // namespace contact_constraints

TUMBLE_NAMESPACE_END
