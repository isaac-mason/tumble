#pragma once

// crashcat's spring-settings.ts: how a soft constraint's spring is specified.

#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace spring_settings {

enum SpringMode : uint32_t {
    /** frequency_or_stiffness is the oscillation frequency in hz, damping the damping ratio (0 none, 1 critical) */
    FREQUENCY_AND_DAMPING = 0,
    /** frequency_or_stiffness is k and damping is c in F = -k x - c v */
    STIFFNESS_AND_DAMPING = 1,
};

struct SpringSettings {
    SpringMode mode = FREQUENCY_AND_DAMPING;
    /** above 0 the constraint is soft; at or below 0 it is hard and damping is ignored */
    float frequency_or_stiffness = 0.0f;
    float damping = 0.0f;
};

/** a hard constraint, no spring */
inline SpringSettings create() { return {}; }

/** true if the spring has a frequency or stiffness (soft constraint); if false the spring is hard */
inline bool has_stiffness(const SpringSettings &settings) { return settings.frequency_or_stiffness > 0.0f; }

/** true if the spring has stiffness or damping, which makes it active; if false the constraint is hard */
inline bool has_stiffness_or_damping(const SpringSettings &settings) {
    return settings.frequency_or_stiffness > 0.0f || (settings.mode != FREQUENCY_AND_DAMPING && settings.damping > 0.0f);
}

}  // namespace spring_settings

TUMBLE_NAMESPACE_END
