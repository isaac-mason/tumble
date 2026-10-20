#pragma once

#include "tumble/physics/constraints/constraint_part/spring_settings.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// the softness and bias a soft constraint adds to its effective mass and target
namespace spring_part {

struct SpringPart {
    float bias;
    float softness;
};

inline SpringPart create() { return {0.0f, 0.0f}; }

inline void reset(SpringPart &part) {
    part.bias = 0.0f;
    part.softness = 0.0f;
}

/** turn off the spring and set a bias only (hard constraint). bias is b in lambda = J v + b */
void calculate_spring_properties_with_bias(SpringPart &part, float bias);

/**
 * spring properties from a frequency (hz) and damping ratio (0 none, 1 critical). c is the value of
 * the constraint equation; set c, frequency or damping to zero to not drive the constraint to zero with
 * a spring. returns the effective mass K^-1 after the spring adjustment.
 */
float calculate_spring_properties_with_frequency_and_damping(SpringPart &part, float delta_time, float inv_effective_mass,
                                                             float bias, float c, float frequency, float damping);

/**
 * spring properties from a stiffness k and damping c, as in F = -k x - c v; with both zero the constraint is hard.
 * returns the effective mass K^-1 after the spring adjustment.
 */
float calculate_spring_properties_with_stiffness_and_damping(SpringPart &part, float delta_time, float inv_effective_mass,
                                                             float bias, float c, float stiffness, float damping);

/** spring properties from settings, by the spring mode. returns the effective mass K^-1 after the spring adjustment. */
float calculate_spring_properties_with_settings(SpringPart &part, float delta_time, float inv_effective_mass, float bias,
                                                float c, const spring_settings::SpringSettings &settings);

/** true if this spring is active (soft constraint) */
bool is_spring_active(const SpringPart &part);

/**
 * the total bias b, the supplied bias plus the spring's: lambda = -K^-1 (J v + softness * total_lambda + b),
 * so this returns softness * total_lambda + b.
 */
float get_spring_bias(const SpringPart &part, float total_lambda);

}  // namespace spring_part

TUMBLE_NAMESPACE_END
