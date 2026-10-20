#include "tumble/physics/constraints/constraint_part/spring_part.h"

#include "tumble/math/scalar.h"

TUMBLE_NAMESPACE_BEGIN

namespace spring_part {

/**
 * the spring properties from a stiffness and damping; returns the effective mass K^-1 after the spring
 * adjustment.
 *
 * the calculation of beta and gamma below are based on the solution of an implicit euler integration
 * scheme. this scheme is unconditionally stable but has built-in damping, so even when you set the
 * damping ratio to 0 there will still be damping. see erin catto gdc 2011 slides page 16 and 32.
 */
static float calculate_spring_properties_helper(SpringPart &part, float delta_time, float inv_effective_mass, float bias,
                                                float c, float stiffness, float damping) {
    // softness = 1 / (dt * (c + dt * k)), stiffness k and damping c: gamma divided by dt, as this works in impulses
    part.softness = 1.0f / (delta_time * (damping + delta_time * stiffness));

    // baumgarte: beta = dt * k / (c + dt * k) = dt * k^2 * softness, so b = beta / dt * C = dt * k * softness * C
    part.bias = bias + delta_time * stiffness * part.softness * c;

    // M (v2 - v1) = J^T lambda with J v2 + softness lambda + b = 0 gives (J M^-1 J^T + softness) lambda = -J v1 - b,
    // so the effective mass is (J M^-1 J^T + softness)^-1
    return 1.0f / (inv_effective_mass + part.softness);
}

void calculate_spring_properties_with_bias(SpringPart &part, float bias) {
    part.softness = 0.0f;
    part.bias = bias;
}

float calculate_spring_properties_with_frequency_and_damping(SpringPart &part, float delta_time, float inv_effective_mass,
                                                             float bias, float c, float frequency, float damping) {
    const float effective_mass = 1.0f / inv_effective_mass;

    if (frequency > 0.0f) {
        // calculate angular frequency
        const float omega = 2.0f * scalar::PI * frequency;

        // calculate spring stiffness k and damping constant c (page 45)
        const float k = effective_mass * omega * omega;
        const float damping_coefficient = 2.0f * effective_mass * damping * omega;

        return calculate_spring_properties_helper(part, delta_time, inv_effective_mass, bias, c, k, damping_coefficient);
    }
    calculate_spring_properties_with_bias(part, bias);
    return effective_mass;
}

float calculate_spring_properties_with_stiffness_and_damping(SpringPart &part, float delta_time, float inv_effective_mass,
                                                             float bias, float c, float stiffness, float damping) {
    // damping alone still makes a spring, one that only drives the velocity
    if (stiffness > 0.0f || damping > 0.0f) {
        return calculate_spring_properties_helper(part, delta_time, inv_effective_mass, bias, c, stiffness, damping);
    }
    const float effective_mass = 1.0f / inv_effective_mass;
    calculate_spring_properties_with_bias(part, bias);
    return effective_mass;
}

float calculate_spring_properties_with_settings(SpringPart &part, float delta_time, float inv_effective_mass, float bias,
                                                float c, const spring_settings::SpringSettings &settings) {
    if (settings.mode == spring_settings::FREQUENCY_AND_DAMPING) {
        return calculate_spring_properties_with_frequency_and_damping(part, delta_time, inv_effective_mass, bias, c,
                                                                      settings.frequency_or_stiffness, settings.damping);
    }
    return calculate_spring_properties_with_stiffness_and_damping(part, delta_time, inv_effective_mass, bias, c,
                                                                  settings.frequency_or_stiffness, settings.damping);
}

bool is_spring_active(const SpringPart &part) { return part.softness != 0.0f; }

float get_spring_bias(const SpringPart &part, float total_lambda) { return part.softness * total_lambda + part.bias; }

}  // namespace spring_part

TUMBLE_NAMESPACE_END
