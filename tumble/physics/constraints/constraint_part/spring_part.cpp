#include "physics/constraints/constraint_part/spring_part.h"

#include "math/scalar.h"

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
    // calculate softness (gamma in the slides)
    // see page 34 and note that the gamma needs to be divided by delta time
    // since we're working with impulses rather than forces:
    // softness = 1 / (dt * (c + dt * k))
    // note that the spring stiffness is k and the spring damping is c
    part.softness = 1.0f / (delta_time * (damping + delta_time * stiffness));

    // calculate bias factor (baumgarte stabilization):
    // beta = dt * k / (c + dt * k) = dt * k^2 * softness
    // b = beta / dt * C = dt * k * softness * C
    part.bias = bias + delta_time * stiffness * part.softness * c;

    // update the effective mass
    // see post by erin catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
    //
    // newton's law:
    // M * (v2 - v1) = J^T * lambda
    //
    // velocity constraint with softness and baumgarte:
    // J * v2 + softness * lambda + b = 0
    //
    // where b = beta * C / dt
    //
    // first solve newton's law for v2 in terms of lambda:
    //
    // v2 = v1 + M^-1 * J^T * lambda
    //
    // substitute this expression into the velocity constraint:
    //
    // J * (v1 + M^-1 * J^T * lambda) + softness * lambda + b = 0
    //
    // now collect coefficients of lambda:
    //
    // (J * M^-1 * J^T + softness) * lambda = - J * v1 - b
    //
    // now we define:
    //
    // K = J * M^-1 * J^T + softness
    //
    // so our new effective mass is K^-1
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
