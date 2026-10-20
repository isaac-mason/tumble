#include "tumble/math/scalar.h"
#include "tumble/physics/constraints/constraint_part/spring_part.h"
#include "tumble/physics/constraints/constraint_part/spring_settings.h"
#include "test.h"
#include "test_math.h"

using spring_part::SpringPart;

void test_spring_part_create_and_reset(TestCtx &ctx) {
    SpringPart part = spring_part::create();
    CHECK(ctx, part.bias == 0.0f && part.softness == 0.0f);
    CHECK(ctx, !spring_part::is_spring_active(part));

    part.bias = 3.0f;
    part.softness = 0.5f;
    CHECK(ctx, spring_part::is_spring_active(part));
    spring_part::reset(part);
    CHECK(ctx, part.bias == 0.0f && part.softness == 0.0f);
}

void test_spring_part_with_bias_is_hard(TestCtx &ctx) {
    SpringPart part = {7.0f, 0.25f};
    spring_part::calculate_spring_properties_with_bias(part, -1.5f);
    CHECK(ctx, part.softness == 0.0f);
    CHECK(ctx, part.bias == -1.5f);
    CHECK(ctx, !spring_part::is_spring_active(part));
    // a hard constraint's bias is just b, whatever has accumulated
    CHECK(ctx, spring_part::get_spring_bias(part, 100.0f) == -1.5f);
}

void test_spring_part_frequency_and_damping(TestCtx &ctx) {
    const float dt = 1.0f / 60.0f;
    const float inv_effective_mass = 0.5f;  // effective mass 2
    const float bias = 0.1f;
    const float c = 0.3f;

    // no frequency: hard, the plain effective mass back
    SpringPart part = {9.0f, 9.0f};
    float effective_mass =
        spring_part::calculate_spring_properties_with_frequency_and_damping(part, dt, inv_effective_mass, bias, c, 0.0f, 1.0f);
    CHECK(ctx, effective_mass == 2.0f);
    CHECK(ctx, part.softness == 0.0f && part.bias == bias);

    // 2 hz, half critical: k = m w^2, c = 2 m zeta w (catto gdc 2011, page 45)
    const float frequency = 2.0f;
    const float zeta = 0.5f;
    effective_mass =
        spring_part::calculate_spring_properties_with_frequency_and_damping(part, dt, inv_effective_mass, bias, c, frequency, zeta);
    const float omega = 2.0f * scalar::PI * frequency;
    const float k = 2.0f * omega * omega;
    const float damping = 2.0f * 2.0f * zeta * omega;
    const float softness = 1.0f / (dt * (damping + dt * k));
    CHECK_NEAR(ctx, part.softness, softness, softness * 1e-6f);
    CHECK_NEAR(ctx, part.bias, bias + dt * k * softness * c, 1e-6f);
    CHECK_NEAR(ctx, effective_mass, 1.0f / (inv_effective_mass + softness), 1e-6f);
    CHECK(ctx, spring_part::is_spring_active(part));
    // the soft effective mass is below the hard one
    CHECK(ctx, effective_mass < 2.0f);
    CHECK_NEAR(ctx, spring_part::get_spring_bias(part, 4.0f), softness * 4.0f + part.bias, 1e-5f);
}

void test_spring_part_stiffness_and_damping(TestCtx &ctx) {
    const float dt = 0.02f;

    // no stiffness and no damping is a hard constraint
    SpringPart part = spring_part::create();
    float effective_mass = spring_part::calculate_spring_properties_with_stiffness_and_damping(part, dt, 0.25f, 0.5f, 1.0f, 0.0f, 0.0f);
    CHECK(ctx, effective_mass == 4.0f);
    CHECK(ctx, part.softness == 0.0f && part.bias == 0.5f);

    // damping alone is soft and drives the velocity only: softness = 1 / (dt c), the bias gains nothing from C
    effective_mass = spring_part::calculate_spring_properties_with_stiffness_and_damping(part, dt, 0.25f, 0.5f, 1.0f, 0.0f, 3.0f);
    const float damping_softness = 1.0f / (dt * 3.0f);
    CHECK_NEAR(ctx, part.softness, damping_softness, 1e-5f);
    CHECK(ctx, part.bias == 0.5f);
    CHECK_NEAR(ctx, effective_mass, 1.0f / (0.25f + damping_softness), 1e-6f);

    // softness = 1 / (dt (c + dt k)), b = bias + dt k softness C
    const float k = 100.0f;
    const float damping = 5.0f;
    effective_mass = spring_part::calculate_spring_properties_with_stiffness_and_damping(part, dt, 0.25f, 0.5f, 1.0f, k, damping);
    const float softness = 1.0f / (dt * (damping + dt * k));
    CHECK_NEAR(ctx, part.softness, softness, 1e-5f);
    CHECK_NEAR(ctx, part.bias, 0.5f + dt * k * softness * 1.0f, 1e-5f);
    CHECK_NEAR(ctx, effective_mass, 1.0f / (0.25f + softness), 1e-6f);
}

void test_spring_part_with_settings_dispatches_on_mode(TestCtx &ctx) {
    const float dt = 1.0f / 60.0f;
    SpringPart direct = spring_part::create();
    SpringPart from_settings = spring_part::create();

    spring_settings::SpringSettings settings = spring_settings::create();
    settings.mode = spring_settings::FREQUENCY_AND_DAMPING;
    settings.frequency_or_stiffness = 3.0f;
    settings.damping = 0.7f;
    float a = spring_part::calculate_spring_properties_with_frequency_and_damping(direct, dt, 0.8f, 0.2f, -0.1f, 3.0f, 0.7f);
    float b = spring_part::calculate_spring_properties_with_settings(from_settings, dt, 0.8f, 0.2f, -0.1f, settings);
    CHECK(ctx, a == b && direct.softness == from_settings.softness && direct.bias == from_settings.bias);

    settings.mode = spring_settings::STIFFNESS_AND_DAMPING;
    settings.frequency_or_stiffness = 50.0f;
    settings.damping = 2.0f;
    a = spring_part::calculate_spring_properties_with_stiffness_and_damping(direct, dt, 0.8f, 0.2f, -0.1f, 50.0f, 2.0f);
    b = spring_part::calculate_spring_properties_with_settings(from_settings, dt, 0.8f, 0.2f, -0.1f, settings);
    CHECK(ctx, a == b && direct.softness == from_settings.softness && direct.bias == from_settings.bias);

    // the default settings are a hard constraint
    settings = spring_settings::create();
    a = spring_part::calculate_spring_properties_with_settings(from_settings, dt, 0.8f, 0.2f, -0.1f, settings);
    CHECK(ctx, a == 1.0f / 0.8f && !spring_part::is_spring_active(from_settings));
}

// a unit mass on a soft constraint C = x, one implicit solve per step:
// lambda = -K^-1 (v + b), v += lambda / m, x += v dt. returns the first time x crosses zero, or -1
static float spring_first_zero_crossing(float frequency, float damping, float duration, bool &monotonic) {
    const float dt = 1.0f / 240.0f;
    float x = 1.0f;
    float v = 0.0f;
    monotonic = true;
    for (float t = 0.0f; t < duration; t += dt) {
        SpringPart part = spring_part::create();
        const float effective_mass =
            spring_part::calculate_spring_properties_with_frequency_and_damping(part, dt, 1.0f, 0.0f, x, frequency, damping);
        const float lambda = -effective_mass * (v + spring_part::get_spring_bias(part, 0.0f));
        v += lambda;
        const float next = x + v * dt;
        if (next > x) monotonic = false;
        if (next <= 0.0f) return t + dt;
        x = next;
    }
    return -1.0f;
}

void test_spring_part_drives_a_mass_like_a_spring(TestCtx &ctx) {
    // undamped at 1 hz: crosses zero after about a quarter period
    bool monotonic = false;
    const float crossing = spring_first_zero_crossing(1.0f, 0.0f, 2.0f, monotonic);
    CHECK(ctx, crossing > 0.2f && crossing < 0.3f);

    // critically damped: decays towards zero without ever crossing it or turning back
    const float critical = spring_first_zero_crossing(1.0f, 1.0f, 4.0f, monotonic);
    CHECK(ctx, critical < 0.0f);
    CHECK(ctx, monotonic);
}
