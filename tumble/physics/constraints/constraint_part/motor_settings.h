#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/scalar.h"
#include "tumble/physics/constraints/constraint_part/spring_settings.h"

TUMBLE_NAMESPACE_BEGIN

enum class MotorState : uint32_t {
    /** free spinning */
    OFF = 0,
    /** drives to a target velocity */
    VELOCITY = 1,
    /** drives to a target position through the spring */
    POSITION = 2,
    /** drives to both the target position and the target velocity */
    POSITION_AND_VELOCITY = 3,
};

namespace motor_settings {

/** whether the motor state drives to a target velocity */
inline bool is_velocity_motor(MotorState state) { return ((uint32_t)state & (uint32_t)MotorState::VELOCITY) != 0; }

/** whether the motor state drives to a target position */
inline bool is_position_motor(MotorState state) { return ((uint32_t)state & (uint32_t)MotorState::POSITION) != 0; }

}  // namespace motor_settings

struct MotorSettings {
    /** drives the position motor; unused by a velocity motor. 2 hz, critically damped by default */
    SpringSettings spring = {SpringMode::FREQUENCY_AND_DAMPING, 2.0f, 1.0f};
    /** force limits of a linear motor, N; usually min is -max unless the motor pushes but never pulls */
    float min_force_limit = -scalar::FLOAT_MAX;
    float max_force_limit = scalar::FLOAT_MAX;
    /** torque limits of an angular motor, N m */
    float min_torque_limit = -scalar::FLOAT_MAX;
    float max_torque_limit = scalar::FLOAT_MAX;
};

namespace motor_settings {

inline MotorSettings create() { return {}; }

inline bool is_valid(const MotorSettings &settings) {
    return settings.spring.frequency_or_stiffness >= 0.0f && settings.spring.damping >= 0.0f &&
           settings.min_force_limit <= settings.max_force_limit && settings.min_torque_limit <= settings.max_torque_limit;
}

}  // namespace motor_settings

TUMBLE_NAMESPACE_END
