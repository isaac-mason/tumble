#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/motion_properties.h"

TUMBLE_NAMESPACE_BEGIN

struct RigidBody;

namespace sleep {

/** the active index of a body off the active list: sleeping or static */
constexpr uint32_t INACTIVE_BODY_INDEX = 0xffffffffu;

/** the centre of mass, and it plus the largest and the second largest bounding box axes */
void get_sleep_test_points(Float3 out_points[3], const RigidBody *body);

/** centred on the points, radius 0 */
void reset_sleep_test_spheres(MotionProperties &motion_properties, const Float3 points[3]);

/** true if the body can sleep */
bool update_sleep_state(RigidBody *body, float delta_time, float max_movement, float time_before_sleep);

/** when a body wakes or its velocity is set */
void reset_sleep_timer(RigidBody *body);

/**
 * the only way onto the active list: a dynamic or kinematic body's sleep timer reset, and, if it was off the list,
 * appended and the activation listener told. a static body is left as it is
 */
void wake(Bodies &bodies, RigidBody *body);

/**
 * the only way off it: an active body swap-removed (so another body's active index changes), its velocities zeroed and
 * the activation listener told. a body off the list is left as it is
 */
void sleep(Bodies &bodies, RigidBody *body);

}  // namespace sleep

TUMBLE_NAMESPACE_END
