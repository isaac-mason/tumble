#pragma once

// crashcat's sleep.ts: the sleep test over a body's three test points, and moving a body on and
// off the active list as it sleeps and wakes.

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

namespace sleep {

/** sentinel value indicating a body is not in the active bodies list (sleeping or static) */
constexpr uint32_t INACTIVE_BODY_INDEX = 0xffffffffu;

/**
 * get the 3 test points for sleep detection:
 * - center of mass
 * - center of mass + largest bounding box axis
 * - center of mass + second largest bounding box axis
 */
void get_sleep_test_points(Float3 out_points[3], const rigid_body::RigidBody *body);

/** reset the sleep test spheres to center around the given points with radius 0 */
void reset_sleep_test_spheres(motion_properties::MotionProperties &motion_properties, const Float3 points[3]);

/** update the sleep state of a body, returns true if the body can sleep, false if it cannot */
bool update_sleep_state(rigid_body::RigidBody *body, float delta_time, float max_movement, float time_before_sleep);

/** reset the sleep timer for a body (called when body is activated or velocity is set) */
void reset_sleep_timer(rigid_body::RigidBody *body);

/** adds a body to the active bodies list, called when a body wakes up or is created as non-sleeping */
void add_body_to_active_bodies(bodies::Bodies &bodies, rigid_body::RigidBody *body);

/** removes a body from the active bodies list using swap-remove, called when a body goes to sleep or is destroyed */
void remove_body_from_active_bodies(bodies::Bodies &bodies, rigid_body::RigidBody *body);

/** puts a body to sleep, sleeping bodies are excluded from physics simulation until woken */
void sleep(bodies::Bodies &bodies, rigid_body::RigidBody *body);

/** wakes a sleeping body */
void wake(bodies::Bodies &bodies, rigid_body::RigidBody *body);

}  // namespace sleep

TUMBLE_NAMESPACE_END
