#pragma once

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

/** the active index of a body off the active list: sleeping or static */
constexpr uint32_t INACTIVE_BODY_INDEX = 0xffffffffu;

/** the centre of mass, and it plus the largest and the second largest bounding box axes */
void get_sleep_test_points(Float3 out_points[3], const rigid_body::RigidBody *body);

/** centred on the points, radius 0 */
void reset_sleep_test_spheres(motion_properties::MotionProperties &motion_properties, const Float3 points[3]);

/** true if the body can sleep */
bool update_sleep_state(rigid_body::RigidBody *body, float delta_time, float max_movement, float time_before_sleep);

/** when a body wakes or its velocity is set */
void reset_sleep_timer(rigid_body::RigidBody *body);

void add_body_to_active_bodies(bodies::Bodies &bodies, rigid_body::RigidBody *body);

/** swap-removes, so another body's active index changes */
void remove_body_from_active_bodies(bodies::Bodies &bodies, rigid_body::RigidBody *body);

/** a sleeping body is left out of the simulation until woken */
void sleep(bodies::Bodies &bodies, rigid_body::RigidBody *body);

void wake(bodies::Bodies &bodies, rigid_body::RigidBody *body);

}  // namespace sleep

TUMBLE_NAMESPACE_END
