#pragma once

// a bare body for the constraint part tests: no shape and no world, only what a part reads

#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"

TUMBLE_NAMESPACE_BEGIN

/** a body of this motion type, inverse mass and inverse inertia diagonal, its center of mass at com, unrotated */
inline rigid_body::RigidBody make_body(motion_type::MotionType motion_type, float inv_mass, const Float3 &inv_inertia_diagonal,
                                       const Float3 &com = {0.0f, 0.0f, 0.0f}) {
    rigid_body::RigidBody body = {};
    body.motion_type = motion_type;
    body.quaternion = {0.0f, 0.0f, 0.0f, 1.0f};
    body.center_of_mass_position = com;
    body.motion_properties = motion_properties::create();
    body.motion_properties.inv_mass = inv_mass;
    body.motion_properties.inv_inertia_diagonal = inv_inertia_diagonal;
    return body;
}

TUMBLE_NAMESPACE_END
