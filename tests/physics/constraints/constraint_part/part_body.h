#pragma once

#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"

TUMBLE_NAMESPACE_BEGIN

/** a body of this motion type, inverse mass and inverse inertia diagonal, its center of mass at com, unrotated */
inline RigidBody make_body(MotionType motion_type, float inv_mass, const Float3 &inv_inertia_diagonal,
                                       const Float3 &com = {0.0f, 0.0f, 0.0f}) {
    RigidBody body = {};
    body.motion_type = motion_type;
    body.quaternion = quatf::identity();
    body.center_of_mass_position = vec3f::load(com);
    body.motion_properties = motion_properties::create();
    body.motion_properties.inv_mass = inv_mass;
    body.motion_properties.inv_inertia_diagonal = vec3f::load(inv_inertia_diagonal);
    return body;
}

TUMBLE_NAMESPACE_END
