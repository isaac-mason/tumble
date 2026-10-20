#pragma once

#include "tumble/math/mat4f.h"
#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace mass_properties {

struct MassProperties {
    /** kg */
    float mass;
    /** the inertia tensor, kg m^2, as the upper 3x3 of a column-major matrix */
    Float4x4 inertia;
};

inline MassProperties create() {
    MassProperties out;
    out.mass = 0.0f;
    mat4f::store(out.inertia, mat4f::identity());
    return out;
}

inline MassProperties &reset(MassProperties &out) {
    out.mass = 0.0f;
    mat4f::store(out.inertia, mat4f::identity());
    return out;
}

inline MassProperties &copy(MassProperties &out, const MassProperties &source) {
    out.mass = source.mass;
    out.inertia = source.inertia;
    return out;
}

/** sets mass and inertia of a solid box of `box_size` at `density`. */
MassProperties &set_mass_and_inertia_of_solid_box(MassProperties &out, Vec3f box_size, float density);
/** scales the mass and inertia by per-axis scale factors. */
MassProperties &scale(MassProperties &out, const MassProperties &source, Vec3f scale);
/** scales the inertia tensor to match a new mass, keeping its distribution; a massless input just takes the mass. */
MassProperties &scale_to_mass(MassProperties &out, float new_mass);
/** rotates the inertia tensor by the upper 3x3 of `rotation`: I' = R I R^T. out may alias source. */
MassProperties &rotate(MassProperties &out, const MassProperties &source, const Mat4f &rotation);
/** translates the inertia tensor (parallel axis theorem). out may alias source. */
MassProperties &translate(MassProperties &out, const MassProperties &source, Vec3f translation);

}  // namespace mass_properties

TUMBLE_NAMESPACE_END
