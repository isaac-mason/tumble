#pragma once

#include "tumble/physics/shapes/shapes.h"
#include "tumble/util/collections.h"

struct MeshVertex {
    float x, y, z;
    float nx, ny, nz;
};

namespace shape_mesh {

/** a shape's surface as triangles in its own space, three vertices each: what a body of it looks like solid */
void build(tumble::collections::Array<MeshVertex> &out, const tumble::Shape *shape);

}
