#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/util/collections.h"

/** a gltf scene's triangles in world space, with each vertex's material colour */
struct Model {
    bool loaded;
    tumble::collections::Array<float> positions;
    tumble::collections::Array<float> colors;
    tumble::collections::Array<uint32_t> indices;
    tumble::Box3f bounds;
};

namespace model {

/** a file in samples/assets, by name */
Model load(const char *name);
void release(Model &model);
/** the first `max_points` vertex positions as points, as a convex hull takes them; the caller releases them */
tumble::collections::Array<tumble::Vec3f> points(const Model &model, uint32_t max_points = UINT32_MAX);

}
