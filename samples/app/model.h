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

}
