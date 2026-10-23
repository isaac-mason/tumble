#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/util/collections.h"

/** a gltf scene's triangles in world space, with each vertex's material colour */
struct Model {
    bool loaded;
    tumble::collections::Vec<float> positions;
    tumble::collections::Vec<float> colors;
    tumble::collections::Vec<uint32_t> indices;
    tumble::Box3f bounds;
};

namespace model {

/** a file in samples/assets, by name */
Model load(const char *name);
void release(Model &model);

}
