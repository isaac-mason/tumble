#pragma once

#include <stdint.h>

#include "tumble/tumble.h"
#include "tumble/util/alloc.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

/** zeroed, so the containers in it start empty; null (error set) when the allocator fails */
template <typename T>
inline T *create() {
    T *object = alloc<T>();
    if (object == nullptr) return nullptr;
    zero(object, sizeof(T));
    return object;
}

template <typename T>
inline void destroy(T *object) {
    free(object);
}

}  // namespace util

TUMBLE_NAMESPACE_END
