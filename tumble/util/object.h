#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/util/alloc.h"
#include "tumble/util/construct.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

/** a T constructed, value initialised: plain fields zeroed, so the containers in it start empty, and the members with
 * constructors constructed */
template <typename T>
inline T *create() {
    T *object = alloc<T>();
    zero(object, sizeof(T));
    return construct<T>(object);
}

/** a T made by create destroyed, then freed; nothing for null, as delete */
template <typename T>
inline void destroy(T *object) {
    if (object == nullptr) return;
    destruct(object);
    free(object);
}

}  // namespace util

TUMBLE_NAMESPACE_END
