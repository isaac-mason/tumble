#pragma once

// objects the wasm hands to js by pointer: made zeroed, so the containers in them start empty, and given back whole.
// js holds them as typed handles, so a pointer of the wrong kind is a type error on the js side.

#include <stdint.h>

#include "util/alloc.h"
#include "util/mem.h"

namespace util {

/** a zeroed object, or null (error set) when the memory cannot grow. */
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

// the C++ allocation operators over the heap (defined in object.cpp). `new T[n]` works because
// clang keeps the count in front of the block when a type needs destroying, and nothing here does.
void *operator new(unsigned long bytes);
void *operator new[](unsigned long bytes);
void operator delete(void *p) noexcept;
void operator delete[](void *p) noexcept;
void operator delete(void *p, unsigned long bytes) noexcept;
void operator delete[](void *p, unsigned long bytes) noexcept;
