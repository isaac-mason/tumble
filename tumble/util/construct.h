#pragma once

#include <stddef.h>

#include "tumble/config.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

/** picks tumble's placement new, which cannot clash with the standard library's where there is one */
struct Placement {};

}  // namespace util

TUMBLE_NAMESPACE_END

inline void *operator new(size_t, void *where, tumble::util::Placement) noexcept { return where; }
inline void operator delete(void *, void *, tumble::util::Placement) noexcept {}

TUMBLE_NAMESPACE_BEGIN

namespace util {

/** a T constructed in memory nothing lives in yet */
template <typename T, typename... Args>
inline T *construct(void *where, Args &&...args) {
    return new (where, Placement{}) T(static_cast<Args &&>(args)...);
}

/** a T's life ended, its memory left for its owner to free or reuse */
template <typename T>
inline void destruct(T *object) {
    object->~T();
}

/** std::move: lets the value be moved from, leaving it empty but alive */
template <typename T>
constexpr __remove_reference_t(T) &&move(T &&value) noexcept {
    return static_cast<__remove_reference_t(T) &&>(value);
}

}  // namespace util

TUMBLE_NAMESPACE_END
