#pragma once

#include <stdint.h>
#include "tumble/config.h"

TUMBLE_NAMESPACE_BEGIN

// a call that cannot do what it was asked sets the error word and backs out; the first error stands until cleared
enum class Error : uint32_t {
    NONE = 0,
    /** the allocator returned null */
    OUT_OF_MEMORY = 1,
    /** a pointer or id was not what the call expected */
    BAD_HANDLE = 2,
    /** a pool is at its most */
    QUOTA = 3,
};

namespace util {

void set_error(Error error);
/** the first error since the last clear; Error::NONE when none */
Error error();
void error_clear();

}  // namespace util

TUMBLE_NAMESPACE_END
