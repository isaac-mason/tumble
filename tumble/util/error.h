#pragma once

#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// a call that cannot do what it was asked sets the error word and backs out; the first error stands until cleared
namespace util {

enum Error : uint32_t {
    ERROR_NONE = 0,
    /** the allocator returned null */
    ERROR_OUT_OF_MEMORY = 1,
    /** a pointer or id was not what the call expected */
    ERROR_BAD_HANDLE = 2,
    /** a pool is at its most */
    ERROR_QUOTA = 3,
};

void set_error(Error error);
/** the first error since the last clear; ERROR_NONE when none */
Error error();
void error_clear();

}  // namespace util

TUMBLE_NAMESPACE_END
