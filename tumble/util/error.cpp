#include "tumble/util/error.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

constinit static Error the_error = ERROR_NONE;

void set_error(Error error) {
    if (the_error == ERROR_NONE) the_error = error;
}

Error error() { return the_error; }

void error_clear() { the_error = ERROR_NONE; }

}  // namespace util

TUMBLE_NAMESPACE_END
