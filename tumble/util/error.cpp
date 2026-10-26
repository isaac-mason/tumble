#include "tumble/util/error.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

constinit static Error the_error = Error::NONE;

void set_error(Error error) {
    if (the_error == Error::NONE) the_error = error;
}

Error error() { return the_error; }

void error_clear() { the_error = Error::NONE; }

}  // namespace util

TUMBLE_NAMESPACE_END
