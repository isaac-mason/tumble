#pragma once

// the one way a function leaves the module. everything else is hidden by -fvisibility=hidden,
// and the import-section check in the build keeps anything from coming in. the shipped surface
// is src/generated/exports.cpp, written by the schema build from schema/bindings.cpp; the test
// harness and the schema runner export a few of their own.
#define WASM_EXPORT extern "C" __attribute__((visibility("default")))

#include <stdint.h>

/** a u32 array js passed, as the pointer to one-word handle structs the bound function takes. */
template <typename P>
struct BindBits {
    P *address;
    template <typename H>
    operator H *() const {
        return reinterpret_cast<H *>(address);
    }
};
template <typename P>
inline BindBits<P> bind_bits(P *address) {
    return {address};
}
