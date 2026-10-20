# musl float math, vendored

The `.c` files here are musl 1.2.6's single-precision `sinf` / `cosf` / `tanf` / `atanf` /
`atan2f` / `acosf` / `asinf` / `expf` / `powf` / `logf` / `fmodf` and their helpers, copied verbatim
from `emsdk/upstream/emscripten/system/lib/libc/musl/src/math`. `COPYRIGHT` is musl's.

They are compiled as C, with `features.h`, `math.h` and `libm.h` here standing in for the libc
headers: the core links no libc, so every function that is a wasm instruction is a builtin and the
rest is declared and vendored. `scalar.h` wraps them for C++.

They exist so the core's transcendentals have one implementation with pinned bits on every host,
rather than whatever libm the browser or Node happens to use.
