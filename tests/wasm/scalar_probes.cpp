#include "tumble/math/scalar.h"
#include "wasm/runtime/export.h"

using namespace tumble;

// run.mjs sweeps these against node's Math
WASM_EXPORT float probe_sin(float x) { return scalar::sin(x); }
WASM_EXPORT float probe_cos(float x) { return scalar::cos(x); }
WASM_EXPORT float probe_atan(float x) { return scalar::atan(x); }
WASM_EXPORT float probe_atan2(float y, float x) { return scalar::atan2(y, x); }
WASM_EXPORT float probe_acos(float x) { return scalar::acos(x); }

