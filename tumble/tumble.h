#pragma once

#define TUMBLE_NAMESPACE_BEGIN namespace tumble {
#define TUMBLE_NAMESPACE_END }

// the simd backend: wasm simd128 when the target has it, else scalar. TUMBLE_NO_SIMD forces scalar; both compute the
// same bits
#if !defined(TUMBLE_NO_SIMD) && !defined(TUMBLE_USE_WASM_SIMD) && defined(__wasm_simd128__)
#define TUMBLE_USE_WASM_SIMD
#endif
#if defined(TUMBLE_NO_SIMD) && defined(TUMBLE_USE_WASM_SIMD)
#error "TUMBLE_NO_SIMD and TUMBLE_USE_WASM_SIMD are both defined"
#endif
