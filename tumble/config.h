#pragma once

#define TUMBLE_NAMESPACE_BEGIN namespace tumble {
#define TUMBLE_NAMESPACE_END }

#if !defined(TUMBLE_SIMD_NONE) && !defined(TUMBLE_SIMD_WASM) && !defined(TUMBLE_SIMD_SSE2) && !defined(TUMBLE_SIMD_NEON)
#if defined(__wasm_simd128__)
#define TUMBLE_SIMD_WASM
#elif defined(__SSE2__) || defined(_M_X64)
#define TUMBLE_SIMD_SSE2
#elif defined(__aarch64__) || defined(_M_ARM64)
#define TUMBLE_SIMD_NEON
#else
#define TUMBLE_SIMD_NONE
#endif
#endif

#if defined(TUMBLE_SIMD_NONE) + defined(TUMBLE_SIMD_WASM) + defined(TUMBLE_SIMD_SSE2) + defined(TUMBLE_SIMD_NEON) > 1
#error "more than one of TUMBLE_SIMD_NONE, TUMBLE_SIMD_WASM, TUMBLE_SIMD_SSE2 and TUMBLE_SIMD_NEON is defined"
#endif

#if defined(TUMBLE_SIMD_WASM)
#define TUMBLE_SIMD_BACKEND_NAME "wasm simd128"
#elif defined(TUMBLE_SIMD_SSE2)
#define TUMBLE_SIMD_BACKEND_NAME "sse2"
#elif defined(TUMBLE_SIMD_NEON)
#define TUMBLE_SIMD_BACKEND_NAME "neon"
#else
#define TUMBLE_SIMD_BACKEND_NAME "scalar"
#endif
