#pragma once

#define TUMBLE_NAMESPACE_BEGIN namespace tumble {
#define TUMBLE_NAMESPACE_END }

#if !defined(TUMBLE_NO_SIMD) && !defined(TUMBLE_USE_WASM_SIMD) && !defined(TUMBLE_USE_SSE2) && !defined(TUMBLE_USE_NEON)
#if defined(__wasm_simd128__)
#define TUMBLE_USE_WASM_SIMD
#elif defined(__SSE2__) || defined(_M_X64)
#define TUMBLE_USE_SSE2
#elif defined(__aarch64__) || defined(_M_ARM64)
#define TUMBLE_USE_NEON
#endif
#endif

#if defined(TUMBLE_NO_SIMD) + defined(TUMBLE_USE_WASM_SIMD) + defined(TUMBLE_USE_SSE2) + defined(TUMBLE_USE_NEON) > 1
#error "more than one of TUMBLE_NO_SIMD, TUMBLE_USE_WASM_SIMD, TUMBLE_USE_SSE2 and TUMBLE_USE_NEON is defined"
#endif

#if defined(TUMBLE_USE_WASM_SIMD)
#define TUMBLE_SIMD_BACKEND_NAME "wasm simd128"
#elif defined(TUMBLE_USE_SSE2)
#define TUMBLE_SIMD_BACKEND_NAME "sse2"
#elif defined(TUMBLE_USE_NEON)
#define TUMBLE_SIMD_BACKEND_NAME "neon"
#else
#define TUMBLE_SIMD_BACKEND_NAME "scalar"
#endif
