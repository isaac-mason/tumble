#pragma once

#include <stdint.h>

// seeded generators for test data, so the random cases are the same every run

inline uint32_t mulberry32(uint32_t &state) {
    uint32_t t = (state += 0x6d2b79f5u);
    t = (t ^ (t >> 15)) * (t | 1u);
    t ^= t + (t ^ (t >> 7)) * (t | 61u);
    return t ^ (t >> 14);
}

/** xorshift32 (13, 17, 5) */
inline uint32_t xorshift32(uint32_t &state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

/** numerical recipes' lcg */
inline uint32_t lcg32(uint32_t &state) {
    state = state * 1664525u + 1013904223u;
    return state;
}

/** the top 24 bits as a float in [0, 1) */
inline float unit24(uint32_t bits) { return (float)(bits >> 8) * (1.0f / 16777216.0f); }

inline float mulberry32_unit(uint32_t &state) { return unit24(mulberry32(state)); }
inline float xorshift32_unit(uint32_t &state) { return unit24(xorshift32(state)); }
inline float lcg32_unit(uint32_t &state) { return unit24(lcg32(state)); }
