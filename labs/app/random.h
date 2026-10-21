#pragma once

#include <stdint.h>

/** a small lcg, seeded per example so a run repeats */
struct Random {
    uint32_t state;
};

namespace random {

inline float next(Random &random) {
    random.state = random.state * 1664525u + 1013904223u;
    return (float)(random.state >> 8) / 16777216.0f;
}

inline float range(Random &random, float low, float high) { return low + (high - low) * next(random); }

}
