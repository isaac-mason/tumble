#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"

TUMBLE_NAMESPACE_BEGIN

namespace color {

constexpr Color make(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) { return {r, g, b, a}; }

constexpr Color RED = make(255, 0, 0);
constexpr Color GREEN = make(0, 255, 0);
constexpr Color BLUE = make(0, 0, 255);
constexpr Color YELLOW = make(255, 255, 0);
constexpr Color PURPLE = make(255, 0, 255);
constexpr Color CYAN = make(0, 255, 255);
constexpr Color ORANGE = make(255, 128, 0);
constexpr Color GREY = make(128, 128, 128);
constexpr Color WHITE = make(255, 255, 255);

}  // namespace color

TUMBLE_NAMESPACE_END
