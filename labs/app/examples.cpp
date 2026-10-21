#include "app/examples.h"

const Example *const EXAMPLES[] = {
    &EXAMPLE_SHAPES,
    &EXAMPLE_CUBE_HEAP,
};

const int EXAMPLE_COUNT = (int)(sizeof(EXAMPLES) / sizeof(EXAMPLES[0]));
