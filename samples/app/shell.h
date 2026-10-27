#pragma once

#include <stddef.h>

#include "app/app.h"
#include "sokol_gfx.h"

struct FrameInput {
    /** the framebuffer's size in pixels */
    float width;
    float height;
    float delta_time;
    float dpi_scale;
    float pointer_x;
    float pointer_y;
    sg_swapchain swapchain;
    /** one world step a frame, drawn as it is: the same picture on every machine */
    bool fixed_step;
    /** the scene and its labels without the tumble window */
    bool hide_ui;
};

namespace shell {

App &app();
void setup(const sg_environment &environment, int initial_example);
void shutdown();

void select_example(int index);
/** an example by its name or its id, -1 when there is none */
int find_example(const char *name);
/** `example_` and the name in snake case: the example's screenshot, and its `?example=` on the web */
void example_id(char *out, size_t capacity, int index);

void frame(const FrameInput &input);

}
