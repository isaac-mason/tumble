#include <stdio.h>
#if defined(__EMSCRIPTEN__)
#include <emscripten.h>
#endif
#include <stdlib.h>
#include <string.h>

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#elif defined(_WIN32)
#include <windows.h>
#include <GL/gl.h>
#elif !defined(__EMSCRIPTEN__)
#include <GL/gl.h>
#endif

#include "app/shell.h"
#include "imgui.h"
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_imgui.h"
#include "sokol_log.h"
#include "sokol_time.h"

using namespace tumble;

struct Input {
    bool down;
    bool dragging;
    sapp_mousebutton button;
    float down_x;
    float down_y;
    float x;
    float y;
};

static Input input;
static int initial_example = 0;
static const char *screenshot_path = nullptr;
static int screenshot_frame = 30;
static int frame_count = 0;
/** seconds a frame, in place of the display's when above 0: a display rate the machine does not have, to try out */
static float frame_time = 0.0f;

static void init() {
    stm_setup();
    shell::setup(sglue_environment(), initial_example);
}

static void save_screenshot() {
#if defined(__EMSCRIPTEN__)
    // the page holds this frame, and the headless browser takes the picture
    shell::app().paused = true;
    EM_ASM({ window.samplesScreenshotReady = true; });
    return;
#else
    const int width = sapp_width();
    const int height = sapp_height();
    uint8_t *pixels = (uint8_t *)malloc((size_t)width * height * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    FILE *file = fopen(screenshot_path, "wb");
    if (file) {
        fprintf(file, "P6\n%d %d\n255\n", width, height);
        for (int y = height - 1; y >= 0; y--) fwrite(pixels + (size_t)y * width * 3, 1, (size_t)width * 3, file);
        fclose(file);
    }
    free(pixels);
#endif
    sapp_request_quit();
}

static void frame() {
    FrameInput frame_input = {};
    frame_input.width = sapp_widthf();
    frame_input.height = sapp_heightf();
    frame_input.delta_time = frame_time > 0.0f ? frame_time : (float)sapp_frame_duration();
    frame_input.dpi_scale = sapp_dpi_scale();
    frame_input.pointer_x = input.x;
    frame_input.pointer_y = input.y;
    frame_input.swapchain = sglue_swapchain();
    frame_input.fixed_step = screenshot_path != nullptr;
    shell::frame(frame_input);

    frame_count++;
    if (screenshot_path && frame_count == screenshot_frame) save_screenshot();
}

static void cleanup() { shell::shutdown(); }

static void event(const sapp_event *e) {
    if (simgui_handle_event(e) && ImGui::GetIO().WantCaptureMouse) {
        input.down = false;
        return;
    }

    App &state = shell::app();
    switch (e->type) {
        case SAPP_EVENTTYPE_MOUSE_DOWN:
            input = {true, false, e->mouse_button, e->mouse_x, e->mouse_y, e->mouse_x, e->mouse_y};
            break;
        case SAPP_EVENTTYPE_MOUSE_MOVE: {
            const float dx = e->mouse_x - input.x;
            const float dy = e->mouse_y - input.y;
            input.x = e->mouse_x;
            input.y = e->mouse_y;
            if (!input.down) break;
            const float moved_x = e->mouse_x - input.down_x;
            const float moved_y = e->mouse_y - input.down_y;
            if (moved_x * moved_x + moved_y * moved_y > 16) input.dragging = true;
            if (!input.dragging) break;
            if (input.button == SAPP_MOUSEBUTTON_LEFT && (e->modifiers & SAPP_MODIFIER_SHIFT) == 0) {
                camera::orbit(state.camera, dx, dy);
            } else {
                camera::pan(state.camera, dx, dy);
            }
            break;
        }
        case SAPP_EVENTTYPE_MOUSE_UP:
            if (input.down && !input.dragging) {
                state.click = {true, e->mouse_button != SAPP_MOUSEBUTTON_LEFT, (e->modifiers & SAPP_MODIFIER_SHIFT) != 0,
                               camera::screen_ray(state.camera, e->mouse_x, e->mouse_y)};
            }
            input.down = false;
            break;
        case SAPP_EVENTTYPE_KEY_DOWN:
            if (!ImGui::GetIO().WantCaptureKeyboard && e->key_code >= 0 && e->key_code < 512) state.keys[e->key_code] = true;
            break;
        case SAPP_EVENTTYPE_KEY_UP:
            if (e->key_code >= 0 && e->key_code < 512) state.keys[e->key_code] = false;
            break;
        case SAPP_EVENTTYPE_MOUSE_SCROLL:
            camera::zoom(state.camera, e->scroll_y);
            break;
        default:
            break;
    }
}

sapp_desc sokol_main(int argc, char **argv) {
    for (int arg = 1; arg < argc; arg++) {
        if (strcmp(argv[arg], "--screenshot") == 0 && arg + 1 < argc) {
            screenshot_path = argv[++arg];
            continue;
        }
        if (strcmp(argv[arg], "--frames") == 0 && arg + 1 < argc) {
            screenshot_frame = atoi(argv[++arg]);
            continue;
        }
        if (strcmp(argv[arg], "--frame-time") == 0 && arg + 1 < argc) {
            frame_time = (float)atof(argv[++arg]);
            continue;
        }
        const int found = shell::find_example(argv[arg]);
        if (found >= 0) initial_example = found;
    }

    sapp_desc desc = {};
    desc.init_cb = init;
    desc.frame_cb = frame;
    desc.cleanup_cb = cleanup;
    desc.event_cb = event;
    desc.width = 1280;
    desc.height = 800;
    desc.sample_count = 4;
    desc.high_dpi = true;
    desc.window_title = "tumble samples";
    desc.logger.func = slog_func;
    return desc;
}
