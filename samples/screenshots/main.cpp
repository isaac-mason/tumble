#include <ImageIO/ImageIO.h>
#include <OpenGL/OpenGL.h>
#include <OpenGL/gl3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/examples.h"
#include "app/shell.h"
#include "sokol_time.h"

using namespace tumble;

static const int FRAMES = 120;
static const int WIDTH = 600;
static const int HEIGHT = 400;
static const int SAMPLES = 4;
static const char *OUT_DIR = "samples/screenshots";

struct Target {
    GLuint msaa_framebuffer;
    GLuint msaa_color;
    GLuint msaa_depth;
    GLuint resolve_framebuffer;
    GLuint resolve_color;
};

static bool create_context() {
    const CGLPixelFormatAttribute attributes[] = {
        kCGLPFAOpenGLProfile, (CGLPixelFormatAttribute)kCGLOGLPVersion_GL4_Core,
        kCGLPFAAccelerated,
        kCGLPFAColorSize, (CGLPixelFormatAttribute)24,
        kCGLPFAAlphaSize, (CGLPixelFormatAttribute)8,
        (CGLPixelFormatAttribute)0,
    };
    CGLPixelFormatObj pixel_format = nullptr;
    GLint format_count = 0;
    if (CGLChoosePixelFormat(attributes, &pixel_format, &format_count) != kCGLNoError || !pixel_format) return false;
    CGLContextObj context = nullptr;
    const CGLError error = CGLCreateContext(pixel_format, nullptr, &context);
    CGLDestroyPixelFormat(pixel_format);
    if (error != kCGLNoError) return false;
    return CGLSetCurrentContext(context) == kCGLNoError;
}

static Target create_target() {
    Target target = {};
    glGenRenderbuffers(1, &target.msaa_color);
    glBindRenderbuffer(GL_RENDERBUFFER, target.msaa_color);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, SAMPLES, GL_RGBA8, WIDTH, HEIGHT);
    glGenRenderbuffers(1, &target.msaa_depth);
    glBindRenderbuffer(GL_RENDERBUFFER, target.msaa_depth);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, SAMPLES, GL_DEPTH24_STENCIL8, WIDTH, HEIGHT);
    glGenFramebuffers(1, &target.msaa_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, target.msaa_framebuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, target.msaa_color);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, target.msaa_depth);

    glGenRenderbuffers(1, &target.resolve_color);
    glBindRenderbuffer(GL_RENDERBUFFER, target.resolve_color);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, WIDTH, HEIGHT);
    glGenFramebuffers(1, &target.resolve_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, target.resolve_framebuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, target.resolve_color);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return target;
}

static sg_environment offscreen_environment() {
    sg_environment environment = {};
    environment.defaults.color_format = SG_PIXELFORMAT_RGBA8;
    environment.defaults.depth_format = SG_PIXELFORMAT_DEPTH_STENCIL;
    environment.defaults.sample_count = SAMPLES;
    return environment;
}

static sg_swapchain offscreen_swapchain(const Target &target) {
    sg_swapchain swapchain = {};
    swapchain.width = WIDTH;
    swapchain.height = HEIGHT;
    swapchain.sample_count = SAMPLES;
    swapchain.color_format = SG_PIXELFORMAT_RGBA8;
    swapchain.depth_format = SG_PIXELFORMAT_DEPTH_STENCIL;
    swapchain.gl.framebuffer = target.msaa_framebuffer;
    return swapchain;
}

static bool write_png(const char *path, const uint8_t *rgba) {
    CGColorSpaceRef color_space = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef bitmap = CGBitmapContextCreate((void *)rgba, WIDTH, HEIGHT, 8, WIDTH * 4, color_space,
                                                (CGBitmapInfo)kCGImageAlphaNoneSkipLast);
    CGImageRef image = CGBitmapContextCreateImage(bitmap);
    CFURLRef url = CFURLCreateFromFileSystemRepresentation(nullptr, (const UInt8 *)path, (CFIndex)strlen(path), false);
    CGImageDestinationRef destination = CGImageDestinationCreateWithURL(url, CFSTR("public.png"), 1, nullptr);
    bool written = false;
    if (destination) {
        CGImageDestinationAddImage(destination, image, nullptr);
        written = CGImageDestinationFinalize(destination);
        CFRelease(destination);
    }
    CFRelease(url);
    CGImageRelease(image);
    CGContextRelease(bitmap);
    CGColorSpaceRelease(color_space);
    return written;
}

static bool save_png(const Target &target, const char *path) {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, target.msaa_framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target.resolve_framebuffer);
    glBlitFramebuffer(0, 0, WIDTH, HEIGHT, 0, 0, WIDTH, HEIGHT, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, target.resolve_framebuffer);

    const size_t row = (size_t)WIDTH * 4;
    uint8_t *pixels = (uint8_t *)malloc(row * HEIGHT);
    uint8_t *image = (uint8_t *)malloc(row * HEIGHT);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    // gl's rows run bottom up
    for (int y = 0; y < HEIGHT; y++) memcpy(image + (size_t)y * row, pixels + (size_t)(HEIGHT - 1 - y) * row, row);
    const bool written = write_png(path, image);
    free(image);
    free(pixels);
    return written;
}

static bool run_example(int index, const Target &target) {
    const size_t blocks_before = util::allocated_blocks();
    const uint64_t start = stm_now();

    char id[128];
    shell::example_id(id, sizeof(id), index);
    char path[512];
    snprintf(path, sizeof(path), "%s/%s.png", OUT_DIR, id);

    shell::setup(offscreen_environment(), index);
    for (int frame = 0; frame < FRAMES; frame++) {
        FrameInput input = {};
        input.width = (float)WIDTH;
        input.height = (float)HEIGHT;
        input.delta_time = 1.0f / 60.0f;
        input.dpi_scale = 1.0f;
        input.pointer_x = WIDTH * 0.5f;
        input.pointer_y = HEIGHT * 0.5f;
        input.swapchain = offscreen_swapchain(target);
        input.fixed_step = true;
        input.hide_ui = true;
        shell::frame(input);
    }
    glFinish();
    const bool written = save_png(target, path);
    shell::shutdown();

    const bool leaked = util::allocated_blocks() != blocks_before;
    const char *status = !written ? "FAIL" : leaked ? "LEAK" : "ok";
    printf("%-4s %-36s %8.1f ms  %s\n", status, examples::at(index).name, stm_ms(stm_since(start)), path);
    fflush(stdout);
    return written && !leaked;
}

int main(int argc, char **argv) {
    stm_setup();
    if (!create_context()) {
        printf("could not create an offscreen OpenGL context\n");
        return 1;
    }
    const Target target = create_target();
    int failures = 0;
    int ran = 0;
    for (int i = 0; i < examples::count(); i++) {
        char id[128];
        shell::example_id(id, sizeof(id), i);
        bool selected = argc < 2;
        for (int arg = 1; arg < argc; arg++) selected |= strstr(id, argv[arg]) != nullptr;
        if (!selected) continue;
        if (!run_example(i, target)) failures++;
        ran++;
    }
    printf("%d examples, %d failed\n", ran, failures);
    return failures == 0 ? 0 : 1;
}
