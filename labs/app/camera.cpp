#include "app/camera.h"

#include <math.h>

using namespace tumble;
using namespace vec;

namespace camera {

static float clamp(float value, float low, float high) { return fminf(fmaxf(value, low), high); }

Camera create() {
    Camera camera = {};
    camera.yaw = -0.8f;
    camera.pitch = 0.5f;
    camera.distance = 20;
    camera.fov_y = 60.0f * 3.14159265f / 180.0f;
    return camera;
}

void look_at(Camera &camera, Float3 eye, Float3 target) {
    const Float3 offset = subtract(eye, target);
    camera.target = target;
    camera.distance = length(offset);
    camera.pitch = clamp(asinf(offset.y / fmaxf(camera.distance, 1e-6f)), -1.5f, 1.5f);
    camera.yaw = atan2f(offset.x, offset.z);
}

void update(Camera &camera, float width, float height) {
    camera.width = width;
    camera.height = height;

    const Float3 offset = {
        cosf(camera.pitch) * sinf(camera.yaw) * camera.distance,
        sinf(camera.pitch) * camera.distance,
        cosf(camera.pitch) * cosf(camera.yaw) * camera.distance,
    };
    camera.eye = add(camera.target, offset);
    camera.forward = normalize(subtract(camera.target, camera.eye));
    camera.side = normalize(cross(camera.forward, {0, 1, 0}));
    camera.up = cross(camera.side, camera.forward);

    const float near_plane = fmaxf(0.05f, camera.distance * 0.01f);
    const float far_plane = camera.distance * 20 + 200;
    const Mat4 projection = matrix::perspective(camera.fov_y, width / fmaxf(height, 1), near_plane, far_plane);
    const Mat4 view = matrix::look_at(camera.eye, camera.target, {0, 1, 0});
    camera.view_projection = matrix::multiply(projection, view);
}

void orbit(Camera &camera, float dx, float dy) {
    camera.yaw -= dx * 0.008f;
    camera.pitch = clamp(camera.pitch + dy * 0.008f, -1.5f, 1.5f);
}

void pan(Camera &camera, float dx, float dy) {
    const float amount = camera.distance * 0.0015f;
    camera.target = scale_and_add(camera.target, camera.side, -dx * amount);
    camera.target = scale_and_add(camera.target, camera.up, dy * amount);
}

void zoom(Camera &camera, float amount) { camera.distance = fmaxf(0.5f, camera.distance * expf(-amount * 0.1f)); }

Ray screen_ray(const Camera &camera, float x, float y) {
    const float ndc_x = (x / camera.width) * 2 - 1;
    const float ndc_y = 1 - (y / camera.height) * 2;
    const float tan_half = tanf(camera.fov_y * 0.5f);
    const float aspect = camera.width / fmaxf(camera.height, 1);

    Float3 direction = camera.forward;
    direction = scale_and_add(direction, camera.side, ndc_x * tan_half * aspect);
    direction = scale_and_add(direction, camera.up, ndc_y * tan_half);
    return {camera.eye, normalize(direction)};
}

}
