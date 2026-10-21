#pragma once

#include "app/matrix.h"
#include "tumble/math/types.h"

struct Ray {
    tumble::Float3 origin;
    tumble::Float3 direction;
};

struct Camera {
    tumble::Float3 target;
    float yaw;
    float pitch;
    float distance;
    float fov_y;
    float width;
    float height;
    tumble::Float3 eye;
    tumble::Float3 forward;
    tumble::Float3 side;
    tumble::Float3 up;
    Mat4 view_projection;
};

namespace camera {

Camera create();
void look_at(Camera &camera, tumble::Float3 eye, tumble::Float3 target);
void update(Camera &camera, float width, float height);
void orbit(Camera &camera, float dx, float dy);
void pan(Camera &camera, float dx, float dy);
void zoom(Camera &camera, float amount);
Ray screen_ray(const Camera &camera, float x, float y);

}
