#pragma once

#include <math.h>

#include "app/vec.h"

struct Mat4 {
    float m[16];
};

namespace matrix {

inline Mat4 identity() { return {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}}; }

inline Mat4 multiply(const Mat4 &a, const Mat4 &b) {
    Mat4 out = {};
    for (int column = 0; column < 4; column++) {
        for (int row = 0; row < 4; row++) {
            float sum = 0;
            for (int k = 0; k < 4; k++) sum += a.m[k * 4 + row] * b.m[column * 4 + k];
            out.m[column * 4 + row] = sum;
        }
    }
    return out;
}

inline Mat4 perspective(float fov_y_radians, float aspect, float near_plane, float far_plane) {
    float f = 1.0f / tanf(fov_y_radians * 0.5f);
    Mat4 out = {};
    out.m[0] = f / aspect;
    out.m[5] = f;
    out.m[10] = (far_plane + near_plane) / (near_plane - far_plane);
    out.m[11] = -1;
    out.m[14] = (2 * far_plane * near_plane) / (near_plane - far_plane);
    return out;
}

inline Mat4 look_at(tumble::Float3 eye, tumble::Float3 target, tumble::Float3 up) {
    using namespace vec;
    tumble::Float3 forward = vec::normalize(vec::subtract(target, eye));
    tumble::Float3 side = vec::normalize(vec::cross(forward, up));
    tumble::Float3 true_up = vec::cross(side, forward);
    return {{
        side.x, true_up.x, -forward.x, 0,
        side.y, true_up.y, -forward.y, 0,
        side.z, true_up.z, -forward.z, 0,
        -vec::dot(side, eye), -vec::dot(true_up, eye), vec::dot(forward, eye), 1,
    }};
}

/** a rotation then a translation */
inline Mat4 compose(tumble::Float3 position, tumble::Float4 q) {
    const float x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
    const float xx = q.x * x2, xy = q.x * y2, xz = q.x * z2, yy = q.y * y2, yz = q.y * z2, zz = q.z * z2;
    const float wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;
    return {{
        1 - (yy + zz), xy + wz, xz - wy, 0,
        xy - wz, 1 - (xx + zz), yz + wx, 0,
        xz + wy, yz - wx, 1 - (xx + yy), 0,
        position.x, position.y, position.z, 1,
    }};
}

inline tumble::Float3 transform_point(const Mat4 &a, tumble::Float3 p) {
    return {
        a.m[0] * p.x + a.m[4] * p.y + a.m[8] * p.z + a.m[12],
        a.m[1] * p.x + a.m[5] * p.y + a.m[9] * p.z + a.m[13],
        a.m[2] * p.x + a.m[6] * p.y + a.m[10] * p.z + a.m[14],
    };
}

}
