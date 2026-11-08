import * as g from 'gpucat';
import { type Vec3, vec3 } from 'math';
import type { Renderer } from './renderer';

/** from the camera through a point of the canvas, the direction unit */
export type PointerRay = { origin: Vec3; direction: Vec3 };

export function createPointerRay(): PointerRay {
    return { origin: [0, 0, 0], direction: [0, 0, -1] };
}

const _ndc: Vec3 = [0, 0, 0.5];

/** a click on the canvas, not the end of a drag that orbited the camera */
export function onClick(renderer: Renderer, handler: (event: PointerEvent) => void): void {
    let downX = 0;
    let downY = 0;
    renderer.canvas.addEventListener('pointerdown', (event) => {
        downX = event.clientX;
        downY = event.clientY;
    });
    renderer.canvas.addEventListener('pointerup', (event) => {
        if (Math.hypot(event.clientX - downX, event.clientY - downY) < 5) handler(event);
    });
}

export function pointerRay(out: PointerRay, renderer: Renderer, clientX: number, clientY: number): PointerRay {
    const rect = renderer.canvas.getBoundingClientRect();
    _ndc[0] = ((clientX - rect.left) / rect.width) * 2 - 1;
    _ndc[1] = -(((clientY - rect.top) / rect.height) * 2 - 1);
    renderer.camera.updateViewMatrix();
    // a point part way into the frustum, on the ray from the camera
    g.unproject(out.direction, _ndc, renderer.camera);
    vec3.copy(out.origin, renderer.camera.position as Vec3);
    vec3.subtract(out.direction, out.direction, out.origin);
    vec3.normalize(out.direction, out.direction);
    return out;
}
