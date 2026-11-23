import * as g from 'gpucat';
import type { Vec3 } from 'math';
import { clearColor } from './theme';

// the drawing, in the worker on the canvas the page handed over

export type View = {
    renderer: g.Renderer<g.DeviceBackend>;
    target: g.CanvasTarget;
    scene: g.Scene;
    camera: g.PerspectiveCamera;
};

/** a backend the page asked for, else whichever there is */
export type RendererChoice = 'webgpu' | 'webgl' | null;

/**
 * webgpu where the worker has it, else webgl2 on the canvas: both run the same node materials. one asked for is used
 * with no fallback
 */
async function createRenderer(target: g.CanvasTarget, choice: RendererChoice): Promise<g.Renderer<g.DeviceBackend>> {
    if (choice === 'webgpu') return g.init(g.webgpu());
    if (choice === 'webgl') return g.init(g.webgl({ target }));
    if (navigator.gpu !== undefined) {
        try {
            if ((await navigator.gpu.requestAdapter()) !== null) return await g.init(g.webgpu());
        } catch (error) {
            console.warn('webgpu failed to start, drawing with webgl2:', error);
        }
    }
    return g.init(g.webgl({ target }));
}

/** a camera at eye looking at lookAt, with the given vertical field of view in radians */
export async function createView(
    canvas: OffscreenCanvas,
    width: number,
    height: number,
    pixelRatio: number,
    eye: Vec3,
    lookAt: Vec3,
    fov: number,
    choice: RendererChoice,
): Promise<View> {
    // the scene is drawn straight onto the canvas, multisampled
    const target = g.createCanvasTarget(canvas, { samples: 4, clearColor });
    const renderer = await createRenderer(target, choice);
    console.log(`renderer: ${renderer.api}`);
    const scene = new g.Scene();
    const camera = new g.PerspectiveCamera(fov, width / height, 0.3, 500);
    camera.position[0] = eye[0];
    camera.position[1] = eye[1];
    camera.position[2] = eye[2];
    camera.lookAt(lookAt);
    scene.add(camera);
    const view: View = { renderer, target, scene, camera };
    resizeView(view, width, height, pixelRatio);
    return view;
}

export function resizeView(view: View, width: number, height: number, pixelRatio: number): void {
    view.target.setPixelRatio(pixelRatio);
    view.target.setSize(width, height);
    view.camera.aspect = width / height;
    view.camera.updateProjectionMatrix();
}

export function renderView(view: View): void {
    view.scene.updateWorldMatrix();
    view.camera.updateViewMatrix();
    const frame = g.frame(view.renderer);
    const pass = frame.pass({ target: view.target, camera: view.camera });
    pass.scene(view.scene);
    pass.end();
    frame.submit();
}
