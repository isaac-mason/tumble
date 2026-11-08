import * as g from 'gpucat';
import type { Vec3 } from 'math';
import { clearColor } from './theme';

// the drawing, in the worker on the canvas the page handed over

export type View = {
    renderer: g.Renderer<g.DeviceBackend>;
    target: g.CanvasTarget;
    scene: g.Scene;
    camera: g.PerspectiveCamera;
    /** the scene, multisampled, drawn over the canvas */
    output: g.Mesh;
};

const FOV = (50 * Math.PI) / 180;

/** webgpu where the worker has it, else webgl2 on the canvas: both run the same node materials */
async function createRenderer(target: g.CanvasTarget): Promise<g.Renderer<g.DeviceBackend>> {
    if (navigator.gpu !== undefined) {
        try {
            if ((await navigator.gpu.requestAdapter()) !== null) return await g.init(g.webgpu());
        } catch (error) {
            console.warn('webgpu failed to start, drawing with webgl2:', error);
        }
    }
    return g.init(g.webgl({ target }));
}

/** a camera at eye looking at lookAt */
export async function createView(
    canvas: OffscreenCanvas,
    width: number,
    height: number,
    pixelRatio: number,
    eye: Vec3,
    lookAt: Vec3,
): Promise<View> {
    const target = g.createCanvasTarget(canvas);
    const renderer = await createRenderer(target);
    const scene = new g.Scene();
    const camera = new g.PerspectiveCamera(FOV, width / height, 0.1, 500);
    camera.position[0] = eye[0];
    camera.position[1] = eye[1];
    camera.position[2] = eye[2];
    camera.lookAt(lookAt);
    scene.add(camera);
    const output = g.fullscreen(g.renderTexture(scene, camera, { clearColor, samples: 4 }).getTextureNode());
    const view: View = { renderer, target, scene, camera, output };
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
    const pass = frame.pass({ target: view.target });
    pass.draw(view.output);
    pass.end();
    frame.submit();
}
