import * as g from 'gpucat';
import type { Vec3 } from 'math';
import { clearColor } from './theme';

export type Renderer = {
    gpu: g.Renderer<g.DeviceBackend>;
    canvas: HTMLCanvasElement;
    target: g.CanvasTarget;
    scene: g.Scene;
    camera: g.PerspectiveCamera;
    controls: g.OrbitControls;
};

/** webgpu where the browser has it, else webgl2 on the canvas: both run the same node materials */
async function startGpu(target: g.CanvasTarget): Promise<g.Renderer<g.DeviceBackend>> {
    if (navigator.gpu !== undefined) {
        try {
            if ((await navigator.gpu.requestAdapter()) !== null) return await g.init(g.webgpu());
        } catch (error) {
            console.warn('webgpu failed to start, drawing with webgl2:', error);
        }
    }
    return g.init(g.webgl({ target }));
}

/** a full window canvas, and a camera at eye orbiting target */
export async function createRenderer(eye: Vec3, target: Vec3): Promise<Renderer> {
    const canvas = document.createElement('canvas');
    document.body.appendChild(canvas);
    // the scene is drawn straight onto the canvas, multisampled. the theme's colours are written as they are, so the
    // canvas shows the hex values unchanged
    const canvasTarget = g.createCanvasTarget(canvas, { samples: 4, clearColor });
    canvasTarget.setPixelRatio(devicePixelRatio);
    canvasTarget.setSize(window.innerWidth, window.innerHeight);
    const gpu = await startGpu(canvasTarget);

    const scene = new g.Scene();
    const camera = new g.PerspectiveCamera((75 * Math.PI) / 180, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position[0] = eye[0];
    camera.position[1] = eye[1];
    camera.position[2] = eye[2];
    scene.add(camera);
    const controls = new g.OrbitControls(camera, canvas);
    controls.target[0] = target[0];
    controls.target[1] = target[1];
    controls.target[2] = target[2];
    controls.update();

    window.addEventListener('resize', () => {
        canvasTarget.setSize(window.innerWidth, window.innerHeight);
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
    });

    return { gpu, canvas, target: canvasTarget, scene, camera, controls };
}

/** the camera follows its controls, and the scene is drawn to the canvas */
export function render(renderer: Renderer): void {
    renderer.controls.update();
    renderer.scene.updateWorldMatrix();
    renderer.camera.updateViewMatrix();
    const frame = g.frame(renderer.gpu);
    const pass = frame.pass({ target: renderer.target, camera: renderer.camera });
    pass.scene(renderer.scene);
    pass.end();
    frame.submit();
}
