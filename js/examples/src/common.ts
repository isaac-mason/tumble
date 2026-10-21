import * as g from 'gpucat';
import { init as initTumble } from 'tumble';
import wasm from 'tumble/tumble.wasm?url';

export type View = {
    renderer: Awaited<ReturnType<typeof g.init>>;
    scene: g.Scene;
    camera: g.PerspectiveCamera;
    controls: g.OrbitControls;
    render: () => void;
};

/** the wasm, a webgpu renderer onto a full window canvas, a camera orbiting the target */
export async function setup(eye: [number, number, number], target: [number, number, number]): Promise<View> {
    await initTumble({ wasm });

    const canvas = document.createElement('canvas');
    canvas.style.display = 'block';
    document.body.appendChild(canvas);
    const view = g.createCanvasTarget(canvas, { samples: 4 });
    view.setPixelRatio(devicePixelRatio);
    view.setSize(window.innerWidth, window.innerHeight);
    const renderer = await g.init(g.webgpu());

    const scene = new g.Scene();
    const camera = new g.PerspectiveCamera(Math.PI / 3, window.innerWidth / window.innerHeight, 0.1, 500);
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
        view.setSize(window.innerWidth, window.innerHeight);
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
    });

    const composite = g.fullscreen(g.renderOutput(g.renderTexture(scene, camera).getTextureNode()));
    const render = () => {
        controls.update();
        scene.updateWorldMatrix();
        camera.updateViewMatrix();
        const frame = g.frame(renderer);
        const pass = frame.pass({ target: view });
        pass.draw(composite);
        pass.end();
        frame.submit();
    };
    return { renderer, scene, camera, controls, render };
}

const materials = new Map<string, g.Material>();

/** lit by a light from above, one material per colour */
export function lit(color: string): g.Material {
    const known = materials.get(color);
    if (known) return known;
    const position = g.attribute('position', g.d.vec3f);
    const normal = g.attribute('normal', g.d.vec3f);
    const world = g.mul(g.modelWorldMatrix, g.vec4(position, g.f32(1)));
    const clip = g.mul(g.cameraProjectionMatrix, g.mul(g.cameraViewMatrix, world));
    const worldNormal = g.varying(g.normalize(g.mul(g.modelNormalMatrix, normal)), 'v_normal');
    const light = g.vec3(g.f32(0.4), g.f32(1), g.f32(0.3)).normalize();
    const shade = g.f32(0.35).add(g.f32(0.65).mul(worldNormal.dot(light).max(g.f32(0))));
    const base = g.rgb(color);
    const material = g.createMaterial({ vertex: clip, fragment: g.vec4(g.vec3(base.x, base.y, base.z).mul(shade), g.f32(1)) });
    materials.set(color, material);
    return material;
}

/** the shapes the examples make, by their settings: a mesh in the shape's own space */
export type ShapeMesh =
    | { kind: 'box'; halfExtents: [number, number, number] }
    | { kind: 'sphere'; radius: number }
    | { kind: 'capsule'; halfHeightOfCylinder: number; radius: number }
    | { kind: 'cylinder'; halfHeight: number; radius: number };

export function meshOf(shape: ShapeMesh, color: string): g.Object3D {
    const material = lit(color);
    switch (shape.kind) {
        case 'box': {
            const [x, y, z] = shape.halfExtents;
            return new g.Mesh(g.createBoxGeometry(2 * x, 2 * y, 2 * z), material);
        }
        case 'sphere':
            return new g.Mesh(g.createSphereGeometry(shape.radius, 24, 16), material);
        case 'cylinder':
            return new g.Mesh(g.createCylinderGeometry(shape.radius, shape.radius, 2 * shape.halfHeight, 24), material);
        case 'capsule': {
            // a cylinder with a sphere on each end
            const group = new g.Object3D();
            group.add(
                new g.Mesh(g.createCylinderGeometry(shape.radius, shape.radius, 2 * shape.halfHeightOfCylinder, 24), material),
            );
            for (const end of [-1, 1]) {
                const cap = new g.Mesh(g.createSphereGeometry(shape.radius, 24, 16), material);
                cap.position[1] = end * shape.halfHeightOfCylinder;
                group.add(cap);
            }
            return group;
        }
    }
}

/** steps at a fixed 60 a second, at most 4 a frame, then renders */
export function loop(step: (deltaTime: number) => void, frame: () => void): void {
    const deltaTime = 1 / 60;
    let last = performance.now();
    let owed = 0;
    const tick = (now: number) => {
        owed += Math.min((now - last) / 1000, 4 * deltaTime);
        last = now;
        while (owed >= deltaTime) {
            step(deltaTime);
            owed -= deltaTime;
        }
        frame();
        requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
}

/** a line of help over the canvas */
export function caption(text: string): void {
    const element = document.createElement('div');
    element.className = 'caption';
    element.textContent = text;
    document.body.appendChild(element);
}
