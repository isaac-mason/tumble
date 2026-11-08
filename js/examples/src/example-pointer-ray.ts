import { type Quat, quat, type Vec3, vec3 } from 'math';
import { mulberry32 } from 'math/random';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    CastRayStatus,
    castRay,
    convexHull,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    INVALID_BODY_ID,
    init,
    MotionType,
    rigidBody,
    sphere,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, setBodyColor } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, drawPoint, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createPointerRay, pointerRay } from './common/pointer';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { debugColor, palette, spectrum } from './common/theme';

await init();
const renderer = await createRenderer([0, 5, 40], [0, 0, 0]);
const worldSettings = createWorldSettings();
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
worldSettings.gravity = [0, 0, 0];
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(23);
const random = () => mulberry32.sample(rng);

const shapes = [
    box.create({ halfExtents: [0.8, 0.8, 0.8] }),
    sphere.create({ radius: 0.8 }),
    convexHull.create({ positions: [-0.8, 0, -0.8, 0.8, 0, -0.8, 0.8, 0, 0.8, -0.8, 0, 0.8, 0, 1.2, 0] }),
];

// a thousand static bodies over shells of a sphere, spread evenly by the golden angle, each breathing in and out along
// its radius and turning: moved by hand with setTransform, as static bodies are
const COUNT = 1000;
type Floater = {
    body: BodyId;
    color: string;
    outward: Vec3;
    nearest: number;
    speed: number;
    phase: number;
    spin: Vec3;
    quaternion: Quat;
};
const floaters: Floater[] = [];
const floaterOf = new Map<BodyId, Floater>();
for (let i = 0; i < COUNT; i++) {
    const theta = (2 * Math.PI * i) / ((1 + Math.sqrt(5)) / 2);
    const phi = Math.acos(1 - (2 * (i + 0.5)) / COUNT);
    const outward: Vec3 = [Math.sin(phi) * Math.cos(theta), Math.sin(phi) * Math.sin(theta), Math.cos(phi)];
    const radius = 10 + (i % 5) * 5;
    const quaternion = quat.fromEuler(
        [0, 0, 0, 1],
        [random() * Math.PI * 2, random() * Math.PI * 2, random() * Math.PI * 2, 'xyz'],
    );
    const body = rigidBody.create(world, {
        shape: shapes[i % shapes.length]!,
        objectLayer: notMoving,
        motionType: MotionType.STATIC,
        position: vec3.scale([0, 0, 0], outward, radius),
        quaternion,
    });
    const color = spectrum[i % spectrum.length]!;
    addBody(bodies, body, color);
    const floater: Floater = {
        body,
        color,
        outward,
        nearest: radius - 3,
        speed: 0.2 + random() * 0.3,
        phase: random() * Math.PI * 2,
        spin: [(random() - 0.5) * 0.5, (random() - 0.5) * 0.5, (random() - 0.5) * 0.5],
        quaternion,
    };
    floaters.push(floater);
    floaterOf.set(body, floater);
}

const _position: Vec3 = [0, 0, 0];
const _turn: Quat = [0, 0, 0, 1];
let time = 0;
function moveFloaters(deltaTime: number): void {
    time += deltaTime;
    for (const floater of floaters) {
        const out = (Math.sin(time * floater.speed + floater.phase) + 1) / 2;
        vec3.scale(_position, floater.outward, floater.nearest + 6 * out);
        const rate = vec3.length(floater.spin);
        quat.setAxisAngle(_turn, vec3.scale([0, 0, 0], floater.spin, 1 / rate), rate * deltaTime);
        quat.normalize(floater.quaternion, quat.multiply(floater.quaternion, _turn, floater.quaternion));
        rigidBody.setTransform(world, floater.body, _position, floater.quaternion, false);
    }
}

// the ray from the camera through the pointer, cast every step: the body it first hits lights up
const ray = createPointerRay();
let pointerX = -1;
let pointerY = -1;
renderer.canvas.addEventListener('pointermove', (event) => {
    pointerX = event.clientX;
    pointerY = event.clientY;
});
const collector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();
const RAY_LENGTH = 100;
const hitPoint: Vec3 = [0, 0, 0];
let lit: BodyId = INVALID_BODY_ID;

function castPointerRay(): void {
    let hit: BodyId = INVALID_BODY_ID;
    if (pointerX >= 0) {
        pointerRay(ray, renderer, pointerX, pointerY);
        collector.reset();
        castRay(world, collector, raySettings, ray.origin, ray.direction, RAY_LENGTH);
        if (collector.hit.status === CastRayStatus.COLLIDING) {
            hit = collector.hit.bodyIdB;
            vec3.scaleAndAdd(hitPoint, ray.origin, ray.direction, collector.hit.fraction * RAY_LENGTH);
        }
    }
    if (hit === lit) return;
    if (lit !== INVALID_BODY_ID) setBodyColor(bodies, lit, floaterOf.get(lit)!.color);
    if (hit !== INVALID_BODY_ID) setBodyColor(bodies, hit, palette.light);
    lit = hit;
}

const panel = createPanel('pointer ray');
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('point at the shapes, and a ray from the camera picks the one in front');

const hitColor = debugColor(palette.light);

loop(
    stepTime,
    (deltaTime) => {
        moveFloaters(deltaTime);
        updateWorld(world, undefined, deltaTime);
        castPointerRay();
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        if (lit !== INVALID_BODY_ID) drawPoint(overlay, hitPoint, hitColor, 0.4, true);
        endOverlay(overlay);
        render(renderer);
    },
);
