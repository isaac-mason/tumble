import type { Quat, Vec3 } from 'math';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    rigidBody,
    updateWorld,
    type World,
} from 'tumble.js';

// the landing page's world, without drawing it: a cube dropped onto a plane

/** a body's look, for whoever draws it */
export type ShapeMesh = { kind: 'box'; halfExtents: [number, number, number] };

export type Drawable = { body: BodyId; mesh: ShapeMesh; color: string };

export type Demo = { world: World; bodies: Drawable[] };

export const STEP = 1 / 60;

export function createDemo(): Demo {
    const settings = createWorldSettings();
    const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
    const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
    enableCollision(settings, moving, moving);
    enableCollision(settings, moving, notMoving);
    const world = createWorld(settings);

    const groundHalf: [number, number, number] = [10, 0.5, 10];
    const ground = rigidBody.create(world, {
        shape: box.create({ halfExtents: groundHalf }),
        objectLayer: notMoving,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    // tipped onto an edge, so it tumbles as it lands
    const cubeHalf: [number, number, number] = [0.5, 0.5, 0.5];
    const turn: Quat = [0.3, 0.2, 0.25, 0.9];
    const length = Math.hypot(...turn);
    const position: Vec3 = [0, 4, 0];
    const cube = rigidBody.create(world, {
        shape: box.create({ halfExtents: cubeHalf }),
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position,
        quaternion: turn.map((v) => v / length) as Quat,
    });

    return {
        world,
        bodies: [
            { body: ground, mesh: { kind: 'box', halfExtents: groundHalf }, color: '#5a5f66' },
            { body: cube, mesh: { kind: 'box', halfExtents: cubeHalf }, color: '#4ecdc4' },
        ],
    };
}

export function stepDemo(demo: Demo): void {
    updateWorld(demo.world, undefined, STEP);
}
