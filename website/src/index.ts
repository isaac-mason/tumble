import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    debug,
    debugDraw,
    enableCollision,
    MotionType,
    rigidBody,
    updateWorld,
} from 'tumble.js';
import {
    addStepPose,
    blendStepPose,
    createDebugLines,
    createStepPoses,
    loop,
    meshOf,
    readStepPoses,
    setup,
    showDebugLines,
} from '../../js/examples/src/common';

const view = await setup([6, 4, 8], [0, 1, 0]);

const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
enableCollision(settings, moving, notMoving);
const world = createWorld(settings);

const groundHalfExtents: [number, number, number] = [10, 0.5, 10];
rigidBody.create(world, {
    shape: box.create({ halfExtents: groundHalfExtents }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
});
const groundMesh = meshOf({ kind: 'box', halfExtents: groundHalfExtents }, '#5a5f66');
groundMesh.position[1] = -0.5;
view.scene.add(groundMesh);

// a cube dropped on its corner, so it tumbles as it lands
const cubeHalfExtents: [number, number, number] = [0.5, 0.5, 0.5];
const cube = rigidBody.create(world, {
    shape: box.create({ halfExtents: cubeHalfExtents }),
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 5, 0],
    quaternion: [0.3, 0.2, 0.1, 0.92736],
});
const cubeMesh = meshOf({ kind: 'box', halfExtents: cubeHalfExtents }, '#4ecdc4');
view.scene.add(cubeMesh);

const poses = createStepPoses(world);
const cubeSlot = addStepPose(poses, cube);

const debugLines = createDebugLines(view.scene);
let showDebug = false;
const debugToggle = document.getElementById('debug-toggle')!;
const toggleDebug = () => {
    showDebug = !showDebug;
    debugToggle.textContent = showDebug ? 'hide debug view [d]' : 'show debug view [d]';
};
debugToggle.addEventListener('click', toggleDebug);
window.addEventListener('keydown', (event) => {
    if (event.key === 'd' || event.key === 'D') toggleDebug();
});

document.getElementById('loading')!.remove();

loop(
    (deltaTime) => {
        updateWorld(world, poses.listener, deltaTime);
        readStepPoses(poses);
    },
    (alpha) => {
        blendStepPose(poses, cubeSlot, alpha, cubeMesh.position, cubeMesh.quaternion);
        debugDraw.clear(debugLines.buffers);
        if (showDebug) debug.bodies(debugLines.buffers, world, {});
        showDebugLines(debugLines);
        view.render();
    },
);
