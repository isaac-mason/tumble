import type * as gpucat from 'gpucat';
import {
    Box,
    Capsule,
    createDefaultUpdateSettings,
    Filter,
    GroundState,
    KCC,
    KCCPolicy,
    MotionType,
    type Quat,
    RigidBody,
    Transformed,
    type Vec3,
    World,
    WorldSettings,
} from 'tumble.js';
import { vec3 } from 'math';
import { addStepPose, blendStepPose, caption, createStepPoses, loop, meshOf, readStepPoses, setup } from './common';

const view = await setup([0, 8, 14], [0, 1, 0]);
caption('WASD to walk, space to jump: up the stairs and slope, onto the belt and the moving platform, into the crates');

const settings = WorldSettings.create();
const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
const notMoving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
WorldSettings.enableCollision(settings, moving, moving);
WorldSettings.enableCollision(settings, moving, notMoving);
const gravity: Vec3 = [0, -20, 0];
settings.gravity = gravity;
const world = World.create(settings);
const poses = createStepPoses(world);
const posed: gpucat.Object3D[] = [];

/** a box body and its mesh: static ones placed once, the rest posed each frame */
function box(
    halfExtents: Vec3,
    position: Vec3,
    color: string,
    motionType: MotionType = MotionType.STATIC,
    quaternion: Quat = [0, 0, 0, 1],
): number {
    const body = RigidBody.create(world, {
        shape: Box.create({ halfExtents }),
        objectLayer: motionType === MotionType.STATIC ? notMoving : moving,
        motionType,
        position,
        quaternion,
        mass: motionType === MotionType.DYNAMIC ? 20 : undefined,
    });
    const mesh = meshOf({ kind: 'box', halfExtents }, color);
    if (motionType === MotionType.STATIC) {
        mesh.position.splice(0, 3, ...position);
        mesh.quaternion.splice(0, 4, ...quaternion);
    } else {
        posed[addStepPose(poses, body)] = mesh;
    }
    view.scene.add(mesh);
    return body;
}

box([30, 0.5, 30], [0, -0.5, 0], '#5a5f66');
for (let i = 0; i < 8; i++) box([1.5, 0.15 * (i + 1), 0.5], [-8, 0.15 * (i + 1), -2 - i], '#8a8f96');
const tilt = (20 * Math.PI) / 180;
box([2, 0.2, 5], [8, 1.6, -4], '#8a8f96', MotionType.STATIC, [Math.sin(tilt / 2), 0, 0, Math.cos(tilt / 2)]);
const belt = box([1.5, 0.1, 6], [0, 0.1, -6], '#3b4bb4');
const platform = box([1.5, 0.2, 1.5], [-8, 1, 6], '#87b6e6', MotionType.KINEMATIC);
for (let i = 0; i < 3; i++) box([0.5, 0.5, 0.5], [5 + i * 1.2, 0.5, 4], '#d6a35c', MotionType.DYNAMIC);

// the belt carries whatever stands on it, as its surface moves
const policy = KCCPolicy.create();
KCCPolicy.set(policy, belt, { surfaceLinearVelocity: [0, 0, 4] });

const pill = Capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.4 });
const standing = Transformed.create({ shape: pill, position: [0, 0.9, 0] });
const character = KCC.create(
    { shape: standing, maxSlopeAngle: (45 * Math.PI) / 180, innerRigidBody: { shape: standing, objectLayer: moving } },
    [0, 0, 4],
    [0, 0, 0, 1],
);
KCC.add(world, character);
const filter = Filter.create(world);
Filter.setFromBody(filter, world, character.innerRigidBodyId);
const characterMesh = meshOf({ kind: 'capsule', halfHeightOfCylinder: 0.5, radius: 0.4 }, '#f2bf4c');
view.scene.add(characterMesh);
// where the character was as the last step began and where the step left it, and where it was drawn last frame
const characterBefore: Vec3 = [...character.position];
const characterNow: Vec3 = [...character.position];
const drawn: Vec3 = [...character.position];
const followed: Vec3 = [...character.position];

const keys = new Set<string>();
window.addEventListener('keydown', (event) => keys.add(event.code));
window.addEventListener('keyup', (event) => keys.delete(event.code));

const updateSettings = createDefaultUpdateSettings();
let steps = 0;
const input: Vec3 = [0, 0, 0];

function steer(deltaTime: number): void {
    // the input on the ground plane, relative to where the camera looks
    const forward = (keys.has('KeyW') ? 1 : 0) - (keys.has('KeyS') ? 1 : 0);
    const right = (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0);
    const { position: eye } = view.camera;
    const target = view.controls.target;
    const lookX = target[0] - eye[0];
    const lookZ = target[2] - eye[2];
    const length = Math.hypot(lookX, lookZ) || 1;
    const fx = lookX / length;
    const fz = lookZ / length;
    const wanted: Vec3 = [fx * forward - fz * right, 0, fz * forward + fx * right];
    const speed = Math.hypot(wanted[0], wanted[2]);
    // eased, so starts and stops are not instant
    for (let k = 0; k < 3; k++) input[k] = input[k]! * 0.8 + (speed > 0 ? (wanted[k]! / speed) * 6 : 0) * 0.2;

    KCC.updateGroundVelocity(world, character, policy);
    const { up, ground } = character;
    const v = character.linearVelocity;
    const along = v[0] * up[0] + v[1] * up[1] + v[2] * up[2];
    let base: Vec3 = [up[0] * along, up[1] * along, up[2] * along];
    if (ground.state === GroundState.ON_GROUND && along - ground.velocity[1] < 0.1) {
        base = [...ground.velocity];
        if (keys.has('Space')) base[1] += 9;
    }
    character.linearVelocity = [
        base[0] + gravity[0] * deltaTime + input[0],
        base[1] + gravity[1] * deltaTime + input[1],
        base[2] + gravity[2] * deltaTime + input[2],
    ];
}

loop(
    (deltaTime) => {
        steps++;
        // the platform sways across, carrying the character
        const time = steps * deltaTime;
        RigidBody.moveKinematic(world, platform, [-8 + Math.sin(time * 0.6) * 3, 1, 6], [0, 0, 0, 1], deltaTime);
        steer(deltaTime);
        vec3.copy(characterBefore, characterNow);
        KCC.update(world, character, deltaTime, gravity, updateSettings, policy, filter);
        vec3.copy(characterNow, character.position);
        World.step(world, undefined, deltaTime);
        readStepPoses(poses);
    },
    (alpha) => {
        for (let slot = 0; slot < posed.length; slot++) {
            const mesh = posed[slot]!;
            blendStepPose(poses, slot, alpha, mesh.position, mesh.quaternion);
        }
        vec3.lerp(drawn, characterBefore, characterNow, alpha);
        // the camera follows where the character is drawn, keeping the turn and zoom it was given
        for (let k = 0; k < 3; k++) {
            view.camera.position[k]! += drawn[k]! - followed[k]!;
            view.controls.target[k]! += drawn[k]! - followed[k]!;
        }
        vec3.copy(followed, drawn);
        characterMesh.position.splice(0, 3, drawn[0], drawn[1] + 0.9, drawn[2]);
        view.render();
    },
);
