import type * as gpucat from 'gpucat';
import { Box, Capsule, Cylinder, MotionType, Poses, RigidBody, type Shape, Sphere, World, WorldSettings } from 'tumble.js';
import { caption, loop, meshOf, type ShapeMesh, setup } from './common';

const view = await setup([18, 14, 22], [0, 2, 0]);
caption('a pile of shapes, each mesh posed from one Poses.read a frame. click to drop more');

const settings = WorldSettings.create();
const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
const notMoving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
WorldSettings.enableCollision(settings, moving, moving);
WorldSettings.enableCollision(settings, moving, notMoving);
const world = World.create(settings);

const groundSize: ShapeMesh = { kind: 'box', halfExtents: [15, 0.5, 15] };
const ground = Box.create({ halfExtents: groundSize.halfExtents });
RigidBody.create(world, { shape: ground, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const groundMesh = meshOf(groundSize, '#5a5f66');
groundMesh.position[1] = -0.5;
view.scene.add(groundMesh);

// each kind's shape, made once and shared by every body of it
const kinds: { shape: Shape; mesh: ShapeMesh; color: string }[] = [
    {
        mesh: { kind: 'box', halfExtents: [0.5, 0.5, 0.5] },
        color: '#4ecdc4',
        shape: Box.create({ halfExtents: [0.5, 0.5, 0.5] }),
    },
    { mesh: { kind: 'sphere', radius: 0.5 }, color: '#ff6b6b', shape: Sphere.create({ radius: 0.5 }) },
    {
        mesh: { kind: 'capsule', halfHeightOfCylinder: 0.5, radius: 0.35 },
        color: '#dfe6e9',
        shape: Capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.35 }),
    },
    {
        mesh: { kind: 'cylinder', halfHeight: 0.4, radius: 0.5 },
        color: '#74b9ff',
        shape: Cylinder.create({ halfHeight: 0.4, radius: 0.5 }),
    },
    { mesh: { kind: 'box', halfExtents: [1, 0.2, 0.4] }, color: '#f39c12', shape: Box.create({ halfExtents: [1, 0.2, 0.4] }) },
];

const poses = Poses.create(world);
// a body's mesh by its slot in the poses
const meshes: gpucat.Object3D[] = [];
let seed = 1;
const random = () => {
    seed = (seed * 1664525 + 1013904223) >>> 0;
    return seed / 4294967296;
};

function drop(count: number): void {
    for (let i = 0; i < count; i++) {
        const kind = kinds[Math.floor(random() * kinds.length)]!;
        const axis = [random() - 0.5, random() - 0.5, random() - 0.5];
        const length = Math.hypot(axis[0]!, axis[1]!, axis[2]!);
        const angle = random() * Math.PI;
        const s = Math.sin(angle / 2) / length;
        const body = RigidBody.create(world, {
            shape: kind.shape,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: [(random() - 0.5) * 8, 8 + random() * 10, (random() - 0.5) * 8],
            quaternion: [axis[0]! * s, axis[1]! * s, axis[2]! * s, Math.cos(angle / 2)],
            friction: 0.5,
        });
        const mesh = meshOf(kind.mesh, kind.color);
        meshes[Poses.add(poses, body)] = mesh;
        view.scene.add(mesh);
    }
}
drop(150);
window.addEventListener('click', () => drop(20));

loop(
    (deltaTime) => World.step(world, undefined, deltaTime),
    () => {
        // only the slots whose bodies moved since the last read
        const { positions, quaternions, changed } = Poses.read(poses);
        for (const slot of changed) {
            const mesh = meshes[slot]!;
            for (let k = 0; k < 3; k++) mesh.position[k] = positions[slot * 3 + k]!;
            for (let k = 0; k < 4; k++) mesh.quaternion[k] = quaternions[slot * 4 + k]!;
        }
        view.render();
    },
);
