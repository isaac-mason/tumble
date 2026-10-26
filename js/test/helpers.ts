import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    destroyWorld,
    enableCollision,
    init,
    MotionType,
    releaseShape,
    rigidBody,
    type Shape,
    updateWorld,
    type World,
    type WorldSettings,
} from '../src';

export type Scene = { world: World; moving: number; notMoving: number; floor: Shape };

/** a world with a moving and a not-moving layer and a static floor, its top at y = 0 */
export async function makeScene(tune?: (settings: WorldSettings) => void): Promise<Scene> {
    await init();
    const settings = createWorldSettings();
    const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
    const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
    enableCollision(settings, moving, moving);
    enableCollision(settings, moving, notMoving);
    tune?.(settings);
    const world = createWorld(settings);
    const floor = box.create({ halfExtents: [20, 0.5, 20] });
    rigidBody.create(world, { shape: floor, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
    return { world, moving, notMoving, floor };
}

export function releaseScene(scene: Scene): void {
    destroyWorld(scene.world);
    releaseShape(scene.floor);
}

export function steps(world: World, count: number): void {
    for (let i = 0; i < count; i++) updateWorld(world, undefined, 1 / 60);
}
