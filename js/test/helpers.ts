import { Box, init, MotionType, RigidBody, type Shape, World, WorldSettings } from '../src';

export type Scene = { world: World; moving: number; notMoving: number; floor: Shape };

/** a world with a moving and a not-moving layer and a static floor, its top at y = 0 */
export async function makeScene(tune?: (settings: WorldSettings) => void): Promise<Scene> {
    await init();
    const settings = WorldSettings.create();
    const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
    const notMoving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
    WorldSettings.enableCollision(settings, moving, moving);
    WorldSettings.enableCollision(settings, moving, notMoving);
    tune?.(settings);
    const world = World.create(settings);
    const floor = Box.create({ halfExtents: [20, 0.5, 20] });
    RigidBody.create(world, { shape: floor, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
    return { world, moving, notMoving, floor };
}

export function releaseScene(scene: Scene): void {
    World.destroy(scene.world);
    Box.destroy(scene.floor);
}

export function steps(world: World, count: number): void {
    for (let i = 0; i < count; i++) World.step(world, undefined, 1 / 60);
}
