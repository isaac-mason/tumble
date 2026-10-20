import * as Low from './generated/tumble';
import type { Vec3 } from './math';

export type World = Low.World;

/** what a world is made with: plain data, kept by the caller, read once by World.create */
export type WorldSettings = {
    gravity: Vec3;
    /** each object layer's broadphase layer, by object layer */
    objectLayers: number[];
    broadphaseLayers: number;
    /** the pairs of object layers that collide */
    collisions: [number, number][];
};

export const WorldSettings = {
    /** the defaults, and no layers */
    create(): WorldSettings {
        return { gravity: [0, -9.81, 0], objectLayers: [], broadphaseLayers: 0, collisions: [] };
    },
    /** a new broadphase layer: a tree of its own, for the object layers in it */
    addBroadphaseLayer(settings: WorldSettings): number {
        return settings.broadphaseLayers++;
    },
    /** a new object layer, in the broadphase layer */
    addObjectLayer(settings: WorldSettings, broadphaseLayer: number): number {
        return settings.objectLayers.push(broadphaseLayer) - 1;
    },
    enableCollision(settings: WorldSettings, objectLayerA: number, objectLayerB: number): void {
        settings.collisions.push([objectLayerA, objectLayerB]);
    },
};

export const World = {
    /** a world over the settings, which it copies */
    create(settings: WorldSettings): World {
        Low.errorClear();
        const native = Low.WorldSettings.create();
        for (let i = 0; i < settings.broadphaseLayers; i++) Low.WorldSettings.addBroadphaseLayer(native);
        for (const broadphaseLayer of settings.objectLayers) Low.WorldSettings.addObjectLayer(native, broadphaseLayer);
        for (const [a, b] of settings.collisions) Low.WorldSettings.enableCollision(native, a, b);
        Low.WorldSettings.writeGravity(native, settings.gravity);
        const world = Low.World.create(native);
        Low.WorldSettings.destroy(native);
        return world;
    },
    /** frees the world and every body and constraint in it */
    destroy(world: World): void {
        Low.World.destroy(world);
    },
    /** one step of delta_time seconds */
    step(world: World, deltaTime: number): void {
        Low.World.step(world, deltaTime);
    },
};
