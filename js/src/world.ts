import { withCopies } from './arrays.js';
import { type Listener, stepWith } from './events.js';
import * as Low from './generated/tumble.js';
import type { Vec3 } from './math.js';

export type World = Low.World;

export type ContactsSettings = Low.ContactsSettings;
export type NarrowphaseSettings = Low.NarrowphaseSettings;
export type SolverSettings = Low.SolverSettings;
export type SleepingSettings = Low.SleepingSettings;
export type CcdSettings = Low.CcdSettings;

/** what a world is made with: plain data, yours, read once by World.create */
export type WorldSettings = {
    /** m/s2 */
    gravity: Vec3;
    gravityEnabled: boolean;
    contacts: ContactsSettings;
    narrowphase: NarrowphaseSettings;
    solver: SolverSettings;
    sleeping: SleepingSettings;
    ccd: CcdSettings;
    /** each object layer's broadphase layer, by object layer */
    objectLayers: number[];
    broadphaseLayers: number;
    /** the pairs of object layers that collide */
    collisions: [number, number][];
};

export const WorldSettings = {
    /** the defaults, and no layers */
    create(): WorldSettings {
        const native = Low.WorldSettings.create();
        const settings: WorldSettings = {
            gravity: Low.WorldSettings.readGravity(Low.Vec3.create(), native),
            gravityEnabled: Low.WorldSettings.gravityEnabled(native),
            contacts: Low.WorldSettings.readContacts(Low.ContactsSettings.create(), native),
            narrowphase: Low.WorldSettings.readNarrowphase(Low.NarrowphaseSettings.create(), native),
            solver: Low.WorldSettings.readSolver(Low.SolverSettings.create(), native),
            sleeping: Low.WorldSettings.readSleeping(Low.SleepingSettings.create(), native),
            ccd: Low.WorldSettings.readCcd(Low.CcdSettings.create(), native),
            objectLayers: [],
            broadphaseLayers: 0,
            collisions: [],
        };
        Low.WorldSettings.destroy(native);
        return settings;
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

/** a surface a shape's or a triangle's material id names */
export type Material = { friction: number; restitution: number };

export const World = {
    /** a world over the settings, which it copies */
    create(settings: WorldSettings): World {
        Low.errorClear();
        const native = Low.WorldSettings.create();
        for (let i = 0; i < settings.broadphaseLayers; i++) Low.WorldSettings.addBroadphaseLayer(native);
        for (const broadphaseLayer of settings.objectLayers) Low.WorldSettings.addObjectLayer(native, broadphaseLayer);
        for (const [a, b] of settings.collisions) Low.WorldSettings.enableCollision(native, a, b);
        Low.WorldSettings.writeGravity(native, settings.gravity);
        Low.WorldSettings.setGravityEnabled(native, settings.gravityEnabled);
        Low.WorldSettings.writeContacts(native, settings.contacts);
        Low.WorldSettings.writeNarrowphase(native, settings.narrowphase);
        Low.WorldSettings.writeSolver(native, settings.solver);
        Low.WorldSettings.writeSleeping(native, settings.sleeping);
        Low.WorldSettings.writeCcd(native, settings.ccd);
        try {
            return Low.World.create(native);
        } finally {
            Low.WorldSettings.destroy(native);
        }
    },
    /** frees the world and every body and constraint in it */
    destroy(world: World): void {
        Low.World.destroy(world);
    },
    /** one step of deltaTime seconds; the listener hears its events and answers them as it runs */
    step(world: World, listener: Listener | undefined, deltaTime: number): void {
        stepWith(world, listener, deltaTime);
    },

    /** the surfaces material ids name, replacing the table */
    setMaterials(world: World, materials: Material[]): void {
        const values: number[] = [];
        for (const material of materials) values.push(material.friction, material.restitution);
        Low.errorClear();
        const ok = withCopies((copy) => Low.World.setMaterials(world, copy.f32(values), materials.length));
        if (!ok) Low.getWasm().check();
    },
};
