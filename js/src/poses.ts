import * as Low from './generated/tumble.js';
import type { BodyId } from './rigid-body.js';
import type { World } from './world.js';

/**
 * what read hands back: views straight onto the wasm's memory, valid until the next call into tumble, so take them from
 * read each time rather than keeping them
 */
export type PoseViews = {
    /** xyz by slot: slot s at s * 3 */
    positions: Float32Array;
    /** xyzw by slot: slot s at s * 4 */
    quaternions: Float32Array;
    /** the slots this read changed */
    changed: Uint32Array;
};

/** the poses of the bodies a renderer draws, each in a slot, refreshed in one call a frame */
export type Poses = {
    handle: Low.Poses;
    views: PoseViews;
    buffer: ArrayBufferLike | null;
    slots: number;
    positionsAddress: number;
    quaternionsAddress: number;
    changedAddress: number;
};

const NO_SLOT = 0xffffffff;
const EMPTY_F32 = new Float32Array(0);
const EMPTY_U32 = new Uint32Array(0);

export const Poses = {
    create(world: World): Poses {
        Low.errorClear();
        return {
            handle: Low.Poses.create(world),
            views: { positions: EMPTY_F32, quaternions: EMPTY_F32, changed: EMPTY_U32 },
            buffer: null,
            slots: 0,
            positionsAddress: 0,
            quaternionsAddress: 0,
            changedAddress: 0,
        };
    },
    destroy(poses: Poses): void {
        Low.Poses.destroy(poses.handle);
    },
    /** a slot for the body, a freed one reused, its pose there from the next read; throws for an id naming no body */
    add(poses: Poses, body: BodyId): number {
        Low.errorClear();
        const slot = Low.Poses.add(poses.handle, body);
        if (slot === NO_SLOT) Low.getWasm().check();
        return slot;
    },
    /** frees the slot for the next add; false for a slot not in use */
    remove(poses: Poses, slot: number): boolean {
        return Low.Poses.remove(poses.handle, slot);
    },
    /** the body in the slot, INVALID_BODY_ID for a free one */
    body(poses: Poses, slot: number): BodyId {
        return Low.Poses.body(poses.handle, slot);
    },
    /**
     * every slot brought to its body's pose, whatever stepped or teleported since the last read; a slot whose body is
     * gone keeps its last pose. changed lists the slots that moved, and every slot added since the last read
     */
    read(poses: Poses): PoseViews {
        const count = Low.Poses.read(poses.handle);
        const handle = poses.handle;
        const buffer = Low.getWasm().mem.memory.buffer;
        const slots = Low.Poses.slots(handle);
        const positionsAddress = Low.Poses.positions(handle);
        const quaternionsAddress = Low.Poses.quaternions(handle);
        const changedAddress = Low.Poses.changed(handle);
        const views = poses.views;
        const moved =
            buffer !== poses.buffer ||
            slots !== poses.slots ||
            positionsAddress !== poses.positionsAddress ||
            quaternionsAddress !== poses.quaternionsAddress;
        if (moved) {
            views.positions = new Float32Array(buffer, positionsAddress, slots * 3);
            views.quaternions = new Float32Array(buffer, quaternionsAddress, slots * 4);
            poses.slots = slots;
            poses.positionsAddress = positionsAddress;
            poses.quaternionsAddress = quaternionsAddress;
        }
        if (moved || changedAddress !== poses.changedAddress || count !== views.changed.length) {
            views.changed = count === 0 ? EMPTY_U32 : new Uint32Array(buffer, changedAddress, count);
            poses.changedAddress = changedAddress;
        }
        poses.buffer = buffer;
        return views;
    },
};
