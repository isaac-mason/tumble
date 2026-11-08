import * as Low from '../generated/tumble.js';
import type { Quat, Vec3 } from '../math.js';
import { type CompoundChild, type Shape, withChildren } from './shapes.js';

export type StaticCompoundShapeSettings = {
    children: CompoundChild[];
    /** children per leaf of the tree over them */
    bvhMaxLeafChildren?: number;
};

/** a compound with a tree over its children, for many children that never change */
export function create(settings: StaticCompoundShapeSettings): Shape {
    const leafChildren = settings.bvhMaxLeafChildren ?? Low.StaticCompound.defaultBvhMaxLeafChildren();
    return withChildren(settings.children, (address) =>
        Low.StaticCompound.create(address, settings.children.length, leafChildren),
    );
}

// a static compound's children, as it holds them: the getters read a static compound only, a child by an index below
// getChildCount

const _child = Low.CompoundShapeChild.create();

function childOf(shape: Shape, index: number): Low.CompoundShapeChild {
    return Low.StaticCompoundData.readChildren(_child, Low.StaticCompound.data(shape as number as Low.StaticCompound)!, index);
}

export function getChildCount(shape: Shape): number {
    return Low.StaticCompoundData.childrenLength(Low.StaticCompound.data(shape as number as Low.StaticCompound)!);
}

/** the child's shape, which the compound holds: not to be released here */
export function getChildShape(shape: Shape, index: number): Shape {
    return childOf(shape, index).shape!;
}

/** in the compound's space */
export function getChildPosition(out: Vec3, shape: Shape, index: number): Vec3 {
    const { position } = childOf(shape, index);
    out[0] = position[0];
    out[1] = position[1];
    out[2] = position[2];
    return out;
}

/** in the compound's space */
export function getChildQuaternion(out: Quat, shape: Shape, index: number): Quat {
    const { quaternion } = childOf(shape, index);
    out[0] = quaternion[0];
    out[1] = quaternion[1];
    out[2] = quaternion[2];
    out[3] = quaternion[3];
    return out;
}
