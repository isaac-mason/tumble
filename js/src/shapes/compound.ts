import * as Low from '../generated/tumble.js';
import type { Quat, Vec3 } from '../math.js';
import { type CompoundChild, type Shape, withChildren } from './shapes.js';

export type CompoundShapeSettings = { children: CompoundChild[] };

/** children placed together; it copies the list and holds the shapes */
export function create(settings: CompoundShapeSettings): Shape {
    return withChildren(settings.children, (address) => Low.Compound.create(address, settings.children.length));
}

/** its children replaced, holding the new and letting go of the old; each body using it takes it again with rigidBody.setShape */
export function update(shape: Shape, settings: CompoundShapeSettings): void {
    withChildren(settings.children, (address) => Low.Compound.setChildren(shape, address, settings.children.length));
}

// a compound's children, as it holds them: the getters read a compound only, a child by an index below getChildCount

const _child = Low.CompoundShapeChild.create();

function childOf(shape: Shape, index: number): Low.CompoundShapeChild {
    return Low.CompoundData.readChildren(_child, Low.Compound.data(shape as number as Low.Compound)!, index);
}

export function getChildCount(shape: Shape): number {
    return Low.CompoundData.childrenLength(Low.Compound.data(shape as number as Low.Compound)!);
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
