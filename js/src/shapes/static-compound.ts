import * as Low from '../generated/tumble.js';
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
