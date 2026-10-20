export type Vec3 = [x: number, y: number, z: number];
export type Quat = [x: number, y: number, z: number, w: number];
/** column-major */
export type Mat4 = [
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
];
export type Box3 = { min: Vec3; max: Vec3 };
