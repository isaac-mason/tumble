export { type BodyId, Dof, INVALID_BODY_ID, MaterialCombineMode, MotionQuality, MotionType } from './body.js';
export {
    type ConstraintId,
    ConstraintSpace,
    ConstraintType,
    INVALID_CONSTRAINT_ID,
    type MotorSettings,
    MotorState,
    SixDOFAxis,
    SpringMode,
    type SpringSettings,
    SwingType,
} from './constraints/common.js';
export type { ConeConstraintSettings } from './constraints/cone-constraint.js';
export * as coneConstraint from './constraints/cone-constraint.js';
export * as constraints from './constraints/constraints.js';
export type { DistanceConstraintSettings } from './constraints/distance-constraint.js';
export * as distanceConstraint from './constraints/distance-constraint.js';
export type { FixedConstraintSettings } from './constraints/fixed-constraint.js';
export * as fixedConstraint from './constraints/fixed-constraint.js';
export type { HingeConstraintSettings } from './constraints/hinge-constraint.js';
export * as hingeConstraint from './constraints/hinge-constraint.js';
export type { PointConstraintSettings } from './constraints/point-constraint.js';
export * as pointConstraint from './constraints/point-constraint.js';
export type { SixDOFConstraintSettings, SixDOFValues } from './constraints/six-dof-constraint.js';
export * as sixDOFConstraint from './constraints/six-dof-constraint.js';
export type { SliderConstraintSettings } from './constraints/slider-constraint.js';
export * as sliderConstraint from './constraints/slider-constraint.js';
export type { SwingTwistConstraintSettings } from './constraints/swing-twist-constraint.js';
export * as swingTwistConstraint from './constraints/swing-twist-constraint.js';
export type { ContactPair } from './contact-pairs.js';
export * as contactPairs from './contact-pairs.js';
export type { BodiesOptions, BodyOptions, JointsOptions, ShapeOptions } from './debug.js';
export * as debug from './debug.js';
export * as debugDraw from './debug-draw.js';
export { DEBUG_VERTEX_STRIDE, type DebugDraw, type DebugViews } from './debug-draw.js';
export type { ContactEvent, ContactRemovedEvent, ContactSettings, Listener } from './events.js';
export type { Filter } from './filter.js';
export * as filter from './filter.js';
export { type InitOptions, init } from './init.js';
export type {
    GroundInfo,
    KCC,
    KCCContact,
    KCCContactEvent,
    KCCSettings,
    KCCSolveResult,
    KCCTuning,
    KCCUpdateSettings,
} from './kcc.js';
export * as kcc from './kcc.js';
export type { KCCBodyPolicy, KCCPolicy } from './kcc-policy.js';
export * as kccPolicy from './kcc-policy.js';
export type { Box3, Mat4, Quat, Vec2, Vec3 } from './math.js';
export * from './queries.js';
export type { RigidBodySettings } from './rigid-body.js';
export * as rigidBody from './rigid-body.js';
export type { BoxShapeSettings } from './shapes/box.js';
export * as box from './shapes/box.js';
export type { CapsuleShapeSettings } from './shapes/capsule.js';
export * as capsule from './shapes/capsule.js';
export type { CompoundShapeSettings } from './shapes/compound.js';
export * as compound from './shapes/compound.js';
export type { ConvexHullShapeSettings } from './shapes/convex-hull.js';
export * as convexHull from './shapes/convex-hull.js';
export type { CylinderShapeSettings } from './shapes/cylinder.js';
export * as cylinder from './shapes/cylinder.js';
export * as emptyShape from './shapes/empty-shape.js';
export type { OffsetCenterOfMassShapeSettings } from './shapes/offset-center-of-mass.js';
export * as offsetCenterOfMass from './shapes/offset-center-of-mass.js';
export type { PlaneShapeSettings } from './shapes/plane.js';
export * as plane from './shapes/plane.js';
export type { ScaledShapeSettings } from './shapes/scaled.js';
export * as scaled from './shapes/scaled.js';
export {
    BakeSupportAdjacency,
    BvhSplitStrategy,
    type CompoundChild,
    ConvexHullBuildError,
    ConvexHullBuildResult,
    computeMassProperties,
    getShapeCenterOfMass,
    getShapeLocalBounds,
    getShapeRefCount,
    getShapeType,
    getShapeVolume,
    type MassProperties,
    releaseShape,
    type Shape,
    ShapeType,
} from './shapes/shapes.js';
export type { SphereShapeSettings } from './shapes/sphere.js';
export * as sphere from './shapes/sphere.js';
export type { StaticCompoundShapeSettings } from './shapes/static-compound.js';
export * as staticCompound from './shapes/static-compound.js';
export type { TransformedShapeSettings } from './shapes/transformed.js';
export * as transformed from './shapes/transformed.js';
export type { TriangleMeshShapeSettings } from './shapes/triangle-mesh.js';
export * as triangleMesh from './shapes/triangle-mesh.js';
export {
    type ActiveBody,
    addBroadphaseLayer,
    addObjectLayer,
    type CcdSettings,
    type ContactsSettings,
    createActiveBody,
    createWorld,
    createWorldSettings,
    destroyWorld,
    enableCollision,
    getActiveBodies,
    getActiveBodyAt,
    getGravity,
    getNumActiveBodies,
    type Material,
    type NarrowphaseSettings,
    type SleepingSettings,
    type SolverSettings,
    setGravity,
    setMaterials,
    updateWorld,
    type World,
    type WorldSettings,
} from './world.js';
