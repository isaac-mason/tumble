import type { BodyId } from './body.js';
import * as Low from './generated/tumble.js';
import { copyVec3, type Vec3 } from './math.js';
import { defaultsOf, type Given, over } from './settings.js';

/** how a character treats a body it meets */
export type KCCBodyPolicy = {
    /** the character passes through it */
    ignore: boolean;
    /** false: it blocks the character but does not move it */
    canPushCharacter: boolean;
    /** false: the character does not push it */
    canReceiveImpulses: boolean;
    /** false: no sliding along a still, walkable surface of it (jolt's slide prevention) */
    slideOnWalkable: boolean;
    /** added to the body's velocities where the character meets it, as a conveyor's belt moves */
    surfaceLinearVelocity: Vec3;
    surfaceAngularVelocity: Vec3;
};

/** how characters treat the bodies they meet: a policy per body, the defaults for the rest; free with kccPolicy.destroy */
export type KCCPolicy = Low.Policy;

const NO_POLICY = 0 as unknown as Low.Policy;
const bodyPolicyDefaults = defaultsOf(Low.KccPolicy.defaultBodyPolicy, Low.KCCBodyPolicy.create);
const readPolicy = Low.KCCBodyPolicy.create();

export function policyOf(policy: KCCPolicy | undefined): Low.Policy {
    return policy ?? NO_POLICY;
}

/** every body at the defaults */
export function create(): KCCPolicy {
    Low.errorClear();
    return Low.KccPolicy.create();
}

export function destroy(policy: KCCPolicy): void {
    Low.KccPolicy.destroy(policy);
}

/** for every body without its own, the given fields over the BodyPolicy defaults */
export function setDefaults(policy: KCCPolicy, given: Given<KCCBodyPolicy>): void {
    Low.KccPolicy.setDefaults(policy, over(bodyPolicyDefaults(), given));
}

export function getDefaults(out: KCCBodyPolicy, policy: KCCPolicy): KCCBodyPolicy {
    Low.KccPolicy.getDefaults(readPolicy, policy);
    return copyBodyPolicy(out, readPolicy);
}

/** the body's own, the given fields over the BodyPolicy defaults (not the policy's); a removed body's stays until removed */
export function set(policy: KCCPolicy, body: BodyId, given: Given<KCCBodyPolicy>): void {
    Low.KccPolicy.set(policy, body, over(bodyPolicyDefaults(), given));
}

/** back to the defaults; whether it had its own */
export function remove(policy: KCCPolicy, body: BodyId): boolean {
    return Low.KccPolicy.remove(policy, body);
}

/** the body's own, or the defaults */
export function get(out: KCCBodyPolicy, policy: KCCPolicy, body: BodyId): KCCBodyPolicy {
    Low.KccPolicy.get(readPolicy, policy, body);
    return copyBodyPolicy(out, readPolicy);
}

function copyBodyPolicy(out: KCCBodyPolicy, from: Low.KCCBodyPolicy): KCCBodyPolicy {
    out.ignore = from.ignore;
    out.canPushCharacter = from.canPushCharacter;
    out.canReceiveImpulses = from.canReceiveImpulses;
    out.slideOnWalkable = from.slideOnWalkable;
    copyVec3(out.surfaceLinearVelocity, from.surfaceLinearVelocity);
    copyVec3(out.surfaceAngularVelocity, from.surfaceAngularVelocity);
    return out;
}
