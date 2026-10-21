#include "tumble/physics/character/kcc_policy.h"

TUMBLE_NAMESPACE_BEGIN

namespace kcc_policy {

static constexpr BodyPolicy DEFAULTS = {};

void release(Policy &policy) { policy.bodies.release(); }

bool set(Policy &policy, body_id::BodyId body_id, const BodyPolicy &body_policy) { return policy.bodies.set({body_id}, body_policy); }

bool remove(Policy &policy, body_id::BodyId body_id) { return policy.bodies.remove({body_id}); }

const BodyPolicy &get(const Policy *policy, body_id::BodyId body_id) {
    if (policy == nullptr) return DEFAULTS;
    const BodyPolicy *own = policy->bodies.get({body_id});
    return own != nullptr ? *own : policy->defaults;
}

}  // namespace kcc_policy

TUMBLE_NAMESPACE_END
