#include "app/examples.h"

#include <string.h>

namespace examples {

/** every example's name, in the order of crashcat's examples.json; a build may hold any subset */
static const char *const ORDER[] = {
    "Shapes",
    "Triangle Mesh",
    "Raycast Vehicle",
    "Ragdoll",
    "Kinematic Character Controller",
    "KCC Stress Test",
    "Floating Character Controller",
    "Pointer Raycast",
    "Cluster",
    "Cube Heap",
    "Static Compound Shapes",
    "Continuous Collision Detection",
    "Kinematic Body",
    "Constraints",
    "Constraint Motors",
    "Hinge Motor",
    "Voxel Triangle Mesh",
    "Voxel Custom Shape",
    "Restitution",
    "Friction",
    "Conveyor Belt",
    "Linear Damping",
    "Angular Damping",
    "Gravity Factor",
    "Mass Properties",
    "Add Impulse at Position",
    "Cast Ray",
    "Cast Shape",
    "Collide Point",
    "Collide Shape",
    "Sensor",
    "Object Layer Filtering",
    "Collision Filtering",
    "Collide Shape vs Shape",
    "Collide Point vs Shape",
    "Cast Ray vs Shape",
    "Cast Shape vs Shape",
    "GJK Collision Detection",
    "Get Supporting Face",
    "Convex Hull Builder",
    "Test Scenarios",
    "Debug Renderer",
};
static constexpr int ORDER_COUNT = (int)(sizeof(ORDER) / sizeof(ORDER[0]));

struct Registry {
    const Example *examples[ORDER_COUNT];
    int count;
    bool sorted;
};

/** a function's static, made before any file's registration reaches it */
static Registry &registry() {
    static Registry registry = {};
    return registry;
}

static int rank(const Example &example) {
    for (int i = 0; i < ORDER_COUNT; i++) {
        if (strcmp(ORDER[i], example.name) == 0) return i;
    }
    return ORDER_COUNT;
}

Registration::Registration(const Example &example) {
    Registry &r = registry();
    if (r.count < ORDER_COUNT) r.examples[r.count++] = &example;
}

static void sort() {
    Registry &r = registry();
    if (r.sorted) return;
    for (int i = 1; i < r.count; i++) {
        for (int j = i; j > 0 && rank(*r.examples[j]) < rank(*r.examples[j - 1]); j--) {
            const Example *swap = r.examples[j];
            r.examples[j] = r.examples[j - 1];
            r.examples[j - 1] = swap;
        }
    }
    r.sorted = true;
}

int count() {
    sort();
    return registry().count;
}

const Example &at(int index) {
    sort();
    return *registry().examples[index];
}

}
