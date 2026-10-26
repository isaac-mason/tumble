# tumble.js bench

Physics scenarios from known benchmark suites, measured from JavaScript with
[`@pmndrs/labs`](https://www.npmjs.com/package/@pmndrs/labs). The scenarios match crashcat's bench one for one, with the
same names, sizes and step counts. So the two engines can be compared directly.

## Layout

```
scenarios/   the scenarios, in plain tumble.js
benches/     one file per scenario, wiring it into labs
smoke.ts     one untimed pass over every scenario
```

A scenario has a name, a description, a `steps` count and a `create()`. `create()` returns
`{ world, bodies, reset, step(stepIndex) }`.

`step` runs one world step, plus any work the scenario does each step. `raycast-mesh` casts its rays there, and
`character-terrain` moves its characters there.

`reset` puts the world back as `create()` left it. Every body that is not static returns to its first pose, at rest and
awake. Anything else the scenario holds is rewound too, such as a random number generator.

## Running

The bench imports `tumble.js` from `dist`, so build first. In `js/`:

```
pnpm run build
pnpm run bench                        # every scenario, saved under the current commit
pnpm run bench pyramid                # one scenario
pnpm run bench "convex-pile pyramid"  # several
pnpm run bench --no-save              # a run that is not saved
pnpm run bench:smoke                  # the smoke pass
```

labs has more commands, such as `baseline` and `compare`. Run them from `bench/` with `pnpm exec labs`.

## What one op measures

One op is `reset()` and then `steps` steps. The world is built once, outside the timed region.

The reset matters. labs sizes its sample count from how long the first samples take, and these scenarios settle over
time. Without a reset, a faster engine would run further into the cheap, settled part of a scenario. It would look
faster than it is. With a reset, every op does the same work.

A reset cannot rewind the contact cache or the shape of the broadphase tree. So the first window on a new world differs
a little from the ones after it. Each bench runs two untimed windows first, with `warm()`. The smoke pass checks this.
It runs six windows per scenario and flags any whose awake body count still changes after the first two. That means the
scenario's reset misses some state.

`steps` covers the part of a scenario worth measuring. More steps is not better. `ccd-cascade` stops at 150 steps,
because after that its bodies have landed and there is little left for CCD to do.

World construction is not measured. labs' heap column only covers the JS heap. tumble's memory is in the wasm, so it does
not show there.

## The scenarios

Each scenario comes from a named test in PEEL (Pierre Terdiman's Physics Engine Evaluation Lab) or in Jolt's
`PerformanceTest`. The file header names the test and any change from it.

| scenario | source | loads |
| --- | --- | --- |
| `box-stacks` | PEEL `ManySmallBoxStacks10` | stacking, many small islands |
| `pyramid` | Jolt `PyramidScene` | one deep contact island, plus an impact |
| `convex-pile` | PEEL `PileOfMediumConvexes` | gjk, epa and convex hull support |
| `compound-pile` | PEEL `PileOfSmallCompounds` | compound dispatch and sub shape ids |
| `ten-thousand-boxes` | PEEL `TenThousandsBoxes` | the broadphase and sleeping at 10k bodies |
| `sea-of-static-boxes` | PEEL `SeaOfStaticBoxes` | the cost of stepping a world at rest |
| `convex-vs-mesh` | Jolt `ConvexVsMeshScene` | the mesh bvh and colliding with triangles |
| `raycast-mesh` | PEEL raycast against a static mesh | ray queries against a triangle mesh |
| `ragdoll-pile` | PEEL `PileOfRagdolls_16` | swing-twist constraints and islands |
| `hinge-chain` | PEEL `HingeJointChain` | long chains of constraints |
| `ccd-cascade` | PEEL `CCDTest_DynamicDynamic_ConvexCascade` | the linear cast CCD sweep |
| `character-terrain` | Jolt `CharacterVirtualScene` | character controller casts against a mesh |

## Adding one

1. Write `scenarios/<name>.ts`, exporting a `defineScenario({...})`. Name the test it comes from in the file header, and
   any change from it.
2. Give it a `reset`. `captureBodyState` and `restoreBodyState` cover the bodies. Rewind anything else it holds by hand.
3. Add it to `SCENARIOS` in `scenarios/index.ts`, and add a `benches/<name>.bench.ts`.
4. Run `pnpm run bench:smoke <name>` and check its windows repeat.
