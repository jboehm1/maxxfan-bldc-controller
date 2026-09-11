# MaxxFan firmware roadmap

## Branch status

### `main`

Hardware-known-good baseline only. Current baseline: `v0.3.7-TEST` reference. Do not merge experimental control changes directly.

### `develop`

Current integration candidate: v0.3.6.3 line with:

- true Hall-independent OPEN mode;
- physical inline current feedback;
- monotonic quintic S-curve;
- continuous OPEN reverse through zero without EN off/on;
- GUI command coalescing / quiet window;
- NVS configuration and diagnostics.

Still requires full hardware regression before promotion to `main`.

### `feature/dynamic-current`

Implemented, software-tested, hardware validation pending.

Purpose: separate the OPEN-mode q-current **request** from the hard current ceiling. Dynamic feed-forward current is based on commanded RPM, with a smooth low-speed floor and startup boost.

Merge gate: reliable start/reverse, requested-vs-measured current sanity, thermal test and no regression in GUI/rotation smoothness.

### `research/sensorless-foc`

Passive observer probe implemented; closed-loop sensorless handover intentionally not implemented yet.

Purpose: validate whether the existing MKS current ADCs + commanded alpha/beta voltage vector can estimate rotor flux angle/RPM sufficiently well without Hall wiring.

Merge gate: this is a research branch, not a normal feature merge. Closed-loop work begins only after passive observer data is stable in both directions over the useful speed range.

## Promotion policy

1. Develop feature/research work on its own branch.
2. Run host/static tests.
3. Flash and execute the branch-specific hardware test plan.
4. Merge successful feature work into `develop`.
5. Run the full regression plan on `develop`.
6. Merge qualified `develop` into `main`.
7. Create a release tag only for a hardware-qualified state.

## Next hardware sessions

### Dynamic current

- 200 / 400 / 600 / 800 / 1000 / 1200 rpm;
- both directions;
- reverse during ramp-up;
- requested versus measured Iq;
- motor and board temperature;
- compare with fixed-current mode.

### Sensorless probe

- run Serial-only, no GUI/Wi-Fi;
- 600 / 800 / 1000 / 1200 rpm;
- compare observer RPM sign/magnitude and continuity;
- repeat reverse direction;
- tune effective R/L/KV only after baseline recordings;
- no sensorless closed-loop handover until passive data passes.
