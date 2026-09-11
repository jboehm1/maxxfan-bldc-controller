# Branching and releases

`main` is the hardware-known-good branch.

`develop` is where compatible improvements are integrated before hardware validation.

Feature work branches from `develop`:
- `feature/dynamic-current`
- future `feature/...`

Experimental architecture work also branches from `develop` but uses `research/...` so it cannot be confused with production-ready firmware:
- `research/sensorless-foc`

When a candidate passes the hardware regression test plan:
1. merge feature branches into `develop`;
2. test `develop` on hardware;
3. merge `develop` into `main`;
4. create a signed/annotated release tag (next proposed: `v0.4.0`);
5. attach the Arduino-ready ZIP to the GitHub Release.
