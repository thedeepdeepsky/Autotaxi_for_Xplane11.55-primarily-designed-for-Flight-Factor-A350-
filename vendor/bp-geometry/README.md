# BetterPushback Geometry Provenance

`Segments.cpp` derives from the geometry-only portion of `src/driving.c` in
[BetterPushbackMod v1.10](https://github.com/olivierbutler/BetterPusbackMod),
commit `c360cd43f38beaddfd73cf817443f8229656d4dc`.
The original CDDL header and Saso Kiselkov copyright are retained.
`COPYING` contains the Common Development and Distribution License 1.0.

Changes: compile as C++17 inside namespace `bpgeometry`; substitute minimal
vector/list helpers from `BpGeometryCompat.h` for libacfutils; remove unrelated
driving-speed declarations. The upstream straight/arc/oblique geometry
construction is retained. The compatibility layer is project code.

The plugin output includes the license and these source files under `licenses`.
Distributions containing the derived geometry must retain its notices, license
and source availability under CDDL. Original reference files are in the project
directory `vendor/BetterPushbackMod-v1.10`; they are not a replacement BP binary.
