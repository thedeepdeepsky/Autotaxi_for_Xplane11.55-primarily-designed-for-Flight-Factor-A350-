# Contributing

Thanks for helping improve FF A350 AutoTaxi.

## Before opening an issue

- Reproduce the problem with the simulator paused or at normal simulation speed.
- Include X-Plane version, aircraft version, plugin version or commit, and the
  relevant `Log.txt` lines.
- Do not upload commercial scenery, aircraft files, DSF/apt.dat extracts,
  BetterPushback caches, or private DataRefTool dumps. Describe the location
  and provide a small synthetic fixture when possible.
- Remove personal paths, usernames, and unrelated plugin information from logs.

## Building and testing

Build with CMake and a 64-bit compiler using the instructions in `README.md`.
Run:

```text
ctest --test-dir build-release --output-on-failure -j 1
```

The ZSPD tests are optional and use scenery installed on the developer's own
machine. They must not add scenery data to the repository.

## Code changes

- Keep routing and control behavior deterministic in the core tests.
- Preserve the FlightFactor compatibility checks and control-release behavior.
- Add or update a focused test for planner, controller, parser, or UI-layout
  changes.
- Keep generated binaries and local simulator paths out of commits.
- Use C++17 and the existing formatting style.

Pull requests should explain the user-visible behavior, the test command used,
and any limitation that still requires validation inside X-Plane.

## Licensing

Project code is MIT licensed. The vendored BetterPushback geometry compatibility
code has its own CDDL notice in `vendor/bp-geometry/COPYING`; retain that notice
when redistributing it. X-Plane SDK headers, FlightFactor A350, BetterPushback,
and third-party scenery remain governed by their respective licenses.
