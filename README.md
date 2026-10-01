# FF A350 AutoTaxi

C++17 X-Plane plugin, initially configured for X-Plane 11 and FlightFactor A350 v1.6.16.
Open this directory in CLion. The local SDK default is `D:/XPLANESDK`.

This repository contains the plugin source, tests, and the small compatibility
geometry component needed by the BetterPushback integration. It does **not**
contain X-Plane, the FlightFactor aircraft, BetterPushback binaries, X-Plane SDK
headers, commercial scenery, DSF files, apt.dat files, textures, or extracted
coordinates from private scenery. The plugin reads those resources from the
user's local installation at runtime.

## Project Status

The project is an experimental community plugin for X-Plane 11. It has been
tested with synthetic kinematics, a fake X-Plane SDK, and local ZSPD regression
fixtures. Live FlightFactor A350 physics, aircraft loading states, scenery
clearance, wingtip clearance, traffic, terrain, and ATC behavior still require
careful user validation. It is not an autopilot, takeoff assistant, or ATC
clearance system. Always monitor the aircraft and keep the **Manual control**
command available.

The implementation is intentionally conservative about control ownership. It
does not take over an already-owned throttle or steering override except for
the documented FlightFactor A350 compatibility path, and it releases controls
when stopped, completed, disabled, or on a watchdog fault.

## Requirements

- Windows 10/11 for the supplied X-Plane 11 plugin target.
- X-Plane 11 with the XPLM SDK headers available locally.
- FlightFactor A350 v1.6.16 for the aircraft adapter and live testing.
- A 64-bit MinGW or MSVC C++17 toolchain and CMake 3.20 or newer.
- BetterPushback 1.10 only when automatic reverse departures are used.
- Optional: XPTools `DSFTool.exe` for custom DSF centerline inspection/loading.

The core parser, route planner, controller, UI layout, and fake-SDK tests can
be built without an installed simulator or aircraft. The plugin target requires
the SDK headers and X-Plane's Windows linker environment described below.

## Build And Install

Use a 64-bit MinGW or MSVC toolchain. CLion CMake options:

```text
-DXPLANE_SDK=D:/XPLANESDK -DCMAKE_BUILD_TYPE=Release
```

Build target `A350AutoTaxi`. The output is `<build>/A350AutoTaxi/64/win.xpl`,
with `<build>/A350AutoTaxi/A350AutoTaxi.ini` beside the `64` folder.
Copy the **whole A350AutoTaxi folder** into:

```text
D:/X Plane 11.55-g/Resources/plugins/A350AutoTaxi/
    A350AutoTaxi.ini
    64/win.xpl
```

The plugin gets the running simulator's root from `XPLMGetSystemPath`; there is no
hardcoded simulator install path. Automatic pushback updates BetterPushback's
route cache under `Output/caches`, with an original-cache backup.
The Windows build statically links the MinGW runtime.

Run `ctest --test-dir <build> --output-on-failure` for core, panel layout and fake-SDK control checks.
`AUTOTAXI_BUILD_PLUGIN=OFF` allows core tests without any SDK.
If the local A_ZSPD apt.dat exists, CMake also enables stand 539, 590 and S1-122 regressions:
eight network-supported runway directions, complete kinematic taxi and runway alignment.
Override `AUTOTAXI_ZSPD_APT` with the path to that scenery, or an empty value to omit this test.

## In X-Plane

<img width="1080" height="720" alt="panel-1080" src="https://github.com/user-attachments/assets/e2155667-964e-456c-ac81-2904a6abfeb3" />
<img width="2815" height="1741" alt="image" src="https://github.com/user-attachments/assets/24d4d18d-f82b-4aaa-b67f-a406e355790d" />


1. Load the A350, prepare engines/hydraulics for taxi and remove chocks.
2. Open **Plugins > FF A350 AutoTaxi > Open AutoTaxi**. Initial airport indexing runs
   in the background. The resizable window displays the departure stand or nearest node,
   a north-up airport map and live ground speed, wheel angle, remaining distance and track error.
   On Windows, panel text and tooltips use the installed Consolas font at 14 px. The plugin
   rasterizes it once and uses the same fixed character advances for layout and rendering.
   Missing fonts or unsupported characters fall back to X-Plane's built-in font; the native
   window title bar remains under X-Plane's control. No font file is bundled with the plugin.
3. Select **Runways**, **Stands** or **Nodes**. Search by name and scroll the destination list.
   Selecting a destination previews its route. Runways show availability or an entry/network error.
   Choose **Local** or **Airport** in the map header. Reverse departures default to a local
   view containing the entire pushback path and handoff node; Airport shows the full network.
   A manual mode selection persists through previews and starting. Drag the map to pan,
   use the wheel or +/- icons to zoom, and the fit icon to reset the selected view.
4. Select a taxi speed. **Runway clearance** permits active runway zones and lineup
   on the selected runway; it represents clearance you have obtained yourself.
5. **Preview** displays the route, runway crossings and remaining runway distance.
   The map and status can preview runway routes before clearance is checked, but starting requires it.
   Preview node IDs are written to X-Plane's `Log.txt`.
6. For forward taxi, center the nosewheel, release toe-brake pedals, keep the parking brake set
   and click **Start taxi**. Start below 2 kt. After checks succeed, the adapter acquires controls,
   sets thrust idle and applies service brakes before releasing the parking brake.
   Starting recomputes the route using the current position. If a forward apron exit is feasible,
   taxi starts from the current position on a paved, curved connector.
7. If the stand exit requires reversing, **Begin departure** uses installed **BetterPushback 1.10**
   to connect a tug, import the automatic reverse route, push to a real taxi node and disconnect.
   Start with parking brake set and throttle levers at idle; wait until stationary.
   Keep chocks removed, taxi/landing lights off, and
   doors/GPU/ASU prepared as BetterPushback requires. The plugin sets/releases the parking brake
   during connection, pushing and disconnection. AutoTaxi acquires no steering/throttle/toe-brake
   overrides while towing. Purple is the reverse path; green is the following taxi route.
   Once BP finishes, the actual position, heading and nosewheel must settle; the forward route is
   recomputed from that position with its first edge constrained to the planned node and heading.
   At taxi handoff the parking brake remains held until the adapter acquires controls and applies
   service brakes; it then releases the parking brake without a separate rolling wait tick.
   Brake/control interlocks still apply. An unsuccessful taxi handoff holds the parking brake.
   Set `automatic_pushback=0` to retain the external/manual pushback waiting workflow.
8. **Stop** closes thrust and brakes to a halt. **Manual control** immediately
   relinquishes overrides. Arrival holds the parking brake and returns control.
   Both controls cancel an armed departure. During automatic towing they request BP to stop;
   finish tug disconnection in BetterPushback after cancellation. Taxi will not start afterward.

Bind `autotaxi/a350/stop` and `autotaxi/a350/disconnect` to keys or joystick buttons
in X-Plane's keyboard/joystick settings. Closing the window leaves taxi running.
Only one AutoTaxi instance or other steering/throttle override plugin should run.
Pause freezes controller and feedback timers. Test at normal simulation speed.

## BetterPushback Integration

The installed 1.10 manual and pinned source expose commands and state datarefs,
but no public coordinate-route submission API. This implementation uses the stock
`BetterPushback_routes.dat` format instead of replacing the BetterPushback binary.
It verifies `version|1.10` in the installed updater metadata and requires an enabled,
idle, non-slave BP instance. Other/missing version metadata is rejected.

Before submission it disables/re-enables **only idle BetterPushback** to clear an
earlier in-memory plan, rebinds its SDK references, preserves unrelated cached routes,
and imports the new route through BP's late planner. BP's planner/camera may appear
briefly while importing. A live BP operation is never reinitialized.
Original cache backup: `Output/caches/BetterPushback_routes.dat.autotaxi.bak`.
Unknown or malformed cache formats are rejected before replacement.

The workflow uses `connect_first`, `start`, `stop_planner`, and repeated `disconnect`
commands. `bp/plan_complete` means the plan was accepted; it does not mean the tow
finished. `bp/op_complete` precedes final wheel/heading correction. Taxi handoff requires
position within 6 m of the planned CG endpoint, heading within 3 degrees, physical
nosewheel below 5 degrees, a stationary interval, and **both** `bp/started` and
`bp/connected` cleared. Aircraft-owned wheel/toe overrides do not block BP start:
BP manages steering during towing, and these flags are not pilot tiller inputs.
Forward taxi checks override ownership with the FF A350 exception described below.
BP connection/tow/disconnection watchdogs cancel the departure on failure.

Reverse geometry derives from BP v1.10's own straight/arc/oblique segment constructor,
with conservative steering limits, continuous paved centerline checks and a forward
exit test at each candidate node. If a direct reverse connection fails, the planner
tries continuous reverse legs through intermediate apron positions/headings, retaining
the turning-radius and pavement checks and a 450 m total tow limit. Candidate handoff
nodes must have a usable E/F taxi edge; disconnected stand markers are excluded.
CG-to-main-axle offset is configured in the INI;
incorrect aircraft geometry may prevent endpoint validation. Winch-style tugs that
ask for brake release before `bp/connected` need pilot assistance during connection;
the adapter will time out rather than guess when to release the brake.

Source: [BetterPushbackMod v1.10](https://github.com/olivierbutler/BetterPusbackMod),
commit `c360cd43f38beaddfd73cf817443f8229656d4dc`. Vendored geometry and its CDDL license
are in `vendor/bp-geometry`; the build copies them into the plugin output's `licenses`
folder. Keep these source files and notices when distributing the compiled plugin.
No installed BP, aircraft or scenery binaries are modified by building this project.

For UI inspection without launching X-Plane, build `panel_preview` and run:

```text
panel_preview.exe "D:/X Plane 11.55-g/Custom Scenery/A_ZSPD/Earth nav data/apt.dat" "ui-preview" 72.3
```

This exports the real panel drawing commands as SVG at three window sizes. It is a
static layout/map preview; tug/aircraft behavior uses the fake-SDK and kinematic tests.
Append a stand name after the heading to preview a specific ramp, for example:

```text
panel_preview.exe "D:/X Plane 11.55-g/Custom Scenery/A_ZSPD/Earth nav data/apt.dat" "stand590-preview" 72.41 590
```

## Nosewheel And FlightFactor

Default **direct** mode writes **degrees, positive right** to
`sim/flightmodel2/gear/tire_steer_command_deg[0]`, with
`sim/operation/override/override_wheel_steer=1`. It writes only gear 0,
not the main gear, and never injects rudder/yaw inputs. The v1.6.16 ACF has
the nosewheel at gear 0 and approximately 28.35 m between nose and main gear.
The controller uses the main-axle position with an initial 2.2 m aft-of-CG offset;
loading/CG changes may require adjustment in the INI.

Steering has angle/speed limits and a 16 degrees/second slew limit. The enacted
angle comes from `sim/flightmodel2/gear/tire_steer_actual_deg[0]` after physics.
A sustained discrepancy above 12 degrees stops taxi; reading the command back
would not reveal FlightFactor overwriting it. Override ownership is also checked.
When an enabled plugin with signature `1-sim A350` and an A35 ICAO aircraft are loaded,
existing wheel/toe overrides are permitted for cooperative control. The adapter writes
configured steering/brake inputs and checks actual nosewheel response. It sets and clears
only override flags that were initially free; existing flags are preserved on release.
Other aircraft still reject an already-active wheel/toe override. X-Plane's boolean
override flags do not identify which plugin owns them.
Throttle uses `ENGN_thro_use` with `override_throttles` when the override is free.
If already active, it automatically uses `sim/cockpit2/engine/actuators/throttle_ratio`
as lever input, leaving the aircraft-owned override and final engine output untouched.
The selected input is logged on engagement. This fallback still needs validation
with FF; if FF ignores the standard lever input, configure its verified private input.
Toe brakes use
`override_toe_brakes`. FlightFactor may still compute these controls itself.
For the detected FF A350, the writable cockpit parking-brake input `1-sim/parckBrake`
is also used when available: 0 holds the brake and 1 releases it. This inverted mapping
comes from the v1.6.16 cockpit manipulator and checklist. BP connection/disconnection,
tow release, taxi interlocks and completion use this input alongside standard brake refs.
Failed start checks retain the planned map so the route remains visible.

**Compiled and tested with synthetic kinematics and a fake SDK; not yet verified
inside the running FF A350.** The SDK's writable flag cannot prove that FF accepts
the input or that the visible wheel animation agrees with the physical wheel.
First test on an empty apron at low speed, observe both wheel direction and aircraft
turning, and use Manual control if the aircraft behaves incorrectly.

If FF rejects direct control, use DataRefTool to identify the **actual writable
tiller input for your aircraft version**. The project intentionally does not invent
a private FF dataref name. Edit the installed INI and click **Reload config**:

```ini
steering_mode=custom
steering_dataref=<verified FF tiller input>
steering_index=-1
steering_scale=0.015384615
steering_sign=1
```

`index=-1` means a scalar; an array uses its real index. `scale=1/65` maps a
65-degree command to a normalized input of 1. Set sign to -1 only if necessary.
Keep feedback on an independent, physical wheel-angle dataref in degrees.
The private FF throttle/brake inputs can also be configured; standard throttle/toe
overrides are used only with the corresponding standard datarefs.

## Airport Data And Limits

- Active `scenery_packs.ini` order wins per airport ID. Disabled/unlisted scenery
  is excluded. X-Plane 11 default apt.dat and the X-Plane 12 global-airport location
  are supported. The XP12 `*GLOBAL_AIRPORTS*` entry keeps its exact position in the list;
  disabled or unlisted global-airport packs are not reintroduced by a fallback when the
  list exists. UTF-8 BOMs, absolute package paths and paths containing spaces are supported.
  Lower-priority stand names within 25 m are available as aliases of active
  scenery stands; lower-priority taxi networks are not merged.
- Reads land runways (100), taxi nodes (1201), directional taxi edges (1202),
  active-runway zones (1204), ramps (1300), ramp width (1301), paved areas (110/111-114),
  and ground marking chains (120/111-116). Bezier handles are sampled for both painted lines
  and pavement boundaries, including polygon holes.
- Route search retains the incoming direction at each edge, rejects turns above 110 degrees,
  respects one-way edges and rejects known classes below E. It can find a longer route when
  a short route has an incompatible turn. Other runway crossings require explicit clearance
  and are penalized during search; the chosen crossings appear before starting.
  Older edges with unknown width are accepted by default; set
  `allow_unknown_width=0` to require explicit E/F width.
- Taxi speed can be selected from 2 to **20 kt**. The controller still reduces speed for
  bends, runway alignment and braking distance.
- The usual graph join limit is 35 m. Gate/apron connectors extend to 180 m when the entire
  connector remains on a paved surface. Forward departure curves match aircraft heading
  and the taxi-edge direction, with a minimum 20 m geometric turning radius (increased
  when the configured wheelbase/steering limit requires it). The planner may join farther
  along the edge to make room for the turn. Feasible forward routes are preferred over a tow.
  Pavement checks concern the route centerline, not the aircraft's full footprint.
- Runway entries use either explicit ATC associations or a connected painted lineup curve and must be
  near its departure end (first 700 m or first 30 percent). The final leg extends
  along the centerline, reducing speed and requiring heading error <=2 degrees
  and lateral error <=2 m before completion. It does not start a takeoff roll.
  `allow_intersection_departure=1` permits later entries; the UI shows the reduced remaining distance.
- Missing networks, disconnected routes and tight reversals report an error.
  There is no fabricated straight-line taxi route across arbitrary airport areas.
- ATC-only fallback links are a centerline approximation, not a surveyed aircraft path.
  Tight bends may be cut by the pursuit controller; the cross-track watchdog
  stops it if the configured limit is exceeded. This version does not model
  wingtip clearance, terrain/pavement boundaries, parked aircraft, traffic or ATC.
- Node display means the **nearest** node; on an edge it shows that node and distance.
  Automatic idle airport refresh occurs after moving 500 m. Detect airport forces a refresh;
  Reload config also rebuilds the scenery index.
- Disable/unload returns controls immediately. It cannot continue braking after unload.

## Painted Centerlines

The map shows actual apt.dat ground markings in yellow. Only marking styles **1, 7, 51, 57**
become taxi routes: ordinary/enhanced centerlines and their black-bordered variants.
Holding lines, broken boundary lines, double edge lines and stand outlines remain display data.
Runway geometry follows the row 100 axis.

An ATC runway edge is not required when a continuous painted entry approaches from outside
the runway strip and ends within 2 m of its surveyed axis, with its final tangent within
35 degrees of the axis. The endpoint remains a graph junction. Route search still requires
a connected path, runway clearance, compatible arrival direction, aircraft turn radius and
the departure-end/intersection limits. No missing taxi connection or full runway graph is
generated for these entries; the final alignment leg follows the row 100 axis.

Painted features are split at intersections and sub-metre endpoint joins, including apron
branches omitted from the ATC network. Curved geometry remains in the route and map;
navigation does not reduce it to a straight chord between ATC nodes. Nearby ATC nodes
are associated with the physical line within 40 m. Matched ATC direction, width and active-runway
rules are retained; runway-strip crossings also require runway clearance. Unclassified apron
links retain unknown width, governed by `allow_unknown_width`. Centerline paint alone does
not establish A350 wingtip clearance. Tight painted bends and incompatible junction turns
are excluded from route search.

When no connected painted path is available for an ATC link, that link remains as fallback.
It is drawn as a thin grey dashed line, and route previews report the selected fallback count.
Airport loading logs the ground-marking and fallback counts. Virtual painted junctions have
generated node IDs; these can differ from the original ATC IDs shown by previous builds.

For this installed ZSPD package, apron lines are available directly in apt.dat. With the optional
runtime DSFTool path, supported yellow `.lin` chains from the matching DSF tile are also loaded;
yellow stripes baked solely into textures or orthophotos remain display-only/unavailable.
It does not infer an arbitrary centerline through a broad apron from pavement boundaries.
The reader operates locally on the user's installed scenery; commercial scenery files, extracted
coordinates and textures are not included in the plugin distribution or uploaded.

### DSF Inspection

The local ZSPD DSF was inspected with the official
[XPTools DSFTool](https://developer.x-plane.com/tools/xptools/).
It contains custom painted `.lin` instances as well as textured `.pol` stand labels.
In this package, the single-yellow line resource has 36 instances, including six Bezier
instances. Its geometry needs separate inspection from apt.dat markings; not all custom
DSF paint is already represented by the apt.dat centerlines.

`dsf_audit` is an offline diagnostic built with `BUILD_TESTING=ON`. It consumes an existing
DSFTool text conversion and the local apt.dat, lists instantiated line resources and compares
candidate anchor positions to the apt.dat painted centerlines or surveyed runway axes:

```text
DSFTool.exe --dsf2text "path/to/tile.dsf" "local-dsf.txt"
dsf_audit.exe "path/to/apt.dat" "local-dsf.txt"
```

Anchor comparison alone does not prove full curve equivalence or a safe taxi connection.
The runtime loader resolves the active package tile, interprets `.lin` chains, deduplicates
geometry through the centerline graph, and applies explicit marking classification.
DSF overlays can coexist across active scenery packs; selection must respect loaded overlay
and exclusion rules rather than applying apt.dat's one-source-per-airport rule to DSFs.
Double-yellow boundaries, service roads, wheel marks, stand-number textures and miscellaneous
polygons must not be indiscriminately routed. DSF does not provide aircraft E/F width,
one-way or ATC clearance semantics for arbitrary painted lines. Scenery conversion and
inspection remain local; decoded paid scenery is not part of the plugin package.

Runtime DSF loading is available when the official `DSFTool.exe` is installed. Set
`dsf_tool_path` in `A350AutoTaxi.ini` to its absolute path, or put `DSFTool.exe` on the
X-Plane plugin process `PATH`. During airport detection the plugin converts only the active
airport package's matching DSF tile into a temporary local text file, imports supported
yellow `.lin` chains, keeps single-yellow chains routable and double-yellow chains display-only,
deletes the temporary file, and rebuilds the centerline graph. The log reports
`DSF yellow marking lines loaded`. If DSFTool is unavailable, the plugin
continues with apt.dat and reports `apt.dat only`; it does not silently invent DSF geometry.

For display, DSF line geometry is appended after apt.dat geometry, so the active custom
scenery line is the top visual layer when the two differ. For routing, apt.dat 1202 remains
the source of taxiway width, one-way direction and runway-activity semantics; DSF supplies
the visible centerline shape. The audit currently reports that ZSPD's 36 custom single-yellow
chains differ substantially from the 1202 node chords, so they must not be treated as proof
that every yellow painted line is an aircraft taxiway.

While taxiing, the panel reports the next turn distance and angle, the next Node and ETA,
the next taxiway change and ETA, plus total remaining ETA. ETAs use the selected taxi speed
and update from the controller's measured route progress.

## Local ZSPD Regression

The current active scenery is `Custom Scenery/A_ZSPD/Earth nav data/apt.dat`.
Stand 539 from the global metadata is at approximately `31.135482, 121.802825`,
matching the custom stand `S1-118` within 13 m. The departure display therefore includes
`S1-118 / 539`. It is distinct from taxi-network **Node 539**.

From this position facing north (0 degrees) or roughly parallel to the runways (342 degrees),
the regression completes forward taxi and lineup to **17R, 35L, 16L, 34R, 16R, 34L, 17L, 35R, 15, 33**.
Facing the original nose-in stand heading (about 72 degrees), the automatic reverse planner
joins the painted network. Each planned endpoint then completes forward taxi and runway alignment
in the kinematic regression.
Runways **15/33** have real painted entry curves in this apt.dat even though their ATC runway
edges are missing. Both directions are now reachable through those curves.
These are route/kinematic results, not a validation of FlightFactor's live physics or dataref behavior.

Stand **590** is a separate E-width ramp. Its nearest original ATC stop marker is disconnected.
Using the painted apron network reduces the tow from about 256-334 m to about 127 m.
All ten runway directions complete forward taxi and lineup in the regression.
The screenshot's **S1-122** is a different stand and is tested separately.
These checks do not establish full-aircraft clearance or live FF/BP behavior.

Official references: [apt.dat format](https://developer.x-plane.com/article/airport-data-apt-dat-12-00-file-format-specification/),
[X-Plane datarefs](https://developer.x-plane.com/datarefs/).

## Repository And Licensing

Project source is released under the [MIT License](LICENSE). Contributions are
covered by the same license unless a pull request states otherwise. The vendored
BetterPushback geometry compatibility code is separately licensed under the CDDL;
its notice is in `vendor/bp-geometry/COPYING` and must remain with that code.
The full BetterPushback source snapshots under `vendor/` are retained for
reference and carry their original notices. X-Plane, FlightFactor, BetterPushback
and scenery copyrights are not transferred by this repository.

Do not commit simulator logs containing personal paths, private aircraft data,
commercial scenery, DSF conversions, or BetterPushback route caches. Use a
small synthetic test fixture when a parser or planner issue needs a reproducible
input. See [CONTRIBUTING.md](CONTRIBUTING.md) for issue and pull request rules.

## Known Limitations

- The route centerline is not a guarantee of full A350 wingtip clearance.
- DSF paint has no inherent taxiway width, one-way, or runway-clearance
  semantics; apt.dat remains authoritative for those routing rules.
- FlightFactor can change private datarefs between aircraft updates. The
  configured custom steering/throttle datarefs must be verified locally.
- BetterPushback's public API does not expose general coordinate-route
  submission, so the integration uses its documented route-cache workflow and
  refuses to interfere with an active tow.
- This release targets X-Plane 11 on Windows. Linux/macOS builds need platform
  adapter and packaging work before they can be considered supported.
