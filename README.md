# FF A350 AutoTaxi

C++17 X-Plane plugin, initially configured for X-Plane 11 and FlightFactor A350 v1.6.16.
Open this directory in CLion. The local SDK default is `D:/XPLANESDK`.

**Engine thrust is manual. AutoTaxi never changes throttle levers, engine thrust
datarefs or `override_throttles`. Set a modest taxi thrust yourself; speed control
uses symmetric service brakes.** With insufficient thrust the aircraft cannot
reach the selected speed. Excessive thrust can overpower braking; reduce it yourself.
The supplied INI contains FF A350 geometry, but other aircraft are permitted after
their geometry and steering/brake datarefs have been configured.

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

The implementation checks steering and brake control ownership, with the
documented FlightFactor A350 compatibility path, and releases controls
when stopped, completed, disabled, or on a watchdog fault.

## Requirements

- Windows 10/11 for the supplied X-Plane 11 plugin target.
- X-Plane 11 with the XPLM SDK headers available locally.
- FlightFactor A350 v1.6.16 for the supplied profile; another aircraft needs its own INI settings.
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

After configuring the build directory, the Windows PowerShell build helper
uses the compiler and CMake recorded in that directory's `CMakeCache.txt`:

```powershell
.\tools\Build.ps1 -BuildDirectory build-release -RunTests
```

It prepends the configured compiler's directory to PATH for this build and
restores PATH afterward. This prevents MinGW from loading unrelated
`libwinpthread-1.dll` or `libssp-0.dll` files installed by other applications.
If GCC exits without diagnostics, check this DLL search order before reinstalling
the compiler. For a CLion CMake profile, prepend the same MinGW `bin` directory
to the profile's PATH environment variable while retaining the existing PATH.
The helper requires an already configured build directory; its default is
`build-release` relative to the repository root.

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
ten runway directions, kinematic taxi and runway alignment. Synthetic route-feature
tests also cover painted stand arrivals, runway exits, route policies, emergency holds
and DSF contours with holes.
Override `AUTOTAXI_ZSPD_APT` with the path to that scenery, or an empty value to omit this test.
`zspd_arrival_exit` also checks the local final lead-ins/stops at 590 and S1-122
and a tangent-continuous painted runway exit. It does not replace live flight-model testing.

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
   window title bar remains under X-Plane's control. No UI font file is bundled; the
   separate airport-sign font is included with its license.
3. Select **Runways**, **Stands** or **Nodes**. Search by name and scroll the destination list.
   All apt.dat stands are listed, including smaller stands. Undersized stands display
   their class and are disabled unless **Allow undersized stands** is selected; the planner checks
   stand width. Legacy row 15 startup locations are supported with unknown width.
   Selecting a destination previews its route. Runways show availability or an entry/network error.
   Choose **Local** or **Airport** in the map header. Reverse departures default to a local
   view containing the entire pushback path and handoff node; Airport shows the full network.
   A manual mode selection persists through previews and starting. Drag the map to pan,
   use the wheel or +/- icons to zoom, and the fit icon to reset the selected view.
   Every apt.dat stand is drawn as a small map marker; stand names are revealed after
   zooming in (including Gate names), with the selected stand highlighted even when
   nearby labels overlap.
   Airport sign boards use the bundled Open Taxiway Mandatory Sign font from
   [Open-Runway-Fonts](https://github.com/ryo-a/Open-Runway-Fonts), selected to match
   MH/T 6011-2015 Appendix B airport sign glyph proportions. The UI itself remains
   Consolas. `|` is rendered as the standard vertical panel divider (with its own
   width and spacing), never as `/`; the attribution and license are in
   `licenses/Open-Runway-Fonts-LICENSE.txt`.
4. Select a taxi speed, including while taxiing. The initial ceiling is 20 kt and
   `max_taxi_speed_kt` can change it. **Runway clearance** permits active runway zones and lineup
   on the selected runway; it represents clearance you have obtained yourself.
5. **Preview** displays the route, runway crossings and remaining runway distance.
   The map and status can preview runway routes before clearance is checked, but starting requires it.
   Preview node IDs are written to X-Plane's `Log.txt`.
6. For forward taxi, center the nosewheel, release toe-brake pedals, keep the parking brake set
   and click **Start taxi**. Start below 2 kt. After checks succeed, the adapter acquires controls,
   preserves your thrust setting and applies service brakes before releasing the parking brake.
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
8. **Emergency brake** holds the service brakes and keeps the route and control ownership.
   **Resume taxi** continues the same route at the current selected speed. It does not
   bypass invalid telemetry, airborne or lost-control checks; if braking carries the
   aircraft outside its tracking allowance, normal tracking checks apply on resume.
   ETA is unavailable during a hold because its duration is unknown.
9. **Stop taxi** ends the route and brakes to a halt, preserving manual thrust. **Manual control** immediately
   relinquishes overrides. Arrival holds the parking brake and returns control.
   Both controls cancel an armed departure. During automatic towing they request BP to stop;
   finish tug disconnection in BetterPushback after cancellation. Taxi will not start afterward.

Bind `autotaxi/a350/stop`, `autotaxi/a350/disconnect` and
`autotaxi/a350/emergency_brake` (hold/resume toggle) to keys or joystick buttons
in X-Plane's keyboard/joystick settings. Closing the window leaves taxi running.
Only one AutoTaxi instance or other steering/brake controller should run.
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
nodes must have a usable taxi edge under the configured width policy; disconnected stand markers are excluded.
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
Painted routes use the longitudinal cockpit position as the tracking reference.
The FF v1.6.16 ACF puts the pilot eyepoint about 1.8 m ahead of the nose gear;
with a 28.35 m wheelbase and 2.2 m main-axle aft-of-CG offset, the reference is
approximately 27.95 m ahead of the reported CG. ATC-only routes retain the
legacy main-axle pursuit controller. Loading/CG changes can require adjustment.

Airport curve design uses **cockpit over the taxiway centre line**, rather than
CG or nose gear over the line: see [EASA CS ADR-DSN.D.240 and D.250](https://www.easa.europa.eu/en/document-library/easy-access-rules/online-publications/easy-access-rules-aerodromes-regulation-eu?erules-id=ERULES-1963177438-2192),
which also references ICAO Doc 9157. These are airport design assumptions,
not a replacement for the A350 FCOM/FCTM or an airline's taxi technique.

For a steady circular cockpit path of radius R and cockpit-to-main-axle distance
L, the rear axle radius is `sqrt(R*R - L*L)`. The planner estimates this inner
cut and compares it with known pavement width, an outer main-wheel half-span
of 6.5 m, 4 m wheel-edge margin and 3 m tracking allowance. When necessary it
plans a gradual outward cockpit-path offset (oversteer), checks the compensated
steering radius and approximate swept nose/main wheel envelope, and limits speed to
the configured turn speed. The planner first checks a zero-offset trajectory using transient
aircraft headings, steering limits and swept wheel positions. It adds small local
amounts of outward compensation only around failed samples and checks final
stand/runway heading too. Feasible bends stay on the painted line; the fallback
does not offset every bend just because one junction needs compensation. If this
local search cannot fit a trajectory, the steady-circle clearance model provides
an additional gradual lateral-offset candidate, subject to the same INI limits.
The controller still tracks the **cockpit**; a painted route
does not silently switch to main-axle pursuit. The resulting cockpit path uses
uniform 2 m sampling and continuous distance-based heading estimates, retains the
stand cockpit stop, and checks the wheel envelope using swept aircraft headings.
Automatic painted apron connectors also use the larger radius required by a
cockpit reference. ATC-only geometry retains legacy main-axle control.
The UI reports planned/current oversteer in metres, not a measured tiller angle.
`max_oversteer_m` defaults to 15 m and accepts 0..30 m; this is an allowance, not
a promise of pavement clearance. Steering and pavement limits remain enforced.
`cockpit_ahead_nose_m`, `main_gear_half_span_m`, `wheel_edge_margin_m` and
`max_oversteer_m` are configurable. The painted line itself remains unchanged.

CTE measures the selected tracking reference against the planned guidance path:
the cockpit in cockpit mode, or the main axle in main-axle mode. It is not CG-to-paint
distance. Planned outward compensation is reported separately as oversteer, and
does not itself increase CTE against the compensated path. Lookahead corner cutting,
wheel response delay, rate limits, discretized paint and tyre physics can leave a
small CTE; this controller does not guarantee exactly zero. Cockpit guidance uses
a shorter 6..12 m lookahead to reduce anticipation-induced corner cutting.
The map's unfilled
aircraft symbol shows **COCKPIT**, **AXLE** (main-axle midpoint), and the left/right
main gear centres as hollow rings, joined by an axle, longitudinal centre line and
two outer lines. No filled aircraft triangle or CG point obscures the geometry.
Cockpit-to-axle separation is 30.15 m; main gear track is 10.3632 m in the FF A350
profile (`main_gear_half_track_m=5.1816`, from the ACF's +/-17 ft gear centres).
These points remain visible before route selection. Map and controller use the
same geometry calculation.
All runtime geometry comes from the loaded INI profile. The ACF was only used
to choose shipped FF A350 defaults; it is not read to override user settings.
Change the installed INI and use **Reload settings** while idle to update the
symbol, tracking reference and future routes.
Green shows the expected **cockpit movement**, regardless of the control reference.
When a background forecast is available, green uses its live cockpit trajectory,
refreshed approximately once per second. Display filtering matches old/new points
by route progress rather than sample index. Braking uses a 2.5 s response time,
up to 1.5 m/s of interior-point movement, and bounded spatial smoothing (up to
0.6 m) to reduce teeth and refresh jumps. Normal refreshes respond faster. The
actual cockpit and predicted stop stay anchored, and travelled points are removed.
These filters affect only the map; steering, braking, raw simulation and CTE
references are unchanged. Orange pavement warnings are preserved.
During a transient forecast failure, ETA becomes unavailable while the last
display trajectory is retained for up to five seconds. Continued failures ease
the display toward the geometric planning path. A route change, airport change,
pushback or emergency hold clears the display filter. Without a live forecast,
green uses geometric planning data. In main-axle mode the
green cockpit path moves outside the yellow bend although axle CTE can be small.
Yellow remains scenery paint, and the CTE metric names its control reference.
The panel always shows **COCKPIT CTE** and **MAIN AXLE CTE** together. The active
control reference is highlighted in green and retains the controller's reported
error. The other value compares its actual INI-derived position to its corresponding
fixed planned path near current route progress. In cockpit mode the planned axle
path includes expected main-wheel offtracking; in axle mode the planned cockpit
path includes the outward movement. These diagnostic paths are fixed at planning
time. CTE does not use the periodically refreshed forecast path, whose start is
the actual aircraft position, and is not necessarily distance to the yellow paint.
Forward CTE values are unavailable during towing or without a route.
Large shortcuts caused
by an early straight stand connector or an unpainted runway exit are planning defects.

Stand arrivals with painted geometry terminate on the closest compatible lead-in,
matching the stand's inbound heading. They retain the curve and painted straight,
truncate the last edge at the stop reference, and prefer cockpit guidance. Enabling
oversteer no longer unconditionally selects main-axle guidance for every stand.
The displayed stand endpoint is a **cockpit stopping target**. In main-axle mode
the controller route ends one cockpit-to-axle distance earlier on the aligned
lead-in, so the cockpit still stops at the same displayed target. Braking begins
before reaching it; completion requires a cockpit position within 1 m, heading
within 2 degrees and speed below 0.12 m/s for a second. Physical brake response
and geometry calibration affect precision; an exact zero-error stop is not promised.
The ground paint's stop bars and docking-system indications are not decoded, so
apt.dat stand position/heading remain the stopping metadata, projected onto the lead-in.
This interpretation of the metadata as a cockpit target is a plugin convention;
it does not establish that a scenery's ramp start is its actual docking stop.

Within `apron_approach_distance_m` of a stand (default 180 m), or after entering
a named APRON/RAMP/LEAD-IN/STAND segment in the last 600 m, the target speed is
reduced smoothly toward `apron_speed_kt` (default 3 kt) at the configured planned
deceleration. Once engaged, the arrival cap can only
decrease until taxi is stopped/restarted; completing a bend or increasing the
selected taxi speed does not raise it. Engine thrust remains manual, so the
plugin uses brakes to enforce this target and cannot command acceleration.

Missing pavement boundaries are shown as **Turn pavement clearance unverified**;
the planner does not claim wheel clearance there. Compensation exceeding the
configured allowance or nosewheel capability is rejected. Known, complete pavement
coverage requires the sampled wheel envelope to stay within it, unless the user
explicitly enables the pavement-limit override. Incomplete pavement coverage
reports unverified clearance if the envelope extends beyond the
available pavement polygons. This is not proof of an excursion: custom DSF meshes
may contain additional paved surface absent from apt.dat. A data source explicitly
marked as having complete pavement coverage instead rejects such an envelope.
The wheel-envelope check is a geometric approximation; it does not model tyre
slip, suspension, bogie articulation, wings, engines, traffic or obstacles.
[Airbus Safety First: 180 degree turns on runway](https://safetyfirst.airbus.com/180-turns-on-runway/)
also advises against braking one main gear to a stop for pivot turns on dry
runways on A350/A330/A340/A300/A310. AutoTaxi continues to steer only the
nosewheel and use symmetric braking.

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
Thrust remains entirely manual throughout starting, taxiing, stopping and disconnecting.
An existing throttle override does not block AutoTaxi. Legacy `throttle_dataref` and
`max_throttle` keys are unused by the controller and should be removed from old profiles.
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
Verified aircraft brake inputs can also be configured; the toe-brake override is
used only with the corresponding standard brake datarefs.

## Route Options And ATC Instructions

The **Route** panel exposes three independent policies while taxi is idle:

| Panel Option | INI Key | Behavior |
| --- | --- | --- |
| Allow oversteer | `allow_oversteer` | Permits outward cockpit compensation through bends while retaining cockpit control on painted routes. Off rejects turns needing compensation. |
| Relax pavement clearance | `ignore_pavement_limits` | Allows inadequate known taxiway class or wheel-edge clearance. The route still follows its painted guidance geometry. It does not waive steering limits, runway clearance or one-way rules. |
| Allow undersized stands | `ignore_stand_size` | Allows a known undersized stand and marks its endpoint orange. |

Green is predicted cockpit movement; purple is towing. Orange forward segments
indicate insufficient width, an approximate wheel envelope outside known pavement,
or unverified clearance in incomplete pavement data. An orange stop marker indicates
insufficient/required-but-unknown stand size. Relaxation permits those risks rather
than converting them to a clearance guarantee. Wingtip, traffic and obstacle clearance
are not evaluated. Changes in this panel are session settings; INI supplies the defaults.

Enter ordered taxiway names and/or `#node` IDs in **ORDERED TAXIWAYS / NODES**,
then **Apply**. Example: `A B #123 C`. Names are case-insensitive; spaces, commas,
semicolons and `>` separate tokens. A numeric token becomes a node ID. Up to 32
mandatory visits are supported. **Pick nodes** adds clicks on visible map nodes in
order; numbered markers show the selection, with **Undo** and **Clear** available.
**Route** reopens the editor after picking. Clearing the sequence restores automatic routing.
`route_via` supplies the same sequence from INI.

Taxiway names do not require manually supplied intersection nodes. For example,
`A > B > C` searches the connected graph for an A-to-B-to-C path, choosing reachable
intersections with compatible arrival directions. Painted feature crossings and
endpoint junctions are split into shared graph nodes; multiple intersections are
resolved by the complete route cost and restrictions, not the first name match.

The sequence requires visits in order; it is not a prohibition on every unnamed
connector or intermediate taxiway. Inspect the preview against the complete ATC
instruction. A missing name, impossible order or restricted connection reports an
error rather than silently dropping the instruction. Route edits require stopping
taxi first; live speed changes and emergency holds do not discard the route.

## Aircraft And INI Configuration

Edit the installed `A350AutoTaxi.ini`, then use **Reload settings** while idle.
The default profile remains FF A350 v1.6.16. `require_a350=0` permits other aircraft;
it does not prove their custom flight-model plugins accept these inputs. Keep each
aircraft profile separately and install the correct one before taxi.

| Parameters | Meaning / Units |
| --- | --- |
| `wheelbase_m`, `main_axle_aft_m` | Nose-to-main-axle distance; main-axle position aft of the reported CG, in metres. |
| `cockpit_ahead_nose_m` | Longitudinal pilot reference ahead of the nose gear, in metres. |
| `main_gear_half_span_m` | Outer main-wheel half-span; does not represent wingspan. |
| `main_gear_half_track_m` | Lateral distance from main-axle midpoint to each main gear centre, used by the unfilled map symbol. Separate from the outer-wheel clearance envelope. |
| `minimum_width_class` | Required apt.dat taxiway/stand wingspan class A through F; choose for the aircraft's wingspan. |
| `wheel_edge_margin_m`, `tracking_allowance_m` | Desired pavement clearance and tracking allowance, in metres. |
| `minimum_turn_radius_m`, `max_oversteer_m` | Geometric turn radius floor and maximum outward compensation; physical steering capability remains enforced. |
| `max_steer_deg`, `steer_rate_deg_s` | Nosewheel steering angle and slew limits. |
| `steering_mode`, `steering_dataref`, `steering_index`, `steering_scale`, `steering_sign` | Direct physical-wheel degrees or verified custom tiller input. Index -1 is scalar; 0+ selects an array element. |
| `feedback_dataref`, `feedback_index`, `feedback_timeout_s` | Independent actual wheel angle in degrees and mismatch timeout. Feedback cannot be the command dataref. |
| `left_brake_dataref`, `right_brake_dataref`, corresponding `_index`, `brake_scale` | Symmetric service brake inputs, scalar/array selection, and multiplier converting 0..1 to the aircraft input range. |
| `parking_brake_dataref`, `parking_brake_index`, `parking_brake_set`, `parking_brake_released` | Writable parking brake input and its actual set/released values. |
| `latitude_dataref`, `longitude_dataref`, `heading_dataref` | Scalar telemetry in degrees: latitude, longitude and true heading. |
| `groundspeed_dataref`, `on_ground_dataref` | Scalar ground speed in m/s and nonzero-on-ground state. |
| `taxi_speed_kt`, `max_taxi_speed_kt`, `turn_speed_kt` | Initial selected speed, UI ceiling and tight-turn speed. Ceiling accepts 2..100 kt; the configured value is not an aircraft operating recommendation. |
| `apron_speed_kt`, `apron_approach_distance_m` | Stand arrival speed cap and fallback distance before the cockpit stop at which it engages. Arrival targets do not increase after engagement. |
| `planned_deceleration_m_s2` | Controller stopping-speed envelope, in m/s squared. |
| `nominal_acceleration_m_s2`, `nominal_brake_authority_m_s2` | Initial ETA model: free acceleration at manual taxi thrust and acceleration removed by a brake ratio of 1, in m/s squared. Telemetry adapts both values while taxiing. |
| `max_brake_ratio`, `emergency_brake_ratio` | Normal/stop and emergency service-brake limits, 0.1..1 before `brake_scale`. |
| `max_cross_track_m` | Tracking fault threshold in metres. |
| `allow_unknown_width`, `allow_intersection_departure`, `painted_runway_exits_only` | Unknown-class handling, intersection departure permission and marked runway exit requirement. |
| `automatic_pushback`, `dsf_tool_path`, `dsf_pavement_resources` | BP integration, official DSFTool executable, verified paved `.pol` resource paths separated by semicolons. |

For a different aircraft, change the geometry, width class, turning limits,
speed/brake settings and verified steering/feedback/brake/parking references first.
If its steering input is normalized, use `steering_mode=custom` and a scale based
on the actual full-travel wheel angle. Scalar telemetry can usually retain X-Plane's
standard references. Existing wheel/toe overrides on generic aircraft are still
rejected; use the aircraft's verified custom inputs rather than claiming its overrides.
Automatic towing additionally requires BetterPushback compatibility with that aircraft's
brakes and geometry; `automatic_pushback=0` retains manual/external towing.

## Airport Data And Limits

- Active `scenery_packs.ini` order wins per airport ID. Disabled/unlisted scenery
  is excluded. X-Plane 11 default apt.dat and the X-Plane 12 global-airport location
  are supported. The XP12 `*GLOBAL_AIRPORTS*` entry keeps its exact position in the list;
  disabled or unlisted global-airport packs are not reintroduced by a fallback when the
  list exists. UTF-8 BOMs, absolute package paths and paths containing spaces are supported.
  Lower-priority stand names within 25 m are available as aliases of active
  scenery stands; lower-priority taxi networks are not merged.
- Reads legacy startup locations (15), land runways (100), taxi nodes (1201), directional taxi edges (1202),
  active-runway zones (1204), ramps (1300), ramp width (1301), paved areas (110/111-114),
  ground marking chains (120/111-116), hold-short markings, and airport/taxiway signs (20). Bezier handles are sampled for both painted lines
  and pavement boundaries, including polygon holes. Attributes on `110` pavement rings stay
  pavement metadata; only `120` linear features enter the ground-marking layer.
- The Runways tab includes every detected hold-short marking as an independently selectable
  `RWY xx HOLD n` destination. Stand markers in the map are clickable and select the complete
  gate/stand name. Airport and Local maps show surveyed runway edges as heavy white lines,
  signs when zoomed in, and one-way runway entry arrows with a red no-entry bar at the
  prohibited end.
- The navigation strip reports the active taxiway and its remaining distance. ETA uses the
  live measured acceleration/brake model when available; while a preview is stopped or its
  asynchronous forecast is pending, it shows an immediate current-speed estimate instead
  of going blank.
- Route search retains the incoming direction at each edge, rejects turns above 110 degrees,
  respects one-way edges and applies the configured minimum width class (E by default). It can find a longer route when
  a short route has an incompatible turn. Other runway crossings require explicit clearance
  and are penalized during search; the chosen crossings appear before starting.
  Older edges with unknown width are accepted by default; set
  `allow_unknown_width=0` to require explicit E/F width.
- Taxi speed can be selected from 1 kt to `max_taxi_speed_kt` (**20 kt** by default). The controller reduces speed for
  bends, runway alignment and braking distance.
- The usual graph join limit is 35 m. Stand lead-in matching allows 45 m when the line direction
  agrees with the stand. Gate/apron connectors extend to 180 m when the entire
  connector remains on a paved surface. Forward departure curves match aircraft heading
  and the taxi-edge direction, with a geometric turning-radius floor set to the main-gear
  track (10.36 m for the default A350 profile; increased
  when the configured wheelbase/steering limit requires it). The planner may join farther
  along the edge to make room for the turn. Feasible forward routes are preferred over a tow.
  Sampled nose/main wheel-envelope checks supplement connector checks. They do not
  establish clearance for the aircraft's wings, engines or complete articulated bogies.
- Runway entries use either explicit ATC associations or a connected painted lineup curve and must be
  near its departure end (first 700 m or first 30 percent). The final leg extends
  along the centerline, reducing speed and requiring heading error <=2 degrees
  and lateral error <=2 m before completion. It does not start a takeoff roll.
  `allow_intersection_departure=1` permits later entries; the UI shows the reduced remaining distance.
  A converging paint chain can end within the runway strip before reaching the
  surveyed axis (up to 15 m lateral gap, subject to runway width). The planner
  preserves its final observed point, then constructs a tangent-continuous cubic
  continuation confined to the runway, with at least 35 m radius. This is marked
  as inferred geometry, not added to the scenery's yellow lines. Connected
  painted entries are preferred over ATC-only entries; runway clearance,
  direction, wingspan restrictions and intersection-departure rules still apply.
- Runway exits require a connected painted exit by default. An unmarked ATC
  runway-to-taxiway chord is not an exit. A paint chain ending before the axis
  uses its tangent-preserving continuation in reverse for exit routing. Where
  the ATC runway axis is missing, a temporary row-100 axis can connect only to
  detected painted exits; the airport graph is unchanged. The first compatible
  forward exit is determined by connectivity, heading, clearance and any ordered
  waypoints. A particular exit is not guaranteed without selecting its node.
  The same origin-runway exit policy applies when the destination is another runway.
  `painted_runway_exits_only=0` explicitly restores legacy unmarked exit routing.
- Missing networks, disconnected routes and tight reversals report an error.
  There is no fabricated straight-line taxi route across arbitrary airport areas.
- ATC-only fallback links are a centerline approximation, not a surveyed aircraft path.
  Tight bends may be cut by the pursuit controller; the cross-track watchdog
  stops it if the configured limit is exceeded. Painted-turn compensation checks
  known pavement boundaries but does not model wingtip clearance, terrain,
  parked aircraft, traffic or ATC.
- Node display means the **nearest** node; on an edge it shows that node and distance.
  Automatic idle airport refresh occurs after moving 500 m. Detect airport forces a refresh;
  Reload config also rebuilds the scenery index.
- Disable/unload returns controls immediately. It cannot continue braking after unload.

## Painted Centerlines

The map shows actual apt.dat ground markings in yellow. Taxiway centerlines and stand lead-ins
with line types **1, 7, 51, 57** become taxi routes: ordinary/enhanced yellow centerlines and
their black-bordered variants. Centerline light strings **101** and **105** are also routable
when an airport supplies them without painted yellow geometry. The official specification defines
**20** and **22** as white roadway markings, so they remain display-only even if they are drawn
on an apron. Vendor extensions 19/30/31 are treated the same way. White roadway/service markings
remain display-only; they are never promoted from proximity alone. Stand Lead-in association is
applied to painted yellow centerlines when an endpoint is within 40 m of a stand stop position,
the feature is within 45 m at its local segment, and its tangent agrees with the
stand heading within 35 degrees (in either direction), with the other endpoint joining the
painted taxiway network within 60 m and the stand endpoint remaining terminal. This topology check prevents
service roads and main taxiways from becoming aircraft guidance lines. The closest matching stand owns the lead-in,
so pushback handoff candidates do not use another stand's white/custom guidance line.
Handoff selection weights each metre of towing as six metres of forward taxi to discourage
long reverse manoeuvres that save only a modest taxi detour.
This is a geometric inference, not confirmation that a white road marking is aircraft guidance.
Holding lines (**4, 5, 6,
103, 104**), boundaries/edge lines (**2, 3, 52, 53**),
queue markings (**8, 9, 58, 59**), runway lead-off lights (**107, 108**) and stand outlines
remain display data; they do not establish an aircraft taxi route.
Runway geometry follows the row 100 axis.
When the aircraft starts on a runway whose ATC axis omits a terminal portion, planning
extends that terminal portion along the surveyed row 100 axis to the existing runway node.
The connection remains inside the physical runway and preserves the directions allowed by
the existing runway edges. Runway clearance and the painted-exit policy still apply; this
does not create an unmarked taxiway exit or join across an interior gap in the runway graph.

An ATC runway edge is not required when a continuous painted entry approaches from outside
the runway strip and ends within 2 m of its surveyed axis, with its final tangent within
35 degrees of the axis. The endpoint remains a graph junction. Route search still requires
a connected path, runway clearance, compatible arrival direction, aircraft turn radius and
the departure-end/intersection limits. No missing taxi connection or full runway graph is
generated for these entries; the final alignment leg follows the row 100 axis.

When an entry curve ends before reaching that axis, a converging endpoint inside
the runway strip and within 15 m of the axis can receive a tangent-continuous
cubic continuation. The continuation must stay inside the runway, respect a
35 m minimum curve radius and match the selected runway direction. It is stored
separately from observed paint and appended after route search reaches the actual
paint endpoint; it cannot create graph shortcuts. The departure-end distance limit
also uses the observed endpoint. A reachable painted entry takes priority over an
ATC runway-entry chord, and its final painted samples are retained before the
unpainted continuation joins the surveyed runway axis.

Painted features are split at intersections and sub-metre endpoint joins, including apron
branches omitted from the ATC network. Curved geometry remains in the route and map;
navigation does not reduce it to a straight chord between ATC nodes. Nearby ATC nodes
are associated with the physical line within 40 m. Matched ATC direction, width and active-runway
rules are retained; runway-strip crossings also require runway clearance. Unclassified apron
links retain unknown width, governed by `allow_unknown_width`. Centerline paint alone does
not establish A350 wingtip clearance. Tight painted bends and incompatible junction turns
are excluded from route search.

Short breaks between two dangling painted centerlines can be repaired at airport
load time. The endpoints must be no more than 20 m apart, each face the gap within
25 degrees, and admit a tangent-preserving curve with radius at least 35 m. The
entire inferred connection is sampled at intervals of at most 0.5 m and must be
inside identified pavement or the surveyed runway surface. Connections preserve
both sides' one-way permissions, the narrower known width and all active-runway
restrictions. Each endpoint receives at most one repair. Unknown pavement, holes,
parallel lanes and longer breaks do not qualify. These links infer continuity;
they are not additional yellow paint read from scenery. Original map markings
remain unchanged, and airport logs report the number of repaired paint gaps.

A dangling endpoint may also extend its observed tangent by up to 20 m to meet
another painted taxi centerline at a shallow angle (at most 35 degrees). This
creates a shared intersection on that line; it cannot join an unpainted ATC axis
or fabricate a right-angle turn. Identified pavement, one-way/width/runway rules
and the aircraft's normal turn and oversteer checks still apply. Tight existing
paint remains subject to `minimum_turn_radius_m`; closing gaps does not disable
that limit. In the local 35L regression, the short route still reports one 108 m
P1 ATC fallback because a painted P1 bend has radius about 14.3 m, below the
  default main-gear-track floor.

For a selected stand with a disconnected painted lead-in, the planner can infer
an entry from nearby apron paint when the lead-in root faces a gap no longer than
20 m. It starts the turn upstream on the apron, joins the real lead-in downstream,
and keeps the remaining painted approach and stand stop. Cubic entry curves must
stay on identified pavement and satisfy the configured turn radius; normal wheel
envelope, width, one-way and runway-clearance checks still apply. No entry is inferred
across unknown pavement. The route records these as inferred connections, named
`Inferred stand entry`; they are not new scenery paint.

Search cost includes cumulative turning inside curved links as well as junction
turns, discouraging repeated reversals when a simpler connected path exists.
Curvature checks use a 5 m neighbourhood to tolerate the graph's 0.3 m junction
weld error; genuinely undersized bends are still rejected. Arrival/departure
tangents use up to 3 m of geometry instead of a potentially tiny first/last sample.

When no connected painted path is available for an ATC link, that link remains as fallback.
It is drawn as a thin grey dashed line, and route previews report the selected fallback count.
Airport loading logs the ground-marking and fallback counts. Virtual painted junctions have
generated node IDs; these can differ from the original ATC IDs shown by previous builds.

To audit every stand in an installed airport, run the local diagnostic:

```text
stand_lead_in_audit.exe "D:/X Plane 11.55-g/Custom Scenery/Global Airports/Earth nav data/apt.dat" ZSSS
stand_lead_in_audit.exe "D:/X Plane 11.55-g/Custom Scenery/Global Airports/Earth nav data/apt.dat" ZSSS "Gate 271 T2"
```

It reports the source line type, distance and heading match, nearby unclassified white lines,
and whether a local painted approach can be planned. A `MATCH` is a geometry match only; the
route planner still applies taxiway width, one-way, runway clearance and aircraft turn checks.

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
yellow `.lin` chains and `.pol` contours, keeps single-yellow chains routable and double-yellow chains display-only,
deletes the temporary file, and rebuilds the centerline graph. The log reports
`DSF yellow marking lines loaded`. If DSFTool is unavailable, the plugin
continues with apt.dat and reports `apt.dat only`; it does not silently invent DSF geometry.

For display, DSF line geometry is appended after apt.dat geometry, so the active custom
scenery line is the top visual layer when the two differ. For routing, apt.dat 1202 remains
the source of taxiway width, one-way direction and runway-activity semantics; DSF supplies
the visible centerline shape. The audit currently reports that ZSPD's 36 custom single-yellow
chains differ substantially from the 1202 node chords, so they must not be treated as proof
that every yellow painted line is an aircraft taxiway.

### DSF Pavement Contours

The runtime parser keeps separate `.pol` windings, including holes. It uses a
polygon as pavement only when its resolved local resource declares `SURFACE asphalt`
or `SURFACE concrete` with `LAYER_GROUP taxiways` or `runways`, or when the user lists
that verified resource in `dsf_pavement_resources`. Other polygon outlines are map
context only. A road marking with `SURFACE asphalt` is not automatically a taxiway.
Orthophoto rectangles may cover grass as well as concrete and are never automatically
considered pavement. Surface counts and incomplete coverage are logged after loading.

The route is sampled at intervals up to 4 m to compare approximate nose/main-wheel
clearance against apt.dat plus identified DSF surfaces. Live telemetry checks the
actual configured nose/main wheel positions against cached pavement at about 4 Hz;
`Wheels outside known pavement` means outside the surfaces the parser knows, not
proof of leaving all real visible pavement. This status does not automatically
cancel taxi, because custom surface coverage can be incomplete.

The DSF reader samples four-dimensional Bezier `.lin` curves and `.pol` boundaries,
including repeated anchors for split handles and closed rings. For orthophotos,
four-dimensional UV coordinates are distinguished from curve controls; eight-dimensional
curved UV polygons also use their geographic controls. This follows the official
[DSF usage](https://developer.x-plane.com/article/dsf-usage-in-x-plane/) and
[painted line specification](https://developer.x-plane.com/article/painted-line-lin-file-format-specification/).
The implementation does not reconstruct pavement from OBJ meshes, base-terrain
triangles, texture pixels/alpha or library aliases. DSF routing and boundaries
cannot yet be promised to match every visible yellow line or paved edge in every
commercial package. All conversion and checking stay on the user's machine.

While taxiing, the panel reports the next turn distance and angle, the next Node and ETA,
the next taxiway change and ETA, plus total remaining ETA. Once a forward route is
ready, a background worker predicts the entire remaining drive by running a copy
of the actual controller, including its current braking integral, tracking state,
speed-dependent lookahead, steering rate, upcoming-bend limits, runway alignment
and final stopping/settling logic. It integrates the aircraft's motion at 0.1 s
steps. Live control and watchdog frequency remain unchanged.

The initial motion model uses nominal free acceleration (0.25 m/s squared) and
brake authority (1.5 m/s squared at brake ratio 1). Half-second telemetry windows
adapt free acceleration from measured motion and applied brake force. Rolling
increases in brake demand also calibrate brake authority; near-zero speed with
brakes held is excluded because ground friction hides the engine acceleration.
The forecast refreshes approximately once per simulated second; measured
route progress updates the displayed remaining times between predictions. Node,
taxiway and total times come from the same predicted timeline. Live speed changes
discard the old prediction; emergency holds display `--:--` until resumed.

Steady small brake corrections, acceleration after a slowdown and steering/CTE
corrections are already simulated by the forecast controller. Live timeline
refreshes blend passage and arrival times at the same measured progress with a
5 s response, limiting forecast correction to 2 seconds per simulated second.
This reduces model-noise jumps without slowing the actual control loop. Sustained
changes still update the prediction; a changed speed setting resets it immediately.
This is display stabilization, not a claim of measured FlightFactor ETA accuracy.
Turn speed setpoints change progressively, with braking distance anticipated
before upcoming bends/oversteer. Brake integration avoids saturation windup and
releases residual braking promptly when underspeed; steering limits vary smoothly
with speed instead of switching at a single threshold.

The entire route is planned ahead, but steering and braking commands are feedback
outputs, so future manual throttle changes, traffic holds, slope changes and
aircraft response cannot be known exactly in advance. Preview ETA uses the initial
model; live ETA learns the current response and assumes it continues. No
automatic engine/throttle control is added. If the model cannot reach the endpoint
(for example, no accelerating thrust or insufficient braking), ETA displays
`--:--` rather than inventing an arrival time. Pushback route previews show
`--:--` for timing because tug connection, towing and disconnection duration cannot
be predicted by the taxi speed model.

If a stand can be seen in the scenery but cannot be found by name, first check
the active apt.dat path logged as `Airport source`. Some scenery uses a different
name for its row 1300/15 ramp than the number painted on the ground. Nearby names
from lower-priority airport definitions can appear as aliases, but those files
do not replace the active taxi geometry. A DSF stand-number texture or object
alone is not a ramp-start record and does not establish a stop position, heading,
or aircraft size. Such visual labels are not automatically invented as destinations.

## Local ZSPD Regression

The current active scenery is `Custom Scenery/A_ZSPD/Earth nav data/apt.dat`.
Stand 539 from the global metadata matches custom stand `S1-118` within 13 m.
The departure display therefore includes
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
  configured custom steering/brake datarefs must be verified locally.
- BetterPushback's public API does not expose general coordinate-route
  submission, so the integration uses its documented route-cache workflow and
  refuses to interfere with an active tow.
- This release targets X-Plane 11 on Windows. Linux/macOS builds need platform
  adapter and packaging work before they can be considered supported.

## UI Performance

The controller caches segment lengths, remaining distances and bend angles when
taxi starts. Every physics cycle still updates steering, brakes and
safety checks; remaining distance uses a lookup instead of summing the entire
remaining route. Unused per-cycle status-string formatting has been removed.
The forward bend scan retains the same 70 m horizon and speed limits.

The live window caches panel drawing commands between telemetry updates (about
4 Hz). Aircraft control and safety monitoring keep their original flight-loop
frequency. Mouse interactions and window resizing invalidate the layout immediately.
Static pavement, markings and fallback taxi links reuse projected/clipped drawing
data until the airport or viewport changes. Lines with the same width are submitted
in consecutive OpenGL batches, preserving their order and individual colors.
The static airport layer is compiled into an OpenGL display list and replayed
with one call per frame. It is rebuilt only when its geometry or viewport changes,
including pan, zoom, resize, map mode and airport replacement. Moving the window
and updating aircraft telemetry reuse the compiled layer. Route overlays, runway
labels and aircraft markers retain their drawing order and update behavior.
If the driver cannot allocate a display list, drawing uses the immediate fallback.

Consecutive Consolas labels share one textured draw batch. Each batch sets the
graphics state, binds the font atlas and restores its texture environment once,
instead of doing that separately for every label. Painter order and colors are
preserved; native-font fallback resets graphics state before subsequent geometry.
Panel updates reserve space for the cached airport and route overlay to avoid
moving tens of thousands of commands when the command buffer grows.

ETA forecasts run on a separate background worker using the controller's cached
segment lengths and bend headings. UI drawing reads the resulting timeline and
interpolates marker times; it does not simulate the route during a draw callback.
Cancellation checks prevent a replaced route or speed setting from publishing a
stale forecast. No forecast calls the X-Plane SDK or writes flight controls.

Runway availability scanning runs on a lower-priority worker on Windows. Closing
the window, selecting a route or starting taxi requests cooperative cancellation
inside the current runway calculation. A hidden window does not start
new availability scans. Reopening resumes a cancelled scan as needed.

Route previews, taxi engagement and forward-route planning after pushback run on
one lower-priority worker. Repeated destination/policy/node edits keep only the
latest queued request, and an older result cannot replace it. Airport changes and
manual/stop actions invalidate requests without waiting for a running calculation.
The old calculation checks its cancellation token inside graph search, apron
connector sampling, reverse-detour search and turn/pavement assessment; it exits
at the next checkpoint so the latest request can run. Cancellation is kept
separate from route failure and cannot trigger a pushback/oversteer fallback.
Unloading still joins the cancelled worker. No XPLM calls occur on the planning worker.

Pavement rings index their edges by vertical range. A point test examines only
edges that can intersect its horizontal ray or its 0.5 m boundary tolerance,
instead of rescanning every edge. Polygon holes, runway rectangles and clearance
thresholds retain their existing meanings; no scenery geometry is simplified.
The status bar shows the planning phase, reverse candidate count and elapsed
wall-clock seconds. A selected destination updates its availability immediately
when its preview completes, without waiting for every runway to be scanned.

Engagement computes from an aircraft-state snapshot and then checks the current
position and heading on the main thread before acquiring controls. A movement of
1 m or a heading change of 3 degrees rejects a stale start; keep the aircraft
stopped during planning. Pushback handoff also requires a stopped aircraft and an
idle tug. Previews complete while paused; pausing defers engagement and a ready
route reports that the simulator must resume. Changes to selected taxi speed
while planning or towing are applied when taxi starts. The simulator control and
BetterPushback adapters remain entirely on the main thread.

Workers share an immutable airport snapshot instead of repeatedly copying the
entire graph and DSF contours for each request. The airport loader also constructs
the pavement query in its background worker. Wheel boundary telemetry reuses
that query at the UI cadence; control continues at every physics cycle.

For a local CPU benchmark without X-Plane, build with `BUILD_TESTING=ON` and run:

```powershell
.\build-release\panel_benchmark.exe "path/to/your/apt.dat"
.\build-release\panel_benchmark.exe "path/to/your/apt.dat" "path/to/X-Plane" "path/to/DSFTool.exe"
.\build-release\controller_benchmark.exe
.\build-release\planning_benchmark.exe "path/to/your/apt.dat" "590" "15"
```

It compares 120 uncached layouts with 120 draws using eight telemetry updates,
then reports line segment and GL batch counts. It also measures a synthetic
5000 m route with ETA in overview and local views. The optional simulator-root and
DSFTool arguments include local DSF markings and contours. Results measure layout work, not live X-Plane FPS or GPU
performance. The input and decoded scenery remain on the user's machine.
The controller benchmark runs 20,000 cycles on synthetic 5 km routes at several
point densities. It compares the original geometry calculation alone with a
complete cached controller update and checks their distance/speed checksums;
these timings also do not measure simulator FPS.
The planning benchmark times one departure, including automatic pushback planning
when needed. It reads local scenery and reports aggregate distance/timing only.

On Windows, the `map_rendering` test creates a hidden OpenGL window and compares
cached and immediate pixels, including different colors, widths, window positions,
dynamic overlays and geometry replacement. It checks GL state restoration and
exports only a synthetic test image into the test working directory. To benchmark
the real static airport layer locally at 1080x720, run:

```powershell
Set-Location .\build-release
.\map_renderer_tests.exe "path/to/your/apt.dat"
```

This measures 120 draws including GPU completion on the reported graphics driver,
after warming the cache. It is a standalone render benchmark, not X-Plane FPS;
initial cache compilation and airport layout are excluded from the draw timings.
No decoded commercial scenery is exported.

The Windows `ui_rendering` test compares the actual panel drawing callback with
the previous unbatched text path, checking pixel equality, window movement,
resizing, colors and native-font fallback. It reports graphics-state/texture-bind
counts and a synthetic panel render benchmark. `background_planning` checks the
worker queue and cancellation; `plugin_background_handoff` checks asynchronous
engagement, pause, aircraft movement, cancellation, live speed and tow handoff
using fake simulator adapters. These tests do not establish live FlightFactor
behavior or a specific improvement in X-Plane FPS.
The `indexed_pavement` test compares indexed and exhaustive polygon containment
on a detailed concave synthetic surface, including holes, near-boundary points,
duplicate/tiny edges and runway wheel envelopes.
