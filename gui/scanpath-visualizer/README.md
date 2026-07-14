# ScanPath — Metrology Scan-Path Planner

Import a 3D STEP file, and the app inspects the geometry and lays out a robot
trajectory for a 3D scanner that holds a fixed **standoff distance (20–30 cm)**
from the surface it faces, aiming to cover the whole part.

## Run it

STEP parsing uses a WASM CAD kernel loaded from a CDN, and the app uses ES module
imports, so it must be served over `http://` (not opened as a `file://` path).

```bash
cd scanpath
python3 -m http.server 8000
# open http://localhost:8000
```

Then **Import STEP file** (or drag one onto the viewport) → **Generate scan path**.

## Controls

- **LMB** orbit · **RMB** pan · **scroll** zoom · **ISO/TOP/FRONT/RIGHT/FIT** preset views
- **Standoff distance** — 20–30 cm, the fixed camera-to-surface offset
- **Coverage density** — more rings + viewpoints = denser sampling
- **Path smoothing** — Catmull-Rom smoothed vs. raw waypoint segments
- **Display** toggles — model, path line, camera icons, robot arm, grid

## Simulate the scan

Once a path exists, a **transport bar** appears at the bottom of the viewport:

- **Play / Pause** (or press **Space**) — animate a scanner rig traveling the path
- **Stop** — reset to the start
- **Scrub** — drag to any point along the path
- **Speed** — 0.25× to 4× (scales with part size, so travel looks natural)
- **POV** — ride along *through* the scanner, seeing what it sees at the current pose
- **Loop** — repeat playback

While it runs, the transport bar reads out the live **waypoint index** and
**standoff distance** — the standoff turns amber if it ever leaves the 20–30 cm band.

## Table, gantry & part placement

The part sits on a **3 m × 2 m worktable** whose top surface is the floor plane
(y = 0), drawn with a 0.5 m grid and dimension labels. **Nothing goes below the
table** — scan viewpoints and every arm link are constrained to y ≥ 0.

The arm is mounted **upside-down at the centre of the table** (X = 0, Z = 0), hanging
from an overhead mount and reaching *down* onto the part. There is no reserved footprint
strip — the whole table top is the work area and the part seats at the table centre,
directly under the mount.

**Mount height** (Display panel) — a slider raises/lowers the overhead mount. It starts
on **auto** (high enough to clear the part and standoff, but within the arm's downward
reach) and shows the current height; drag it to set a fixed height in cm.

An **X-slider** switch (Display panel) chooses how the arm covers X:
- **On** (default) — the base rides a gantry rail along X through the table centre, so
  the arm only has to solve the Y-Z reach at each carriage stop.
- **Off** — the base is **fixed** at the table centre and the arm covers X with its
  own joints (base yaw + reach). Good for a stationary pedestal cell; wide parts may
  show more `HELD` points since the base can't chase them.

The transport bar shows the live **carriage X** position in metres as it plays.

**Auto-orientation on import.** When a part loads it is automatically rotated into
the best pose for scanning — its **longest** dimension along X (so the gantry does
the long travel), its **shortest** along Y (lying flat, lowest and easiest to reach),
and the middle along Z. This only permutes/flips axes, so the part's **scale is never
changed**. You can override it with the placement controls.

Under **Placement on table**:
- **Rotate X / Y / Z** — rotate the part 90° about each world axis (cumulative), not
  just the vertical — so any face can be turned to point up or toward the rail.
- **Centre** — centres the part in the parts area (on) or seats it toward the far edge (off).
- **Drag to move** — just grab the part (or the coloured **XYZ arrow gizmo** on it)
  with the mouse and slide it across the table in the X-Z plane. No mode button —
  click-drag on the part moves it, click-drag on empty space orbits the view. The
  part is clamped to stay fully within the parts area (off the gantry footprint), and
  the scan path regenerates when you release.

**Reach note:** hanging over the table centre, a true-scale SR5 (~0.9 m reach) can't
cover the full 1.5 m depth (or a very wide part with the X-slider off), so out-of-reach
points show as `HELD`. Turn off **True scale** for a scaled demo (the arm and mount grow
to span the whole cell — HELD points drop to a handful).

## Rokae SR5 arm (from your URDF)

The arm is the **real xMate SR5** built from your `xMateSR5_urdf.xacro`: the six
joint origins, axes, and limits are taken verbatim from the URDF, and the actual
visual STL meshes (`meshes/xMateSR5_base…link6.stl`) are loaded and attached to each
link. The URDF is Z-up (ROS); the arm is rotated into the app's Y-up world and then
**inverted** so it hangs upside-down from the overhead gantry.

Joint limits used (radians, from the URDF): J1 ±6.283, J2 −2.793…2.618,
J3 −2.967…2.443, J4 ±6.283, J5 ±6.283, J6 ±6.283. The tool point (TCP) sits on the
link-6 flange; IK drives the TCP onto each scanner pose.

The STL meshes must sit in a **`meshes/` subfolder** next to the app (already
included). Because they load over `http://`, keep serving the folder rather than
opening the file directly.

**True scale (1:1)** — on by default: the arm is at real SR5 size (~0.92 m reach),
so reach limits are honest and points it can't reach show as `HELD`. Turn it off for
a fit-to-table demo where the model is scaled up to span the whole table (useful to
see the full trajectory, but no longer a real reach envelope).

Toggle the arm with **Display → Rokae SR5 arm**.

### Scanner mount & aim

The 3D scanner is a **rigid fixture bolted to the arm's mounting plate (flange)** —
it's a child of the flange, seated flush on the plate face with the lens protruding,
so it moves and rotates only with the arm and can never face the plate. Its optical
axis is parallel to the plate, facing outward along the flange's tool axis (+Z).
The IK aims that axis at the surface using the wrist joints, then applies a **roll
correction** about the tool axis so the scanner stays upright — it won't tilt
sideways. Orientation is never set independently of the arm.

### Collision & reach handling

The arm hangs from the overhead gantry at a fixed height; the carriage tracks the
scanner's X, so the arm only has to solve the Y-Z reach. A **collision-aware
trajectory is precomputed** along evenly-spaced checkpoints (each solved from the
last **collision-free** configuration for continuity) to seed the motion, and during
playback the arm is refined with **live IK each frame** so its flange stays locked on
the scanner it carries. Collision testing samples points densely along every link and
flags a link if any sample is *inside* the part or within a clearance margin of the
surface — so a joint or link body penetrating the part is caught, not just the thin
centerline.

The inside/outside test uses a **multi-ray majority vote**: several rays are cast in
independent directions and the odd/even surface-crossing counts are pooled, so a stray
ray that leaks through a crack, hits a coincident face, or grazes an edge is outvoted.
This is far more reliable than a single-ray parity test on the imperfect triangle soup
STEP tessellation often produces. The proximity test casts along the axes **and the
cube diagonals**, so a link skimming the surface at an oblique angle is still caught.

The key rule: **a penetrating pose is never stored.** If no collision-free
configuration can be found for a checkpoint, the arm holds its last safe pose instead
of dipping into the part, and the transport bar reads `HELD (no safe path)`. During
playback every stored pose is collision-free, so the joints don't pass through the
geometry.

The scan path itself is a serpentine up the part with evenly-spaced checkpoints and
no backtracking, and every viewpoint sits on the standoff shell *outside* the part.

**Honest limits.** No bottom-cap pass (a seated part can't be scanned from beneath by
a fixed arm). A fixed-base arm can't reach a full 360° wrap from one mount, so some
points will show as held/unreachable — that's real geometry, and the arm now stays
out rather than faking it. The inside/clearance test assumes a reasonably watertight
STEP mesh; extremely open or non-manifold geometry can weaken the inside test. For a
real cell you'd add a turntable or reposition between passes — send me your fixturing
and I'll model it.

## What you see

- **Line** — the ordered scanner trajectory (teal→violet = start→end; orange dot = start)
- **Camera icons** — Blender-style frustum + up-triangle showing scanner **position and rotation** at each waypoint
- **Rokae SR5 arm** — line illustration whose tool point follows the scanner (see above)

## How the path is planned

The planner uses a **Multi-Face Raster algorithm** — every reachable face of the
part is swept with its own regular grid of rays, so a long part is covered end to
end and on its sides, not just from above:

1. **Reachable faces** — the arm approaches from the −Z side and slides along X, so
   three surfaces are reachable and each is rastered independently:
   the **top** (+Y), the **near side** (−Z, the vertical face facing the rail), and
   **both ±X ends** (reached by moving the gantry carriage). The far +Z side and the
   underside are skipped — a fixed gantry-mounted arm genuinely can't reach them.
2. **Grid sampling** — each face is sampled on a regular grid at a fixed surface
   spacing (spacing scales with the Coverage Density setting), so coverage is even
   regardless of the part's proportions.
3. **Local-normal standoff** — each ray's hit becomes a viewpoint offset along that
   hit's **true local surface normal** by the standoff distance, so the 20–30 cm
   standoff stays correct even on curved and angled faces — something a world-space
   raster cannot do (its standoff varies as `cos θ`, leaving a 45° face 41% too far).
4. **Reachability filter** — hits on the inaccessible far side (normal ≈ +Z) or the
   underside (normal ≈ −Y) are discarded, and every viewpoint is kept on or above the
   table.
5. **Occlusion rejection** — each candidate is line-of-sight checked: the scanner must
   actually see its target point (the first thing along the sight line is the target,
   not a nearer wall), or the viewpoint is dropped.
6. **De-duplication** — a surface patch already covered by one face's raster isn't
   re-added by another, keeping coverage even and the count down.
7. **Orientation** — each pose is oriented so the scanner's optical axis (−Z) looks at
   the surface point, stored as a quaternion.
8. **Rail-aware boustrophedon ordering** — viewpoints are bucketed into X columns so
   the carriage advances monotonically left-to-right; within each column the sweep runs
   bottom→top and alternates direction each column, so the arm's Y-Z motion continues
   from where the last column ended with no wasted long returns.

This replaced a single-axis "arc" raster that aimed every ray at one column-centre
point — so it really only saw the top and a sliver of the near side and never reached
the ends of a long part — which had in turn replaced a normal-clustering planner (one
viewpoint per normal direction, under-covering large faces) and, before that, a
cylindrical-ring path that circled the part 360°, half of which the arm couldn't reach.

The **Path Report** shows waypoint count, path length, an estimated coverage %,
and the standoff standard deviation (σ) across all viewpoints.

## Export

**Export path (JSON)** writes `scanpath.json`: ordered positions in **mm** and
scanner **quaternions**, in a Y-up machine frame with the model centred at origin,
plus the surface target point per waypoint. Feed this into your robot post-processor.

## Notes / limits

- Coverage is a heuristic estimate from viewpoint validity plus a per-viewpoint
  line-of-sight check, not a full occlusion simulation. Deep pockets and undercuts
  may still need manual viewpoints.
- Arm reachability and link-vs-part collision **are** checked (see *Collision & reach
  handling* above); poses the SR5 can't reach at true scale are flagged `HELD`.
