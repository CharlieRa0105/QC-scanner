# Calibration

> Part of [[Quality Control Scanner]]

The system relies on three coordinate relationships being correct. Most
scan-quality problems trace back to one of these being off, so treat
calibration as the foundation everything else sits on.

## What needs calibrating

The MIRACO Plus is **arm-mounted, not externally referenced** — it uses its
own photogrammetry for pose tracking, with no separate tracker unit. That
means the SR5's own repeatability (±0.03mm) sits in the measurement chain,
and there are no cameras to calibrate against each other. What must be
calibrated:

1. **Corner-reference-to-arm** — the relationship between the table's marked
   corner reference (where the CAD model is registered, and therefore where
   the part actually is) and the arm's base frame (where waypoints are
   commanded). Without this the planned standoff is wrong everywhere.
2. **Hand-eye transform** — the scanner's pose relative to the arm's tool
   flange (fixed once mounted, but must be measured accurately). This is the
   **dominant accuracy lever**: a 0.01° rotation error ≈ 52µm at 300mm
   standoff — on the order of the whole accuracy budget. Validate against a
   certified reference artefact (ceramic sphere bar / low-CTE scale bar).
3. **Tool-changer repose** — after any tool change (scanner ↔ thread probe),
   re-validate the tool pose; the tool changer's own repeatability re-enters
   the chain each time.

## Corner-reference-to-arm calibration

Touch a probe (or the scanner itself) to several known points at the marked
corner reference and solve for the transform into the arm's base frame. The
exact procedure depends on the final cell layout — document the chosen
method here once it is settled on site. See
[src/arm_control/arm_controller.py](../src/arm_control/arm_controller.py)
for the pose-execution interface this transform feeds into.

## Hand-eye calibration

Vendor + MoveIt calibration routine against a certified ceramic sphere bar
or low-CTE scale bar, run through several arm poses. Validate against the
traceable artefact before trusting any scan-vs-CAD deviation numbers.

## Verifying the full chain

After both are done, run a scan of a known part and check the quality
gate's hole detection. Systematic missing coverage on one side usually
points to a frame misalignment rather than a planning bug.

## When to recalibrate

- The arm or track is repositioned, or the table/corner reference moves →
  redo corner-reference-to-arm.
- The scanner is remounted or bumped on the tool changer → redo hand-eye,
  then re-validate against the certified artefact.
- After any tool change on the changer → re-validate the tool pose.
