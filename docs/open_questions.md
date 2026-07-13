# Open questions

> Part of [[Quality Control Scanner]]

These need resolving before or during build. Each blocks specific work.
The canonical, fuller list with reasoning lives in
[[Quality Control Scanner - Options Report]] §7.

## Phase 0 — vendor confirmations that gate procurement

- **Accuracy bar — RESOLVED (owner, 2026-07-01), no longer open.**
  "As accurate as achievable within constraints" — best-achievable **≈ 65–75µm
  volumetric**, whole-part scan-vs-CAD (not per-feature); the current 50µm is a
  *manual touch-probe* figure. A **CMM referee** covers any guaranteed-tight
  feature + the threads; no full-part CMM gate is mandated.

- **Scanner selection — RESOLVED (2026-07-07): Revopoint MIRACO Plus** (specified
  in project brief). Arm-mounted structured-light; no external tracker unit;
  ~940g payload; RevoLink SDK (custom ROS2 bridge required). Blocks cleared:
  automation interface, mounting, bridge scope are now scoped to the MIRACO Plus.

- **Thread gauging (hardest risk)** — no scanner-ecosystem probe gauges thread PD
  and no mature system does automated inline dimensional PD (internal threads are
  the hard case). **Plan = two-tier:** inline GO/NO-GO gauging + offline
  dimensional-PD referee (Johnson Gage / Gagemaker; CMM escalation). Scope the
  referee station. Blocks: the thread-inspection subsystem / tool changer.

- **Custom ROS2 bridge scope** — Revopoint does not ship an official ROS2 driver;
  a bespoke ROS2-Humble bridge over the RevoLink SDK is required.

## Hardware

- **Custom rail drive interface** — the 3m rail is custom-built (commercial
  1.5m ~£7k, need 3m). Length and layout decided. Open: motor/drive hardware
  selection (stepper+step-dir / servo+CAN / servo+EtherCAT) and the resulting
  ROS2 interface (basic ROS2 node for index-and-shoot vs full `ros2_control`
  hardware_interface for coordinated motion). Carriage must support ~16.5kg.
  Blocks: track_controller.py, collision model in MoveIt2.

- **Scan-head weight budget — RESOLVED (2026-07-07):** MIRACO Plus ~940g + tool
  changer + cabling comfortably under the 5kg payload. Verify the exact mount +
  cabling assembly weight once hardware is in hand.

- **`rokae_ros2` beta maturity** — the official ROKAE ROS 2 driver is v0.0.4
  (0 stars, unproven). Verify a clean Humble build, MoveIt 2 planning into
  Gazebo, and the separately-downloaded xCore SDK integration early.
  Blocks: the whole simulation-first dev loop.

- **Maximum part height** — determines camera mounting height and arm
  approach angles.
  Blocks: config/system_config.yaml, waypoint_generator.py, camera mounts.
  Resolve by: reviewing CAD files.

- **Clamp / fixture design** — locks the part for contact thread probing
  without occluding scan/thread access or distorting the part.

## Software

- **CAD-to-waypoint pipeline** — does `surface_coverage_planning` / noether
  accept a mesh directly and honour the 300mm standoff constraint natively, or
  must `waypoint_generator.py` post-process its output? Resolve by: testing in
  Gazebo + MoveIt 2 once the ROS2 environment is set up.

- **CAD file format and library** — confirm which mesh format the pipeline uses
  (STL / STEP / OBJ) and which Open3D function to load it for normal estimation.
  Resolve by: testing with a representative part CAD file.

## Future phases (document for the next engineer, not this one)

- QC comparison pipeline: scan vs CAD model, pass/fail per tolerance spec
  (each pass is compared to CAD independently — no cross-pass merge).
- Supplier reliability tracking and historical trend dashboard.
