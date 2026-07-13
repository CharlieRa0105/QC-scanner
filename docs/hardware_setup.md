# Hardware setup

> Part of [[Quality Control Scanner]]

Physical setup guide for the scanning cell. Several values here are still
unresolved — see [open_questions.md](open_questions.md). This document
records what is known and what each piece of hardware needs.

## Components

### Scanner — Revopoint MIRACO Plus (arm-mounted structured-light)

- **Revopoint MIRACO Plus**: arm-mounted structured-light 3D scanner (specified in
  project brief). Weight ~940g — well within the SR5's 5kg payload budget.
- Uses its own photogrammetry for position tracking; **no external camera tracker
  unit required**. The arm's repeatability (±0.03mm on the SR5) enters the
  measurement chain.
- **Accuracy bar = best-achievable ≈ 65–75µm volumetric** (whole-part scan-vs-CAD;
  current manual-process 50µm is a *touch-probe* figure); a CMM referee covers
  any guaranteed-tight feature.
- Optimal standoff: 300mm; acceptable range ~200–400mm.
- Controlled via the **Revolink SDK** — Revopoint does not ship an official ROS2
  driver; a custom ROS2-Humble bridge over the SDK is required.
- No line-of-sight constraint from a separate tracker unit — simpler poses than
  the previously-considered optical-tracker architecture.

### Robot arm — ROKAE xMate SR5-5/0.9C (6-axis force-controlled cobot)

- Reach: **919mm**. Payload: **5kg**. Repeatability: **±0.03mm**. 6-axis,
  force-controlled (joint-torque sensing, 1 kHz), IP54, any-orientation mount.
  This is the arm **on-hand** (confirmed 2026-07-02, superseding the earlier
  UR20 placeholder).
- Quick-change **tool changer** fitted: carries the scanner and a
  tracker-referenced **contact thread probe** (dimensional thread gauging —
  see [adding_new_tools.md](adding_new_tools.md)).
- **Payload is comfortable with the MIRACO Plus** (~940g + mount + cabling, well
  under 5kg). The arm is still **reach-constrained** (919mm), so the traverse
  track carries most of the coverage.
- Controlled via the **xCore SDK** (C++ primary / Python binding; up to 1 kHz;
  position/impedance/torque modes) and the **`rokae_ros2`** ROS 2 Humble driver
  (ships SR5 URDF + MoveIt 2 config + Gazebo; **beta — v0.0.4**, de-risk early).

### Carrier — custom-built 3m linear floor track

- A custom-built 3m motorised floor track running along the table. The SR5's
  short **919mm reach** means the track carries most of the coverage; 3m gives
  full access to a 2.5m part from one side (commercial 1.5m off-the-shelf
  options were rejected — cost ~£7k for half the length needed).
- **Index-and-shoot:** the track moves to a station, stops, then the arm scans.
  Not a coordinated kinematic joint in the current design. Cornering handled as
  straight segments with index-at-corner stops.
- Motor/drive interface: TBD at build time. For ROS2 integration, two paths:
  **(a) index-and-shoot** — a basic ROS2 node commanding the motor controller
  over serial/Modbus/CAN, waiting for a done signal (simplest, current plan);
  **(b) coordinated 7-DOF** — a custom `ros2_control` hardware_interface exposing
  the rail as a prismatic joint (harder, unlocks smoother paths).
- Carriage must support the SR5's ~16.5kg + cabling.

### Table & parts

- Table ~**2.5×2.5m** fixed surface; working height TBD. The part sits on
  the table and does not move during a pass (it is flipped by an operator
  between the two passes).
- **Marked corner reference** on the table defines part position and orientation.
  The operator aligns the part to this corner; all scan paths are generated from
  the CAD file in this fixed coordinate frame.
- Largest part ~**2.5×1.5m**, wide variety of geometry. Threads (external +
  internal) are distributed all over every part and carry a dimensional tolerance.
  Maximum part height is TBD.

## Connections

- The SR5 (xCore controller) and the custom track controller connect over
  Ethernet (arm IP) and the track's motor drive interface (TBD at build time —
  serial/Modbus/CAN/EtherCAT). IPs/ports in
  [config/system_config.yaml](../config/system_config.yaml) (placeholders
  until set on site).
- The MIRACO Plus connects via USB 3 or WiFi per Revopoint's setup; a custom
  ROS2 bridge over the RevoLink SDK feeds captures to the quality gate.

## Setup order

1. Mount and level the table; mark the corner reference (physical datum for part placement).
2. Install the custom 3m track and arm; confirm the track is controllable.
3. Fit the MIRACO Plus (and the thread probe) to the tool changer.
4. Run calibration — see [calibration.md](calibration.md).
5. Verify arm and track connectivity with
   [scripts/test_arm_connection.py](../scripts/test_arm_connection.py).
