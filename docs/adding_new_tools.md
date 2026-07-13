# Adding a new end effector

> Part of [[Quality Control Scanner]]

The ROKAE xMate SR5 has a quick-change tool changer fitted, so the cell is built
to take more than the scanner. **Mind the 5kg payload budget** — every tool
(scanner, thread probe, changer, cabling) competes for the same tight envelope,
so weigh each addition before fitting it. **Dimensional thread gauging is in scope** and
is the first planned addition: threads (external + internal) are distributed
all over every part and carry a dimensional tolerance (pitch diameter /
pitch), which an optical surface scan alone cannot verify. No scanner-ecosystem
probe gauges thread PD and no mature system does automated inline dimensional PD
(internal threads are the hard case), so the plan is **two-tier**: an **inline
GO/NO-GO gauging tool on the changer** (functional accept) plus an **offline
dimensional-PD referee** bench (Johnson Gage / Gagemaker; CMM escalation). The
scan locates the threaded features and generates the tool path. The steps below
apply to any new tool.

Keep new-tool logic separate from the scanning pipeline so the two do not
entangle.

## Steps

1. **Physical fitting**
   - Get the tool's CAD model and mounting interface.
   - Confirm it works with the quick-change tool changer.
   - Add the tool to the MoveIt2 collision model so collision checking
     accounts for its geometry. Its tool-tip offset (the transform from
     the changer flange to the working point) must be measured and
     recorded.

2. **Configuration**
   - Add a section for the tool in
     [config/system_config.yaml](../config/system_config.yaml) — at
     minimum its tool-tip offset and any standoff or speed constraints.

3. **Control module**
   - Add a module under [src/](../src) for the tool's behaviour (for
     thread inspection, e.g. `src/thread_inspection/`). Reuse
     [src/arm_control/](../src/arm_control) for motion; only the
     tool-specific logic is new.

4. **Path planning**
   - If the tool needs a different motion pattern than surface coverage
     (thread inspection, for example, targets specific features rather
     than whole surfaces), add a dedicated waypoint generator rather than
     bending the scanning one.

5. **Pipeline integration**
   - Decide whether the new tool runs as its own job or as a stage within
     an existing job, and wire it into
     [src/orchestrator.py](../src/orchestrator.py) accordingly. Tool
     changes mid-job must home safely and re-confirm the tool offset.

6. **Tests**
   - Add tests under [tests/](../tests) covering the new tool's planning
     and control logic, mocking hardware where it is unavailable.

## Principle

Keep each tool's logic in its own module and let the orchestrator compose
them. Avoid special-casing tools inside shared modules — that is what
makes a multi-tool cell hard to maintain.
