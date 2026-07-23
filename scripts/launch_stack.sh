#!/usr/bin/env bash
# Full-stack QC Scanner launcher — the desktop-icon target.
#
# Brings the WHOLE system up with one double-click:
#   1. the ROS 2 mission graph in the `qc-humble` Docker container
#      (rosbridge :9090 + move_group + arm_driver[mock] + path_planner +
#       movement_driver + task_manager + scanner/inspection)
#   2. the console HTTP server on the host (:8000, .venv312)
#   3. the browser at the console URL
#
# Ctrl-C (or closing this terminal) tears the ROS graph container back down.
#
# This launcher runs the MOCK arm (no hardware). The mock starts at HOME and only
# moves when you command it (jog / go-to / execute). To drive the REAL SR5, add
# 'arm_backend:=rokae sdk_root:=/opt/rokae_sdk' to the ROS_LAUNCH line below — ONLY
# with Ra present and the physical E-stop in reach.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER=qc_mission
URL="http://127.0.0.1:8000/"
# REAL SR5 (rokae). The host xCore SDK (~/rokae_sdk) is mounted into the container
# at /opt/rokae_sdk; it has a cpython-310 build matching the container's Python 3.10.
# The arm is at robot_ip (default 192.168.2.160), reachable via --net=host. Change
# 'arm_backend:=rokae' back to ':=mock' to run without hardware.
SDK_HOST="$HOME/rokae_sdk"
# Operator CAD library — the SAME source the console/host backend uses (QC_PARTS_DIR,
# default ~/Desktop/Parts). Mounted into the container so the ROS PathPlanner finds
# the exact parts the console lists (else: "no CAD for part_id …").
PARTS_HOST="${QC_PARTS_DIR:-$HOME/Desktop/Parts}"
# MOCK backend (default; no hardware). Append 'arm_backend:=rokae sdk_root:=/opt/rokae_sdk'
# for the real SR5 (supervised, E-stop in reach).
ROS_LAUNCH="ros2 launch qc_bringup qc_mission.launch.py"

log(){ printf '\n\033[1;32m[qc-stack]\033[0m %s\n' "$*"; }
port_up(){ (exec 3<>/dev/tcp/127.0.0.1/"$1") 2>/dev/null && { exec 3>&- 3<&-; return 0; } || return 1; }

# ---------------------------------------------------------------------------
# 1. ROS graph (Docker). Skipped gracefully if docker is unavailable — the
#    console still starts, but the arm shows "ROS graph down".
# ---------------------------------------------------------------------------
GRAPH_UP=0
if command -v docker >/dev/null 2>&1; then
  if ! docker image inspect qc-humble >/dev/null 2>&1; then
    log "qc-humble image missing — building it (first run, a few minutes)…"
    "$REPO/docker/build.sh"
  fi
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  trap 'echo; log "stopping ROS graph ($CONTAINER)…"; docker stop "$CONTAINER" >/dev/null 2>&1 || true' EXIT INT TERM

  # Host gmsh (the coverage planner loads it lazily, only when planning a part) —
  # mounted read-only onto PYTHONPATH/LD_LIBRARY_PATH so /mission/plan works.
  GMSH_PY="$REPO/.venv312/lib/python3.12/site-packages/gmsh.py"
  GMSH_SO="$REPO/.venv312/lib/libgmsh.so.4.15"
  GMSH_MOUNTS=()
  if [ -f "$GMSH_PY" ] && [ -f "$GMSH_SO" ]; then
    GMSH_MOUNTS=(-v "$GMSH_PY:/scratch/pydeps/gmsh.py:ro" -v "$GMSH_SO:/scratch/pydeps/libgmsh.so.4.15:ro")
  else
    log "note: host gmsh not found — the graph starts, but PART PLANNING will fail until gmsh is available."
  fi

  # xCore SDK for the REAL arm (rokae backend). Mounted read-only; the ArmDriver's
  # sdk_root points here. Missing SDK => rokae connect fails and SILENTLY falls back
  # to mock, so warn loudly.
  SDK_MOUNTS=()
  if [ -d "$SDK_HOST/Release/linux" ]; then
    SDK_MOUNTS=(-v "$SDK_HOST:/opt/rokae_sdk:ro")
  else
    log "WARNING: xCore SDK not found at $SDK_HOST/Release/linux — arm_backend:=rokae will"
    log "         FAIL to connect and fall back to MOCK (watch the backend chip in the console)."
  fi

  # Operator CAD library — shared with the console so the planner finds the same parts.
  PARTS_MOUNTS=(); PARTS_ENV=()
  if [ -d "$PARTS_HOST" ]; then
    PARTS_MOUNTS=(-v "$PARTS_HOST:/parts:ro"); PARTS_ENV=(-e QC_PARTS_DIR=/parts)
  else
    log "note: parts dir $PARTS_HOST not found — planner falls back to repo config/cad."
  fi

  log "starting ROS graph in container '$CONTAINER' (building changed packages first)…"
  docker run -d --rm --name "$CONTAINER" --net=host \
    -v "$REPO/ros2_ws:/ros2_ws" \
    -v "$REPO:/repo:ro" \
    "${GMSH_MOUNTS[@]}" \
    "${SDK_MOUNTS[@]}" \
    "${PARTS_MOUNTS[@]}" \
    -v qc_humble_build:/ros2_ws/build \
    -v qc_humble_install:/ros2_ws/install \
    -e QC_REPO_ROOT=/repo \
    "${PARTS_ENV[@]}" \
    -e PYTHONPATH=/scratch/pydeps \
    -e LD_LIBRARY_PATH=/scratch/pydeps \
    qc-humble \
    bash -lc "source /opt/ros/humble/setup.bash && cd /ros2_ws && \
      { colcon build --packages-select qc_msgs qc_moveit_config sr5_arm_driver \
          path_planner movement_driver task_manager scanner_driver inspection qc_bringup \
          >/tmp/qc_build.log 2>&1 || echo '[qc-stack] colcon build had errors (using previous install) — see /tmp/qc_build.log'; } && \
      source install/setup.bash && exec $ROS_LAUNCH" >/dev/null

  log "waiting for rosbridge on :9090…"
  for _ in $(seq 1 180); do
    port_up 9090 && { GRAPH_UP=1; break; }
    docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { log "ROS graph exited early — inspect: docker logs $CONTAINER"; break; }
    sleep 1
  done
  [ "$GRAPH_UP" = 1 ] && log "rosbridge is up (:9090) — ROS graph ready." \
                      || log "rosbridge did not come up in time — the console will show 'ROS graph down'."
else
  log "docker not found — starting the console only (arm will show 'ROS graph down')."
fi

# ---------------------------------------------------------------------------
# 2 + 3. Console server + browser.
# ---------------------------------------------------------------------------
# Open the browser once :8000 is actually listening (probe, don't race the bind).
( for _ in $(seq 1 60); do port_up 8000 && break; sleep 0.5; done
  xdg-open "$URL" >/dev/null 2>&1 || sensible-browser "$URL" >/dev/null 2>&1 || true ) &

if port_up 8000; then
  # A console is already serving :8000 — don't start a second one. Keep this
  # terminal (and the ROS graph) alive until the graph stops or you Ctrl-C.
  log "console already running on :8000 — opened the browser to it."
  log "leave this window open to keep the ROS graph running (Ctrl-C to stop it)."
  while [ "$GRAPH_UP" = 1 ] && docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; do sleep 5; done
else
  log "starting the console at $URL  (Ctrl-C stops the whole stack)…"
  export QC_JOG_SPEED=30
  "$REPO/scripts/run_console.sh"   # foreground; NOT exec, so the EXIT trap still tears the graph down
fi
