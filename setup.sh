#!/usr/bin/env bash
# =============================================================================
# QC Scanner — one-shot setup for a fresh machine.
#
#   git clone https://github.com/CharlieRa0105/QC-scanner.git
#   cd QC-scanner
#   ./setup.sh                 # prepare the env to run the web console from source
#
# What it does (idempotent — safe to re-run):
#   1. Ensures `uv` is available (prints the install command if not).
#   2. Installs a standalone CPython 3.12 via uv.  <-- REQUIRED: the ROKAE xCore
#      SDK only ships 3.8-3.12 builds; the console can't talk to the arm on 3.13+.
#   3. Creates the .venv312 interpreter the console runs under.
#   4. Locates the ROKAE xCore SDK (needed for the physical arm) and reports
#      how to supply it if it's missing.
#
# The console is a web app served from source — start it with
# scripts/run_console.sh (http://127.0.0.1:8000). There is no packaged binary.
#
# Nothing here is committed to git (venv, SDK are gitignored / external) —
# that's why this script exists.
# =============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

# ---- options ----------------------------------------------------------------
for arg in "$@"; do
  case "$arg" in
    -h|--help)
      grep '^#' "$0" | sed 's/^# \{0,1\}//' | head -25
      exit 0 ;;
    *) echo "unknown option: $arg (try --help)" >&2; exit 2 ;;
  esac
done

# ---- pretty output ----------------------------------------------------------
say()  { printf '\n\033[1;36m==>\033[0m %s\n' "$*"; }
ok()   { printf '  \033[1;32m✓\033[0m %s\n' "$*"; }
warn() { printf '  \033[1;33m!\033[0m %s\n' "$*"; }
die()  { printf '\n\033[1;31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }

PY_VERSION="3.12"
VENV=".venv312"

# ---- 0. make repo scripts executable ---------------------------------------
# git may deliver these without the +x bit on a fresh clone; fix it so the
# documented `scripts/run_console.sh` etc. work directly.
chmod +x setup.sh scripts/*.sh 2>/dev/null || true

# ---- 1. uv ------------------------------------------------------------------
say "Checking for uv (Python toolchain manager)"
export PATH="$HOME/.local/bin:$PATH"
if ! command -v uv >/dev/null 2>&1; then
  die "uv is not installed. Install it once with:
       curl -LsSf https://astral.sh/uv/install.sh | sh
     then re-run ./setup.sh  (uv is user-space; no root needed)."
fi
ok "uv found: $(uv --version)"

# ---- 2. Python 3.12 ---------------------------------------------------------
say "Installing CPython ${PY_VERSION} (SDK requires 3.8-3.12)"
uv python install "${PY_VERSION}"
ok "Python ${PY_VERSION} available"

# ---- 3. venv ----------------------------------------------------------------
say "Creating ${VENV} (the Python 3.12 interpreter the console runs under)"
# Reuse a healthy venv, but rebuild one that's missing or broken. A venv copied
# or moved between paths has stale absolute shebangs, so verify it actually runs.
if [ -x "${VENV}/bin/python" ] && "${VENV}/bin/python" -c "pass" >/dev/null 2>&1; then
  ok "${VENV} already exists and works — reusing"
else
  [ -e "${VENV}" ] && warn "${VENV} missing/broken (e.g. moved from another path) — rebuilding"
  uv venv --clear --python "${PY_VERSION}" "${VENV}"
fi
ok "${VENV} ready"

# ---- 3b. planner Python deps ------------------------------------------------
# The console BACKEND (server.py) is Python-stdlib only, but the PATH PLANNER
# CLIs it shells out to (scripts/plan_*.py -> libs/path_planning) need these:
#   numpy, gmsh              CAD load + maths
#   trimesh + scipy/networkx/rtree   mesh slicing & down-raycast (lawnmower/contour planners)
say "Installing path-planner dependencies into ${VENV}"
uv pip install --python "${VENV}/bin/python" \
  numpy gmsh trimesh scipy networkx rtree
ok "planner deps installed"

# ---- 4. ROKAE SDK -----------------------------------------------------------
say "Locating the ROKAE xCore SDK (needed to talk to the physical arm)"
SDK_FOUND=""
for cand in "${QC_SDK_PATH:-}" "$REPO_ROOT/vendor/rokae_sdk" "$HOME/rokaeProject" "$HOME/rokae_sdk"; do
  [ -n "$cand" ] || continue
  if [ -d "$cand/Release/linux" ] && ls "$cand"/Release/linux/xCoreSDK_python.cpython-312*.so >/dev/null 2>&1; then
    SDK_FOUND="$cand"; break
  fi
done
if [ -n "$SDK_FOUND" ]; then
  ok "SDK found at: $SDK_FOUND"
else
  warn "ROKAE SDK not found. The console will run but stay Offline (no arm)."
  warn "Supply it in ONE of these ways, then re-run setup:"
  warn "  • copy the SDK tree to ~/rokaeProject  (must contain Release/linux/*.so)"
  warn "  • or set QC_SDK_PATH=/path/to/sdk before launching the console"
  warn "  • it must include a CPython 3.12 build (xCoreSDK_python.cpython-312*.so)"
fi

# ---- done -------------------------------------------------------------------
say "Setup complete."
cat <<EOF

  Run the console (web app from source, connects to the arm):
      scripts/run_console.sh          →  http://127.0.0.1:8000

  Reminder: the arm must be on the network at 192.168.2.160 (or edit the IP in
  the console header). Motion is enabled by default — set QC_ALLOW_MOTION=0 for
  a read-only session.
EOF
