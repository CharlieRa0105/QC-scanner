#!/usr/bin/env bash
# =============================================================================
# QC Scanner — one-shot setup for a fresh machine.
#
#   git clone https://github.com/CharlieRa0105/QC-scanner.git
#   cd QC-scanner
#   ./setup.sh                 # full setup: env + deps + build the qc-console app
#   ./setup.sh --all           # also build the qc-humble RViz Docker image
#   ./setup.sh --no-binary      # env + deps only (run from source, no packaged app)
#
# What it does (idempotent — safe to re-run):
#   1. Ensures `uv` is available (prints the install command if not).
#   2. Installs a standalone CPython 3.12 via uv.  <-- REQUIRED: the ROKAE xCore
#      SDK only ships 3.8-3.12 builds; the console can't talk to the arm on 3.13+.
#   3. Creates .venv312 and installs the Python deps (numpy, gmsh, pywebview,
#      pyinstaller).
#   4. Locates the ROKAE xCore SDK (needed for the physical arm) and reports
#      how to supply it if it's missing.
#   5. Builds the single-file desktop app -> dist/qc-console  (skip: --no-binary).
#   6. (--with-docker / --all) Builds the qc-humble RViz container (large).
#
# Nothing here is committed to git (venv, binary, Docker image, SDK are all
# gitignored / external) — that's why this script exists.
# =============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

# ---- options ----------------------------------------------------------------
WITH_DOCKER=0
WITH_BINARY=1          # build the desktop app by default — it IS the console
for arg in "$@"; do
  case "$arg" in
    --with-docker) WITH_DOCKER=1 ;;
    --no-binary)   WITH_BINARY=0 ;;
    --with-binary) WITH_BINARY=1 ;;
    --all)         WITH_DOCKER=1; WITH_BINARY=1 ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^# \{0,1\}//' | head -30
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
chmod +x setup.sh scripts/*.sh docker/*.sh 2>/dev/null || true

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

# ---- 3. venv + deps ---------------------------------------------------------
say "Creating ${VENV} and installing Python dependencies"
# Reuse a healthy venv, but rebuild one that's missing or broken. A venv copied
# or moved between paths has stale absolute shebangs, so verify it actually runs.
if [ -x "${VENV}/bin/python" ] && "${VENV}/bin/python" -c "pass" >/dev/null 2>&1; then
  ok "${VENV} already exists and works — reusing"
else
  [ -e "${VENV}" ] && warn "${VENV} missing/broken (e.g. moved from another path) — rebuilding"
  uv venv --clear --python "${PY_VERSION}" "${VENV}"
fi
uv pip install --python "${VENV}/bin/python" numpy gmsh pywebview pyinstaller
ok "${VENV} ready with numpy, gmsh, pywebview, pyinstaller"

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

# ---- 5. Docker image (optional) --------------------------------------------
if [ "$WITH_DOCKER" = 1 ]; then
  say "Building the qc-humble RViz container (large — ~5.6 GB, needs network)"
  if ! command -v docker >/dev/null 2>&1; then
    warn "docker not installed — skipping. RViz won't be available."
  elif [ ! -f docker/build.sh ]; then
    warn "docker/build.sh missing — skipping."
  elif bash docker/build.sh; then
    ok "qc-humble image built"
  else
    warn "docker build failed — see output above. RViz won't be available."
  fi
else
  warn "Skipping Docker image (RViz). Add --with-docker or --all to build it."
fi

# ---- 6. Desktop binary (optional) ------------------------------------------
if [ "$WITH_BINARY" = 1 ]; then
  say "Building the QC console desktop app (dist/qc-console)"
  bash scripts/build_console.sh || die "build failed — see output above"
  ok "dist/qc-console built — double-click or run ./dist/qc-console"
else
  warn "Skipping the packaged binary (--no-binary). Run from source instead:"
  warn "  scripts/run_console.sh"
fi

# ---- done -------------------------------------------------------------------
say "Setup complete."
cat <<EOF

  Run the console (from source, connects to the arm):
      scripts/run_console.sh          →  http://127.0.0.1:8000

  Packaged desktop app (if built with --with-binary/--all):
      ./dist/qc-console

  RViz arm view (if built with --with-docker/--all):
      docker/run_arm.sh

  Reminder: the arm must be on the network at 192.168.2.160 (or edit the IP in
  the console header). Motion is enabled by default — set QC_ALLOW_MOTION=0 for
  a read-only session.
EOF
