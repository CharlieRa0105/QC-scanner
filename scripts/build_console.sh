#!/usr/bin/env bash
# Build the QC Scanner operator console as a single-file Linux desktop app.
#
# Produces ./dist/qc-console (double-click to launch a native window; falls
# back to the default browser if no GUI webview backend is present).
#
# IMPORTANT — Python version: the ROKAE xCore SDK ships CPython builds for
# 3.8-3.12 ONLY, so the frozen app must bundle a Python in that range or it
# can never talk to the arm (it would fall back to "Offline"). We build with
# the pinned 3.12 env at .venv312, NOT the system python3 (3.14 here).
#
# One-time setup (uv-managed 3.12 + planner + build tooling):
#   uv venv --python 3.12 .venv312
#   uv pip install --python .venv312/bin/python numpy gmsh pywebview pyinstaller
#
# NOTE: PyInstaller does not cross-compile. This builds a LINUX binary on
# Linux. A Windows .exe must be built by running qc_console.spec on Windows.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
PY=".venv312/bin/pyinstaller"
[ -x "$PY" ] || { echo "error: $PY not found — create the 3.12 venv first (see header)"; exit 1; }
exec "$PY" qc_console.spec --noconfirm
