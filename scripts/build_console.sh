#!/usr/bin/env bash
# Build the QC Scanner operator console as a single-file Linux desktop app.
#
# Produces ./dist/qc-console (double-click to launch a native window; falls
# back to the default browser if no GUI webview backend is present).
#
# One-time setup (venv with system numpy/gmsh + build tooling):
#   python3 -m venv --system-site-packages .venv
#   .venv/bin/pip install pywebview pyinstaller
#
# NOTE: PyInstaller does not cross-compile. This builds a LINUX binary on
# Linux. A Windows .exe must be built by running qc_console.spec on Windows.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
PY=".venv/bin/pyinstaller"
[ -x "$PY" ] || { echo "error: $PY not found — create the venv first (see header)"; exit 1; }
exec "$PY" qc_console.spec --noconfirm
