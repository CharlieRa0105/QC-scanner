#!/usr/bin/env bash
# Launch the QC Scanner operator console (backend + GUI).
# Serves gui/ and the real API (/api/robot/*, /api/parts, /api/scan/*).
#
# IMPORTANT — Python version: the ROKAE xCore SDK ships CPython builds for
# 3.8-3.12 ONLY. The console must run under one of those or it cannot talk to
# the arm. This repo keeps a pinned 3.12 env at .venv312 (created by ./setup.sh
# with `uv venv --python 3.12 .venv312`). The console backend is stdlib-only, so
# the venv exists purely to pin that interpreter — there are no pip deps to
# install. The system python3 here is 3.14, which silently fails to load the SDK
# (the import falls through to an empty namespace dir) and leaves the arm
# stuck "Offline" — that was the "mock IP still connected" symptom.
#
# Usage:
#   scripts/run_console.sh              # http://127.0.0.1:8000
#   scripts/run_console.sh --port 9000
#   scripts/run_console.sh --host 0.0.0.0
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Pick an SDK-compatible interpreter: prefer the pinned 3.12 venv.
if [ -x "$REPO_ROOT/.venv312/bin/python" ]; then
  PY="$REPO_ROOT/.venv312/bin/python"
else
  PY="python3"
  echo "warning: .venv312 not found — using $(command -v python3)." >&2
  echo "         If that is Python 3.13+ the ROKAE SDK will NOT load and the" >&2
  echo "         arm stays Offline. See this script's header to create .venv312." >&2
fi

# Point the robot bridge at the xCore SDK (contains Release/linux/*.so). Honour
# an existing QC_SDK_PATH; otherwise pick the first location that actually has it.
if [ -z "${QC_SDK_PATH:-}" ]; then
  for cand in "$HOME/rokaeProject" "$HOME/rokae_sdk"; do
    if [ -d "$cand/Release/linux" ]; then QC_SDK_PATH="$cand"; export QC_SDK_PATH; break; fi
  done
fi

exec "$PY" "$REPO_ROOT/backend/server.py" "$@"
