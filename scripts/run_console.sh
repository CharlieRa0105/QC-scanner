#!/usr/bin/env bash
# Launch the QC Scanner operator console (backend + GUI).
# Serves gui/ and the real /api/plan endpoint. Stdlib Python only.
#
# Usage:
#   scripts/run_console.sh              # http://127.0.0.1:8000
#   scripts/run_console.sh --port 9000
#   scripts/run_console.sh --host 0.0.0.0
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$REPO_ROOT/backend/server.py" "$@"
