#!/usr/bin/env bash
# Desktop-launcher wrapper for the QC Scanner console.
# Opens the browser at the console URL once the server has had a moment to bind,
# then runs run_console.sh in the foreground (this terminal shows the server
# logs; closing it stops the console).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

URL="http://127.0.0.1:8000/"
( sleep 2; xdg-open "$URL" >/dev/null 2>&1 || true ) &

export QC_ALLOW_SCAN_TRACE=1
export QC_JOG_SPEED=30
exec "$HERE/run_console.sh" "$@"
