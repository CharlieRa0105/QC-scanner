#!/usr/bin/env bash
# Desktop-launcher wrapper for the QC Scanner console.
# Opens the browser at the console URL once the server has had a moment to bind,
# then runs run_console.sh in the foreground (this terminal shows the server
# logs; closing it stops the console).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

URL="http://127.0.0.1:8000/"
# Open the browser ONCE the server is actually listening on :8000 (a fixed sleep
# raced the cold-start server-bind, so the browser opened to a connection error
# and looked like "it didn't open the site"). Probe the port with bash's /dev/tcp
# (no curl dependency); give up after ~30 s and open anyway.
(
  for _ in $(seq 1 60); do
    (exec 3<>/dev/tcp/127.0.0.1/8000) 2>/dev/null && { exec 3>&- 3<&-; break; }
    sleep 0.5
  done
  xdg-open "$URL" >/dev/null 2>&1 || sensible-browser "$URL" >/dev/null 2>&1 || true
) &

export QC_ALLOW_SCAN_TRACE=1
export QC_JOG_SPEED=30
exec "$HERE/run_console.sh" "$@"
