#!/usr/bin/env bash
# Install a proper double-clickable launcher for the QC Scanner console.
#
# WHY: ./dist/qc-console is a raw executable — running it from a terminal works,
# but double-clicking a bare binary in a file manager does nothing (file
# managers don't execute ELF binaries). This registers a .desktop entry with an
# absolute path, so the console shows up in the applications menu and launches
# on double-click.
#
# Run once (re-run if you move the repo):  scripts/install_app.sh
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$REPO/dist/qc-console"
[ -x "$BIN" ] || { echo "error: $BIN not found — build it first: scripts/build_console.sh"; exit 1; }

ICON="$REPO/gui/assets/logomark.svg"
APPS="$HOME/.local/share/applications"
DEST="$APPS/qc-scanner-console.desktop"
mkdir -p "$APPS"

cat > "$DEST" <<EOF
[Desktop Entry]
Type=Application
Name=QC Scanner Console
Comment=Dexory 3D Scan QC operator interface (Scan Cell 01)
Exec=$BIN
Icon=$ICON
Terminal=false
Categories=Utility;
StartupNotify=true
EOF

chmod +x "$DEST"
# Mark trusted / refresh the menu database where those tools exist.
gio set "$DEST" metadata::trusted true 2>/dev/null || true
update-desktop-database "$APPS" 2>/dev/null || true

echo "installed: $DEST"
echo "  -> launches: $BIN"
echo "Find 'QC Scanner Console' in your applications menu, or double-click the"
echo "entry. (From a terminal, '$BIN' also works directly.)"
