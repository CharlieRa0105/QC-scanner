# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec for the QC Scanner operator console (Linux desktop build).

Produces a single-file executable `dist/qc-console` that:
  * bundles the GUI (gui/), the test CAD (config/cad/), and the planner
    source (libs/, scripts/, backend/) as data;
  * bundles the heavy third-party deps the planner needs (numpy, gmsh)
    including gmsh's compiled shared library;
  * bundles pywebview for the native window (falls back to the system
    browser at runtime if no GUI backend is present).

Build (from the repo root, inside the 3.12 venv — the ROKAE SDK only has
3.8-3.12 builds, so the bundled interpreter must be in that range):
    .venv312/bin/pyinstaller qc_console.spec --noconfirm
    # or simply: scripts/build_console.sh
Run:
    ./dist/qc-console                 # native window (or browser fallback)
    QC_HEADLESS=1 ./dist/qc-console   # server only (no window)

NOTE: PyInstaller does not cross-compile. This spec builds a Linux binary on
Linux. A Windows .exe would need to be built by running this same spec on a
Windows machine.
"""

from PyInstaller.utils.hooks import collect_all

# gmsh ships a compiled shared library that PyInstaller's import analysis
# won't find on its own -- collect_all grabs its data, binaries, and modules.
gmsh_datas, gmsh_bins, gmsh_hidden = collect_all("gmsh")
# numpy has official PyInstaller hooks, but collect_all is belt-and-braces for
# the frozen import to resolve cleanly.
np_datas, np_bins, np_hidden = collect_all("numpy")
# pywebview + its platform backend.
wv_datas, wv_bins, wv_hidden = collect_all("webview")

# The planner source is imported at runtime via sys.path (see app.py), not
# followed by PyInstaller's static analysis, so ship it as data files. The
# GUI and CAD are pure data. (source_dir, dest_dir_in_bundle)
project_datas = [
    ("gui", "gui"),
    ("config/cad", "config/cad"),
    ("libs", "libs"),
    ("scripts", "scripts"),
    ("backend", "backend"),
]

a = Analysis(
    ["app.py"],
    pathex=["libs", "scripts", "backend", "ros2_ws/src/sr5_arm_driver"],
    binaries=gmsh_bins + np_bins + wv_bins,
    datas=project_datas + gmsh_datas + np_datas + wv_datas,
    hiddenimports=gmsh_hidden + np_hidden + wv_hidden + [
        # imported at runtime through sys.path; name them so they're bundled.
        "server",
        "robot_bridge",
        "sr5_arm_driver",
        "sr5_arm_driver.backends",
        "plan_path",
        "libs.path_planning.cad_loader",
        "libs.path_planning.normal_estimation",
        "libs.path_planning.waypoint_generator",
        "libs.path_planning.incidence_cone_modifier",
    ],
    hookspath=[],
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="qc-console",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=True,          # keep a console so backend logs / errors are visible
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
