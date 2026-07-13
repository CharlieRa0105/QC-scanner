#!/usr/bin/env python3
"""
app.py

Desktop entry point for the QC Scanner operator console.

Wraps the existing backend (backend/server.py) so the console runs as a
double-click desktop app instead of "open a browser at localhost":

  1. Starts the stdlib HTTP backend on a free 127.0.0.1 port in a daemon
     thread (same server that serves gui/ and the real /api/plan endpoint).
  2. Opens a native desktop window (pywebview) pointed at that local URL.
  3. Tears the server down when the window closes.

Fallbacks so it still works in constrained environments:
  * If pywebview has no GUI backend available (e.g. missing webkit2gtk),
    it falls back to opening the system default browser.
  * QC_HEADLESS=1 (or --headless) runs the server only, no window -- used
    for build/bundle verification and as a last-resort "just serve it".

Frozen (PyInstaller) vs source: when frozen, data files (gui/, config/cad,
libs/, scripts/) are unpacked next to the executable in sys._MEIPASS; the
backend resolves paths from there. See qc_console.spec.
"""

import os
import sys
import threading
from http.server import HTTPServer


def _base_dir():
    """Directory that holds gui/, config/, libs/, scripts/.

    When running from a PyInstaller one-file build, data is unpacked into
    sys._MEIPASS; otherwise it's just this file's directory.
    """
    if getattr(sys, "frozen", False):
        return sys._MEIPASS  # type: ignore[attr-defined]
    return os.path.dirname(os.path.abspath(__file__))


BASE_DIR = _base_dir()

# The backend imports the planner via repo-relative paths; make sure both the
# base dir and its scripts/ are importable before importing the server module.
sys.path.insert(0, BASE_DIR)
sys.path.insert(0, os.path.join(BASE_DIR, "scripts"))

# backend/server.py resolves GUI_DIR/CAD_DIR from its own location
# (<repo>/backend/..). In a frozen build the tree is preserved under _MEIPASS,
# so importing it as a module still resolves correctly.
sys.path.insert(0, os.path.join(BASE_DIR, "backend"))
# Pin the backend's data root to our base dir so gui/ and config/cad resolve
# whether we're running from source or from a frozen one-file bundle.
os.environ.setdefault("QC_BASE_DIR", BASE_DIR)
import server as backend  # noqa: E402  (path set up above)


def start_server():
    """Launch the HTTP backend on a free localhost port; return (httpd, url)."""
    # Port 0 lets the OS pick any free port, so two launches never collide.
    httpd = HTTPServer(("127.0.0.1", 0), backend.QCRequestHandler)
    port = httpd.server_address[1]
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd, f"http://127.0.0.1:{port}/"


def main():
    headless = os.environ.get("QC_HEADLESS") == "1" or "--headless" in sys.argv

    httpd, url = start_server()
    print(f"QC Scanner console backend: {url}", flush=True)

    if headless:
        # Server-only mode: keep serving until Ctrl-C. Used to verify a build
        # (the window needs a desktop session; the server does not).
        print("headless mode — serving only, press Ctrl-C to stop")
        try:
            threading.Event().wait()
        except KeyboardInterrupt:
            pass
        httpd.shutdown()
        return

    # Preferred: a native desktop window via pywebview.
    try:
        import webview
        webview.create_window("QC Scanner — Scan Cell 01", url,
                              width=1440, height=900, min_size=(1100, 720))
        webview.start()  # blocks until the window is closed
    except Exception as e:  # noqa: BLE001 -- no GUI backend, bad display, etc.
        # Fallback: open the default browser and keep the server alive.
        print(f"native window unavailable ({e}); opening in default browser")
        import webbrowser
        webbrowser.open(url)
        try:
            threading.Event().wait()
        except KeyboardInterrupt:
            pass

    httpd.shutdown()


if __name__ == "__main__":
    main()
