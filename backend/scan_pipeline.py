"""
scan_pipeline.py

Scan-job lifecycle + results store for the operator console.

Status (2026-07-14):
    This is a STUB pipeline. The arm connection is real (robot_bridge.py), but
    the subsystems a scan actually needs -- scanner capture (MIRACO), point-cloud
    registration, and the QC quality gate -- do not exist in the codebase yet.

    So this module does NOT fabricate scan data. It provides the real plumbing
    the UI drives against:

      * a ScanStore   -- JSON-file-backed results store (starts empty)
      * a ScanJob     -- the scan lifecycle state machine
      * ScanManager   -- owns the current job + the store, one per process

    A scan run walks the real phases (execute -> capture -> register -> quality
    gate) and marks each unbuilt phase honestly. It finishes with a persisted
    record whose status is "incomplete" and whose metrics are null, with notes
    naming exactly which subsystems are missing. When those land, fill the
    phases in and the same UI + store keep working.

    IMPORTANT: a scan does NOT command arm motion. The console's motion controls
    (robot_bridge) are for direct operator jogging only.

Env:
    QC_DATA_DIR   default <repo>/data     where scans.json is written
"""

import json
import os
import threading
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(os.environ.get("QC_BASE_DIR") or Path(__file__).resolve().parent.parent)
DATA_DIR = Path(os.environ.get("QC_DATA_DIR") or (REPO_ROOT / "data"))
SCANS_FILE = DATA_DIR / "scans.json"

# The phases a real scan will move through. Each is marked implemented=False
# until its subsystem exists; the job reports them so the UI can show real
# progress instead of a fabricated timer.
PHASES = [
    {"key": "execute",  "label": "Execute trajectory",  "implemented": False},
    {"key": "capture",  "label": "Capture point cloud",  "implemented": False},
    {"key": "register", "label": "Register + merge",     "implemented": False},
    {"key": "quality",  "label": "Quality gate",         "implemented": False},
]

_MISSING_NOTES = [
    "Scanner capture not implemented — no point cloud was captured.",
    "Point-cloud registration not implemented.",
    "QC quality gate not evaluated — pass/fail undetermined.",
]


def _now_iso():
    return datetime.now().isoformat(timespec="seconds")


def _new_scan_id():
    # SCAN-YYYYMMDD-HHMMSS — unique enough for a single cell at human cadence.
    return "SCAN-" + datetime.now().strftime("%Y%m%d-%H%M%S")


def _empty_metrics():
    """The real metrics schema, all null until the QC pipeline fills it."""
    return {
        "meanDeviationMm": None,
        "rmseMm": None,
        "stdMm": None,
        "coveragePct": None,
        "pointCount": None,
    }


class ScanStore:
    """JSON-file-backed list of scan records. Thread-safe. Starts empty."""

    def __init__(self, path=SCANS_FILE):
        self._path = Path(path)
        self._lock = threading.Lock()

    def _read_locked(self):
        if not self._path.is_file():
            return []
        try:
            with self._path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, list) else []
        except (ValueError, OSError):
            # A corrupt/partial file must not take the whole console down;
            # treat it as empty (the next add() rewrites it cleanly).
            return []

    def _write_locked(self, records):
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._path.with_suffix(".json.tmp")
        with tmp.open("w", encoding="utf-8") as f:
            json.dump(records, f, indent=2)
        tmp.replace(self._path)  # atomic swap so a reader never sees half a file

    def list(self):
        with self._lock:
            return self._read_locked()

    def get(self, scan_id):
        with self._lock:
            for r in self._read_locked():
                if r.get("scanId") == scan_id:
                    return r
        return None

    def add(self, record):
        with self._lock:
            records = self._read_locked()
            records.append(record)
            self._write_locked(records)
        return record


class ScanJob:
    """One scan run. Because capture/QC don't exist yet, the job completes
    immediately with an honest 'incomplete' record rather than pretending to
    work. The phase list still reports which steps ran vs. are unbuilt."""

    def __init__(self, part_id):
        self.scan_id = _new_scan_id()
        self.part_id = part_id or ""
        self.started_at = _now_iso()
        self.finished_at = None
        self.state = "running"          # running | done | error | stopped
        self.record = None

    def phases(self):
        # Every phase is not-yet-implemented (its subsystem doesn't exist), so
        # each is reported honestly rather than faked as done.
        out = []
        for p in PHASES:
            status = "not_implemented" if not p["implemented"] else "pending"
            out.append({**p, "status": status})
        return out

    def finalize(self, store, state="done"):
        """Complete the job and persist an honest record. Idempotent."""
        if self.record is not None:
            return self.record
        self.state = state
        self.finished_at = _now_iso()
        self.record = {
            "scanId": self.scan_id,
            "partId": self.part_id,
            "startedAt": self.started_at,
            "finishedAt": self.finished_at,
            "status": "stopped" if state == "stopped" else "incomplete",
            "result": None,               # pass|rescan|flagged once QC exists
            "metrics": _empty_metrics(),
            "toleranceMm": None,
            "cloudFile": None,
            "phases": self.phases(),
            "notes": (["Scan stopped by operator."] if state == "stopped"
                      else list(_MISSING_NOTES)),
        }
        store.add(self.record)
        return self.record

    def status(self):
        return {
            "scanId": self.scan_id,
            "partId": self.part_id,
            "state": self.state,
            "startedAt": self.started_at,
            "finishedAt": self.finished_at,
            "phases": self.phases(),
            "record": self.record,
        }


class ScanManager:
    """Owns the store and the current/last job for the process."""

    def __init__(self):
        self.store = ScanStore()
        self._lock = threading.Lock()
        self._job = None

    def start(self, part_id):
        with self._lock:
            if self._job is not None and self._job.state == "running":
                return {"ok": False, "error": "a scan is already running",
                        "status": self._job.status()}
            job = ScanJob(part_id)
            # No capture/QC to run, so finalize straight away with an honest,
            # persisted 'incomplete' record instead of a fake progress timer.
            job.finalize(self.store, state="done")
            self._job = job
            return {"ok": True, "status": job.status()}

    def stop(self):
        with self._lock:
            if self._job is None:
                return {"ok": False, "error": "no scan to stop"}
            if self._job.state == "running":
                self._job.finalize(self.store, state="stopped")
            return {"ok": True, "status": self._job.status()}

    def status(self):
        with self._lock:
            if self._job is None:
                return {"state": "idle", "record": None, "phases": [
                    {**p, "status": "pending"} for p in PHASES]}
            return self._job.status()


# One manager per process, shared across requests.
MANAGER = ScanManager()
