"""Quick check that robot_bridge talks to the project's arm driver backend and
that its motion commands are correctly gated.

Run:  python3 backend/test_robot_bridge.py                 # tries the real arm
      QC_ROBOT_IP=10.255.255.1 python3 backend/test_robot_bridge.py   # force "unreachable"

There is no mock backend: if the SR5 isn't reachable the bridge stays honestly
disconnected, and every motion command must refuse with "not connected".
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from robot_bridge import BRIDGE

st = BRIDGE.connect()
print("connect  -> kind=%s connected=%s note=%r" % (st["kind"], st["connected"], st["note"]))
print("info     -> %s  power=%s mode=%s sdk=%s"
      % (st.get("info"), st.get("power"), st.get("mode"), st.get("sdkVersion")))
j = BRIDGE.joints()
print("joints   -> kind=%s n=%s" % (j.get("kind"), len(j.get("joints", []))))
if j.get("joints"):
    print("joint[0] -> %s" % j["joints"][0])

# Motion commands: each returns a status dict tagged {ok, action, error?}.
# When disconnected they must all refuse rather than reaching the SDK.
print("\nmotion commands:")
for label, call in [
    ("power on",    lambda: BRIDGE.set_power(True)),
    ("drag on",     lambda: BRIDGE.set_drag(True)),
    ("stop",        lambda: BRIDGE.stop()),
    ("estop",       lambda: BRIDGE.estop()),
    ("clear_alarm", lambda: BRIDGE.clear_alarm()),
    ("jog 6-joint", lambda: BRIDGE.move_joints([0, 0, 0, 0, 0, 0])),
]:
    r = call()
    print("  %-12s -> ok=%s action=%s error=%r" % (label, r.get("ok"), r.get("action"), r.get("error")))
    if not BRIDGE.status()["connected"]:
        assert r.get("ok") is False, f"{label} should refuse while disconnected"
        assert r.get("error"), f"{label} should give a reason while disconnected"

# Bad jog targets are rejected before any SDK call.
r = BRIDGE.move_joints(["not", "a", "number"])
print("  bad jog      -> ok=%s error=%r" % (r.get("ok"), r.get("error")))
assert r.get("ok") is False

BRIDGE.disconnect()
print("OK")
