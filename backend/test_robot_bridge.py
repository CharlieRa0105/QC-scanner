"""Quick check that robot_bridge is powered by the project's arm driver backend.
Run:  QC_ROBOT_MODE=mock python3 backend/test_robot_bridge.py
      QC_ROBOT_MODE=auto python3 backend/test_robot_bridge.py   # tries the real arm
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
BRIDGE.disconnect()
print("OK")
