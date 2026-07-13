"""Dexory Robot Teach (ROS 2 edition).

A Tkinter GUI that is ITSELF a ROS 2 node. It never touches hardware directly --
it publishes command messages to and subscribes to state topics from the
sr5_arm_driver and slider_driver nodes. Teaches waypoints for BOTH the arm
(6 joints) and the slider (linear position), then replays them.

  Command topics : /armCMD, /railCMD   (std_msgs/Float64MultiArray)
  State topics   : /armPos, /railPos (sensor_msgs/JointState)
                   /arm/status, /rail/status             (std_msgs/String)
  Services       : {/arm,/slider}/set_power, set_drag, stop, home; /arm/clear_alarm

Set DEXORY_TEACH_SELFTEST=1 to run a scripted teach+replay and exit (headless CI).
"""

import os
import sys
import json
import math
import time
import threading
import tkinter as tk
from tkinter import ttk, filedialog, font as tkfont

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray, Bool
from std_srvs.srv import SetBool, Trigger

# --- Dexory branding (identical to the standalone app) ---
NAVY = "#0B1A48"; LIME = "#CBFE34"; LIME_DK = "#7A9A1A"; DARK = "#1A1A1A"
WHITE = "#FFFFFF"; BG = "#F5F6F8"; GREY = "#E3E6EA"; MUTED = "#6B7280"; DANGER = "#D64545"

STALE_S = 1.5   # a driver is "offline" if no state seen within this many seconds


def _pkg_images():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("dexory_teach_ros"), "images")
    except Exception:  # noqa: BLE001
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")


# ==========================================================================
# ROS 2 node
# ==========================================================================
class TeachNode(Node):
    def __init__(self, n_joints=6):
        super().__init__("dexory_teach_gui")
        self.n = n_joints
        # live state (updated by subscription callbacks on the executor thread)
        self.arm_joints = [0.0] * n_joints
        self.arm_status = "off"
        self.slider_pos = 0.0
        self.slider_status = "off"
        self.arm_rx = 0.0
        self.slider_rx = 0.0
        self.drag_button = False
        self.arm_backend = ""
        self.slider_backend = ""

        self.pub_arm = self.create_publisher(Float64MultiArray, "/armCMD", 10)
        self.pub_slider = self.create_publisher(Float64MultiArray, "/railCMD", 10)
        self.pub_arm_connect = self.create_publisher(String, "/arm/connect", 10)
        self.pub_slider_connect = self.create_publisher(String, "/rail/connect", 10)

        self.create_subscription(JointState, "/armPos", self._cb_arm_js, 10)
        self.create_subscription(String, "/arm/status", self._cb_arm_status, 10)
        self.create_subscription(Bool, "/arm/drag_button", self._cb_button, 10)
        self.create_subscription(String, "/arm/backend", self._cb_arm_backend, 10)
        self.create_subscription(JointState, "/railPos", self._cb_slider_js, 10)
        self.create_subscription(String, "/rail/status", self._cb_slider_status, 10)
        self.create_subscription(String, "/rail/backend", self._cb_slider_backend, 10)

        self.cli = {
            "arm/set_power": self.create_client(SetBool, "/arm/set_power"),
            "arm/set_drag": self.create_client(SetBool, "/arm/set_drag"),
            "arm/clear_alarm": self.create_client(Trigger, "/arm/clear_alarm"),
            "arm/stop": self.create_client(Trigger, "/arm/stop"),
            "arm/home": self.create_client(Trigger, "/arm/home"),
            "arm/sim_drag_button": self.create_client(SetBool, "/arm/sim_drag_button"),
            "rail/set_power": self.create_client(SetBool, "/rail/set_power"),
            "rail/set_drag": self.create_client(SetBool, "/rail/set_drag"),
            "rail/stop": self.create_client(Trigger, "/rail/stop"),
            "rail/home": self.create_client(Trigger, "/rail/home"),
        }

    # -- subscription callbacks --
    def _cb_arm_js(self, msg):
        if msg.position:
            self.arm_joints = list(msg.position)
        self.arm_rx = time.time()

    def _cb_arm_status(self, msg):
        self.arm_status = msg.data

    def _cb_button(self, msg):
        self.drag_button = bool(msg.data)

    def _cb_arm_backend(self, msg):
        self.arm_backend = msg.data

    def _cb_slider_backend(self, msg):
        self.slider_backend = msg.data

    def connect_arm(self, ip):
        self.pub_arm_connect.publish(String(data=ip))

    def connect_slider(self, dev):
        self.pub_slider_connect.publish(String(data=dev))

    def _cb_slider_js(self, msg):
        if msg.position:
            self.slider_pos = float(msg.position[0])
        self.slider_rx = time.time()

    def _cb_slider_status(self, msg):
        self.slider_status = msg.data

    # -- freshness --
    def arm_online(self):
        return (time.time() - self.arm_rx) < STALE_S

    def slider_online(self):
        return (time.time() - self.slider_rx) < STALE_S

    # -- commands --
    def move_arm(self, joints_rad, speed_pct):
        m = Float64MultiArray()
        m.data = [float(x) for x in joints_rad] + [float(speed_pct)]
        self.pub_arm.publish(m)

    def move_slider(self, pos_m, speed_pct):
        m = Float64MultiArray()
        m.data = [float(pos_m), float(speed_pct)]
        self.pub_slider.publish(m)

    def call(self, key, value=None, log=None):
        cli = self.cli[key]
        if isinstance(cli.srv_type, type) and cli.srv_type is SetBool:
            req = SetBool.Request(); req.data = bool(value)
        else:
            req = Trigger.Request()
        if not cli.wait_for_service(timeout_sec=1.0):
            if log:
                log(f"service {key} not available")
            return
        fut = cli.call_async(req)
        if log:
            def _done(f):
                try:
                    r = f.result()
                    log(f"{key}: {getattr(r, 'message', '') or ('ok' if r.success else 'failed')}")
                except Exception as e:  # noqa: BLE001
                    log(f"{key}: error {e}")
            fut.add_done_callback(_done)


# ==========================================================================
# Toggle switch widget (branded pill switch)
# ==========================================================================
class ToggleSwitch(tk.Frame):
    OFF_TRACK = "#C4C9D1"

    def __init__(self, parent, text, command, font, bg=WHITE, on_color=LIME):
        super().__init__(parent, bg=bg)
        self.command = command
        self.on_color = on_color
        self.state = False
        tk.Label(self, text=text, bg=bg, fg=DARK, font=font).pack(side="left")
        self.cv = tk.Canvas(self, width=52, height=28, bg=bg, highlightthickness=0, cursor="hand2")
        self.cv.pack(side="left", padx=8)
        self.cv.bind("<Button-1>", lambda _e: self.command and self.command(not self.state))
        self._draw()

    def _draw(self):
        c = self.cv; c.delete("all")
        col = self.on_color if self.state else self.OFF_TRACK
        c.create_oval(2, 4, 22, 24, fill=col, outline=col)
        c.create_oval(30, 4, 50, 24, fill=col, outline=col)
        c.create_rectangle(12, 4, 40, 24, fill=col, outline=col)
        kx = 32 if self.state else 4
        c.create_oval(kx, 6, kx + 16, 22, fill=WHITE, outline="#B8BCC4")

    def set(self, on):
        self.state = bool(on); self._draw()


# ==========================================================================
# GUI
# ==========================================================================
class App:
    def __init__(self, root, node, executor):
        self.root = root
        self.node = node
        self.executor = executor
        self.waypoints = []          # list of {joints:[rad], slider:m, desc:str}
        self.active_speed = 150.0    # mm/s applied to moves; set via the "Set speed" button
        self.run_thread = None
        self.stop_flag = threading.Event()
        self.selftest = os.environ.get("DEXORY_TEACH_SELFTEST") == "1"
        self._prev_button = False    # (kept for the optional keypad indicator)
        # auto-capture-on-settle state
        self._cap_prev_j = None
        self._cap_prev_s = 0.0
        self._cap_moving = False
        self._cap_still_t = 0.0

        root.title("Dexory Robot Teach — ROS 2")
        root.configure(bg=BG)
        root.geometry("820x820")
        root.minsize(720, 720)
        self._set_fonts()
        self._load_icon()
        self._build_header()
        self._build_body()

        self.log("GUI node started. Drivers auto-discovered over the ROS graph.")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        # WSLg (WSL2's Wayland/XWayland compositor) often shows a blank window
        # until the first explicit repaint. Nudge the geometry and force an
        # update so the interface paints immediately instead of staying empty.
        try:
            self.root.update_idletasks()
            self.root.deiconify()
            self.root.lift()
            w, h = 820, 820
            self.root.geometry(f"{w + 1}x{h + 1}")
            self.root.update()
            self.root.geometry(f"{w}x{h}")
            self.root.update()
        except tk.TclError:
            pass

        self.root.after(100, self._refresh)
        if self.selftest:
            self.root.after(800, self._run_selftest)

    # ---- style ----
    def _set_fonts(self):
        fams = set(tkfont.families())
        base = "IBM Plex Sans" if "IBM Plex Sans" in fams else "DejaVu Sans"
        self.f = tkfont.Font(family=base, size=10)
        self.f_bold = tkfont.Font(family=base, size=10, weight="bold")
        self.f_h = tkfont.Font(family=base, size=13, weight="bold")
        self.f_mono = tkfont.Font(family="DejaVu Sans Mono", size=9)

    def _load_icon(self):
        try:
            self._icon = tk.PhotoImage(file=os.path.join(_pkg_images(), "dexory_icon.png"))
            self.root.iconphoto(True, self._icon)
        except Exception:  # noqa: BLE001
            pass

    def _btn(self, parent, text, cmd, kind="primary"):
        colors = {"primary": (LIME, DARK, "#B7E82A"), "dark": (NAVY, WHITE, "#16265F"),
                  "ghost": (GREY, DARK, "#D2D6DC"), "danger": (DANGER, WHITE, "#B93B3B")}
        bg, fg, active = colors[kind]
        return tk.Button(parent, text=text, command=cmd, font=self.f_bold, bg=bg, fg=fg,
                         activebackground=active, activeforeground=fg, relief="flat", bd=0,
                         padx=14, pady=8, cursor="hand2", highlightthickness=0)

    def _card(self, parent):
        return tk.Frame(parent, bg=WHITE, highlightbackground=GREY, highlightthickness=1)

    def _section(self, parent, text):
        tk.Label(parent, text=text, bg=WHITE, fg=NAVY, font=self.f_h).pack(anchor="w", padx=12, pady=(10, 6))

    def _field(self, parent, label, default, width=8):
        f = tk.Frame(parent, bg=WHITE); f.pack(fill="x", padx=12, pady=3)
        tk.Label(f, text=label, bg=WHITE, fg=DARK, font=self.f, width=17, anchor="w").pack(side="left")
        var = tk.StringVar(value=default)
        tk.Entry(f, textvariable=var, font=self.f, width=width, relief="solid", bd=1).pack(side="left")
        return var

    # ---- header ----
    def _build_header(self):
        h = tk.Frame(self.root, bg=NAVY, height=72); h.pack(side="top", fill="x"); h.pack_propagate(False)
        try:
            img = tk.PhotoImage(file=os.path.join(_pkg_images(), "dexory_logo.png"))
            factor = max(1, round(img.height() / 43))
            self._logo = img.subsample(factor, factor)
            tk.Label(h, image=self._logo, bg=NAVY).pack(side="left", padx=20, pady=14)
        except Exception:  # noqa: BLE001
            tk.Label(h, text="DEXORY", bg=NAVY, fg=WHITE,
                     font=tkfont.Font(size=18, weight="bold")).pack(side="left", padx=20)
        tk.Label(h, text="Robot Teach · ROS 2", bg=NAVY, fg=LIME, font=self.f_h).pack(side="right", padx=24)
        tk.Frame(self.root, bg=LIME, height=3).pack(side="top", fill="x")

    # ---- body ----
    def _build_body(self):
        body = tk.Frame(self.root, bg=BG); body.pack(fill="both", expand=True, padx=16, pady=12)

        # connection card
        conn = self._card(body); conn.pack(fill="x", pady=(0, 10))
        self._section(conn, "Connection")

        # -- arm row: IP address --
        ar = tk.Frame(conn, bg=WHITE); ar.pack(fill="x", padx=12, pady=2)
        self.lbl_arm = tk.Label(ar, text="●", bg=WHITE, fg=DANGER, font=self.f_bold); self.lbl_arm.pack(side="left")
        tk.Label(ar, text="Arm IP", bg=WHITE, fg=DARK, font=self.f_bold, width=8, anchor="w").pack(side="left", padx=(4, 4))
        self.ip_var = tk.StringVar(value="192.168.2.160")
        tk.Entry(ar, textvariable=self.ip_var, font=self.f, width=16, relief="solid", bd=1).pack(side="left")
        self._btn(ar, "Connect", self._connect_arm, "dark").pack(side="left", padx=(6, 2))
        self._btn(ar, "Disconnect", self._disconnect_arm, "ghost").pack(side="left")
        self.arm_be_lbl = tk.Label(ar, text="mock", bg=WHITE, fg=MUTED, font=self.f_mono); self.arm_be_lbl.pack(side="right", padx=8)

        # -- slider row: USB device --
        sr = tk.Frame(conn, bg=WHITE); sr.pack(fill="x", padx=12, pady=2)
        self.lbl_slider = tk.Label(sr, text="●", bg=WHITE, fg=DANGER, font=self.f_bold); self.lbl_slider.pack(side="left")
        tk.Label(sr, text="Slider USB", bg=WHITE, fg=DARK, font=self.f_bold, width=8, anchor="w").pack(side="left", padx=(4, 4))
        self.usb_var = tk.StringVar(value="/dev/ttyUSB0")
        tk.Entry(sr, textvariable=self.usb_var, font=self.f, width=16, relief="solid", bd=1).pack(side="left")
        self._btn(sr, "Connect", self._connect_slider, "dark").pack(side="left", padx=(6, 2))
        self._btn(sr, "Disconnect", self._disconnect_slider, "ghost").pack(side="left")
        self.slider_be_lbl = tk.Label(sr, text="mock", bg=WHITE, fg=MUTED, font=self.f_mono); self.slider_be_lbl.pack(side="right", padx=8)

        # -- live state readout --
        self.lbl_state = tk.Label(conn, text="arm: —   slider: —", bg=WHITE, fg=MUTED, font=self.f_mono)
        self.lbl_state.pack(anchor="w", padx=12, pady=(2, 10))

        cols = tk.Frame(body, bg=BG); cols.pack(fill="both", expand=True)

        # -- left: settings --
        left = self._card(cols); left.pack(side="left", fill="y", padx=(0, 8))
        self._section(left, "Settings")
        self.speed_var = self._field(left, "Speed (mm/s)", "150")
        spd = tk.Frame(left, bg=WHITE); spd.pack(fill="x", padx=12, pady=(0, 4))
        self._btn(spd, "Set speed", self._set_speed, "dark").pack(side="left")
        self.speed_lbl = tk.Label(spd, text="active: 150 mm/s (30%)", bg=WHITE, fg=LIME_DK, font=self.f_mono)
        self.speed_lbl.pack(side="left", padx=8)
        self.loops_var = self._field(left, "Loops", "3")
        self.home_var = self._field(left, "Arm home (deg)", "0,0,0,0,0,0", width=18)
        self.slhome_var = self._field(left, "Slider home (m)", "0.0")
        self.jog_var = self._field(left, "Slider jog step (m)", "0.10")
        mt = tk.Frame(left, bg=WHITE); mt.pack(fill="x", padx=12, pady=(6, 4))
        tk.Label(mt, text="Motion", bg=WHITE, fg=DARK, font=self.f_bold).pack(side="left")
        self.motion_var = tk.StringVar(value="Joints")
        for m in ("Joints", "MoveL"):
            tk.Radiobutton(mt, text=m, variable=self.motion_var, value=m, bg=WHITE, font=self.f,
                           activebackground=WHITE, selectcolor=LIME).pack(side="left", padx=4)
        tk.Label(left, text="Joints = replay taught angles (robust).\nMoveL = straight-line (real arm only).",
                 bg=WHITE, fg=MUTED, font=self.f, justify="left").pack(anchor="w", padx=12, pady=(0, 6))
        self.btn_home = self._btn(left, "⌂  Home arm + slider", self._home, "ghost")
        self.btn_home.pack(fill="x", padx=12, pady=(6, 12))

        # -- right: teach --
        right = self._card(cols); right.pack(side="left", fill="both", expand=True)
        self._section(right, "Teach — arm")
        drag = tk.Frame(right, bg=WHITE); drag.pack(fill="x", padx=12, pady=(0, 4))
        self.sw_drag = ToggleSwitch(drag, "Arm drag", self._toggle_arm_drag, self.f_bold); self.sw_drag.pack(side="left")
        self.btn_capture = self._btn(drag, "◉ Capture waypoint", self._capture, "dark"); self.btn_capture.pack(side="right")

        acr = tk.Frame(right, bg=WHITE); acr.pack(fill="x", padx=12, pady=(0, 6))
        self.autocap_var = tk.BooleanVar(value=True)
        tk.Checkbutton(acr, text="Auto-capture when the arm settles (hold still while dragging)",
                       variable=self.autocap_var, bg=WHITE, fg=DARK, font=self.f,
                       activebackground=WHITE, selectcolor=LIME).pack(side="left")
        self.btn_lbl = tk.Label(acr, text="end-btn ●", bg=WHITE, fg=MUTED, font=self.f_bold)
        self.btn_lbl.pack(side="right")

        # slider teach subsection
        self._section(right, "Teach — slider")
        sl = tk.Frame(right, bg=WHITE); sl.pack(fill="x", padx=12, pady=(0, 4))
        self.sw_sldrag = ToggleSwitch(sl, "Slider drag", self._toggle_slider_drag, self.f_bold); self.sw_sldrag.pack(side="left")
        self._btn(sl, "◀ jog", self._jog_minus, "ghost").pack(side="left", padx=(12, 4))
        self._btn(sl, "jog ▶", self._jog_plus, "ghost").pack(side="left", padx=4)
        self.slpos_lbl = tk.Label(sl, text="pos: 0.000 m", bg=WHITE, fg=NAVY, font=self.f_mono); self.slpos_lbl.pack(side="right")

        # waypoint list
        lf = tk.Frame(right, bg=WHITE); lf.pack(fill="both", expand=True, padx=12, pady=(6, 0))
        self.listbox = tk.Listbox(lf, font=self.f_mono, height=8, activestyle="none", relief="solid", bd=1,
                                  highlightthickness=0, selectbackground=NAVY, selectforeground=WHITE)
        self.listbox.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(lf, orient="vertical", command=self.listbox.yview); sb.pack(side="right", fill="y")
        self.listbox.config(yscrollcommand=sb.set)

        row = tk.Frame(right, bg=WHITE); row.pack(fill="x", padx=12, pady=6)
        for txt, fn in (("▲", self._move_up), ("▼", self._move_down), ("Go to", self._goto),
                        ("Delete", self._delete), ("Clear", self._clear),
                        ("Save", self._save), ("Load", self._load)):
            self._btn(row, txt, fn, "ghost").pack(side="left", padx=(0, 5))

        mrow = tk.Frame(right, bg=WHITE); mrow.pack(fill="x", padx=12, pady=(4, 0))
        self.sw_motors = ToggleSwitch(mrow, "Motors", self._toggle_motors, self.f_bold); self.sw_motors.pack(side="left")
        self.btn_run = self._btn(right, "▶  RUN  (loop all waypoints)", self._run, "primary"); self.btn_run.pack(fill="x", padx=12, pady=(4, 4))
        self.btn_stop = self._btn(right, "■  STOP", self._stop, "danger"); self.btn_stop.pack(fill="x", padx=12, pady=(0, 6))
        self.btn_clear = self._btn(right, "⟲  Clear arm e-stop alarm", self._clear_alarm, "dark"); self.btn_clear.pack(fill="x", padx=12, pady=(0, 12))

        # log
        logc = self._card(body); logc.pack(fill="both", expand=False, pady=(10, 0))
        self._section(logc, "Log")
        self.logbox = tk.Text(logc, height=7, font=self.f_mono, bg="#0E1730", fg="#D5E8B0",
                              relief="flat", bd=0, wrap="word", state="disabled")
        self.logbox.pack(fill="both", expand=True, padx=12, pady=(0, 12))

        tk.Label(body, text="⚠  In-app STOP is a soft stop. The physical e-stop is the only true "
                            "emergency stop — keep it in hand.", bg=BG, fg=MUTED, font=self.f).pack(anchor="w", pady=(8, 0))

    # ---- periodic UI refresh (main thread) ----
    def _refresh(self):
        n = self.node
        self.lbl_arm.config(fg=LIME_DK if n.arm_online() else DANGER)
        self.lbl_slider.config(fg=LIME_DK if n.slider_online() else DANGER)
        real_arm = n.arm_backend.startswith("rokae") and not n.arm_status.startswith("error")
        real_sl = n.slider_backend.startswith("roboteq") and not n.slider_status.startswith("error")
        self.arm_be_lbl.config(text=n.arm_backend or "—", fg=LIME_DK if real_arm else MUTED)
        self.slider_be_lbl.config(text=n.slider_backend or "—", fg=LIME_DK if real_sl else MUTED)
        jdeg = ", ".join(f"{math.degrees(a):.0f}" for a in n.arm_joints)
        self.lbl_state.config(text=f"arm[{n.arm_status}]: {jdeg}   slider[{n.slider_status}]: {n.slider_pos:.3f}m")
        self.slpos_lbl.config(text=f"pos: {n.slider_pos:.3f} m")

        # keypad button indicator (only meaningful when read_keypad is enabled)
        self.btn_lbl.config(fg=LIME_DK if n.drag_button else MUTED)

        # AUTO-CAPTURE ON SETTLE: while dragging, once the arm moves and then holds
        # still for a short dwell, capture one waypoint (no keypad needed).
        if self.autocap_var.get() and self.sw_drag.state:
            now = time.time()
            j, s = list(n.arm_joints), n.slider_pos
            if self._cap_prev_j is not None:
                d = sum(abs(a - b) for a, b in zip(j, self._cap_prev_j)) + abs(s - self._cap_prev_s)
                if d > 0.012:                       # moving
                    self._cap_moving = True
                    self._cap_still_t = now
                elif self._cap_moving and (now - self._cap_still_t) > 0.6:   # settled
                    self._cap_moving = False
                    self.log("arm settled → auto-capturing waypoint")
                    self._capture()
            self._cap_prev_j, self._cap_prev_s = j, s
        else:
            self._cap_prev_j, self._cap_moving = None, False

        self.root.after(100, self._refresh)

    # ---- helpers ----
    def _num(self, var, default):
        try:
            return float(var.get())
        except (ValueError, tk.TclError):
            self.log(f"bad number '{var.get()}', using {default}"); return float(default)

    def _floats(self, text, n):
        try:
            vals = [float(x) for x in text.replace(" ", "").split(",")]
            if len(vals) != n:
                raise ValueError
            return vals
        except ValueError:
            self.log(f"arm home must be {n} comma-separated numbers; using zeros"); return [0.0] * n

    def log(self, msg):
        line = str(msg)
        print(line, flush=True)     # also to stdout (useful headless)
        def do():
            self.logbox.config(state="normal"); self.logbox.insert("end", line + "\n")
            self.logbox.see("end"); self.logbox.config(state="disabled")
        try:
            self.root.after(0, do)
        except Exception:  # noqa: BLE001
            pass

    # ---- teach actions ----
    def _ui(self, fn):
        """Run a Tk widget mutation on the main thread (safe from any thread)."""
        try:
            self.root.after(0, fn)
        except Exception:  # noqa: BLE001
            pass

    # ---- connection ----
    def _connect_arm(self):
        ip = self.ip_var.get().strip()
        self.node.connect_arm(ip)
        self.log(f"arm connect → {ip} (falls back to mock if unreachable)")

    def _disconnect_arm(self):
        self.node.connect_arm("")
        self.log("arm → mock (disconnected)")

    def _connect_slider(self):
        dev = self.usb_var.get().strip()
        self.node.connect_slider(dev)
        self.log(f"slider connect → {dev} (falls back to mock if unavailable)")

    def _disconnect_slider(self):
        self.node.connect_slider("")
        self.log("slider → mock (disconnected)")

    def _toggle_arm_drag(self, desired):
        self.node.call("arm/set_drag", desired, self.log)
        self._ui(lambda: self.sw_drag.set(desired))
        if desired:
            self._ui(lambda: self.sw_motors.set(False))

    def _toggle_slider_drag(self, desired):
        self.node.call("rail/set_drag", desired, self.log)
        self._ui(lambda: self.sw_sldrag.set(desired))

    def _toggle_motors(self, desired):
        self.node.call("arm/set_power", desired, self.log)
        self.node.call("rail/set_power", desired, self.log)
        self._ui(lambda: self.sw_motors.set(desired))
        if desired:
            self._ui(lambda: (self.sw_drag.set(False), self.sw_sldrag.set(False)))

    def _jog(self, sign):
        step = self._num(self.jog_var, 0.10) * sign
        target = self.node.slider_pos + step
        self.node.move_slider(target, self.active_speed)
        self.log(f"slider jog -> {target:.3f} m @ {self.active_speed:.0f} mm/s")

    def _jog_plus(self):
        self._jog(+1)

    def _jog_minus(self):
        self._jog(-1)

    def _capture(self):
        wp = {"joints": list(self.node.arm_joints), "slider": self.node.slider_pos}
        wp["desc"] = self._desc(wp)
        self.waypoints.append(wp)
        self.listbox.insert("end", f" W{len(self.waypoints):02d}  {wp['desc']}")
        self.log(f"captured W{len(self.waypoints):02d}: {wp['desc']}")

    def _desc(self, wp):
        jdeg = " ".join(f"{math.degrees(a):5.0f}" for a in wp["joints"])
        return f"J[{jdeg}]  S{wp['slider']:.3f}m"

    def _sel(self):
        s = self.listbox.curselection(); return s[0] if s else None

    def _delete(self):
        i = self._sel()
        if i is None:
            return
        self.waypoints.pop(i); self._refresh_list()

    def _clear(self):
        self.waypoints.clear(); self._refresh_list(); self.log("cleared all waypoints")

    def _move_up(self):
        i = self._sel()
        if i is None or i == 0:
            return
        self.waypoints[i - 1], self.waypoints[i] = self.waypoints[i], self.waypoints[i - 1]
        self._refresh_list(i - 1)

    def _move_down(self):
        i = self._sel()
        if i is None or i >= len(self.waypoints) - 1:
            return
        self.waypoints[i + 1], self.waypoints[i] = self.waypoints[i], self.waypoints[i + 1]
        self._refresh_list(i + 1)

    def _refresh_list(self, select=None):
        self.listbox.delete(0, "end")
        for k, wp in enumerate(self.waypoints, 1):
            self.listbox.insert("end", f" W{k:02d}  {wp['desc']}")
        if select is not None:
            self.listbox.selection_set(select)

    def _save(self):
        if not self.waypoints:
            self.log("nothing to save"); return
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("Waypoints", "*.json")]) if not self.selftest \
            else os.path.join(os.path.expanduser("~"), "dexory_waypoints.json")
        if not path:
            return
        with open(path, "w") as f:
            json.dump(self.waypoints, f, indent=2)
        self.log(f"saved {len(self.waypoints)} waypoints -> {path}")

    def _load(self):
        path = filedialog.askopenfilename(filetypes=[("Waypoints", "*.json")]) if not self.selftest else ""
        if not path:
            return
        with open(path) as f:
            self.waypoints = json.load(f)
        for wp in self.waypoints:
            wp["desc"] = self._desc(wp)
        self._refresh_list(); self.log(f"loaded {len(self.waypoints)} waypoints")

    # ---- speed ----
    @staticmethod
    def _speed_band(mms):
        return 10 if mms < 100 else 30 if mms < 200 else 50 if mms < 500 else 80 if mms < 800 else 100

    def _set_speed(self):
        v = self._num(self.speed_var, 150)
        self.active_speed = v
        band = self._speed_band(v)
        self.speed_lbl.config(text=f"active: {v:.0f} mm/s ({band}%)")
        self.log(f"Speed set → {v:.0f} mm/s  (≈{band}% joint-speed band; used for RUN / Go to / Home / jog)")

    # ---- motion ----
    def _home(self):
        # move arm to the home joints (from the field) and slider to its home, at the active speed
        target_rad = [math.radians(a) for a in self._floats(self.home_var.get(), 6)]
        self.node.move_arm(target_rad, self.active_speed)
        self.node.move_slider(self._num(self.slhome_var, 0.0), self.active_speed)
        self.log(f"homing arm + slider @ {self.active_speed:.0f} mm/s")

    def _goto(self):
        i = self._sel()
        if i is None:
            self.log("select a waypoint to go to"); return
        self._start_run([self.waypoints[i]], loops=1, tag="goto")

    def _run(self):
        if not self.waypoints:
            self.log("capture at least one waypoint first"); return
        loops = int(self._num(self.loops_var, 3))
        self._start_run(list(self.waypoints), loops=loops, tag="run")

    def _start_run(self, wps, loops, tag):
        if self.run_thread and self.run_thread.is_alive():
            self.log("already running — press STOP first"); return
        self.stop_flag.clear()
        speed = self.active_speed
        self.run_thread = threading.Thread(target=self._run_worker, args=(wps, loops, speed, tag), daemon=True)
        self.run_thread.start()

    def _run_worker(self, wps, loops, speed, tag):
        for lp in range(1, loops + 1):
            for j, wp in enumerate(wps, 1):
                if self.stop_flag.is_set():
                    self.log("STOP: sequence halted"); return
                self.log(f"{tag} cycle {lp}/{loops} -> W{j:02d}")
                self.node.move_arm(wp["joints"], speed)
                self.node.move_slider(wp["slider"], speed)
                if not self._wait_arrival():
                    return
        self.log(f"{tag}: finished {loops} loop(s)")

    def _wait_arrival(self, start_timeout=3.0, move_timeout=30.0):
        """Two-phase wait mirroring the real controller: first wait for motion to
        START (status 'moving'), then wait for both axes to return to 'idle'.
        Avoids racing past a waypoint while the driver is still 'idle' for a tick."""
        n = self.node
        t0 = time.time()
        while time.time() - t0 < start_timeout:
            if self.stop_flag.is_set():
                return False
            if n.arm_status == "moving" or n.slider_status == "moving":
                break
            time.sleep(0.03)
        t0 = time.time()
        while time.time() - t0 < move_timeout:
            if self.stop_flag.is_set():
                return False
            if n.arm_status in ("idle", "off") and n.slider_status in ("idle", "off"):
                return True
            time.sleep(0.05)
        self.log("timed out waiting for arrival"); return False

    def _stop(self):
        self.stop_flag.set()
        self.node.call("arm/stop", log=self.log)
        self.node.call("rail/stop", log=self.log)
        self.log("STOP requested")

    def _clear_alarm(self):
        self.node.call("arm/clear_alarm", log=self.log)

    # ---- self-test (headless validation) ----
    def _run_selftest(self):
        threading.Thread(target=self._selftest_worker, daemon=True).start()

    def _selftest_worker(self):
        log = self.log
        log("=== SELFTEST START ===")
        t0 = time.time()
        while not (self.node.arm_online() and self.node.slider_online()):
            if time.time() - t0 > 10:
                log("SELFTEST FAIL: drivers never came online"); self.root.after(0, self.root.quit); return
            time.sleep(0.1)
        log("drivers online")
        self._toggle_motors(True); time.sleep(0.5)
        # teach 3 waypoints via AUTO-CAPTURE ON SETTLE: mock drag moves-then-holds every
        # ~2 s; each hold triggers a capture (no keypad involved).
        self._toggle_arm_drag(True)
        t0 = time.time()
        while len(self.waypoints) < 3 and time.time() - t0 < 16:
            time.sleep(0.2)
        self._toggle_arm_drag(False); time.sleep(0.5)
        self._toggle_motors(True); time.sleep(0.3)
        if len(self.waypoints) < 3:
            log(f"SELFTEST FAIL: only {len(self.waypoints)} waypoints captured")
            self.root.after(0, self.root.quit); return
        self._save()
        # replay discrete waypoints
        self.stop_flag.clear()
        self._run_worker(list(self.waypoints), loops=1, speed=100, tag="selftest-replay")

        # exercise Set speed + Home (home should move the arm back toward 0)
        self._set_speed()
        self._toggle_motors(True); time.sleep(0.3)
        self._home(); time.sleep(2.5)
        log(f"home @ {self.active_speed:.0f} mm/s -> j1={self.node.arm_joints[0]:.3f} rad")

        log(f"=== SELFTEST PASS: {len(self.waypoints)} waypoints taught & replayed ===")
        time.sleep(0.5)
        self.root.after(0, self.root.quit)

    def _on_close(self):
        self.stop_flag.set()
        try:
            self.root.quit()
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TeachNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        root = tk.Tk()
    except Exception as e:  # noqa: BLE001
        print(f"Cannot open a display ({e}). On WSL make sure WSLg is available "
              f"(echo $DISPLAY should print :0).", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    app = App(root, node, executor)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
