/*
 * debug.js — the console's Debug 3D viewport (popup).
 *
 * Focused on the CAD SCAN PATH: orient the part with 90° flip buttons (it stays
 * grounded on the table) -- the scan path REGENERATES automatically for each
 * orientation, so there is no "generate" step. "Preview" replays the scan in the
 * sim (flies the arm + scanner along the path around the part); "Scan -> arm" runs
 * the same path on the real arm. A table-height slider sets the arm<->table gap.
 *
 * Scene/arm/path all come from the shared module viewer3d.js.
 */
'use strict';

const $ = (id) => document.getElementById(id);
const setStatus = (t) => { $('statusText').textContent = t; };
let ready = false;

(async function main() {
  const config = await QCViewer.fetchConfig();
  const v = QCViewer.create($('canvas'), { config, sizeToWindow: true, onFrame });

  await v.buildArm('assets/arm/');

  // ---- live arm mirror (matches the main viewer) ---------------------------
  // Push the controller's live joint angles onto the sim arm (FK) so the Debug
  // viewport ALWAYS shows the current arm pose. Yields while a local replay
  // (play.on) or a planned-trajectory playback owns the arm.
  // Single-source arm posing by priority (see viewer.js): preview > fresh live
  // telemetry > HOME. A disconnected arm holds HOME, steady, until commanded.
  const HOME_RAD = [-71.38, 93.33, 127.11, 157.33, -61.23, -77.06].map((d) => d * Math.PI / 180);
  let previewing = false;
  let liveJoints = null, liveAt = 0;
  const LIVE_FRESH_MS = 1500;
  v.setJoints(HOME_RAD);   // start at HOME until telemetry arrives
  function syncArm() {
    if (!v.play.on && !previewing && v.setJoints) {
      const fresh = liveJoints && (performance.now() - liveAt < LIVE_FRESH_MS);
      v.setJoints(fresh ? liveJoints : HOME_RAD);
    }
    setTimeout(syncArm, 125);
  }
  syncArm();

  // ---- rosbridge: calculated-joint preview + mission runner ----------------
  // Animate the EXACT MoveIt-planned joint trajectory (/plan/trajectory) — the
  // same joints the real arm runs (IK owned by MoveIt/KDL, not the controller).
  // "Scan → arm" plans+executes that trajectory via the ROS mission. Degrades
  // silently if the ROS graph / rosbridge is down.
  let rosConn = null;
  let mission = { active: false, awaitExec: false, lastPhase: null };
  let lastTrajSig = '';   // dedupe latched /plan/trajectory re-delivery (flicker fix)
  async function execMission() {
    if (!rosConn || !rosConn.callService) return;
    setStatus('mission: executing planned joint trajectory…');
    let r; try { r = await rosConn.callService('/mission/execute', 'std_srvs/Trigger', {}, 600000); }
    catch (e) { r = { unavailable: true }; }
    if (r.unavailable) setStatus('mission: execute request failed (rosbridge)');
    else if (!(r.values && r.values.success)) setStatus('mission: execute rejected — ' + ((r.values && r.values.message) || 'unknown'));
  }
  function onMissionState(m) {
    const phase = m.phase || '';
    if (mission.active && phase !== mission.lastPhase) {
      setStatus('mission: ' + phase + (m.detail ? ' — ' + m.detail : '')); mission.lastPhase = phase;
    }
    if (!mission.active) return;
    if (phase === 'planned' && mission.awaitExec) {
      mission.awaitExec = false;
      // DECOUPLED: plan does not auto-move the arm — inspect the preview, then confirm.
      if (confirm('Plan ready — trajectory previewing in the view.\n\nExecute on the ARM now? The arm WILL move; keep the E-stop in reach.')) execMission();
      else { mission.active = false; setStatus('plan ready — execution cancelled (not run)'); }
    }
    else if (phase === 'complete') { mission.active = false; setStatus('mission complete'); }
    else if (phase === 'aborted') { mission.active = false; setStatus('mission aborted'); }
    else if (phase === 'error') { mission.active = false; setStatus('mission error — ' + (m.detail || 'unknown')); }
  }
  if (window.QCRos) {
    const rosUrl = 'ws://' + (location.hostname || '127.0.0.1') + ':9090';
    rosConn = QCRos.connect(rosUrl, {
      subscribe: [
        { topic: '/plan/trajectory', type: 'trajectory_msgs/JointTrajectory' },
        { topic: '/mission/state', type: 'qc_msgs/MissionState' },
        { topic: '/arm/joint_states', type: 'sensor_msgs/JointState' },
      ],
      onMsg: (topic, msg) => {
        if (topic === '/plan/trajectory' && msg.points && msg.points.length) {
          const pts = msg.points;
          const sig = pts.length + ':' + JSON.stringify(pts[0].positions) + ':' + JSON.stringify(pts[pts.length - 1].positions);
          if (sig !== lastTrajSig) {                 // skip re-delivered identical trajectory
            lastTrajSig = sig;
            previewing = true;
            v.playJointTrajectory(pts, { onDone: () => { previewing = false; } });
            if (!mission.active) setStatus('MoveIt preview — ' + pts.length + ' calculated joint points');
          }
        } else if (topic === '/mission/state') {
          onMissionState(msg);
        } else if (topic === '/arm/joint_states' && msg.position && msg.position.length) {
          liveJoints = msg.position; liveAt = performance.now();   // radians + freshness
        }
      },
    });
  }

  let scanWps = [];                        // planner scan path (part frame)
  let domeInfo = null;                     // enclosing hemisphere {center,radius} (dome planner)
  let boxInfo = null;                      // enclosing rectangle {center,half_dims} (box planner)
  let primitive = 'dome';                  // which fit shape the current bundle was planned on
  let standoffMm = 80;                     // scanner standoff (mm); slider drives it, bundle reports it
  try {
    const bundle = await (await fetch('/api/viewer_bundle')).json();
    if (bundle.part) v.buildPart(bundle.part);
    scanWps = bundle.waypoints || [];
    domeInfo = bundle.dome || null;
    boxInfo = bundle.box || null;
    primitive = bundle.primitive || (boxInfo ? 'box' : 'dome');
    if (bundle.standoff_m != null) standoffMm = Math.round(bundle.standoff_m * 1000);
  } catch (e) { /* no part planned yet */ }
  v.frameView();

  // ---------------- enclosing fit shape (debug overlay, toggle) -------------
  // Draws whichever primitive the bundle was planned on -- the hemisphere (dome
  // planner) or the rectangle (table-aligned box planner). Rebuilt on every
  // (re)load so a primitive switch swaps the mesh. Both are FIXED in the table
  // frame (the part flips INSIDE them).
  let fitMesh = null;
  function drawFit() {
    if (fitMesh) { v.scene.remove(fitMesh); fitMesh = null; }
    const T = v.THREE;
    const show = !!($('showDome') && $('showDome').checked);
    if (domeInfo) {
      const geo = new T.SphereGeometry(domeInfo.radius, 48, 24, 0, Math.PI * 2, 0, Math.PI / 2);
      fitMesh = new T.Mesh(geo, new T.MeshBasicMaterial(
        { color: 0x5b8dd6, transparent: true, opacity: 0.10, side: T.DoubleSide, depthWrite: false }));
      fitMesh.add(new T.LineSegments(new T.WireframeGeometry(geo),
        new T.LineBasicMaterial({ color: 0x5b8dd6, transparent: true, opacity: 0.28 })));
      fitMesh.rotation.x = Math.PI / 2;    // pole +Y -> +Z (flat face on table)
      fitMesh.position.set(domeInfo.center[0], domeInfo.center[1], domeInfo.center[2]);
    } else if (boxInfo) {
      const geo = new T.BoxGeometry(boxInfo.half_dims[0] * 2, boxInfo.half_dims[1] * 2, boxInfo.half_dims[2] * 2);
      fitMesh = new T.Mesh(geo, new T.MeshBasicMaterial(
        { color: 0x5b8dd6, transparent: true, opacity: 0.06, side: T.DoubleSide, depthWrite: false }));
      fitMesh.add(new T.LineSegments(new T.EdgesGeometry(geo),
        new T.LineBasicMaterial({ color: 0x5b8dd6, transparent: true, opacity: 0.5 })));
      fitMesh.position.set(boxInfo.center[0], boxInfo.center[1], boxInfo.center[2]);
      if (boxInfo.quaternion) {             // table-aligned box orientation (yaw)
        const q = boxInfo.quaternion;
        fitMesh.quaternion.set(q[0], q[1], q[2], q[3]);
      }
    }
    if (fitMesh) { fitMesh.visible = show; v.scene.add(fitMesh); }
  }
  function setFitActive(prim) {
    const seg = $('fitSeg'); if (!seg) return;
    seg.querySelectorAll('button').forEach((b) => b.classList.toggle('on', b.dataset.fit === prim));
  }
  drawFit(); setFitActive(primitive);
  const domeChk = $('showDome');
  if (domeChk) domeChk.onchange = () => { if (fitMesh) fitMesh.visible = domeChk.checked; };

  // ---------------- orient the part (90° flips -> RE-PLAN) ------------------
  // Each flip RE-PLANS the scan for the new pose (like the main viewport), so the
  // fitted shape + path always follow the part. (Rotating the mesh WITHOUT
  // re-planning -- the old behaviour -- left the path around the previous pose, so
  // the rotated mesh overlapped stale waypoints and they looked like they were
  // inside the part.)
  let orient = [0, 0, 0];            // accumulated rx,ry,rz degrees
  let replanning = false;
  async function reorient(dx, dy, dz, reset) {
    if (replanning) return;
    orient = reset ? [0, 0, 0] : [orient[0] + dx, orient[1] + dy, orient[2] + dz];
    replanning = true;
    setStatus(`re-planning at orient [${orient.join(', ')}]°…`);
    try {
      const r = await (await fetch('/api/plan/reorient', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ orientRpyDeg: orient }),
      })).json();
      if (!r.ok) { setStatus('re-plan failed: ' + (r.error || 'unknown')); return; }
      await reloadBundle();
      v.frameView();
      setStatus(`re-planned — ${scanWps.length} waypoints at orient [${orient.join(', ')}]°`);
    } catch (e) { setStatus('re-plan error: ' + e.message); }
    finally { replanning = false; }
  }
  const wire = (id, dx, dy, dz) => { const b = $(id); if (b) b.onclick = () => reorient(dx, dy, dz, false); };
  wire('flipX', 90, 0, 0); wire('flipY', 0, 90, 0); wire('spinZ', 0, 0, 90);
  const rb = $('flipReset');
  if (rb) rb.onclick = () => reorient(0, 0, 0, true);

  // ---------------- path preview + scan ------------------------------------
  let drawnPoses = [];                      // the planner path currently on screen
  function drawClientPath() {
    // The bundle is grounded (already sits on the table) and is RE-PLANNED for the
    // current orientation, so the path matches the part's pose. Draw the waypoints
    // as-is; line_id is carried through so buildPath draws one polyline per ring.
    drawnPoses = scanWps
      .map((w) => ({ position: w.position, target: (w.target || w.position),
                     line_id: w.line_id == null ? 0 : w.line_id }))
      .filter((w) => w.position[2] >= -0.001);
    v.buildPath(drawnPoses);
    v.buildArmPath(drawnPoses);   // continuous orange arm-travel line (thin GL line)
    return drawnPoses.length;
  }
  // PREVIEW = replay the scan in the sim: fly the arm + scanner along the path
  // around the part. The path itself is already drawn (auto, on load and on every
  // reorient), so there is no separate "generate" step -- this just animates it.
  function replay() {
    if (!drawnPoses.length && !drawClientPath()) {
      setStatus('no path to replay (plan a part first)'); return;
    }
    v.play.poseArm = true;                 // the arm follows the path (sim IK, not the real arm)
    v.play.speed = Math.max(8, v.waypoints().length / 12);   // ~12 s regardless of point count
    v.play.t = 0;
    v.play.on = true;
    v.play.onDone = () => setStatus('replay complete — reorient or Scan → arm');
    setStatus('replaying the scan around the part…');
  }
  const previewBtn = $('genPathBtn'); if (previewBtn) previewBtn.onclick = replay;

  // ---------------- table height (arm -> table gap) -------------------------
  // Slider moves the arm mount live (cheap, local); on release we tell the
  // backend so the reachability preview uses the new gap, then re-check.
  const tH = $('tableH'), tHVal = $('tableHVal');
  if (tH) {
    const initMm = Math.round(v.mountHeightMm ? v.mountHeightMm() : 1200);
    tH.value = String(Math.min(1400, Math.max(400, initMm)));
    const showH = (mm) => { if (tHVal) tHVal.textContent = mm + ' mm'; };
    showH(+tH.value);
    tH.oninput = () => {
      const mm = +tH.value; showH(mm);
      v.setMountHeight(mm);                                 // live: moves the arm mount only
    };
    tH.onchange = async () => {
      const mm = +tH.value;
      try {
        await fetch('/api/robot/table_height', {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ mm }),                     // tell the backend the new gap
        });
      } catch (e) { /* backend optional; local preview already moved */ }
    };
  }

  // ---------------- standoff (waypoint <-> fit shape distance) --------------
  // Slider sets the scanner standoff (mm): for the hemisphere the waypoint-to-centre
  // distance is radius + standoff; for the rectangle it's the per-face offset. On
  // release we ask the backend to RE-PLAN at the new standoff (the planner CLI's
  // --standoff-mm), then reload + redraw. oninput only updates the label (re-planning
  // is a subprocess -- too heavy to run on every drag tick).
  const sOff = $('standoff'), sOffVal = $('standoffVal');
  function syncStandoffUI() {
    if (!sOff) return;
    sOff.value = String(Math.min(+sOff.max, Math.max(+sOff.min, standoffMm)));
    if (sOffVal) sOffVal.textContent = sOff.value + ' mm';
  }
  if (sOff) {
    syncStandoffUI();
    sOff.oninput = () => { if (sOffVal) sOffVal.textContent = sOff.value + ' mm'; };
    let standingOff = false;
    sOff.onchange = async () => {
      if (standingOff) return;
      if (!scanWps.length) { setStatus('no part planned yet — standoff not applied'); return; }
      standingOff = true;
      const mm = +sOff.value;
      setStatus(`re-planning at ${mm}mm standoff…`);
      try {
        const r = await (await fetch('/api/plan/standoff', {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ standoffMm: mm }),
        })).json();
        if (!r.ok) { setStatus('standoff re-plan failed: ' + (r.error || 'unknown')); syncStandoffUI(); return; }
        await reloadBundle();
        v.frameView();
        setStatus(`${scanWps.length} waypoints at ${standoffMm}mm standoff`);
      } catch (e) { setStatus('standoff error: ' + e.message); syncStandoffUI(); }
      finally { standingOff = false; }
    };
  }

  // ---------------- scan speed (real-arm end-effector, mm/s) ----------------
  // Sets the Cartesian sweep speed sent to 'Scan -> arm'. Local only -- read at
  // send time, no backend call on drag; motion itself stays gated as before.
  let scanSpeedMms = 60;
  const spd = $('scanSpeed'), spdVal = $('scanSpeedVal');
  if (spd) {
    spd.value = String(scanSpeedMms);
    const showSpd = () => { if (spdVal) spdVal.textContent = spd.value + ' mm/s'; };
    showSpd();
    spd.oninput = () => { scanSpeedMms = +spd.value; showSpd(); };
  }

  // "Scan → arm" now runs the scan as a MoveIt JOINT TRAJECTORY via the ROS
  // mission (plan on MoveIt/KDL, then execute on the MovementDriver as absolute
  // joint commands — no controller IK). The SAME /plan/trajectory animates the
  // sim arm above, so the sim and the real arm run identical joints. Orientation /
  // standoff / speed are owned by the planner, so the old per-send speed no longer
  // applies here. The mission runs by part id (fetched from /api/plan/last).
  const scanBtn = $('scanArmBtn');
  if (scanBtn) scanBtn.onclick = async () => {
    if (mission.active) { setStatus('a mission is already running'); return; }
    if (!rosConn || !rosConn.callService) {
      setStatus('ROS graph not reachable (rosbridge :9090) — cannot run the joint-trajectory scan'); return;
    }
    let partId = '';
    try { const lp = await (await fetch('/api/plan/last')).json(); partId = (lp && lp.part_id) || ''; } catch (e) { /* none */ }
    if (!partId) { setStatus('no planned part — plan a part in the console first'); return; }
    if (!confirm(`Plan and run the scan for ${partId} on the arm as a MoveIt joint trajectory?\n` +
                 `The arm will MOVE. Ensure the cell is clear and the E-stop is in reach.`)) return;
    mission = { active: true, awaitExec: true, lastPhase: null };
    setStatus('mission: planning (MoveIt/KDL) for ' + partId + '…');
    let r; try { r = await rosConn.callService('/mission/plan', 'qc_msgs/StartMission', { part_id: partId }, 120000); }
    catch (e) { r = { unavailable: true }; }
    if (r.unavailable) { mission.active = false; setStatus('mission: plan request failed (rosbridge)'); return; }
    if (!(r.values && r.values.accepted)) {
      mission.active = false; setStatus('mission: plan rejected — ' + ((r.values && r.values.message) || 'unknown')); return;
    }
    // /mission/execute fires from onMissionState() once phase === 'planned'.
  };

  // live arm-comms monitor (separate window)
  const commsBtn = $('commsBtn');
  if (commsBtn) commsBtn.onclick = () => window.open('comms.html', 'qc_comms', 'width=780,height=640');

  // camera view segment (orbit / scanner POV)
  const seg = $('viewSeg');
  if (seg) seg.querySelectorAll('button').forEach((b) => b.onclick = () => {
    seg.querySelectorAll('button').forEach((x) => x.classList.remove('on'));
    b.classList.add('on');
    v.setView(b.dataset.view);
  });

  // layer toggles (part / path / aim rays / arm / table) -- moved here from main
  document.querySelectorAll('.toggle[data-layer]').forEach((el) => el.onclick = () => {
    const on = !el.classList.contains('on'); el.classList.toggle('on', on); v.setLayer(el.dataset.layer, on);
  });

  // Reload the planned bundle (part + path + fit shape) after a re-plan.
  async function reloadBundle() {
    const b = await (await fetch('/api/viewer_bundle')).json();
    if (b.part) v.buildPart(b.part);
    scanWps = b.waypoints || [];
    domeInfo = b.dome || null;
    boxInfo = b.box || null;
    primitive = b.primitive || (boxInfo ? 'box' : 'dome');
    if (b.standoff_m != null) { standoffMm = Math.round(b.standoff_m * 1000); syncStandoffUI(); }
    drawnPoses = [];
    drawClientPath();           // mesh comes pre-oriented + grounded from the bundle
    drawFit();                  // swap the fit shape to the re-fitted primitive
    setFitActive(primitive);
  }

  // ---------------- generate path (explicit re-plan) -----------------------
  // Regenerate the scan path for the current part on the currently-selected
  // primitive (dome or rectangle), preserving orientation + table placement on
  // the backend, then reload + redraw. Explicit trigger for the debug workflow.
  const genPlanBtn = $('genPlanBtn');
  let generating = false;
  if (genPlanBtn) genPlanBtn.onclick = async () => {
    if (generating) return;
    generating = true;
    const label = primitive === 'box' ? 'rectangle' : 'hemisphere';
    setStatus(`generating path on ${label}…`);
    try {
      const r = await (await fetch('/api/plan/primitive', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ primitive }),
      })).json();
      if (!r.ok) { setStatus('generate failed: ' + (r.error || 'unknown')); return; }
      await reloadBundle();
      setStatus(`${scanWps.length} waypoints generated on ${primitive === 'box' ? 'rectangle' : 'hemisphere'}`);
    } catch (e) { setStatus('generate error: ' + e.message); }
    finally { generating = false; }
  };

  // ---------------- fit primitive: hemisphere <-> rectangle -> RE-PLAN -----
  // Regenerate the path on the selected primitive (dome raster vs table-aligned
  // box); orientation + table placement are preserved by the backend.
  const fitSeg = $('fitSeg');
  let refitting = false;
  if (fitSeg) fitSeg.querySelectorAll('button').forEach((b) => b.onclick = async () => {
    const prim = b.dataset.fit;
    if (refitting || prim === primitive) return;
    refitting = true;
    setStatus(`re-planning on ${prim === 'box' ? 'rectangle' : 'hemisphere'}…`);
    try {
      const r = await (await fetch('/api/plan/primitive', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ primitive: prim }),
      })).json();
      if (!r.ok) { setStatus('re-plan failed: ' + (r.error || 'unknown')); setFitActive(primitive); return; }
      await reloadBundle();
      setStatus(`${scanWps.length} waypoints on ${primitive === 'box' ? 'rectangle' : 'hemisphere'}`);
    } catch (e) { setStatus('re-plan error: ' + e.message); setFitActive(primitive); }
    finally { refitting = false; }
  });

  if (scanWps.length) drawClientPath();
  setStatus(scanWps.length ? 'ready — flip to orient, Generate to plan, Preview to replay, Scan → arm' : 'no scan path (plan a part first)');
  window.__dbg = { v, replay, orient: () => orient };
  ready = true;

  // Camera-follow REMOVED: it re-posed the arm with CCD-IK toward the orbit target
  // every frame, which fought the live/HOME posing (syncArm) and the trajectory
  // preview -> the flicker/alternation. The arm is now driven ONLY by syncArm
  // (preview > live > HOME). onFrame is intentionally a no-op.
  function onFrame() {}
})();
