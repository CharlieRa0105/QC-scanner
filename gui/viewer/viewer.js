/*
 * viewer.js — the MAIN Run-tab scan-path viewport.
 *
 * Scene/arm/path/playback all live in the shared module viewer3d.js. This file
 * wires the page UI: transport (play/scrub), layer + camera toggles, and — ported
 * from the debug viewport — the fit primitive (hemisphere/rectangle), Generate
 * path, orient-the-part (all of which RE-PLAN), table height, show-fit-shape, and
 * the live comms monitor.
 */
'use strict';

const $ = (id) => document.getElementById(id);
const setStatus = (t) => { $('statusText').textContent = t; };

(async function main() {
  const config = await QCViewer.fetchConfig();
  const v = QCViewer.create($('canvas'), { config, sizeToWindow: true });
  window.__viewer = v;

  // ---- load / reload the planned bundle (part + path + dome) ----------------
  let scanWps = [], domeInfo = null, domeMesh = null, boxInfo = null, boxMesh = null;
  let primitive = 'dome';   // which fit shape the current bundle was planned on
  async function loadBundle() {
    const r = await fetch('/api/viewer_bundle');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const bundle = await r.json();
    if (bundle.part) v.buildPart(bundle.part);
    scanWps = bundle.waypoints || [];
    domeInfo = bundle.dome || null;
    boxInfo = bundle.box || null;
    primitive = bundle.primitive || (boxInfo ? 'box' : 'dome');
    drawPath();
    buildDome();
    buildBox();
    setFitActive(primitive);
    $('scrub').max = String(Math.max(0, scanWps.length - 1));
    return bundle;
  }
  function drawPath() {
    // path is in the fixed table frame (bundle is grounded); draw as-is.
    const poses = scanWps
      .map((w) => ({ position: w.position, target: (w.target || w.position),
                     line_id: w.line_id == null ? 0 : w.line_id }))
      .filter((w) => w.position[2] >= -0.001);
    v.buildPath(poses);
    v.buildArmPath(poses);   // continuous orange arm-travel line (thin GL line)
  }
  function buildDome() {
    if (domeMesh) { v.scene.remove(domeMesh); domeMesh = null; }
    if (!domeInfo) return;
    const T = v.THREE;
    const geo = new T.SphereGeometry(domeInfo.radius, 48, 24, 0, Math.PI * 2, 0, Math.PI / 2);
    domeMesh = new T.Mesh(geo, new T.MeshBasicMaterial(
      { color: 0x5b8dd6, transparent: true, opacity: 0.10, side: T.DoubleSide, depthWrite: false }));
    domeMesh.add(new T.LineSegments(new T.WireframeGeometry(geo),
      new T.LineBasicMaterial({ color: 0x5b8dd6, transparent: true, opacity: 0.28 })));
    domeMesh.rotation.x = Math.PI / 2;           // pole +Y -> +Z (flat face on table)
    domeMesh.position.set(domeInfo.center[0], domeInfo.center[1], domeInfo.center[2]);
    domeMesh.visible = !!($('showDome') && $('showDome').checked);
    v.scene.add(domeMesh);
  }
  function buildBox() {
    if (boxMesh) { v.scene.remove(boxMesh); boxMesh = null; }
    if (!boxInfo) return;
    const T = v.THREE;
    const geo = new T.BoxGeometry(boxInfo.half_dims[0] * 2, boxInfo.half_dims[1] * 2, boxInfo.half_dims[2] * 2);
    boxMesh = new T.Mesh(geo, new T.MeshBasicMaterial(
      { color: 0x5b8dd6, transparent: true, opacity: 0.06, side: T.DoubleSide, depthWrite: false }));
    boxMesh.add(new T.LineSegments(new T.EdgesGeometry(geo),
      new T.LineBasicMaterial({ color: 0x5b8dd6, transparent: true, opacity: 0.5 })));
    boxMesh.position.set(boxInfo.center[0], boxInfo.center[1], boxInfo.center[2]);
    if (boxInfo.quaternion) {               // table-aligned box orientation (yaw)
      const q = boxInfo.quaternion;
      boxMesh.quaternion.set(q[0], q[1], q[2], q[3]);
    }
    boxMesh.visible = !!($('showDome') && $('showDome').checked);
    v.scene.add(boxMesh);
  }
  function setFitActive(prim) {
    const seg = $('fitSeg'); if (!seg) return;
    seg.querySelectorAll('button').forEach((b) => b.classList.toggle('on', b.dataset.fit === prim));
  }

  // ---- transport (play / reset / scrub) -------------------------------------
  v.play.onStep = (t, n) => { $('scrub').value = String(t); $('wptLabel').textContent = `${Math.round(t) + 1} / ${n}`; };
  v.play.onDone = () => { $('playBtn').textContent = '▶ Play'; };
  $('playBtn').onclick = () => { v.play.on = !v.play.on; $('playBtn').textContent = v.play.on ? '❚❚ Pause' : '▶ Play'; };
  $('resetBtn').onclick = () => { v.play.on = false; $('playBtn').textContent = '▶ Play'; v.placeAt(0); };
  $('scrub').oninput = (e) => { v.play.on = false; $('playBtn').textContent = '▶ Play'; v.placeAt(parseFloat(e.target.value)); };

  // ---- camera ---------------------------------------------------------------
  // (Layer toggles live in the Debug viewport, not here.)
  $('viewSeg').querySelectorAll('button').forEach((b) => b.onclick = () => {
    $('viewSeg').querySelectorAll('button').forEach((x) => x.classList.remove('on'));
    b.classList.add('on'); v.setView(b.dataset.view);
  });

  // ---- orient the part -> RE-PLAN the path for the new pose -----------------
  // Each flip accumulates a part rotation and asks the backend to regenerate the
  // dome/path for that orientation, then reloads the bundle -- rotating/moving the
  // part yields a NEW path, not the old one rotated with it.
  let orient = [0, 0, 0];   // accumulated rx,ry,rz degrees
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
      await loadBundle();
      v.frameView();
      setStatus(`re-planned — ${scanWps.length} waypoints at orient [${orient.join(', ')}]°`);
    } catch (e) { setStatus('re-plan error: ' + e.message); }
    finally { replanning = false; }
  }
  const wireFlip = (id, dx, dy, dz) => { const b = $(id); if (b) b.onclick = () => reorient(dx, dy, dz, false); };
  wireFlip('flipX', 90, 0, 0); wireFlip('flipY', 0, 90, 0); wireFlip('spinZ', 0, 0, 90);
  const fr = $('flipReset');
  if (fr) fr.onclick = () => reorient(0, 0, 0, true);

  // ---- table height (arm <-> table gap) -------------------------------------
  const tH = $('tableH'), tHVal = $('tableHVal');
  if (tH) {
    const init = Math.round(v.mountHeightMm ? v.mountHeightMm() : 1200);
    tH.value = String(Math.min(1400, Math.max(400, init)));
    const show = (mm) => { if (tHVal) tHVal.textContent = mm + ' mm'; };
    show(+tH.value);
    tH.oninput = () => { const mm = +tH.value; show(mm); if (v.setMountHeight) v.setMountHeight(mm); };
    tH.onchange = async () => {
      try {
        await fetch('/api/robot/table_height', { method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ mm: +tH.value }) });
      } catch (e) { /* backend optional */ }
    };
  }

  // ---- show fit-shape toggle (whichever primitive is active) ----------------
  const dc = $('showDome');
  if (dc) dc.onchange = () => {
    if (domeMesh) domeMesh.visible = dc.checked;
    if (boxMesh) boxMesh.visible = dc.checked;
  };

  // ---- fit primitive: hemisphere <-> rectangle -> RE-PLAN on the new shape --
  // The path is regenerated on whichever primitive is selected (dome raster vs
  // table-aligned box); the backend preserves orientation.
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
      await loadBundle();
      v.frameView();
      setStatus(`${scanWps.length} waypoints on ${primitive === 'box' ? 'rectangle' : 'hemisphere'}`);
    } catch (e) { setStatus('re-plan error: ' + e.message); setFitActive(primitive); }
    finally { refitting = false; }
  });

  // ---- generate path (explicit re-plan) -------------------------------------
  // Regenerate the scan path for the current part on the currently-selected
  // primitive, preserving orientation on the backend, then reload + redraw.
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
      await loadBundle();
      v.frameView();
      setStatus(`${scanWps.length} waypoints generated on ${primitive === 'box' ? 'rectangle' : 'hemisphere'}`);
    } catch (e) { setStatus('generate error: ' + e.message); }
    finally { generating = false; }
  };

  // (No "Scan -> arm" here: scanning is driven by the console's Run process.)
  // (Arm comms monitor lives in the Debug viewport, not here.)

  // ---- sim arm mirrors the PHYSICAL arm (no IK) -----------------------------
  // The controller reports live joint angles (deg); we push them onto the sim arm
  // via forward kinematics (setJoints, radians) so the sim always matches the real
  // arm. Only mirrors when connected AND no local playback is running (playback
  // owns the arm during a replay). Poll ~8 Hz.
  // The sim arm is driven by EXACTLY ONE source each tick, by priority:
  //   1. a planned-trajectory preview (previewing) — owns the arm while playing;
  //   2. live telemetry (/arm/joint_states) when it's FRESH (arm connected);
  //   3. otherwise HOME — so a disconnected arm holds the home pose, steady, until
  //      commanded (no flicker, no stale/zero pose).
  // Single-source => the live mirror and the preview can never both write the joints
  // (that double-write was the flicker).
  const HOME_RAD = [-71.38, 93.33, 127.11, 157.33, -61.23, -77.06].map((d) => d * Math.PI / 180);
  let previewing = false;
  let liveJoints = null, liveAt = 0;   // last /arm/joint_states + when (performance.now)
  // /plan/trajectory is LATCHED — it re-delivers on every rosbridge (re)connect.
  // Dedupe so an identical trajectory isn't replayed (that re-play was part of the flicker).
  let lastTrajSig = '';
  const LIVE_FRESH_MS = 1500;
  function syncArm() {
    if (!v.play.on && !previewing && v.setJoints) {
      const fresh = liveJoints && (performance.now() - liveAt < LIVE_FRESH_MS);
      v.setJoints(fresh ? liveJoints : HOME_RAD);
    }
    setTimeout(syncArm, 125);
  }
  syncArm();

  // ---- content --------------------------------------------------------------
  setStatus('loading…');
  await v.buildArm('assets/arm/');
  v.setJoints(HOME_RAD);   // start at HOME (not the zero pose) until telemetry arrives

  // ---- MoveIt trajectory preview over rosbridge (Phase 3) --------------------
  // If the ROS graph is up (qc_bringup → rosbridge on :9090), animate the arm
  // through the EXACT joints MoveIt planned (accurate sim). Silently retries if
  // rosbridge isn't reachable, so the viewer still works off the HTTP API alone.
  // A received trajectory plays ONCE, taking arm ownership (previewing=true) for the
  // duration and handing back to the live mirror on completion (onDone) — so the two
  // sources are mutually exclusive and the steady state is "shows the current arm".
  if (window.QCRos) {
    const rosUrl = 'ws://' + (location.hostname || '127.0.0.1') + ':9090';
    QCRos.connect(rosUrl, {
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
            setStatus('MoveIt preview — ' + pts.length + ' trajectory points');
          }
        } else if (topic === '/mission/state') {
          setStatus('mission: ' + msg.phase + (msg.detail ? ' — ' + msg.detail : ''));
        } else if (topic === '/arm/joint_states' && msg.position && msg.position.length) {
          liveJoints = msg.position; liveAt = performance.now();   // radians + freshness stamp
        }
      },
    });
  }
  try {
    const bundle = await loadBundle();
    v.frameView();
    setStatus(`${scanWps.length} waypoints · part ${bundle.part ? bundle.part.vertices.length : 0} verts`);
  } catch (e) {
    v.frameView();
    setStatus('no plan yet — pick a part (' + e.message + ')');
  }
})();
