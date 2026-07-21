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
  let mirror = true;
  async function syncArm() {
    if (mirror && !v.play.on) {
      try {
        const j = await (await fetch('/api/robot/joints')).json();
        if (j && j.connected && Array.isArray(j.joints) && j.joints.length && v.setJoints) {
          v.setJoints(j.joints.map((q) => (q.deg || 0) * Math.PI / 180));
        }
      } catch (e) { /* offline — leave the sim arm as-is */ }
    }
    setTimeout(syncArm, 125);
  }
  syncArm();

  // ---- content --------------------------------------------------------------
  setStatus('loading…');
  await v.buildArm('assets/arm/');
  try {
    const bundle = await loadBundle();
    v.frameView();
    setStatus(`${scanWps.length} waypoints · part ${bundle.part ? bundle.part.vertices.length : 0} verts`);
  } catch (e) {
    v.frameView();
    setStatus('no plan yet — pick a part (' + e.message + ')');
  }
})();
