/*
 * viewer.js — the MAIN Run-tab scan-path viewport.
 *
 * Scene/arm/path/playback all live in the shared module viewer3d.js. This file
 * wires the page UI: transport (play/scrub), layer + camera toggles, and — ported
 * from the debug viewport — orient-the-part (which RE-PLANS the path for the new
 * pose), table height, show-hemisphere, Scan -> arm, and the live comms monitor.
 */
'use strict';

const $ = (id) => document.getElementById(id);
const setStatus = (t) => { $('statusText').textContent = t; };

(async function main() {
  const config = await QCViewer.fetchConfig();
  const v = QCViewer.create($('canvas'), { config, sizeToWindow: true });
  window.__viewer = v;

  // ---- load / reload the planned bundle (part + path + dome) ----------------
  let scanWps = [], domeInfo = null, domeMesh = null;
  async function loadBundle() {
    const r = await fetch('/api/viewer_bundle');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const bundle = await r.json();
    if (bundle.part) v.buildPart(bundle.part);
    scanWps = bundle.waypoints || [];
    domeInfo = bundle.dome || null;
    drawPath();
    buildDome();
    $('scrub').max = String(Math.max(0, scanWps.length - 1));
    return bundle;
  }
  function drawPath() {
    // path is in the fixed table frame (bundle is grounded); draw as-is.
    v.buildPath(scanWps
      .map((w) => ({ position: w.position, target: (w.target || w.position),
                     line_id: w.line_id == null ? 0 : w.line_id }))
      .filter((w) => w.position[2] >= -0.001));
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

  // ---- transport (play / reset / scrub) -------------------------------------
  v.play.onStep = (t, n) => { $('scrub').value = String(t); $('wptLabel').textContent = `${Math.round(t) + 1} / ${n}`; };
  v.play.onDone = () => { $('playBtn').textContent = '▶ Play'; };
  $('playBtn').onclick = () => { v.play.on = !v.play.on; $('playBtn').textContent = v.play.on ? '❚❚ Pause' : '▶ Play'; };
  $('resetBtn').onclick = () => { v.play.on = false; $('playBtn').textContent = '▶ Play'; v.placeAt(0); };
  $('scrub').oninput = (e) => { v.play.on = false; $('playBtn').textContent = '▶ Play'; v.placeAt(parseFloat(e.target.value)); };

  // ---- camera + layers ------------------------------------------------------
  $('viewSeg').querySelectorAll('button').forEach((b) => b.onclick = () => {
    $('viewSeg').querySelectorAll('button').forEach((x) => x.classList.remove('on'));
    b.classList.add('on'); v.setView(b.dataset.view);
  });
  document.querySelectorAll('.toggle[data-layer]').forEach((el) => el.onclick = () => {
    const on = !el.classList.contains('on'); el.classList.toggle('on', on); v.setLayer(el.dataset.layer, on);
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

  // ---- show hemisphere toggle ----------------------------------------------
  const dc = $('showDome');
  if (dc) dc.onchange = () => { if (domeMesh) domeMesh.visible = dc.checked; };

  // (No "Scan -> arm" here: scanning is driven by the console's Run process.)

  // ---- live arm-comms monitor ----------------------------------------------
  const cb = $('commsBtn');
  if (cb) cb.onclick = () => window.open('comms.html', 'qc_comms', 'width=780,height=640');

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
