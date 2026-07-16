/*
 * debug.js — the console's Debug 3D viewport (popup).
 *
 * Focused on the CAD SCAN PATH: orient the part with 90° flip buttons (it stays
 * grounded on the table), "Generate path" to preview the scan path for that
 * orientation (registered to home-0.15z + per-point reachability when the arm is
 * connected), and "Scan -> arm" to run the continuous oriented scan. Camera-
 * follow (the preview arm re-aims at the orbit look-at) is always on.
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
  let scanWps = [];                        // planner scan path (part frame)
  try {
    const bundle = await (await fetch('/api/viewer_bundle')).json();
    if (bundle.part) v.buildPart(bundle.part);
    scanWps = bundle.waypoints || [];
  } catch (e) { /* no part planned yet */ }
  v.frameView();

  // ---------------- orient the part (90° flips, stays grounded) -------------
  const R2D = 180 / Math.PI, D2R = Math.PI / 180;
  let partQuat = new THREE.Quaternion();

  function reground() {
    // sit the (rotated) part on the table: shift Z so its lowest point is z=0
    v.partGroup.position.set(0, 0, 0);
    v.partGroup.quaternion.copy(partQuat);
    v.partGroup.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(v.partGroup);
    if (isFinite(box.min.z)) v.partGroup.position.z = -box.min.z;
    v.partGroup.updateMatrixWorld(true);
  }
  function flip(axis, deg) {
    partQuat.premultiply(new THREE.Quaternion().setFromAxisAngle(axis, deg * D2R));
    reground();
    drawClientPath();                      // keep the path on the part as it flips
    setStatus('part flipped — press "Generate path" to check reachability');
  }
  function orientDeg() {
    const e = new THREE.Euler().setFromQuaternion(partQuat, 'XYZ');
    return [e.x * R2D, e.y * R2D, e.z * R2D];
  }
  const X = new THREE.Vector3(1, 0, 0), Y = new THREE.Vector3(0, 1, 0), Z = new THREE.Vector3(0, 0, 1);
  const wire = (id, ax, d) => { const b = $(id); if (b) b.onclick = () => flip(ax, d); };
  wire('flipX', X, 90); wire('flipY', Y, 90); wire('spinZ', Z, 90);
  const rb = $('flipReset');
  if (rb) rb.onclick = () => { partQuat = new THREE.Quaternion(); reground(); drawClientPath(); setStatus('orientation reset'); };

  // ---------------- path preview + scan ------------------------------------
  function drawClientPath() {
    // draw the path ON the grounded/oriented part; drop below-table (z<0) points
    // (down-facing surfaces the overhead arm can't reach -- they clip the floor).
    v.partGroup.updateMatrixWorld(true);
    const m = v.partGroup.matrixWorld;
    const pts = scanWps.map((w) => {
      const p = new THREE.Vector3(...w.position).applyMatrix4(m);
      const t = new THREE.Vector3(...(w.target || w.position)).applyMatrix4(m);
      return { position: [p.x, p.y, p.z], target: [t.x, t.y, t.z] };
    }).filter((w) => w.position[2] >= 0);
    v.buildPath(pts);
    return pts.length;
  }
  async function generatePath() {
    if (!scanWps.length) { setStatus('no scan path planned (pick a part + plan first)'); return; }
    const rpy = orientDeg();
    const nShown = drawClientPath();       // always redraw (no arm needed)
    setStatus(`orient [${rpy.map((x) => x.toFixed(0)).join(', ')}]° — ${nShown} pts above table · checking reachability…`);
    try {
      const r = await (await fetch('/api/robot/scan_preview', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ orientRpyDeg: rpy, incidenceDeg: 10 }),
      })).json();
      if (!r.ok) { setStatus(`drawn (connect arm for reachability: ${r.error})`); return; }
      const good = r.poses.filter((p) => p.reachable && p.position[2] >= -0.005);
      v.buildPath(good);
      setStatus(`orient [${rpy.map((x) => x.toFixed(0)).join(', ')}]° — ${good.length}/${r.n} reachable`);
    } catch (e) { setStatus(`drawn (backend unreachable: ${e.message})`); }
  }
  const genBtn = $('genPathBtn'); if (genBtn) genBtn.onclick = generatePath;

  const scanBtn = $('scanArmBtn');
  if (scanBtn) scanBtn.onclick = async () => {
    if (!scanWps.length) { setStatus('no scan path to send'); return; }
    const rpy = orientDeg();
    if (!confirm(`Run the scan at orient [${rpy.map((x) => x.toFixed(0)).join(', ')}]° (aim at part ±10°)?\n` +
                 `The arm will MOVE. Ensure the cell is clear and the E-stop is in reach.`)) return;
    setStatus('sending scan to the arm…');
    try {
      const r = await (await fetch('/api/robot/scan_trace', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ confirm: true, incidenceDeg: 10, orientRpyDeg: rpy, speedMms: 60 }),
      })).json();
      if (!r.ok || !r.started) { setStatus('arm refused: ' + (r.error || 'unknown')); return; }
      const poll = setInterval(async () => {
        try {
          const st = await (await fetch('/api/robot/follow_status')).json();
          if (st.running) { setStatus(`scanning ${st.completed || 0}/${st.total || 0}…`); return; }
          clearInterval(poll);
          setStatus(st.ok ? `scan complete — ${st.completed}/${st.total}` :
                    `scan ${st.aborted ? 'aborted' : 'failed'} at ${st.completed}/${st.total}${st.error ? ' — ' + st.error : ''}`);
        } catch (e) { clearInterval(poll); }
      }, 300);
    } catch (e) { setStatus('backend unreachable: ' + e.message); }
  };

  // camera view segment (orbit / scanner POV)
  const seg = $('viewSeg');
  if (seg) seg.querySelectorAll('button').forEach((b) => b.onclick = () => {
    seg.querySelectorAll('button').forEach((x) => x.classList.remove('on'));
    b.classList.add('on');
    v.setView(b.dataset.view);
  });

  reground();
  if (scanWps.length) drawClientPath();
  setStatus(scanWps.length ? 'ready — flip to orient, Generate path, Scan → arm' : 'no scan path (plan a part first)');
  window.__dbg = { v, orientDeg, generatePath };
  ready = true;

  // camera-follow: ALWAYS ON. The preview arm re-aims at the orbit look-at.
  let acc = 0;
  function onFrame(dt) {
    if (!ready || !v.arm.ready) return;
    acc += dt; if (acc < 0.05) return; acc = 0;
    v.solveIK(v.tipWorld(), v.controls.target.clone(), 8);
  }
})();
