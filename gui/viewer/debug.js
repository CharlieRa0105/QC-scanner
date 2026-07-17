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
  let scanWps = [];                        // planner scan path (part frame)
  let domeInfo = null;                     // enclosing hemisphere {center,radius} (dome planner)
  try {
    const bundle = await (await fetch('/api/viewer_bundle')).json();
    if (bundle.part) v.buildPart(bundle.part);
    scanWps = bundle.waypoints || [];
    domeInfo = bundle.dome || null;
  } catch (e) { /* no part planned yet */ }
  v.frameView();

  // ---------------- enclosing hemisphere (debug overlay, toggle) ------------
  let domeMesh = null;
  function buildDome() {
    if (!domeInfo || domeMesh) return;
    const T = v.THREE;
    const geo = new T.SphereGeometry(domeInfo.radius, 48, 24, 0, Math.PI * 2, 0, Math.PI / 2);
    domeMesh = new T.Mesh(geo, new T.MeshBasicMaterial(
      { color: 0x5b8dd6, transparent: true, opacity: 0.10, side: T.DoubleSide, depthWrite: false }));
    domeMesh.add(new T.LineSegments(new T.WireframeGeometry(geo),
      new T.LineBasicMaterial({ color: 0x5b8dd6, transparent: true, opacity: 0.28 })));
    domeMesh.matrixAutoUpdate = false;     // we drive its matrix to follow the part
    domeMesh.visible = false;
    v.scene.add(domeMesh);
  }
  function updateDome() {
    if (!domeMesh) return;
    const T = v.THREE;
    // FIXED in the table frame: pole +Y -> +Z so the flat face lies on the table,
    // placed at the fitted centre. The dome does NOT follow the part's flip -- the
    // part rotates INSIDE a fixed scan dome.
    domeMesh.matrix.compose(
      new T.Vector3(domeInfo.center[0], domeInfo.center[1], domeInfo.center[2]),
      new T.Quaternion().setFromAxisAngle(new T.Vector3(1, 0, 0), Math.PI / 2),
      new T.Vector3(1, 1, 1));
  }
  buildDome(); updateDome();
  const domeChk = $('showDome');
  if (domeChk) domeChk.onchange = () => { if (domeMesh) domeMesh.visible = domeChk.checked; };

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
    // The scan path + dome are FIXED in the table frame (the dome already covers
    // every orientation), so a flip rotates ONLY the part inside them -- the path
    // is not dragged around, and no re-plan is needed.
    setStatus('part rotated inside the fixed scan dome');
  }
  function orientDeg() {
    const e = new THREE.Euler().setFromQuaternion(partQuat, 'XYZ');
    return [e.x * R2D, e.y * R2D, e.z * R2D];
  }
  const X = new THREE.Vector3(1, 0, 0), Y = new THREE.Vector3(0, 1, 0), Z = new THREE.Vector3(0, 0, 1);
  const wire = (id, ax, d) => { const b = $(id); if (b) b.onclick = () => flip(ax, d); };
  wire('flipX', X, 90); wire('flipY', Y, 90); wire('spinZ', Z, 90);
  const rb = $('flipReset');
  if (rb) rb.onclick = () => { partQuat = new THREE.Quaternion(); reground(); setStatus('orientation reset'); };

  // ---------------- path preview + scan ------------------------------------
  let drawnPoses = [];                      // the planner path currently on screen
  function drawClientPath() {
    // The scan path is a FIXED dome in the table frame (the bundle is grounded so it
    // already sits on the table), NOT glued to the part's flip rotation. Draw the
    // waypoints as-is; a part flip rotates only the part mesh, not this path.
    // line_id is carried through so buildPath draws one polyline per ring.
    drawnPoses = scanWps
      .map((w) => ({ position: w.position, target: (w.target || w.position),
                     line_id: w.line_id == null ? 0 : w.line_id }))
      .filter((w) => w.position[2] >= -0.001);
    v.buildPath(drawnPoses);
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
      v.setMountHeight(mm); reground(); drawClientPath();   // live, no backend round-trip
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

  reground();
  if (scanWps.length) drawClientPath();
  setStatus(scanWps.length ? 'ready — flip to orient, Preview to replay, Scan → arm' : 'no scan path (plan a part first)');
  window.__dbg = { v, orientDeg, replay };
  ready = true;

  // Camera-follow re-aims the preview arm at the orbit look-at -- EXCEPT during a
  // replay, when the arm follows the PATH (placeAt IK-poses it), so we must not
  // fight it.
  let acc = 0;
  function onFrame(dt) {
    if (!ready || !v.arm.ready) return;
    if (v.play.on) return;                 // replay owns the arm; skip camera-follow
    acc += dt; if (acc < 0.05) return; acc = 0;
    v.solveIK(v.tipWorld(), v.controls.target.clone(), 8);
  }
})();
