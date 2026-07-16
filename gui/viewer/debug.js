/*
 * debug.js — the console's Debug 3D viewport (popup).
 *
 * Uses the SAME scene module as the main viewer (viewer3d.js). Adds:
 *  - placeable outline primitives (circle / square / triangle) with a
 *    translate/rotate gizmo (vendored THREE.TransformControls) and a live pose
 *    readout in the ARM-BASE frame (mm / deg)
 *  - "Trace outline": converts the selected outline into an ordered waypoint
 *    list (standoff above the outline plane, aiming at it; waypoints below the
 *    table or outside the workspace box are DROPPED and counted), previews the
 *    trace in the viewport (arm posed by the preview IK), and offers the same
 *    gated backend path as scan tracing (mock free; real arm needs
 *    QC_ALLOW_SCAN_TRACE=1 — the Task-9 ladder)
 *  - camera-follow (D2): the end-effector re-aims at the orbit camera's
 *    look-at point, throttled; optional gated send-to-arm. An explicit trace
 *    suspends camera-follow and it resumes afterwards.
 */
'use strict';

const $ = (id) => document.getElementById(id);
const setStatus = (t) => { $('statusText').textContent = t; };
let ready = false;   // set once main() finishes wiring (guards onFrame's TDZ)

(async function main() {
  const config = await QCViewer.fetchConfig();
  const v = QCViewer.create($('canvas'), { config, sizeToWindow: true, onFrame });
  const SIZE = (config.debug_shapes.default_size_mm || 200) / 1000;   // m
  const Z0 = (config.debug_shapes.initial_z_mm || 600) / 1000;        // m above table
  const STANDOFF = (config.planner.standoff_mm || 250) / 1000;        // m
  const DIMS = config.workspace.dims_mm.map((x) => x / 1000);

  await v.buildArm('assets/arm/');
  try {
    const bundle = await (await fetch('/api/viewer_bundle')).json();
    if (bundle.part) v.buildPart(bundle.part);
  } catch (e) { /* no part planned yet — fine for debug */ }
  v.frameView();
  setStatus('ready — add an outline');

  // ---------------- shapes -------------------------------------------------
  const shapes = [];        // { group, kind, id }
  let selected = null;
  let shapeSeq = 0;
  const shapeMat = () => new THREE.LineBasicMaterial({ color: 0x2b6cb8 });

  function outlinePoints(kind, size) {
    const pts = [];
    if (kind === 'circle') {
      const r = size / 2;
      for (let i = 0; i <= 48; i++) { const a = (i / 48) * 2 * Math.PI; pts.push(new THREE.Vector3(r * Math.cos(a), r * Math.sin(a), 0)); }
    } else if (kind === 'square') {
      const h = size / 2;
      [[-h, -h], [h, -h], [h, h], [-h, h], [-h, -h]].forEach((p) => pts.push(new THREE.Vector3(p[0], p[1], 0)));
    } else { // triangle (equilateral, edge = size)
      const rOut = size / Math.sqrt(3);
      for (let i = 0; i <= 3; i++) { const a = (i / 3) * 2 * Math.PI + Math.PI / 2; pts.push(new THREE.Vector3(rOut * Math.cos(a), rOut * Math.sin(a), 0)); }
    }
    return pts;
  }

  function addShape(kind) {
    const group = new THREE.Group();
    group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(outlinePoints(kind, SIZE)), shapeMat()));
    group.position.set(0, 0, Z0);          // spawn horizontal, centred, at Z0
    v.scene.add(group);
    const s = { group, kind, id: ++shapeSeq };
    shapes.push(s);
    select(s);
    renderChips();
    setStatus(`${kind} #${s.id} added — drag with the gizmo`);
  }

  function select(s) {
    selected = s;
    gizmo.detach();
    if (s) gizmo.attach(s.group);
    renderChips();
  }

  function deleteSelected() {
    if (!selected) return;
    gizmo.detach();
    v.scene.remove(selected.group);
    selected.group.traverse((o) => { o.geometry && o.geometry.dispose(); o.material && o.material.dispose && o.material.dispose(); });
    shapes.splice(shapes.indexOf(selected), 1);
    selected = shapes[shapes.length - 1] || null;
    if (selected) gizmo.attach(selected.group);
    renderChips();
    setStatus('shape deleted');
  }

  function renderChips() {
    const el = $('shapeChips');
    el.innerHTML = shapes.length ? '' : '<span style="font-size:11px;color:var(--ink-dim)">none yet</span>';
    shapes.forEach((s) => {
      const b = document.createElement('button');
      b.textContent = `${s.kind[0].toUpperCase()}${s.id}`;
      b.className = s === selected ? 'on' : '';
      b.onclick = () => select(s);
      el.appendChild(b);
    });
  }

  // gizmo (translate + rotate in 3D)
  const gizmo = new THREE.TransformControls(v.orbitCam, v.canvas);
  gizmo.setMode('translate');
  gizmo.addEventListener('dragging-changed', (e) => { v.controls.enabled = !e.value; });
  v.scene.add(gizmo);

  $('addCircle').onclick = () => addShape('circle');
  $('addSquare').onclick = () => addShape('square');
  $('addTriangle').onclick = () => addShape('triangle');
  $('delShape').onclick = deleteSelected;
  $('gizmoSeg').querySelectorAll('button').forEach((b) => b.onclick = () => {
    $('gizmoSeg').querySelectorAll('button').forEach((x) => x.classList.remove('on'));
    b.classList.add('on');
    gizmo.setMode(b.dataset.mode);
  });
  $('viewSeg').querySelectorAll('button').forEach((b) => b.onclick = () => {
    $('viewSeg').querySelectorAll('button').forEach((x) => x.classList.remove('on'));
    b.classList.add('on');
    v.setView(b.dataset.view);
  });

  // live pose readout — ARM-BASE frame, mm / deg (global constraint)
  const FLIP = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
  function updatePoseReadout() {
    if (!selected) { $('poseName').textContent = '—'; $('posePos').textContent = '—'; $('poseRot').textContent = '—'; return; }
    $('poseName').textContent = `${selected.kind} #${selected.id}`;
    const p = v.tableToArmMm(selected.group.position);
    $('posePos').textContent = p.map((x) => x.toFixed(1)).join('  ');
    const qArm = FLIP.clone().multiply(selected.group.quaternion);
    const e = new THREE.Euler().setFromQuaternion(qArm, 'XYZ');
    $('poseRot').textContent = [e.x, e.y, e.z].map((r) => (r * 180 / Math.PI).toFixed(1)).join('  ');
    const tip = v.tableToArmMm(v.tipWorld());
    $('tipPos').textContent = tip.map((x) => x.toFixed(1)).join('  ');
  }

  // ---------------- trace -----------------------------------------------------
  function outlineToWaypoints(s) {
    // POSITION-ONLY: the tool tip follows the wireframe outline EXACTLY (Ra: "just
    // position, I don't care about head rotation"). Waypoints are the outline
    // points themselves (no standoff, no aim) -- orientation is left to the arm
    // (the backend searches a reachable one per point). A dummy identity quat is
    // included only to satisfy the waypoint schema. Points outside the workspace
    // box are dropped + counted.
    const wps = [];
    let dropped = 0;
    const pts = outlinePoints(s.kind, SIZE);
    for (const lp of pts) {
      const world = lp.clone().applyQuaternion(s.group.quaternion).add(s.group.position);
      const inBox = Math.abs(world.x) <= DIMS[0] / 2 && Math.abs(world.y) <= DIMS[1] / 2 &&
                    world.z >= 0 && world.z <= DIMS[2];
      if (!inBox) { dropped++; continue; }
      wps.push({
        position: [world.x, world.y, world.z],
        quaternion: [0, 0, 0, 1],                 // ignored (position-only)
        target: [world.x, world.y, world.z],      // tip sits on the point (no aim)
      });
    }
    return { wps, dropped };
  }

  // "Trace outline" = PREVIEW ONLY. It builds the position-only wireframe path and
  // plays it in the viewport; NOTHING is sent to the arm. When you're happy with
  // the preview you press "Send to arm" (below). This replaces the old auto-send.
  let previewWps = null;
  let sending = false;
  $('traceBtn').onclick = () => {
    if (!selected) { setStatus('add / select an outline first'); return; }
    const { wps, dropped } = outlineToWaypoints(selected);
    if (!wps.length) { setStatus('outline entirely outside the workspace — nothing to trace'); $('sendBtn').disabled = true; return; }
    previewWps = wps;
    follow.suspended = true;                 // trace overrides camera-follow
    v.buildPath(wps);
    v.play.speed = 8;
    v.play.on = true;
    v.play.onDone = () => { follow.suspended = false; };
    $('sendBtn').disabled = false;
    setStatus(`preview: ${selected.kind} — ${wps.length} points` +
              (dropped ? ` (${dropped} dropped: out of workspace)` : '') +
              ' · press "Send to arm" when happy');
  };

  // "Send to arm" — commit the previewed path to the real arm, POSITION-ONLY
  // (the tip follows the wireframe; the backend searches a reachable orientation
  // per point). Gated: mock free, real arm needs QC_ALLOW_SCAN_TRACE=1.
  $('sendBtn').onclick = async () => {
    if (!previewWps || !previewWps.length) { setStatus('trace a preview first'); return; }
    if (sending) { setStatus('already sending'); return; }
    if (!confirm(`Send ${previewWps.length} points to the arm? It will MOVE (position-only). ` +
                 `Ensure the cell is clear and the E-stop is in reach.`)) return;
    sending = true;
    follow.suspended = true;
    setStatus(`sending ${previewWps.length} points to the arm…`);
    try {
      const r = await (await fetch('/api/robot/follow_path', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ confirm: true, waypoints: previewWps, positionOnly: true, speedMms: 60, settleS: 0.05 }),
      })).json();
      if (r.ok && r.started) {
        const poll = setInterval(async () => {
          try {
            const st = await (await fetch('/api/robot/follow_status')).json();
            if (st.running) { setStatus(`arm tracing ${st.completed || 0}/${st.total || 0}…`); return; }
            clearInterval(poll); sending = false; follow.suspended = false;
            setStatus(st.ok ? `arm trace complete — ${st.completed}/${st.total}` :
                      `arm trace ${st.aborted ? 'aborted' : 'failed'} at ${st.completed}/${st.total}${st.error ? ' — ' + st.error : ''}`);
          } catch (e) { clearInterval(poll); sending = false; follow.suspended = false; }
        }, 300);
      } else {
        sending = false; follow.suspended = false;
        setStatus(`arm refused: ${r.error || 'unknown'}`);
      }
    } catch (e) { sending = false; follow.suspended = false; setStatus('backend unreachable: ' + e.message); }
  };

  // ---------------- camera-follow (preview only) -----------------------------
  const follow = { preview: false, suspended: false };
  $('followTgl').onclick = () => {
    follow.preview = !follow.preview;
    $('followTgl').classList.toggle('on', follow.preview);
    setStatus(follow.preview ? 'camera-follow on — the arm re-aims at the view target (preview)' : 'camera-follow off');
  };

  // test/inspection handle (playwright + manual debugging from the console)
  window.__dbg = { v, shapes, follow, select, addShape, outlineToWaypoints, get selected() { return selected; } };

  ready = true;   // onFrame may now touch the consts above (see guard below)

  let followAccum = 0;
  function onFrame(dt) {
    if (!ready) return;   // first frames fire before init completes (TDZ guard)
    updatePoseReadout();
    if (!follow.preview || follow.suspended || !v.arm.ready) return;
    followAccum += dt;
    if (followAccum < 0.05) return;          // preview re-aim at <=20 Hz
    followAccum = 0;
    const aim = v.controls.target.clone();
    const tip = v.tipWorld();
    v.solveIK(tip, aim, 8);                  // preview only: re-aim, never commands the arm
  }
})();
