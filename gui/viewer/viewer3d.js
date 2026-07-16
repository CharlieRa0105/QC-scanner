/*
 * viewer3d.js — reusable 3D scene module for the QC Scanner console.
 *
 * ONE implementation of the cell scene (workspace box + table + grid), the SR5
 * arm (URDF meshes + forward kinematics + a small numerical IK for PREVIEW
 * posing), the part mesh, the scan path, playback and cameras — consumed by
 * BOTH the main Run-tab viewer (viewer.js) and the debug popup (debug.js).
 * No copy-paste forks: anything scene-level belongs here.
 *
 * Frames: the scene is the `table` frame (metres, Z-up, origin under the arm
 * base). The workspace box + mount come from /api/config (Task 0's single
 * source of truth); helpers convert table-frame <-> arm-base-frame mm for the
 * debug UI (the arm base hangs at table (0,0,H), rolled 180deg, so
 *   arm = [x, -y, H - z] * 1000  and back).
 *
 * The in-browser IK is a damped CCD solver over the joint chain
 * (assets/arm/chain.json) used ONLY to pose the preview arm (aim the tool at a
 * target from a probe position). It is not the controller's solution and never
 * commands hardware — MoveIt remains the authority for real trajectories.
 *
 * dispose() tears the WebGL context down (renderer.dispose + forceContextLoss)
 * so a popup can open/close repeatedly without leaking contexts.
 */
'use strict';

(function () {
  const LIME = 0xc3ef00, PATH = 0x5c7300, AIM = 0x1f8a78;

  // ---------- config fetch (workspace box etc.) -----------------------------
  async function fetchConfig() {
    const FALLBACK = {
      workspace: { dims_mm: [2000, 750, 1200], mount: { base_xyz_mm: [1000, 375, 1200] }, table_z_mm: 0 },
      debug_shapes: { default_size_mm: 200, initial_z_mm: 600 },
      planner: { standoff_mm: 250 },
    };
    try {
      const r = await fetch('/api/config');
      const d = await r.json();
      const c = (d && d.config) || {};
      return {
        workspace: c.workspace || FALLBACK.workspace,
        debug_shapes: c.debug_shapes || FALLBACK.debug_shapes,
        planner: c.planner || FALLBACK.planner,
      };
    } catch (e) { return FALLBACK; }
  }

  function create(canvas, opts = {}) {
    const cfg = opts.config;                        // resolved config (fetchConfig())
    const ws = cfg.workspace;
    const dims = ws.dims_mm.map((v) => v / 1000);   // box, metres
    const H = ws.mount.base_xyz_mm[2] / 1000;       // mount height above table

    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xeef1f4);

    const orbitCam = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
    orbitCam.up.set(0, 0, 1);
    orbitCam.position.set(1.0, -1.0, 1.0);
    const scannerCam = new THREE.PerspectiveCamera(40, 1, 0.005, 100);
    scannerCam.up.set(0, 0, 1);
    const controls = new THREE.OrbitControls(orbitCam, canvas);
    controls.enableDamping = true;
    controls.target.set(0, 0, 0.3);

    scene.add(new THREE.HemisphereLight(0xffffff, 0xc8ccd2, 1.0));
    scene.add(new THREE.AmbientLight(0xffffff, 0.35));
    const key = new THREE.DirectionalLight(0xffffff, 0.55);
    key.position.set(1, -1, 2); scene.add(key);

    // ---------- cell: grid + table + WORKSPACE BOX (Task 0/1 config) --------
    const cellGroup = new THREE.Group(); scene.add(cellGroup);
    {
      const grid = new THREE.GridHelper(2.4, 48, 0xaab4c0, 0xd2d9e1);
      grid.rotation.x = Math.PI / 2;
      cellGroup.add(grid);
      // table surface = the box footprint at z=0
      const table = new THREE.Mesh(
        new THREE.PlaneGeometry(dims[0], dims[1]),
        new THREE.MeshStandardMaterial({ color: 0xe4e8ee, roughness: 0.95 })
      );
      table.position.z = -0.001;
      cellGroup.add(table);
      // translucent workspace box, arm-base-centred: x,y centred on the mount,
      // z from the table plane up to the mount.
      const box = new THREE.Mesh(
        new THREE.BoxGeometry(dims[0], dims[1], dims[2]),
        new THREE.MeshBasicMaterial({ color: 0x5b8dd6, transparent: true, opacity: 0.06, depthWrite: false })
      );
      box.position.set(0, 0, dims[2] / 2);
      cellGroup.add(box);
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(box.geometry),
        new THREE.LineBasicMaterial({ color: 0x5b8dd6, transparent: true, opacity: 0.5 })
      );
      edges.position.copy(box.position);
      cellGroup.add(edges);
    }

    // ---------- groups -------------------------------------------------------
    const partGroup = new THREE.Group(); scene.add(partGroup);
    const pathGroup = new THREE.Group(); scene.add(pathGroup);
    const aimGroup = new THREE.Group(); scene.add(aimGroup);
    const armGroup = new THREE.Group(); scene.add(armGroup);

    // ---------- SR5 arm: chain + meshes + FK + preview IK --------------------
    // Link pivots are built as a nested Object3D chain so setJoints() re-poses
    // the real meshes (needed for trace playback + camera-follow preview).
    const arm = { pivots: [], axes: [], chain: null, tip: null, ready: false };

    async function buildArm(base = 'assets/arm/') {
      let chain;
      try { chain = await (await fetch(base + 'chain.json')).json(); }
      catch (e) { console.warn('arm chain load failed', e); return; }
      arm.chain = chain;
      const loader = new THREE.STLLoader();
      const mat = new THREE.MeshStandardMaterial({ color: 0xc8ccd2, metalness: 0.55, roughness: 0.45 });

      // mount: table -> arm base (height + roll come from the URDF/config)
      const mount = new THREE.Object3D();
      mount.position.set(0, 0, H);
      mount.rotation.set(Math.PI, 0, 0, 'ZYX');
      armGroup.add(mount);

      let parent = mount;
      for (const link of chain.links) {
        const pivot = new THREE.Object3D();
        pivot.position.set(link.xyz[0], link.xyz[1], link.xyz[2]);
        parent.add(pivot);
        if (link.type === 'revolute') {
          arm.pivots.push(pivot);
          arm.axes.push(new THREE.Vector3(...link.axis).normalize());
        }
        await new Promise((resolve) => loader.load(base + link.mesh, (geom) => {
          geom.scale(0.001, 0.001, 0.001);
          pivot.add(new THREE.Mesh(geom, mat));
          resolve();
        }, undefined, () => resolve()));
        parent = pivot;
      }
      // tool tip frame (end of link6)
      arm.tip = new THREE.Object3D();
      parent.add(arm.tip);
      arm.ready = true;
      setJoints(chain.config || [0, 0, 0, 0, 0, 0]);
    }

    function setJoints(q) {
      if (!arm.ready && !arm.pivots.length) return;
      arm.pivots.forEach((p, i) => p.quaternion.setFromAxisAngle(arm.axes[i], q[i] || 0));
      scene.updateMatrixWorld(true);
    }
    function getJoints() { return arm.pivots.map((p, i) => { const a = arm.axes[i]; const e = new THREE.Vector3(); const angle = p.quaternion.angleTo(new THREE.Quaternion()); return angle * Math.sign(p.quaternion.dot(new THREE.Quaternion().setFromAxisAngle(a, 1)) || 1); }); }
    function tipWorld() { return arm.tip ? arm.tip.getWorldPosition(new THREE.Vector3()) : new THREE.Vector3(); }
    function tipDirWorld() {
      // the tool's local +Z (out of the flange) in world coordinates
      if (!arm.tip) return new THREE.Vector3(0, 0, -1);
      const q = arm.tip.getWorldQuaternion(new THREE.Quaternion());
      return new THREE.Vector3(0, 0, 1).applyQuaternion(q);
    }

    // Damped CCD: put the TOOL TIP at `pos` with the tool +Z aiming at `aim`.
    // Preview-quality (bounded iterations), never commands hardware.
    let ikState = [0, 0.4, 0.8, 0, 0.8, 0];   // warm start between solves
    function solveIK(pos, aim, iters = 40) {
      if (!arm.ready) return null;
      const q = ikState.slice();
      const target = pos.clone();
      for (let it = 0; it < iters; it++) {
        for (let j = arm.pivots.length - 1; j >= 0; j--) {
          setJoints(q);
          const tip = tipWorld();
          if (tip.distanceTo(target) < 0.002) break;
          const pivot = arm.pivots[j];
          const pw = pivot.getWorldPosition(new THREE.Vector3());
          const axisW = arm.axes[j].clone().applyQuaternion(pivot.getWorldQuaternion(new THREE.Quaternion())).normalize();
          const toTip = tip.clone().sub(pw), toTgt = target.clone().sub(pw);
          // project both onto the plane perpendicular to the joint axis
          toTip.sub(axisW.clone().multiplyScalar(toTip.dot(axisW)));
          toTgt.sub(axisW.clone().multiplyScalar(toTgt.dot(axisW)));
          if (toTip.lengthSq() < 1e-10 || toTgt.lengthSq() < 1e-10) continue;
          let ang = toTip.angleTo(toTgt);
          if (axisW.dot(new THREE.Vector3().crossVectors(toTip, toTgt)) < 0) ang = -ang;
          q[j] += Math.max(-0.3, Math.min(0.3, ang * 0.9));   // damped step
          q[j] = Math.max(-Math.PI, Math.min(Math.PI, q[j]));
        }
      }
      // aim pass: rotate wrist joints (4..6) so tool +Z points at `aim`
      if (aim) {
        for (let it = 0; it < 10; it++) {
          for (let j = 3; j < arm.pivots.length; j++) {
            setJoints(q);
            const tip = tipWorld();
            const want = aim.clone().sub(tip).normalize();
            const have = tipDirWorld();
            if (have.angleTo(want) < 0.01) break;
            const pivot = arm.pivots[j];
            const axisW = arm.axes[j].clone().applyQuaternion(pivot.getWorldQuaternion(new THREE.Quaternion())).normalize();
            const h = have.clone().sub(axisW.clone().multiplyScalar(have.dot(axisW)));
            const w = want.clone().sub(axisW.clone().multiplyScalar(want.dot(axisW)));
            if (h.lengthSq() < 1e-10 || w.lengthSq() < 1e-10) continue;
            let ang = h.angleTo(w);
            if (axisW.dot(new THREE.Vector3().crossVectors(h, w)) < 0) ang = -ang;
            q[j] += Math.max(-0.4, Math.min(0.4, ang));
          }
        }
      }
      setJoints(q);
      ikState = q.slice();
      return q;
    }

    // ---------- part + path --------------------------------------------------
    let waypoints = [];
    let scanner = null;

    function buildPart(part) {
      clearGroup(partGroup);
      if (!part || !part.vertices) return;
      const g = new THREE.BufferGeometry();
      const pos = new Float32Array(part.vertices.length * 3);
      part.vertices.forEach((v, i) => { pos[3 * i] = v[0]; pos[3 * i + 1] = v[1]; pos[3 * i + 2] = v[2]; });
      const idx = [];
      (part.triangles || []).forEach((t) => idx.push(t[0], t[1], t[2]));
      g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      g.setIndex(idx); g.computeVertexNormals();
      partGroup.add(new THREE.Mesh(g, new THREE.MeshStandardMaterial({
        color: 0x7c8a9c, metalness: 0.3, roughness: 0.55, side: THREE.DoubleSide })));
    }

    function buildPath(wps) {
      clearGroup(pathGroup); clearGroup(aimGroup);
      waypoints = (wps || []).map((w) => ({
        pos: new THREE.Vector3(...w.position),
        target: new THREE.Vector3(...(w.target || w.position)),
      }));
      if (!waypoints.length) { scanner && (scanner.visible = false); return; }
      pathGroup.add(new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(waypoints.map((w) => w.pos)),
        new THREE.LineBasicMaterial({ color: PATH })));
      const dot = new THREE.SphereGeometry(0.0025, 8, 8);
      const dm = new THREE.MeshBasicMaterial({ color: PATH });
      const am = new THREE.LineBasicMaterial({ color: AIM, transparent: true, opacity: 0.55 });
      waypoints.forEach((w) => {
        const s = new THREE.Mesh(dot, dm); s.position.copy(w.pos); pathGroup.add(s);
        aimGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([w.pos, w.target]), am));
      });
      if (!scanner) {
        scanner = new THREE.Mesh(new THREE.ConeGeometry(0.012, 0.03, 16),
          new THREE.MeshStandardMaterial({ color: LIME, emissive: 0x2a3a00 }));
        scene.add(scanner);
      }
      scanner.visible = true;
      placeAt(0);
    }

    // ---------- playback ------------------------------------------------------
    const play = { t: 0, on: false, speed: 6, poseArm: true, onStep: null, onDone: null };
    function placeAt(t) {
      if (!waypoints.length) return;
      const n = waypoints.length;
      t = Math.max(0, Math.min(n - 1, t));
      const i = Math.floor(t), f = t - i, j = Math.min(n - 1, i + 1);
      const pos = waypoints[i].pos.clone().lerp(waypoints[j].pos, f);
      const tgt = waypoints[i].target.clone().lerp(waypoints[j].target, f);
      if (scanner) { scanner.position.copy(pos); scanner.lookAt(tgt); scanner.rotateX(Math.PI / 2); }
      scannerCam.position.copy(pos); scannerCam.lookAt(tgt);
      if (play.poseArm && arm.ready) solveIK(pos, tgt, 14);
      play.t = t;
      if (play.onStep) play.onStep(t, n);
    }

    // ---------- layers / cameras / helpers ------------------------------------
    const groupsByLayer = { part: partGroup, path: pathGroup, targets: aimGroup, table: cellGroup, arm: armGroup };
    function setLayer(k, on) { if (groupsByLayer[k]) groupsByLayer[k].visible = on; }
    let view = 'orbit';
    function setView(v) { view = v; controls.enabled = (v === 'orbit'); if (scanner) scanner.visible = (v !== 'scanner') && waypoints.length > 0; }

    function frameView() {
      const box = new THREE.Box3();
      partGroup.traverse((o) => { if (o.isMesh) box.expandByObject(o); });
      waypoints.forEach((w) => box.expandByPoint(w.pos));
      if (box.isEmpty()) box.set(new THREE.Vector3(-0.4, -0.4, 0), new THREE.Vector3(0.4, 0.4, H));
      const c = box.getCenter(new THREE.Vector3());
      const r = Math.max(box.getSize(new THREE.Vector3()).length() * 0.6, 0.4);
      controls.target.copy(c);
      orbitCam.position.set(c.x + r, c.y - r, c.z + r * 0.8);
      controls.update();
    }

    function clearGroup(g) {
      while (g.children.length) {
        const o = g.children.pop();
        o.traverse && o.traverse((m) => { m.geometry && m.geometry.dispose(); m.material && m.material.dispose && m.material.dispose(); });
        g.remove(o);
      }
    }

    // table-frame (m) <-> arm-base-frame (mm) conversions for the debug UI.
    function tableToArmMm(v) { return [v.x * 1000, -v.y * 1000, (H - v.z) * 1000]; }
    function armMmToTable(a) { return new THREE.Vector3(a[0] / 1000, -a[1] / 1000, H - a[2] / 1000); }

    // ---------- loop / dispose -------------------------------------------------
    let lastW = 0, lastH = 0, raf = 0, disposed = false, last = performance.now();
    function tick(now) {
      if (disposed) return;
      const dt = (now - last) / 1000; last = now;
      const w = canvas.clientWidth || canvas.parentElement.clientWidth, h = canvas.clientHeight || canvas.parentElement.clientHeight;
      const W = opts.sizeToWindow ? window.innerWidth : w, Hh = opts.sizeToWindow ? window.innerHeight : h;
      if ((W !== lastW || Hh !== lastH) && W > 0 && Hh > 0) {
        lastW = W; lastH = Hh;
        renderer.setSize(W, Hh, !opts.sizeToWindow ? false : undefined);
        if (opts.sizeToWindow) renderer.setSize(W, Hh);
        [orbitCam, scannerCam].forEach((c) => { c.aspect = W / Hh; c.updateProjectionMatrix(); });
      }
      if (play.on && waypoints.length > 1) {
        let t = play.t + play.speed * dt;
        if (t >= waypoints.length - 1) { t = waypoints.length - 1; play.on = false; play.onDone && play.onDone(); }
        placeAt(t);
      }
      if (opts.onFrame) {
        // never let a consumer callback kill the render loop
        try { opts.onFrame(dt); } catch (e) { console.warn('onFrame error:', e); }
      }
      controls.update();
      renderer.render(scene, view === 'scanner' ? scannerCam : orbitCam);
      raf = requestAnimationFrame(tick);
    }
    raf = requestAnimationFrame(tick);

    function dispose() {
      disposed = true;
      cancelAnimationFrame(raf);
      controls.dispose();
      [partGroup, pathGroup, aimGroup, armGroup, cellGroup].forEach(clearGroup);
      renderer.dispose();
      renderer.forceContextLoss();     // release the WebGL context (popup reopen safety)
    }

    return {
      THREE, scene, renderer, canvas, controls, orbitCam, scannerCam,
      cfg, mountHeight: H,
      buildArm, setJoints, solveIK, tipWorld, arm,
      buildPart, buildPath, placeAt, play, waypoints: () => waypoints,
      setLayer, setView, frameView, tableToArmMm, armMmToTable,
      dispose,
    };
  }

  window.QCViewer = { create, fetchConfig };
})();
