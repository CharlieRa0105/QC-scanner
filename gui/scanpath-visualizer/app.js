import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { buildRokaeSR5, solveIK, getLinkPoints, LINK_RADII } from './robot.js';

// ============================================================
//  ScanPath — metrology scan-path planner
// ============================================================

const $ = (id) => document.getElementById(id);

// ---- State ----
const state = {
  standoff: 250,        // mm  (25 cm)
  density: 2,           // 1..4
  smooth: true,
  modelMesh: null,      // THREE.Mesh of the imported part
  modelGroup: new THREE.Group(),
  pathGroup: new THREE.Group(),
  camGroup: new THREE.Group(),
  rayGroup: new THREE.Group(),
  waypoints: [],        // [{pos:Vector3, quat:Quaternion, normal:Vector3}]
  bbox: null,
  rawBox: null,         // part's untransformed bounds
  tableGroup: null,     // 3m x 2m table group
  partXform: { rotQuat: new THREE.Quaternion(), centered:true, manual:false, manualX:0, manualZ:0 },
  layers: { model:true, path:true, cams:true, rays:true, grid:false },
  // ---- simulation ----
  sim: {
    curve: null,        // CatmullRomCurve3 through waypoint positions
    surfCurve: null,    // CatmullRomCurve3 through surface targets (aim points)
    quats: [],          // per-waypoint quaternions (for slerp)
    rig: null,          // moving scanner icon group
    beam: null,         // scan beam mesh
    t: 0,               // 0..1 along path
    playing: false,
    speed: 1,
    loop: true,
    pov: false,
    length: 1,          // curve length (mm) for constant-speed travel
    robot: null,        // { root, joints, tcp } articulated SR5
    traj: null,         // precomputed [{angles, hits, unreachable}] per checkpoint
    robotBase: new THREE.Vector3(),
    showRobot: true,
    trueScale: true,    // real 1:1 SR5 scale (honest reach) vs fit-to-table demo
    useSlider: true,    // gantry carriage provides X travel; off = fixed base, arm's own X axis
    mountCm: null,      // user mount height (cm); null = auto
  },
};

// ============================================================
//  Three.js scene
// ============================================================
const canvas = $('canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias:true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = false;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0e13);

const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100000);
camera.position.set(600, 450, 700);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN };

// Lighting
scene.add(new THREE.HemisphereLight(0xbcd4e6, 0x1a2028, 0.9));
const key = new THREE.DirectionalLight(0xffffff, 1.4); key.position.set(500, 900, 600); scene.add(key);
const fill = new THREE.DirectionalLight(0x88aacc, 0.5); fill.position.set(-600, 200, -400); scene.add(fill);

// Grid (toggleable)
const grid = new THREE.GridHelper(2000, 40, 0x2a3542, 0x1a222c);
grid.material.transparent = true; grid.material.opacity = 0.5; grid.visible = false;
scene.add(grid);

// Axes gnomon
const axes = new THREE.AxesHelper(120);
scene.add(axes);

scene.add(state.modelGroup, state.pathGroup, state.camGroup, state.rayGroup);

// ============================================================
//  Resize + render loop
// ============================================================
function resize() {
  const r = canvas.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / r.height;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(canvas);
resize();

let lastT = performance.now(), frames = 0, fpsT = 0;
function loop(t) {
  const dt = t - lastT; lastT = t; frames++; fpsT += dt;
  if (fpsT > 500) { $('fpsText').textContent = Math.round(1000/(fpsT/frames)) + ' fps'; frames=0; fpsT=0; }
  advanceSim(dt);
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
}
requestAnimationFrame(loop);

// ============================================================
//  STEP loading  (occt-import-js -> WASM OpenCASCADE)
// ============================================================
let occt = null;
async function ensureOcct() {
  if (occt) return occt;
  setLoader(true, 'Initialising CAD kernel…');
  occt = await occtimportjs();
  return occt;
}

async function loadStepFile(file) {
  try {
    await ensureOcct();
    setLoader(true, 'Parsing STEP geometry…');
    const buf = new Uint8Array(await file.arrayBuffer());
    const result = occt.ReadStepFile(buf, null);
    if (!result || !result.success || !result.meshes || !result.meshes.length) {
      throw new Error('No solid geometry found in file.');
    }
    buildModel(result.meshes);
    $('filechip').textContent = file.name;
    $('dropHint').style.display = 'none';
    status(`Loaded "${file.name}" — ${result.meshes.length} solid(s)`);
    $('planBtn').disabled = false;
    frameView('iso');
  } catch (e) {
    console.error(e);
    status('Import failed: ' + e.message, true);
    alert('Could not read STEP file.\n' + e.message);
  } finally {
    setLoader(false);
  }
}

function buildModel(meshes) {
  clearGroup(state.modelGroup);
  clearGroup(state.pathGroup); clearGroup(state.camGroup); clearGroup(state.rayGroup);
  state.waypoints = [];
  resetReport();
  $('exportBtn').disabled = true;
  $('transport').classList.remove('show');
  setPlaying(false);
  if (state.sim.rig) { if (state.sim.rig.parent) state.sim.rig.parent.remove(state.sim.rig); disposeObj(state.sim.rig); state.sim.rig = null; state.sim.curve = null; }
  if (state.sim.robot) { scene.remove(state.sim.robot.root); disposeObj(state.sim.robot.root); state.sim.robot = null; }
  if (state.sim.gantry) { scene.remove(state.sim.gantry); disposeObj(state.sim.gantry); state.sim.gantry = null; }

  const mat = new THREE.MeshStandardMaterial({
    color: 0x8fa3b3, metalness: 0.55, roughness: 0.45,
    flatShading: false, side: THREE.DoubleSide,
  });
  const edgeMat = new THREE.LineBasicMaterial({ color: 0x3d4a58, transparent:true, opacity:0.6 });

  const merged = new THREE.Group();
  for (const m of meshes) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(m.attributes.position.array, 3));
    if (m.attributes.normal) g.setAttribute('normal', new THREE.Float32BufferAttribute(m.attributes.normal.array, 3));
    else g.computeVertexNormals();
    if (m.index) g.setIndex(new THREE.Uint32BufferAttribute(m.index.array, 1));
    const mesh = new THREE.Mesh(g, mat);
    merged.add(mesh);
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(g, 25), edgeMat);
    mesh.add(edges);
  }
  state.modelGroup.add(merged);
  state.modelMesh = merged;

  // remember the part's own local bounds (before any table transform)
  merged.updateMatrixWorld(true);
  state.rawBox = new THREE.Box3().setFromObject(merged);
  // reset transform for a freshly loaded part, seeded with the AUTO-ORIENTATION
  // (best scanning pose). Scale is untouched — only rotation + placement change.
  state.partXform = {
    rotQuat: computeAutoOrientation(state.rawBox),
    centered: true, manual: false, manualX: 0, manualZ: 0,
  };

  buildTable();
  placePartOnTable();
  applyLayers();
  status(`Loaded — auto-oriented for scanning. Drag the part to reposition.`);
}

// ============================================================
//  TABLE  — 3 m x 2 m worktable; its top surface is the floor (y = 0).
//  A 0.5 m strip along the near (-Z) edge holds a linear SLIDER rail that
//  moves the arm base along X. The remaining 1.5 m x 3 m is the PARTS AREA.
//  Nothing (part, scanner, arm links) may go below y = 0.
// ============================================================
const TABLE  = { x: 3000, z: 2000, thick: 40 };      // mm (3 m x 2 m)
// The arm hangs over the table CENTRE. Its X-travel (when the X-slider is on)
// spans the full table width; there is no reserved near-edge strip any more.
const SLIDER = {
  get xMin() { return -TABLE.x/2; },              // -1500
  get xMax() { return  TABLE.x/2; },              // +1500
};
// parts area = the whole table, centred (the part seats at the table centre,
// directly under the arm mount). A small margin is applied when clamping.
const PARTS = {
  get zMin() { return -TABLE.z/2; },    // -1000
  get zMax() { return  TABLE.z/2; },    // +1000
  get xMin() { return -TABLE.x/2; },
  get xMax() { return  TABLE.x/2; },
  get zCenter() { return 0; },          // table centre
};

function buildTable() {
  if (state.tableGroup) { scene.remove(state.tableGroup); disposeObj(state.tableGroup); }
  const g = new THREE.Group();
  const hx = TABLE.x/2, hz = TABLE.z/2;

  // table top outline (at y=0)
  const topPts = [
    new THREE.Vector3(-hx,0,-hz), new THREE.Vector3(hx,0,-hz),
    new THREE.Vector3(hx,0,hz), new THREE.Vector3(-hx,0,hz), new THREE.Vector3(-hx,0,-hz),
  ];
  g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(topPts),
        new THREE.LineBasicMaterial({ color: 0x5f6f7e })));

  // faint filled top so it reads as a surface
  const topGeo = new THREE.PlaneGeometry(TABLE.x, TABLE.z);
  topGeo.rotateX(-Math.PI/2);
  const top = new THREE.Mesh(topGeo, new THREE.MeshBasicMaterial({
    color: 0x16202a, transparent:true, opacity:0.55, side:THREE.DoubleSide }));
  top.position.y = -0.5;
  g.add(top);

  // legs (down to -thickness*… just a short skirt, purely visual)
  const legH = Math.min(TABLE.x, TABLE.z) * 0.28;
  const corners = [[-hx,-hz],[hx,-hz],[hx,hz],[-hx,hz]];
  const legMat = new THREE.LineBasicMaterial({ color: 0x3d4a58 });
  for (const [cx,cz] of corners) {
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(
      [new THREE.Vector3(cx,0,cz), new THREE.Vector3(cx,-legH,cz)]), legMat));
  }
  const skirt = topPts.map(p => new THREE.Vector3(p.x, -legH, p.z));
  g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(skirt), legMat));

  // grid on the table top (0.5 m spacing)
  const step = 500;
  const gmat = new THREE.LineBasicMaterial({ color: 0x243040, transparent:true, opacity:0.5 });
  const gpts = [];
  for (let x = -hx; x <= hx; x += step) { gpts.push(new THREE.Vector3(x,0,-hz), new THREE.Vector3(x,0,hz)); }
  for (let z = -hz; z <= hz; z += step) { gpts.push(new THREE.Vector3(-hx,0,z), new THREE.Vector3(hx,0,z)); }
  g.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(gpts), gmat));

  // NB: no reserved near-edge strip any more — the arm hangs over the table
  // CENTRE and the part seats there too. A small centre cross marks the mount.
  const cMat = new THREE.LineBasicMaterial({ color: 0x2b6cb0, transparent:true, opacity:0.6 });
  g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(
    [new THREE.Vector3(-120,0,0), new THREE.Vector3(120,0,0)]), cMat));
  g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(
    [new THREE.Vector3(0,0,-120), new THREE.Vector3(0,0,120)]), cMat));

  // labels
  g.add(makeTextSprite(`${(TABLE.x/1000).toFixed(0)} m`, new THREE.Vector3(0, 12, -hz-90), 0x8fb0c8));
  g.add(makeTextSprite(`${(TABLE.z/1000).toFixed(0)} m`, new THREE.Vector3(hx+120, 12, 0), 0x8fb0c8));
  g.add(makeTextSprite('table centre', new THREE.Vector3(0, 12, 160), 0x7fd0b8));

  scene.add(g);
  state.tableGroup = g;
}

// canvas text sprite for dimension labels. The canvas is sized to the MEASURED
// text width (with padding) instead of a fixed 256 px, so longer labels like
// "parts 1.5 × 3 m" are no longer clipped, and the sprite scale follows the
// canvas aspect ratio so the text isn't stretched.
function makeTextSprite(text, pos, color=0xffffff) {
  const font = 'bold 40px monospace';
  const pad = 16;
  // measure first on a scratch context
  const meas = document.createElement('canvas').getContext('2d');
  meas.font = font;
  const textW = Math.ceil(meas.measureText(text).width);

  const cv = document.createElement('canvas');
  cv.width = textW + pad * 2;
  cv.height = 64;
  const cx = cv.getContext('2d');
  cx.fillStyle = '#' + color.toString(16).padStart(6,'0');
  cx.font = font; cx.textAlign = 'center'; cx.textBaseline = 'middle';
  cx.fillText(text, cv.width / 2, cv.height / 2);

  const tex = new THREE.CanvasTexture(cv);
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map:tex, transparent:true, depthTest:false }));
  spr.position.copy(pos);
  // keep a constant on-screen text height (~75 mm) and scale width to match aspect
  const h = 75;
  spr.scale.set(h * (cv.width / cv.height), h, 1);
  spr.userData.isLabel = true;
  return spr;
}

// ============================================================
//  Auto-orientation — choose the best axis-aligned pose for scanning.
//  For a rail-mounted arm that sweeps along X and reaches from the −Z side, the
//  ideal placement puts the part's LONGEST dimension along X (so the slider does
//  the long travel), its SHORTEST along Y (lie flat — lowest, easiest to reach),
//  and the middle along Z (shallow depth). This only permutes/flips axes, so the
//  part's SCALE is never changed. Returns a quaternion to apply to the part.
// ============================================================
function computeAutoOrientation(rawBox) {
  const s = rawBox.getSize(new THREE.Vector3());
  const dims = [s.x, s.y, s.z];
  const order = [0, 1, 2].sort((a, b) => dims[b] - dims[a]); // [longest, mid, shortest]
  const longest = order[0], mid = order[1], shortest = order[2];

  // Build a rotation whose columns send the part's longest local axis → world X,
  // shortest → world Y, mid → world Z. (makeBasis columns = images of e_x,e_y,e_z.)
  const cols = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
  cols[longest].set(1, 0, 0);   // longest local axis → +X
  cols[shortest].set(0, 1, 0);  // shortest local axis → +Y
  cols[mid].set(0, 0, 1);       // middle local axis → +Z
  const m = new THREE.Matrix4().makeBasis(cols[0], cols[1], cols[2]);
  // guarantee a proper rotation (det +1); flip Z column if it came out reflected
  if (m.determinant() < 0) { cols[2].negate(); m.makeBasis(cols[0], cols[1], cols[2]); }
  return new THREE.Quaternion().setFromRotationMatrix(m);
}

// ============================================================
//  Apply the part transform (rotation quaternion + placement) and seat it on
//  the table so its lowest point rests exactly on y = 0.
// ============================================================
function placePartOnTable() {
  const merged = state.modelMesh;
  if (!merged) return;
  const xf = state.partXform;

  // reset, then apply the cumulative rotation (auto-orient + any 90° steps)
  merged.position.set(0,0,0);
  merged.quaternion.copy(xf.rotQuat);
  merged.updateMatrixWorld(true);

  // seat on table: shift so min-Y sits on y=0
  let box = new THREE.Box3().setFromObject(merged);
  const c = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  merged.position.y -= box.min.y;        // bottom on the table surface

  // Position the part within the PARTS AREA (never on the slider strip).
  merged.position.x -= c.x;              // centred in X to start
  if (xf.manual) {
    // explicit user-dragged position (parts-area coords)
    merged.position.x += xf.manualX;
    merged.position.z += (xf.manualZ - c.z);
  } else if (xf.centered) {
    // centre of the parts area (X = 0, Z = parts-area centre)
    merged.position.z += (PARTS.zCenter - c.z);
  } else {
    // seat toward the far (+Z) edge of the parts area
    const targetZ = PARTS.zMax - size.z*0.5 - 100;
    merged.position.z += (targetZ - c.z);
  }
  merged.updateMatrixWorld(true);

  // clamp fully inside the parts area (off the slider strip, on the table)
  const nb = new THREE.Box3().setFromObject(merged);
  if (nb.min.x < PARTS.xMin) merged.position.x += (PARTS.xMin - nb.min.x);
  if (nb.max.x > PARTS.xMax) merged.position.x -= (nb.max.x - PARTS.xMax);
  if (nb.min.z < PARTS.zMin) merged.position.z += (PARTS.zMin - nb.min.z);
  if (nb.max.z > PARTS.zMax) merged.position.z -= (nb.max.z - PARTS.zMax);
  merged.updateMatrixWorld(true);

  state.bbox = new THREE.Box3().setFromObject(merged);
}

// ============================================================
//  PATH PLANNER
//  Strategy: sample the part surface, cluster sample points into
//  a set of viewpoints on scan "rings" at increasing height, offset
//  each viewpoint along the local surface normal by the standoff
//  distance, orient the scanner to face the surface, then order the
//  viewpoints into a continuous serpentine path around the object.
// ============================================================
function planPath() {
  if (!state.modelMesh) return;
  setLoader(true, 'Planning path & checking arm clearance…');
  // let the loader paint
  setTimeout(() => {
    try { doPlan(); }
    catch (e) { console.error(e); status('Planning failed: '+e.message, true); }
    finally { setLoader(false); }
  }, 30);
}

// ============================================================
//  SURFACE-GRID SCAN PATH PLANNER — helpers
//
//  Algorithm: multi-face surface rastering. Each reachable face of the part
//  (TOP, the NEAR −Z side, and BOTH ±X ends) is swept with a regular grid of
//  rays at a fixed surface spacing. Every hit becomes a viewpoint offset along
//  the hit's true LOCAL normal by the standoff, then filtered for reachability,
//  line-of-sight (occlusion) and spatial duplicates, and finally ordered as a
//  rail-aware boustrophedon along X.
//
//  Why this over the previous approaches: the normal-cluster planner emitted one
//  viewpoint per normal direction (under-covering large faces), and the earlier
//  single-axis "arc" raster aimed every ray at one column-centre point, so it
//  really only saw the TOP and a sliver of the near side and never reached the
//  ends of a long part. Rastering each reachable face independently gives even,
//  complete coverage of the sides and the full length of the part, while the
//  per-hit local-normal offset keeps the 20–30 cm standoff correct on curved and
//  angled faces.
// ============================================================

// Cast one ray (from `origin`, travelling along `dir` INTO the part) and return
// the first surface hit as an oriented, standoff-offset viewpoint (or null if the
// hit is unreachable for a rail-mounted arm). The outward surface normal and the
// standoff camera position are derived from the hit's true local normal, so the
// standoff is correct on curved and angled faces.
function castViewpoint(origin, dir, meshes, raycaster, standoff, minCamY) {
  raycaster.set(origin, dir);
  raycaster.far = 3e5;
  const hits = raycaster.intersectObjects(meshes, true);
  if (!hits.length) return null;

  const surfPt = hits[0].point.clone();
  let normal = hits[0].face
    ? hits[0].face.normal.clone().transformDirection(hits[0].object.matrixWorld).normalize()
    : dir.clone().negate();
  if (normal.dot(dir) > 0) normal.negate();            // force outward-facing

  // ---- reachability filter (rail-mounted arm, approaches from the −Z side) ----
  if (normal.z > 0.60 && normal.y < 0.25) return null; // far (+Z) face — unreachable
  if (normal.y < -0.55) return null;                    // underside / bottom cap

  const camPos = surfPt.clone().addScaledVector(normal, standoff);
  // keep the scanner above the table — clamp low viewpoints up rather than
  // rejecting them (rejecting killed all side coverage on short parts).
  if (camPos.y < minCamY) camPos.y = minCamY;
  return { surf: surfPt, normal, camPos };
}

// Line-of-sight test: the scanner at camPos must actually SEE surfPt without a
// nearer piece of geometry blocking it. Cast from camPos toward surfPt; the first
// hit should land at ~standoff (the intended surface). A much closer hit means
// something occludes the view (or camPos is buried) → reject.
function viewpointHasLineOfSight(camPos, surfPt, meshes, raycaster) {
  const toSurf = surfPt.clone().sub(camPos);
  const dist = toSurf.length();
  if (dist < 1e-3) return false;
  raycaster.set(camPos, toSurf.divideScalar(dist));
  raycaster.far = dist * 1.5;
  const hits = raycaster.intersectObjects(meshes, true);
  if (!hits.length) return false;                       // nothing there to scan
  return hits[0].distance > dist * 0.6;                 // first hit is the target
}

// ============================================================
//  Main planner
// ============================================================
function doPlan() {
  clearGroup(state.pathGroup); clearGroup(state.camGroup); clearGroup(state.rayGroup);
  state.waypoints = [];

  const box = state.bbox.clone();
  const size = box.getSize(new THREE.Vector3());
  const centre = box.getCenter(new THREE.Vector3());
  const standoff = state.standoff;
  const density = state.density;

  const meshes = [];
  state.modelMesh.traverse(o => { if (o.isMesh) meshes.push(o); });
  if (!meshes.length) { status('No geometry found.', true); return; }

  const raycaster = new THREE.Raycaster();
  // minimum scanner height above the table — a clamp, not a reject.
  const minCamY = Math.max(size.y * 0.03, 25);

  // ---- grid spacing — scales with the PART SIZE (not the standoff) ----
  // Spacing is a fraction of the part's largest dimension, so a small part gets
  // a fine grid and a large part a coarse one — always enough samples per face.
  // (Tying this to the standoff broke small parts: when the part was smaller than
  // the standoff, every face got a 1×1 grid and the de-dup merged all hits into a
  // single viewpoint.) Higher density → finer spacing.
  const spacing = size3(box) / (6 + density * 4);       // ~1/10 … 1/22 of the part
  const pad = size3(box) * 0.5 + standoff;              // launch rays from outside
  const inset = Math.min(size.x, size.y, size.z) * 0.02 + 0.5;
  const nAxis = (lo, hi) => THREE.MathUtils.clamp(Math.round((hi - lo) / spacing) + 1, 2, 80);

  // The reachable faces for a rail-mounted arm approaching from −Z:
  //   • TOP        (+Y) — rays travel −Y, rastered over X × Z
  //   • NEAR SIDE  (−Z) — rays travel +Z, rastered over X × Y  (the vertical side
  //                        facing the rail — this is what was being missed)
  //   • BOTH ENDS  (±X) — rays travel ∓X, rastered over Z × Y  (the ends of a
  //                        long part — the slider reaches them by moving in X)
  const xLo = box.min.x + inset, xHi = box.max.x - inset;
  const yLo = box.min.y + inset, yHi = box.max.y - inset;
  const zLo = box.min.z + inset, zHi = box.max.z - inset;

  const rasters = [
    { dir:new THREE.Vector3(0,-1,0), // TOP
      uLo:xLo, uHi:xHi, vLo:zLo, vHi:zHi,
      pt:(u,v)=>new THREE.Vector3(u, box.max.y + pad, v) },
    { dir:new THREE.Vector3(0,0,1),  // NEAR SIDE (−Z)
      uLo:xLo, uHi:xHi, vLo:yLo, vHi:yHi,
      pt:(u,v)=>new THREE.Vector3(u, v, box.min.z - pad) },
    { dir:new THREE.Vector3(-1,0,0), // +X END
      uLo:zLo, uHi:zHi, vLo:yLo, vHi:yHi,
      pt:(u,v)=>new THREE.Vector3(box.max.x + pad, v, u) },
    { dir:new THREE.Vector3(1,0,0),  // −X END
      uLo:zLo, uHi:zHi, vLo:yLo, vHi:yHi,
      pt:(u,v)=>new THREE.Vector3(box.min.x - pad, v, u) },
  ];

  // ---- raster every reachable face, collect line-of-sight-valid viewpoints ----
  const cell = spacing * 0.7;                    // global de-dupe cell size
  const seen = new Set();
  const keyOf = (p) => `${Math.round(p.x/cell)},${Math.round(p.y/cell)},${Math.round(p.z/cell)}`;
  const vps = [];
  for (const r of rasters) {
    const nU = nAxis(r.uLo, r.uHi), nV = nAxis(r.vLo, r.vHi);
    for (let iu = 0; iu < nU; iu++) {
      const u = nU === 1 ? (r.uLo+r.uHi)/2 : THREE.MathUtils.lerp(r.uLo, r.uHi, iu/(nU-1));
      for (let iv = 0; iv < nV; iv++) {
        const v = nV === 1 ? (r.vLo+r.vHi)/2 : THREE.MathUtils.lerp(r.vLo, r.vHi, iv/(nV-1));
        const vp = castViewpoint(r.pt(u,v), r.dir, meshes, raycaster, standoff, minCamY);
        if (!vp) continue;
        if (!viewpointHasLineOfSight(vp.camPos, vp.surf, meshes, raycaster)) continue;
        const key = keyOf(vp.surf);
        if (seen.has(key)) continue;              // another face already covered this patch
        seen.add(key);
        vps.push(vp);
      }
    }
  }

  if (!vps.length) { status('No reachable surface found — try a different placement or density.', true); return; }

  // ---- rail-aware boustrophedon ordering ----
  // Bucket viewpoints into X columns (the slider travel direction) so the slider
  // advances monotonically left→right; within each column sweep bottom→top and
  // alternate the direction each column so the arm's Y-Z motion continues from
  // where the previous column ended — no long returns.
  const nCols = Math.max(4, nAxis(box.min.x, box.max.x));
  const xMin = box.min.x - pad, xRange = Math.max((box.max.x + pad) - xMin, 1);
  const buckets = Array.from({ length: nCols }, () => []);
  for (const vp of vps) {
    const ci = THREE.MathUtils.clamp(Math.floor((vp.camPos.x - xMin) / xRange * nCols), 0, nCols - 1);
    buckets[ci].push(vp);
  }
  const ordered = [];
  buckets.forEach((col, i) => {
    if (!col.length) return;
    // sweep up the column; near-side (further from rail, −Z) first at each height
    col.sort((a, b) => (a.camPos.y - b.camPos.y) || (a.camPos.z - b.camPos.z));
    ordered.push(...(i % 2 === 0 ? col : col.reverse()));
  });

  // ---- build oriented waypoints (scanner −Z looks at the surface point) ----
  const wps = ordered.map(p => ({
    pos: p.camPos.clone(),
    quat: quatLookAt(p.camPos, p.surf),
    surf: p.surf.clone(),
    normal: p.normal.clone(),
  }));

  state.waypoints = wps;
  drawPath(wps);
  drawCameras(wps);
  applyLayers();
  report(wps, size);
  buildSim(wps);
  $('exportBtn').disabled = false;
  // don't reframe on replan — keep the user's current view (FIT button resets it)
  status(`Path generated — ${wps.length} waypoints (multi-face raster)`);
}

// Orientation whose -Z axis points from `eye` toward `target`, +Y ~ world up.
// Uses THREE.Matrix4.lookAt, which is the same math Object3D.lookAt uses and
// always yields a correct right-handed frame with -Z on the target.
const _m4 = new THREE.Matrix4();
const _up = new THREE.Vector3(0,1,0);
const _altUp = new THREE.Vector3(0,0,1);
function quatLookAt(eye, target) {
  const dir = target.clone().sub(eye).normalize();
  // choose an up that isn't parallel to the view direction (avoid pole gimbal)
  const up = Math.abs(dir.dot(_up)) > 0.98 ? _altUp : _up;
  // Matrix4.lookAt(eye, target, up) builds a basis with -Z pointing eye->target
  _m4.lookAt(eye, target, up);
  return new THREE.Quaternion().setFromRotationMatrix(_m4);
}

// ============================================================
//  Fixed on-screen marker sizes (mm) — constant regardless of the model size,
//  so the camera icons / waypoint dots / moving scanner look the same whether
//  the part is a 50 mm pin or a 1.4 m casting.
// ============================================================
const ICON_UNIT = 45;   // camera frustum icon at each waypoint
const RIG_UNIT  = 55;   // the moving scanner rig
const DOT_R     = 6;    // waypoint dot radius
const START_R   = 12;   // start marker radius

// ============================================================
//  Draw: path line
// ============================================================
function drawPath(wps) {
  const pts = wps.map(w => w.pos);
  let curvePts = pts;
  if (state.smooth && pts.length > 2) {
    const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal', 0.5);
    curvePts = curve.getPoints(pts.length * 8);
  }
  const geo = new THREE.BufferGeometry().setFromPoints(curvePts);
  // gradient-ish colour by using vertex colors along the path
  const colors = [];
  const cA = new THREE.Color(0x4dd8c4), cB = new THREE.Color(0x7d6cff);
  for (let i=0;i<curvePts.length;i++){
    const t=i/(curvePts.length-1); const c=cA.clone().lerp(cB,t);
    colors.push(c.r,c.g,c.b);
  }
  geo.setAttribute('color', new THREE.Float32BufferAttribute(colors,3));
  const mat = new THREE.LineBasicMaterial({ vertexColors:true, transparent:true, opacity:0.95 });
  const line = new THREE.Line(geo, mat);
  state.pathGroup.add(line);

  // waypoint dots
  const dotGeo = new THREE.SphereGeometry(DOT_R, 8, 8);
  const dotMat = new THREE.MeshBasicMaterial({ color: 0x4dd8c4 });
  pts.forEach(p => { const d=new THREE.Mesh(dotGeo,dotMat); d.position.copy(p); state.pathGroup.add(d); });

  // start marker
  const startMat = new THREE.MeshBasicMaterial({ color: 0xe8a33d });
  const s = new THREE.Mesh(new THREE.SphereGeometry(START_R,10,10), startMat);
  s.position.copy(pts[0]); state.pathGroup.add(s);
}

// ============================================================
//  Draw: Blender-style camera icons at each waypoint
//  (a small pyramid/frustum + a "up" triangle on top)
// ============================================================
function makeCameraIcon(scaleUnit) {
  const g = new THREE.Group();
  const s = scaleUnit;                 // body scale
  const mat = new THREE.LineBasicMaterial({ color: 0x9fd8ff });
  const focal = s * 1.6;               // frustum depth
  const w = s * 1.0, h = s * 0.75;     // sensor half-extents

  // frustum apex at origin (this is the lens/sensor position), opening toward +Z... 
  // but our scanner forward is -Z, so build the cone opening along -Z.
  const apex = new THREE.Vector3(0,0,0);
  const f = -focal;
  const corners = [
    new THREE.Vector3( w,  h, f),
    new THREE.Vector3(-w,  h, f),
    new THREE.Vector3(-w, -h, f),
    new THREE.Vector3( w, -h, f),
  ];
  const seg = [];
  // apex -> 4 corners
  corners.forEach(c => { seg.push(apex.clone(), c.clone()); });
  // rectangle
  for (let i=0;i<4;i++){ seg.push(corners[i].clone(), corners[(i+1)%4].clone()); }
  // "up" triangle marker on top edge (the Blender camera tell)
  const tri = [
    new THREE.Vector3(-w*0.55, h, f), new THREE.Vector3(w*0.55, h, f),
    new THREE.Vector3(w*0.55, h, f), new THREE.Vector3(0, h*1.7, f),
    new THREE.Vector3(0, h*1.7, f), new THREE.Vector3(-w*0.55, h, f),
  ];
  seg.push(...tri);

  const geo = new THREE.BufferGeometry().setFromPoints(seg);
  g.add(new THREE.LineSegments(geo, mat));
  // small solid dot at lens
  g.add(new THREE.Mesh(new THREE.SphereGeometry(s*0.18,6,6), new THREE.MeshBasicMaterial({color:0x9fd8ff})));
  return g;
}

function drawCameras(wps) {
  const unit = ICON_UNIT;
  // show a subset if very dense (keeps viewport readable)
  const stride = wps.length > 120 ? Math.ceil(wps.length/120) : 1;
  wps.forEach((w, i) => {
    if (i % stride !== 0 && i !== wps.length-1) return;
    const icon = makeCameraIcon(unit);
    icon.position.copy(w.pos);
    icon.quaternion.copy(w.quat);
    state.camGroup.add(icon);
  });
}

// ============================================================
//  Report
// ============================================================
function report(wps, size) {
  let len = 0;
  for (let i=1;i<wps.length;i++) len += wps[i].pos.distanceTo(wps[i-1].pos);
  // standoff deviation
  const devs = wps.map(w => w.pos.distanceTo(w.surf) - state.standoff);
  const mean = devs.reduce((a,b)=>a+b,0)/devs.length;
  const sigma = Math.sqrt(devs.reduce((a,b)=>a+(b-mean)**2,0)/devs.length);
  // crude coverage estimate: fraction of viewpoints that hit real surface within band
  const inBand = devs.filter(d => Math.abs(d) < 10).length;
  const cov = Math.round(100 * inBand / devs.length);

  $('stWp').innerHTML = wps.length;
  $('stLen').innerHTML = (len/10).toFixed(0) + '<small> cm</small>';
  $('stCov').innerHTML = cov + '<small>%</small>';
  $('stDev').innerHTML = sigma.toFixed(1) + '<small> mm</small>';
}
function resetReport(){ ['stWp','stLen','stCov','stDev'].forEach(id=>$(id).textContent='—'); }

// ============================================================
//  Export
// ============================================================
function exportPath() {
  if (!state.waypoints.length) return;
  const payload = {
    generator: 'ScanPath',
    units: 'mm',
    frame: 'machine (Y-up, model centred at origin)',
    standoff_mm: state.standoff,
    density: state.density,
    generated: new Date().toISOString(),
    waypoints: state.waypoints.map((w,i) => ({
      i,
      position: [round(w.pos.x), round(w.pos.y), round(w.pos.z)],
      quaternion: [round(w.quat.x,5), round(w.quat.y,5), round(w.quat.z,5), round(w.quat.w,5)],
      target: [round(w.surf.x), round(w.surf.y), round(w.surf.z)],
    })),
  };
  const blob = new Blob([JSON.stringify(payload,null,2)], {type:'application/json'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'scanpath.json';
  a.click();
  URL.revokeObjectURL(a.href);
  status('Exported scanpath.json');
}

// ============================================================
//  SIMULATION  — animate a scanner rig traveling the path
// ============================================================
function buildSim(wps) {
  const sim = state.sim;
  // the rig is a child of the old flange and gets disposed with the old robot;
  // just drop our references here.
  if (sim.rig && sim.rig.parent) sim.rig.parent.remove(sim.rig);
  if (sim.rig) disposeObj(sim.rig);
  sim.curve = null; sim.surfCurve = null; sim.quats = []; sim.rig = null; sim.beam = null;

  if (wps.length < 2) { $('transport').classList.remove('show'); return; }

  // path as a curve for smooth constant-speed travel; match the drawn line
  const pts = wps.map(w => w.pos.clone());
  sim.curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal', 0.5);
  sim.length = sim.curve.getLength();
  sim.quats = wps.map(w => w.quat.clone());
  // parallel curve through the surface targets, so the scanner's aim point
  // moves smoothly and the scanner keeps facing the object between waypoints
  const surfs = wps.map(w => w.surf.clone());
  sim.surfCurve = new THREE.CatmullRomCurve3(surfs, false, 'centripetal', 0.5);

  // ---- build the Rokae SR5 arm first (the scanner mounts on its flange) ----
  buildRobot();

  // ---- scanner rig — driven directly along the planned path ----
  // The scanner is placed each frame on the EXACT path pose (position from the
  // path curve, orientation from quatLookAt(pos → surface target)) rather than
  // inheriting the arm's IK solution. This guarantees the scanner tracks the
  // drawn path precisely and its optical axis (local -Z) always points at the
  // surface — IK solver error can no longer let the scanner drift off the path
  // or tilt. The arm is still driven by IK underneath as a visual illustration
  // of how a real cell would reach each pose. The rig lives in world (scene)
  // space so its transform is set absolutely, not relative to the flange.
  const unit = RIG_UNIT;
  const rig = new THREE.Group();
  const icon = makeCameraIcon(unit);
  icon.traverse(o => { if (o.material) o.material = o.material.clone(); if (o.material?.color) o.material.color.set(0xffd27d); });
  rig.add(icon);
  // device body sits BEHIND the lens (local +Z, away from the viewing direction)
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(unit*1.1, unit*0.8, unit*0.9),
    new THREE.MeshStandardMaterial({ color:0x2a3542, metalness:0.6, roughness:0.4 })
  );
  body.position.set(0, 0, unit*0.6);
  rig.add(body);
  sim.beam = null;

  scene.add(rig);          // world-space: pose is set exactly from the path
  sim.rig = rig;
  sim.t = 0;

  placeRig(0);

  $('transport').classList.add('show');
  $('scrub').value = 0;
  updateTelem(0);
}

// ---- build the SR5 arm mounted on the slider rail ----
function buildRobot() {
  const sim = state.sim;
  if (sim.robot) { scene.remove(sim.robot.root); disposeObj(sim.robot.root); sim.robot = null; }

  const chain = buildRokaeSR5();
  scene.add(chain.root);
  sim.robot = chain;

  const box = state.bbox;
  const size = box.getSize(new THREE.Vector3());
  const centre = box.getCenter(new THREE.Vector3());
  const reach = chain.reach;                     // ~919 mm (from URDF)

  // The arm is mounted UPSIDE-DOWN, hanging over the CENTRE OF THE TABLE (X=0,
  // Z=0) at the mount height, and hangs down to reach the part.
  //   • Slider ON  — a gantry carriage moves the base along X (Z stays at 0).
  //   • Slider OFF — the base is FIXED at table centre and the arm covers X with
  //                  its own joints (base yaw J1 + reach).
  const railZ = 0;                                   // table centre-line (Z)
  const zSpan = Math.max(Math.abs(box.max.z - railZ), Math.abs(box.min.z - railZ));
  // with the slider off the base can't translate, so it must also reach across X
  const xSpan = sim.useSlider ? 0 : Math.max(Math.abs(box.max.x), Math.abs(box.min.x));
  const horiz = Math.hypot(xSpan, zSpan) + state.standoff;
  const needed = Math.hypot(horiz, box.max.y) + size.y*0.15;

  let scale = 1;
  if (!sim.trueScale && needed > reach * 0.95) { scale = needed / (reach * 0.95); }
  chain.root.scale.setScalar(scale);
  sim.robotScale = scale;
  sim.railZ = railZ;

  // INVERT the arm: rotate the root π about X so its base sits at the top and the
  // links hang downward toward the part (composes with the model's built-in Z-up
  // → Y-up rotation so "robot up" points to world −Y).
  chain.root.rotation.x = Math.PI;

  // Mount height. By default it's auto (high enough to clear the part + standoff,
  // but within the arm's downward reach). The user can override it with the Mount
  // height slider; a manual value is clamped so the mount never sits inside the part.
  const R = reach * scale;
  const autoY = THREE.MathUtils.clamp(
    Math.max(box.max.y + state.standoff + size.y * 0.15, R * 0.6),
    R * 0.55, R * 0.95
  );
  if (sim.mountCm == null) {
    sim.mountY = autoY;                                  // auto — reflect it on the slider
    const cm = Math.round(autoY / 10);
    const el = $('mountHeight');
    if (el) { el.value = cm; $('mountHeightVal').textContent = cm + ' cm'; }
  } else {
    sim.mountY = THREE.MathUtils.clamp(sim.mountCm * 10, box.max.y + 20, 3000);
  }

  chain.root.visible = sim.showRobot;

  // cache the part's meshes + a raycaster for link-vs-part collision testing
  sim.partMeshes = [];
  state.modelMesh?.traverse(o => { if (o.isMesh) sim.partMeshes.push(o); });
  sim.collRay = new THREE.Raycaster();
  sim.clearance = Math.max(size3(box) * 0.03, 20) * scale;
  sim.partBox = new THREE.Box3().setFromObject(state.modelMesh)
    .expandByScalar(sim.clearance + 100 * scale);

  // place the base on the gantry at the start-of-path X (trajectory moves it in X)
  const startX = THREE.MathUtils.clamp(state.waypoints[0]?.pos.x ?? 0, SLIDER.xMin, SLIDER.xMax);
  setBaseX(chain, startX);
  sim.robotBase.copy(chain.root.position);

  // build the overhead gantry frame + the carriage marker on it
  buildGantry();
  buildCarriage(chain);

  // precompute the full continuous joint trajectory (with slider X per checkpoint)
  precomputeArmTrajectory();
}

// position the arm base. With the slider ON it moves along X (gantry carriage);
// with it OFF the base is fixed at table centre (X=0) and the arm reaches X itself.
// Z and Y are always the table-centre mount point.
function setBaseX(chain, x) {
  const sim = state.sim;
  const bx = sim.useSlider ? THREE.MathUtils.clamp(x, SLIDER.xMin, SLIDER.xMax) : 0;
  chain.root.position.set(bx, sim.mountY ?? 0, sim.railZ ?? 0);
  chain.root.updateMatrixWorld(true);
  return bx;
}

// overhead mount, centred on the table.
//   • Slider ON  — a twin-rail beam runs the full X travel at the mount height,
//                  carried by two end posts down to the table.
//   • Slider OFF — a single fixed pillar drops from the mount point to the table.
function buildGantry() {
  const sim = state.sim;
  if (sim.gantry) { scene.remove(sim.gantry); disposeObj(sim.gantry); sim.gantry = null; }
  const y = sim.mountY, z = sim.railZ ?? 0, off = 60;
  const g = new THREE.Group();
  const beamMat = new THREE.LineBasicMaterial({ color: 0x4dd8c4 });
  const postMat = new THREE.LineBasicMaterial({ color: 0x3d4a58 });
  const line = (a, b, m) => new THREE.Line(new THREE.BufferGeometry().setFromPoints([a, b]), m);

  if (sim.useSlider) {
    // twin rails at the mount height, spanning the full X travel through centre
    g.add(line(new THREE.Vector3(SLIDER.xMin, y, z-off), new THREE.Vector3(SLIDER.xMax, y, z-off), beamMat));
    g.add(line(new THREE.Vector3(SLIDER.xMin, y, z+off), new THREE.Vector3(SLIDER.xMax, y, z+off), beamMat));
    for (const x of [SLIDER.xMin, SLIDER.xMax]) {          // end posts to the table
      g.add(line(new THREE.Vector3(x, 0, z), new THREE.Vector3(x, y, z), postMat));
      g.add(line(new THREE.Vector3(x-off, 0, z), new THREE.Vector3(x+off, 0, z), postMat));
    }
    g.add(makeTextSprite('X-slider rail', new THREE.Vector3(0, y + 60, z), 0x6fd0e8));
  } else {
    // single fixed central pillar from the mount point down to the table
    g.add(line(new THREE.Vector3(0, 0, z), new THREE.Vector3(0, y, z), postMat));
    // a small mount plate square at the top
    for (const [dx,dz] of [[-off,-off],[off,-off],[off,off],[-off,off]]) {
      g.add(line(new THREE.Vector3(0, y, z), new THREE.Vector3(dx, y, z+dz), postMat));
    }
    // foot cross on the table
    g.add(line(new THREE.Vector3(-off, 0, z), new THREE.Vector3(off, 0, z), postMat));
    g.add(line(new THREE.Vector3(0, 0, z-off), new THREE.Vector3(0, 0, z+off), postMat));
    g.add(makeTextSprite('fixed centre mount', new THREE.Vector3(0, y + 60, z), 0x6fd0e8));
  }
  scene.add(g);
  sim.gantry = g;
}

// a small carriage box drawn at the base so the slider position is visible
function buildCarriage(chain) {
  if (chain._carriage) return;
  const s = 90 * (state.sim.robotScale || 1);
  const pts = [
    [-s,2,-s],[s,2,-s],[s,2,s],[-s,2,s],[-s,2,-s],
  ].map(p => new THREE.Vector3(...p));
  const carriage = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineBasicMaterial({ color: 0x4dd8c4 }));
  chain.root.add(carriage);
  chain._carriage = carriage;
}

// ---- robust collision test -------------------------------------------------
// Sample points densely along every link segment. A point counts as a collision
// if it is INSIDE the part or within the clearance margin of the surface. This
// catches joints/links penetrating the body, not just the thin centerline
// crossing a face.
//
// Better inside/outside test: rather than trusting a single ray's parity (which
// breaks on non-watertight meshes, coincident faces, or a ray that grazes an
// edge), we cast several rays in independent directions and take a MAJORITY VOTE
// on the odd/even crossing count. A stray ray that leaks through a crack or hits
// a degenerate face is outvoted, so the test is reliable on the imperfect
// triangle soup that STEP tessellation often produces. The clearance test also
// uses more directions (cube diagonals as well as axes) so a link skimming the
// surface at an oblique angle is still caught.
const _axisDirs = [
  new THREE.Vector3(1,0,0), new THREE.Vector3(-1,0,0),
  new THREE.Vector3(0,1,0), new THREE.Vector3(0,-1,0),
  new THREE.Vector3(0,0,1), new THREE.Vector3(0,0,-1),
];
const _diagDirs = [
  new THREE.Vector3( 1, 1, 1), new THREE.Vector3(-1, 1, 1),
  new THREE.Vector3( 1,-1, 1), new THREE.Vector3( 1, 1,-1),
  new THREE.Vector3(-1,-1, 1), new THREE.Vector3(-1, 1,-1),
  new THREE.Vector3( 1,-1,-1), new THREE.Vector3(-1,-1,-1),
].map(v => v.normalize());

// Majority-vote inside test. coarse=true uses 3 rays (cheap, for per-frame /
// base-search use); fine uses 6. A point is inside when a strict majority of the
// rays report an odd number of surface crossings.
function pointInsideMesh(pt, meshes, ray, coarse=false) {
  const dirs = coarse ? _axisDirs.slice(0, 3) : _axisDirs;
  let odd = 0;
  for (const d of dirs) {
    ray.set(pt, d);
    ray.far = Infinity;
    if (ray.intersectObjects(meshes, true).length % 2 === 1) odd++;
  }
  return odd * 2 > dirs.length;      // strict majority of rays say "inside"
}
function pointTooClose(pt, meshes, ray, clearance, coarse=false) {
  const dirs = coarse ? _axisDirs.slice(0, 3) : _axisDirs.concat(_diagDirs);
  for (const d of dirs) {
    ray.set(pt, d);
    ray.far = clearance;
    const hits = ray.intersectObjects(meshes, true);
    if (hits.length && hits[0].distance < clearance) return true;
  }
  return false;
}

// returns number of link sample points that collide with the part.
// coarse=true uses fewer samples/rays (for the base-placement search).
function linkCollisionCount(chain, coarse=false) {
  const sim = state.sim;
  const meshes = sim.partMeshes;
  if (!meshes || !meshes.length) return 0;
  const ray = sim.collRay;
  const box = sim.partBox;
  const scale = sim.robotScale || 1;
  const pts = getLinkPoints(chain);
  let count = 0;

  for (let i = 0; i < pts.length - 1; i++) {
    const a = pts[i], b = pts[i+1];
    const seg = b.clone().sub(a);
    const len = seg.length();
    if (len < 1e-3) continue;
    const radius = (LINK_RADII[i] || 40) * scale;
    const spacing = coarse ? Math.max(radius, 40) : Math.max(radius*0.6, 20);
    const steps = Math.max(coarse ? 2 : 3, Math.ceil(len / spacing));
    for (let s = 0; s <= steps; s++) {
      const p = a.clone().addScaledVector(seg, s/steps);
      // FLOOR CONSTRAINT: nothing may pass below the table plane (y = 0)
      if (p.y < -1) { count++; break; }
      if (box && !box.containsPoint(p)) {
        if (box.distanceToPoint(p) > (radius + sim.clearance)) continue;
      }
      const inside = pointInsideMesh(p, meshes, ray, coarse);
      const close = inside ? false : pointTooClose(p, meshes, ray, radius + sim.clearance, coarse);
      if (inside || close) { count++; break; }
    }
  }
  return count;
}

// ============================================================
//  ARM TRAJECTORY (precomputed once, played back smoothly)
//  Instead of solving IK every frame (which snapped the arm through the
//  part), we solve a sequence of evenly-spaced checkpoints ONCE, each
//  continuing from the previous joint state so the motion is continuous.
//  Playback just interpolates the stored joint angles — no teleporting.
// ============================================================
function precomputeArmTrajectory() {
  const sim = state.sim;
  const chain = sim.robot;
  if (!chain) return;

  const n = Math.max(48, Math.min(300, state.waypoints.length * 3));
  sim.traj = [];
  sim.trajN = n;

  setArmNeutral(chain);
  let lastSafe = chain.joints.map(j => angleOf(j));   // last collision-free config

  for (let s = 0; s < n; s++) {
    const t = s / (n - 1);
    const p = poseAt(t);

    // SLIDER: move the carriage in X to track the scanner's X, so the arm only
    // has to reach in the Y-Z plane. Clamp to the rail travel limits.
    const baseX = setBaseX(chain, p.pos.x);

    // start each solve from the last SAFE configuration for continuity
    lastSafe.forEach((a,k) => setJointAngle(chain.joints[k], a));
    chain.root.updateWorldMatrix(true, true);

    solveIKContinuous(chain, p.pos, p.surf);
    let hits = linkCollisionCount(chain);
    if (hits > 0) hits = resolveCollisionLocally(chain, p.pos, p.surf, hits);

    let angles, unreachable;
    if (hits === 0) {
      // safe pose found — accept it and remember it
      angles = chain.joints.map(j => angleOf(j));
      lastSafe = angles;
      const err = tcpError(chain, p.pos);
      unreachable = err > chain.reach * (sim.robotScale||1) * 0.14;
    } else {
      // NO collision-free solution: do NOT let the arm enter the part.
      // Hold the last safe configuration and flag this segment as skipped so
      // the tool never penetrates the geometry.
      angles = lastSafe.slice();
      unreachable = true;                 // shown as out-of-reach / skipped
      hits = 0;                           // we are NOT colliding (we held back)
    }

    sim.traj.push({ angles, baseX, hits, unreachable });
  }
}

// read a joint's scalar angle about its own axis (works for any axis sign)
function angleOf(j) {
  const q = j.group.quaternion;
  const raw = 2 * Math.acos(THREE.MathUtils.clamp(q.w, -1, 1));  // >= 0
  if (raw < 1e-6) return 0;
  const s = Math.sqrt(1 - q.w*q.w);
  const ax = new THREE.Vector3(q.x/s, q.y/s, q.z/s);   // rotation axis (unit)
  return ax.dot(j.axis) >= 0 ? raw : -raw;             // sign relative to joint axis
}
function setJointAngle(j, a) {
  j.group.quaternion.setFromAxisAngle(j.axis, THREE.MathUtils.clamp(a, j.min, j.max));
}

// a stable starting pose (shoulder forward, elbow bent up) for the real SR5
function setArmNeutral(chain) {
  const preset = [0, 0.6, -1.0, 0, 0.9, 0];   // j1..j6 (rad), matches URDF axes
  chain.joints.forEach((j,k) => setJointAngle(j, preset[k] || 0));
  chain.root.updateWorldMatrix(true, true);
}

// CCD but with small damping so a call nudges from the current pose toward the
// target — repeated calls across checkpoints yield continuous motion.
function solveIKContinuous(chain, pos, aim) {
  solveIK(chain, pos, aim, 6);      // fewer iters + inherit previous state
}

// Try to find a collision-free configuration that still reaches the pose,
// searching a spread of shoulder/elbow/base seeds. Returns hits (0 if solved).
function resolveCollisionLocally(chain, pos, aim, hits) {
  const sim = state.sim;
  const saved = chain.joints.map(j => j.group.quaternion.clone());
  let bestQ = saved, bestHits = hits, bestErr = tcpError(chain, pos);

  const shoulder = [0, 0.3, -0.3, 0.6, -0.6, 1.0, -1.0, 1.4];
  const base     = [0, 0.5, -0.5, 1.0, -1.0];
  for (const yb of base) {
    for (const sb of shoulder) {
      chain.joints.forEach((j,k) => j.group.quaternion.copy(saved[k]));
      chain.joints[0].group.rotateOnAxis(chain.joints[0].axis, yb);
      chain.joints[1].group.rotateOnAxis(chain.joints[1].axis, sb*0.5);
      chain.joints[2].group.rotateOnAxis(chain.joints[2].axis, sb);
      solveIK(chain, pos, aim, 7);
      const h = linkCollisionCount(chain);
      const err = tcpError(chain, pos);
      // strongly prefer zero collisions; among those, best reach accuracy
      const better = (h < bestHits) || (h === bestHits && err < bestErr);
      if (better) {
        bestHits = h; bestErr = err;
        bestQ = chain.joints.map(j=>j.group.quaternion.clone());
        if (h === 0 && err < sim.clearance*2) { chain.joints.forEach((j,k)=>j.group.quaternion.copy(bestQ[k])); return 0; }
      }
    }
  }
  chain.joints.forEach((j,k) => j.group.quaternion.copy(bestQ[k]));
  return bestHits;
}

// Drive the arm so its end-effector (flange/TCP) sits exactly on the scanner's
// current path pose, keeping the arm and the scanner rigidly connected on screen.
// The precomputed collision-safe trajectory is used only as a smooth, continuous
// SEED; a short live IK refinement then closes the flange onto the exact pose so
// the tool never drifts away from the scanner it is meant to be carrying.
function driveRobot(t) {
  const sim = state.sim;
  const chain = sim.robot;
  if (!chain || !sim.showRobot) return;

  const p = poseAt(t);

  // Put the rail carriage directly under the scanner so the arm only has to
  // reach in the Y-Z plane, then solve IK onto the scanner's EXACT path pose.
  setBaseX(chain, p.pos.x);

  // The arm keeps its previous-frame joint state as the IK seed (natural
  // continuity — consecutive path poses are close together). We iterate CCD
  // until the flange/TCP actually lands on the scanner, so the arm visibly
  // carries the camera along the path instead of trailing behind it.
  let err = tcpError(chain, p.pos);
  const goal = Math.max(size3(state.bbox) * 0.004, 1.5);   // ~a few mm
  for (let pass = 0; pass < 8 && err > goal; pass++) {
    solveIK(chain, p.pos, p.surf, 12);
    err = tcpError(chain, p.pos);
  }

  // Honest status: if the flange still can't get close, it's a true reach limit.
  const reachTol = chain.reach * (sim.robotScale || 1) * 0.14;
  const unreachable = err > reachTol;
  const hits = unreachable ? 0 : (linkCollisionCount(chain, true) > 0 ? 1 : 0);  // coarse: cheap per-frame

  setArmCollisionState(chain, hits, unreachable);
  sim.currentCollision = hits;
  sim.currentUnreachable = unreachable;
}

function tcpError(chain, pos) {
  return chain.tcp.getWorldPosition(new THREE.Vector3()).distanceTo(pos);
}

// tint the arm: red on collision, dim grey when the pose is unreachable
function setArmCollisionState(chain, hits, unreachable) {
  const stateKey = hits > 0 ? 'coll' : (unreachable ? 'unreach' : 'ok');
  if (chain._collVisual === stateKey) return;      // avoid churn
  chain._collVisual = stateKey;
  chain.root.traverse(o => {
    const m = o.material;
    if (!m || !m.color) return;
    if (o.userData.baseColor === undefined) o.userData.baseColor = m.color.getHex();
    let col = o.userData.baseColor;
    if (stateKey === 'coll') col = 0xc0392b;         // red
    else if (stateKey === 'unreach') col = 0x4a5560; // dim grey
    m.color.setHex(col);
    m.opacity = stateKey === 'unreach' ? 0.35 : 1.0;
    m.transparent = stateKey === 'unreach';
  });
}

// map global t (0..1) to an interpolated pose. Orientation is derived LIVE from
// the interpolated scanner position toward the interpolated surface target, so
// the scanner always points at the object — even between waypoints.
function poseAt(t) {
  const sim = state.sim;
  const wps = state.waypoints;
  const n = wps.length;
  const tc = THREE.MathUtils.clamp(t, 0, 1);
  const f = tc * (n - 1);
  const i = Math.min(Math.floor(f), n - 2);
  const lf = f - i;

  const pos = sim.curve.getPointAt(tc);
  // interpolate the target point the scanner is aiming at
  const surf = sim.surfCurve
    ? sim.surfCurve.getPointAt(tc)
    : wps[i].surf.clone().lerp(wps[i+1].surf, lf);
  // orientation: always look from pos toward the surface target
  const quat = quatLookAt(pos, surf);
  const wpIndex = Math.round(f);
  return { pos, quat, surf, wpIndex };
}

function placeRig(t) {
  const sim = state.sim;
  if (!sim.curve) return;
  const p = poseAt(t);

  // Place the scanner EXACTLY on the planned path: position on the path curve,
  // orientation from quatLookAt so its local -Z always points at the surface
  // target. This is the same pose used to draw the path and the static camera
  // icons, so the moving scanner tracks them precisely and its aim is always
  // correct — independent of any IK solver error.
  if (sim.rig) {
    sim.rig.position.copy(p.pos);
    sim.rig.quaternion.copy(p.quat);
  }

  // Drive the arm along its precomputed IK trajectory purely as a visual
  // illustration of how the cell would reach the pose (it may lag/hold on
  // unreachable points — that no longer affects the scanner's own pose).
  driveRobot(t);

  if (sim.pov) driveCameraPOV(p);
  updateTelem(t, p);
}

function updateTelem(t, p) {
  p = p || poseAt(t);
  const n = state.waypoints.length;
  $('tWp').textContent = `${Math.min(p.wpIndex+1, n)}/${n}`;
  if (p.surf) {
    const cm = (p.pos.distanceTo(p.surf) / 10);
    $('tStand').textContent = cm.toFixed(1) + ' cm';
    // warn colour if outside 20–30 band
    $('tStand').style.color = (cm < 19.5 || cm > 30.5) ? 'var(--warn)' : 'var(--accent)';
  }
  // arm status indicator
  const cs = $('tColl');
  if (cs) {
    if (state.sim.currentCollision > 0) { cs.textContent = 'COLLISION ✗'; cs.style.color = '#c0392b'; }
    else if (state.sim.currentUnreachable) { cs.textContent = 'HELD (no safe path)'; cs.style.color = '#e8a33d'; }
    else { cs.textContent = 'CLEAR ✓'; cs.style.color = 'var(--accent)'; }
  }
  // slider carriage X position (metres)
  const sx = $('tSlider');
  if (sx && state.sim.robot) {
    const xm = state.sim.robot.root.position.x / 1000;
    sx.textContent = xm.toFixed(2) + ' m';
  }
}

function advanceSim(dtMs) {
  const sim = state.sim;
  if (!sim.playing || !sim.curve) return;
  // constant linear speed: cover the path in a base time scaled by 1/speed.
  // base traversal ≈ path_length / (referenceSpeed). Use a sensible mm/s.
  const mmPerSec = (size3(state.bbox) * 0.9) * sim.speed;   // scales with part size
  const dt = dtMs / 1000;
  sim.t += (mmPerSec * dt) / sim.length;
  if (sim.t >= 1) {
    if (sim.loop) sim.t -= 1;
    else { sim.t = 1; setPlaying(false); }
  }
  $('scrub').value = Math.round(sim.t * 1000);
  placeRig(sim.t);
}

function setPlaying(on) {
  const sim = state.sim;
  sim.playing = on;
  $('playIcon').style.display = on ? 'none' : 'block';
  $('pauseIcon').style.display = on ? 'block' : 'none';
  $('playBtn').classList.toggle('active', on);
}

// ---- POV: fly the view camera through the scanner (its real mounted pose) ----
let savedCam = null;
function driveCameraPOV(p) {
  const sim = state.sim;
  if (sim.rig) {
    // use the scanner's actual world transform (its exact path pose)
    const lens = sim.rig.getWorldPosition(new THREE.Vector3());
    const q = sim.rig.getWorldQuaternion(new THREE.Quaternion());
    const fwd = new THREE.Vector3(0,0,-1).applyQuaternion(q); // scanner views along -Z
    camera.position.copy(lens);
    controls.target.copy(lens.clone().addScaledVector(fwd, state.standoff));
  } else {
    camera.position.copy(p.pos);
    const fwd = new THREE.Vector3(0,0,-1).applyQuaternion(p.quat);
    controls.target.copy(p.pos.clone().addScaledVector(fwd, state.standoff));
  }
}
function togglePOV() {
  const sim = state.sim;
  sim.pov = !sim.pov;
  $('rideBtn').classList.toggle('active', sim.pov);
  if (sim.pov) {
    savedCam = { pos: camera.position.clone(), tgt: controls.target.clone() };
    controls.enableRotate = false;                 // lock orbit while riding
    placeRig(sim.t);
  } else {
    controls.enableRotate = true;
    if (savedCam) { camera.position.copy(savedCam.pos); controls.target.copy(savedCam.tgt); }
  }
}


function size3(box){ const s=box.getSize(new THREE.Vector3()); return Math.max(s.x,s.y,s.z)||100; }

function frameView(which='iso') {
  let box = state.bbox || new THREE.Box3(new THREE.Vector3(-100,-100,-100), new THREE.Vector3(100,100,100));
  // include the table so the whole cell frames nicely
  if (state.tableGroup) box = box.clone().union(new THREE.Box3().setFromObject(state.tableGroup));
  // include the robot in the framing so the arm is visible
  if (state.sim.robot && state.sim.showRobot) {
    box = box.clone().union(new THREE.Box3().setFromObject(state.sim.robot.root));
  }
  const c = box.getCenter(new THREE.Vector3());
  const r = size3(box) * 1.3;
  const cam = { pos:new THREE.Vector3() };
  switch(which){
    case 'top':   cam.pos.set(c.x, c.y + r*1.4, c.z+0.001); break;
    case 'front': cam.pos.set(c.x, c.y, c.z + r*1.4); break;
    case 'right': cam.pos.set(c.x + r*1.4, c.y, c.z); break;
    default:      cam.pos.set(c.x + r, c.y + r*0.8, c.z + r);
  }
  camera.position.copy(cam.pos);
  controls.target.copy(c);
  controls.update();
}

// ============================================================
//  Layers
// ============================================================
function applyLayers() {
  state.modelGroup.visible = state.layers.model;
  state.pathGroup.visible  = state.layers.path;
  state.camGroup.visible   = state.layers.cams;
  state.rayGroup.visible   = state.layers.rays;
  grid.visible             = state.layers.grid;
}

// ============================================================
//  UI wiring
// ============================================================
$('importBtn').onclick = () => $('fileInput').click();
$('fileInput').onchange = e => { if (e.target.files[0]) loadStepFile(e.target.files[0]); };
$('planBtn').onclick = planPath;
$('exportBtn').onclick = exportPath;

$('standoff').oninput = e => { state.standoff = parseFloat(e.target.value)*10; $('standoffVal').textContent = e.target.value+' cm'; };
$('density').oninput = e => {
  state.density = parseInt(e.target.value);
  $('densityVal').textContent = ['','low','medium','high','ultra'][state.density];
};

document.querySelectorAll('[data-smooth]').forEach(el => el.onclick = () => {
  document.querySelectorAll('[data-smooth]').forEach(x=>x.classList.remove('on'));
  el.classList.add('on');
  state.smooth = el.dataset.smooth === '1';
  $('smoothVal').textContent = state.smooth ? 'on':'off';
  if (state.waypoints.length) doPlan();
});

document.querySelectorAll('[data-layer]').forEach(el => el.onclick = () => {
  const k = el.dataset.layer;
  state.layers[k] = !state.layers[k];
  el.classList.toggle('on', state.layers[k]);
  applyLayers();
});

document.querySelectorAll('[data-robot]').forEach(el => el.onclick = () => {
  state.sim.showRobot = !state.sim.showRobot;
  el.classList.toggle('on', state.sim.showRobot);
  if (state.sim.robot) {
    state.sim.robot.root.visible = state.sim.showRobot;
    if (state.sim.showRobot) placeRig(state.sim.t);
  }
});

// ---- placement controls (rotate X/Y/Z · centre) ----
// NOTE: placement changes never reframe the camera — the view is only reset by
// the FIT / preset view buttons (top-right). This keeps your current viewpoint
// while you rotate, centre, or drag the part.
function applyPlacement() {
  if (!state.modelMesh) return;
  placePartOnTable();
  // if a path was already generated, regenerate against the new placement
  if (state.waypoints.length) planPath();
}

// rotate the part 90° about a WORLD axis (pre-multiply so it's applied in world
// space, on top of the current orientation) — works for X, Y and Z.
function rotatePart(axis) {
  if (!state.modelMesh) return;
  const q = new THREE.Quaternion().setFromAxisAngle(axis, Math.PI / 2);
  state.partXform.rotQuat.premultiply(q);
  state.partXform.manual = false;   // re-seat within the parts area after a turn
  applyPlacement();
}
$('rotXBtn').onclick = () => rotatePart(new THREE.Vector3(1,0,0));
$('rotYBtn').onclick = () => rotatePart(new THREE.Vector3(0,1,0));
$('rotZBtn').onclick = () => rotatePart(new THREE.Vector3(0,0,1));

$('placeCenter').onclick = () => {
  if (!state.modelMesh) return;
  state.partXform.centered = !state.partXform.centered;
  state.partXform.manual = false;   // preset placement overrides manual drag
  $('placeCenter').classList.toggle('on', state.partXform.centered);
  applyPlacement();
};

$('trueScaleTgl').onclick = () => {
  state.sim.trueScale = !state.sim.trueScale;
  $('trueScaleTgl').classList.toggle('on', state.sim.trueScale);
  if (state.waypoints.length) planPath();   // rebuild arm at new scale
};

$('sliderTgl').onclick = () => {
  state.sim.useSlider = !state.sim.useSlider;
  $('sliderTgl').classList.toggle('on', state.sim.useSlider);
  if (state.waypoints.length) planPath();   // rebuild arm mount + trajectory
};

// Mount height: update the label live while dragging; rebuild the arm on release
// (rebuilding on every input tick would be too heavy).
$('mountHeight').oninput = e => {
  state.sim.mountCm = parseInt(e.target.value);
  $('mountHeightVal').textContent = e.target.value + ' cm';
};
$('mountHeight').onchange = () => { if (state.waypoints.length) planPath(); };

// ---- always-on part drag (grab the part body, slide in X-Z) ----
// Uses only core three.js raycasting. A pointerdown that hits the part starts a
// drag (and suspends orbit for that gesture); a pointerdown on empty space is
// left to OrbitControls, so there is NO mode button and no on-model gizmo.
const partDrag = {
  active:false, ray:new THREE.Raycaster(),
  plane:new THREE.Plane(new THREE.Vector3(0,1,0), 0), grab:new THREE.Vector3(),
};
function ndcFromEvent(e) {
  const r = canvas.getBoundingClientRect();
  return new THREE.Vector2(((e.clientX-r.left)/r.width)*2-1, -((e.clientY-r.top)/r.height)*2+1);
}
function planeHit(e) {
  partDrag.ray.setFromCamera(ndcFromEvent(e), camera);
  const p = new THREE.Vector3();
  return partDrag.ray.ray.intersectPlane(partDrag.plane, p) ? p : null;
}
canvas.addEventListener('pointerdown', e => {
  if (!state.modelMesh || e.button !== 0 || state.sim.pov) return;
  partDrag.ray.setFromCamera(ndcFromEvent(e), camera);
  // does the click land on the part?
  const targets = [];
  state.modelMesh.traverse(o => { if (o.isMesh) targets.push(o); });
  if (!partDrag.ray.intersectObjects(targets, true).length) return;  // empty space → orbit

  const hit = planeHit(e); if (!hit) return;
  const c = new THREE.Box3().setFromObject(state.modelMesh).getCenter(new THREE.Vector3());
  partDrag.grab.set(hit.x - c.x, 0, hit.z - c.z);
  partDrag.active = true;
  controls.enabled = false;                    // suspend orbit during the drag
  canvas.setPointerCapture(e.pointerId);
  status('Moving part — release to replan');
});
canvas.addEventListener('pointermove', e => {
  if (!partDrag.active) return;
  const hit = planeHit(e); if (!hit) return;
  const box = new THREE.Box3().setFromObject(state.modelMesh);
  const c = box.getCenter(new THREE.Vector3());
  const half = box.getSize(new THREE.Vector3()).multiplyScalar(0.5);
  const cx = THREE.MathUtils.clamp(hit.x - partDrag.grab.x, PARTS.xMin + half.x, PARTS.xMax - half.x);
  const cz = THREE.MathUtils.clamp(hit.z - partDrag.grab.z, PARTS.zMin + half.z, PARTS.zMax - half.z);
  state.partXform.manual = true;
  state.partXform.manualX = cx;
  state.partXform.manualZ = cz;
  state.partXform.centered = false;
  $('placeCenter').classList.remove('on');
  placePartOnTable();                          // cheap reposition; no replan mid-drag
});
canvas.addEventListener('pointerup', e => {
  if (!partDrag.active) return;
  partDrag.active = false;
  controls.enabled = true;
  canvas.releasePointerCapture?.(e.pointerId);
  if (state.waypoints.length) planPath();      // replan against the new position
  status('Ready');
});

document.querySelectorAll('[data-view]').forEach(el => el.onclick = () => frameView(el.dataset.view));
$('fitBtn').onclick = () => frameView('iso');

// ---- transport wiring ----
$('playBtn').onclick = () => {
  if (!state.sim.curve) return;
  if (!state.sim.playing && state.sim.t >= 1) state.sim.t = 0;  // restart from end
  setPlaying(!state.sim.playing);
};
$('stopBtn').onclick = () => { setPlaying(false); state.sim.t = 0; $('scrub').value = 0; placeRig(0); };
$('scrub').oninput = e => {
  if (!state.sim.curve) return;
  state.sim.t = parseInt(e.target.value) / 1000;
  placeRig(state.sim.t);
};
$('speedSel').onchange = e => { state.sim.speed = parseFloat(e.target.value); };
$('loopChk').onchange = e => { state.sim.loop = e.target.checked; };
$('rideBtn').onclick = togglePOV;

// spacebar = play/pause
window.addEventListener('keydown', e => {
  if (e.code === 'Space' && state.sim.curve && document.activeElement.tagName !== 'INPUT') {
    e.preventDefault(); $('playBtn').onclick();
  }
});

// cursor readout
canvas.addEventListener('pointermove', () => {
  $('cursorInfo').textContent = `cam ${camera.position.x.toFixed(0)}, ${camera.position.y.toFixed(0)}, ${camera.position.z.toFixed(0)} mm`;
});

// drag & drop
const dropHint = $('dropHint');
['dragenter','dragover'].forEach(ev => canvas.parentElement.addEventListener(ev, e=>{e.preventDefault(); dropHint.classList.add('drag'); dropHint.style.display='flex';}));
['dragleave','drop'].forEach(ev => canvas.parentElement.addEventListener(ev, e=>{e.preventDefault(); dropHint.classList.remove('drag'); if(state.modelMesh)dropHint.style.display='none';}));
canvas.parentElement.addEventListener('drop', e => {
  const f = e.dataTransfer.files[0];
  if (f && /\.(step|stp)$/i.test(f.name)) loadStepFile(f);
  else status('Drop a .step / .stp file', true);
});

// ============================================================
//  Small utils
// ============================================================
function clearGroup(g){ while(g.children.length){ const c=g.children.pop(); c.traverse?.(o=>{o.geometry?.dispose?.(); o.material?.dispose?.();}); } }
function disposeObj(o){ o.traverse?.(c=>{c.geometry?.dispose?.(); if(Array.isArray(c.material))c.material.forEach(m=>m.dispose?.()); else c.material?.dispose?.();}); }
function setLoader(show, txt){ $('loader').classList.toggle('show', show); if(txt)$('loaderText').textContent=txt; }
function status(msg, warn){ $('statusText').textContent = msg; $('statusText').style.color = warn ? '#e8a33d' : ''; }
function round(v,d=2){ return Math.round(v*10**d)/10**d; }
