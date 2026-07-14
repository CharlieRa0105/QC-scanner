import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

// ============================================================
//  Rokae xMate SR5 — built from the user's exact URDF (xMateSR5_urdf.xacro)
//  with the real visual STL meshes. Kinematics are taken verbatim from the
//  URDF (values below are the joint origins in METRES, converted to mm here).
//
//  URDF chain (ROS convention, Z-up):
//    base -> j1 origin(0,0,0.328)   axis(0,0,1)   lim [-6.2832, 6.2832]
//    j1   -> j2 origin(0,0,0)       axis(0,1,0)   lim [-2.7925, 2.618]
//    j2   -> j3 origin(0.05,0,0.4)  axis(0,-1,0)  lim [-2.9671, 2.4435]
//    j3   -> j4 origin(-0.05,0,0.4) axis(0,0,1)   lim [-6.2832, 6.2832]
//    j4   -> j5 origin(0,0.136,0)   axis(0,-1,0)  lim [-6.2832, 6.2832]
//    j5   -> j6 origin(0,0,0.1035)  axis(0,0,1)   lim [-6.2832, 6.2832]
//  All meshes are in mm, placed at each link's local origin (visual origin 0).
//  The whole robot is built Z-up then the root is rotated -90° about X so it
//  matches the app's Y-up world.
// ============================================================

const MM = 1000;  // URDF is in metres; our scene is in mm

// [origin xyz in metres, axis xyz, lower, upper]
const URDF = [
  { origin:[0.0,   0.0, 0.328], axis:[0, 0, 1], lower:-6.2832, upper: 6.2832 }, // j1
  { origin:[0.0,   0.0, 0.0  ], axis:[0, 1, 0], lower:-2.7925, upper: 2.618  }, // j2
  { origin:[0.05,  0.0, 0.4  ], axis:[0,-1, 0], lower:-2.9671, upper: 2.4435 }, // j3
  { origin:[-0.05, 0.0, 0.4  ], axis:[0, 0, 1], lower:-6.2832, upper: 6.2832 }, // j4
  { origin:[0.0, 0.136, 0.0  ], axis:[0,-1, 0], lower:-6.2832, upper: 6.2832 }, // j5
  { origin:[0.0,   0.0, 0.1035],axis:[0, 0, 1], lower:-6.2832, upper: 6.2832 }, // j6
];
const MESH_FILES = [
  'meshes/xMateSR5_base.stl',
  'meshes/xMateSR5_link1.stl',
  'meshes/xMateSR5_link2.stl',
  'meshes/xMateSR5_link3.stl',
  'meshes/xMateSR5_link4.stl',
  'meshes/xMateSR5_link5.stl',
  'meshes/xMateSR5_link6.stl',
];

const ROBOT_WHITE = 0xe9eef3;   // URDF links are "white"
const C_TCP = 0xffd27d;

function linkMaterial() {
  return new THREE.MeshStandardMaterial({ color: ROBOT_WHITE, metalness: 0.25, roughness: 0.6, side: THREE.DoubleSide });
}

// ============================================================
//  Build the exact SR5 kinematic chain. The skeleton (joint groups at the
//  URDF origins) is created synchronously so IK works immediately; the STL
//  visual meshes load asynchronously and attach to each link group.
// ============================================================
export function buildRokaeSR5() {
  // zUp holds the robot in ROS Z-up coords; root rotates it into Y-up.
  const root = new THREE.Group();
  root.name = 'RokaeSR5';
  const zUp = new THREE.Group();
  zUp.rotation.x = -Math.PI/2;         // Z-up (ROS) -> Y-up (scene)
  root.add(zUp);

  const joints = [];       // {group, axis(local), min, max}
  const linkGroups = [];   // where each link's STL mesh attaches (base + 6 links)

  // base link group (fixed)
  const baseGroup = new THREE.Group();
  zUp.add(baseGroup);
  linkGroups.push(baseGroup);

  // build joints 1..6, each nested in the previous link group
  let parent = baseGroup;
  for (let i = 0; i < URDF.length; i++) {
    const j = URDF[i];
    const jg = new THREE.Group();
    jg.position.set(j.origin[0]*MM, j.origin[1]*MM, j.origin[2]*MM);
    parent.add(jg);
    // the link mesh for this child link attaches at the joint group origin
    const linkGroup = new THREE.Group();
    jg.add(linkGroup);
    linkGroups.push(linkGroup);

    joints.push({
      group: jg,
      axis: new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize(),
      min: j.lower, max: j.upper,
    });
    parent = linkGroup;
  }

  // TCP: the tool control point sits ON the flange mounting face. link6's mesh
  // face is at its local Z=0 (the joint6 origin); the body extends back to
  // Z≈-34mm. So the plate face is Z=0 and the scanner bolts on there.
  const tcp = new THREE.Group();
  tcp.position.set(0, 0, 0);           // on the mounting plate face
  linkGroups[6].add(tcp);
  // small tool cross so the TCP is visible before/without a scanner
  const crossPts = [
    new THREE.Vector3(-24,0,0), new THREE.Vector3(24,0,0),
    new THREE.Vector3(0,-24,0), new THREE.Vector3(0,24,0),
  ];
  tcp.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(crossPts),
        new THREE.LineBasicMaterial({ color:C_TCP })));

  const chain = {
    root, joints, tcp,
    flange: linkGroups[6],
    linkGroups,
    // full reach ≈ shoulder + arm segments to the flange face (URDF, mm)
    reach: (0.4 + 0.4 + 0.136 + 0.1035) * MM,   // ≈ 1040 mm to flange face
    meshesLoaded: false,
  };

  loadMeshes(chain);
  return chain;
}

// asynchronously load the STL meshes and attach to each link group
function loadMeshes(chain) {
  const loader = new STLLoader();
  let pending = MESH_FILES.length;
  MESH_FILES.forEach((file, i) => {
    loader.load(file, (geo) => {
      geo.computeVertexNormals();
      const mesh = new THREE.Mesh(geo, linkMaterial());
      mesh.userData.isRobotMesh = true;
      chain.linkGroups[i].add(mesh);
      if (--pending === 0) chain.meshesLoaded = true;
    }, undefined, (err) => {
      console.warn('SR5 mesh load failed:', file, err);
      if (--pending === 0) chain.meshesLoaded = true;
    });
  });
}

// Ordered world-space points along the arm (base -> each joint -> TCP).
// app.js uses consecutive pairs as link segments for collision testing.
export function getLinkPoints(chain) {
  chain.root.updateWorldMatrix(true, true);
  const pts = [];
  pts.push(chain.linkGroups[0].getWorldPosition(new THREE.Vector3()));
  for (const j of chain.joints) pts.push(j.group.getWorldPosition(new THREE.Vector3()));
  pts.push(chain.tcp.getWorldPosition(new THREE.Vector3()));
  return pts;
}

// collision radius (mm) around each link segment — from the real mesh girths
export const LINK_RADII = [90, 75, 70, 60, 55, 50, 42];

// ============================================================
//  CCD Inverse Kinematics
//  Iteratively rotate each joint (from wrist to base) to bring the
//  TCP onto the target position, then bias the final joints to also
//  match the target orientation (scanner facing the surface).
// ============================================================
export function solveIK(chain, targetPos, aimPoint, iterations = 12) {
  const { joints, tcp } = chain;
  const tmp = new THREE.Vector3();
  const tipPos = new THREE.Vector3();
  const jointPos = new THREE.Vector3();
  const toTip = new THREE.Vector3();
  const toTgt = new THREE.Vector3();
  const worldAxis = new THREE.Vector3();
  const q = new THREE.Quaternion();

  chain.root.updateWorldMatrix(true, true);

  for (let it = 0; it < iterations; it++) {
    // position pass: wrist -> base
    for (let i = joints.length - 1; i >= 0; i--) {
      const j = joints[i];
      j.group.updateWorldMatrix(true, true);
      tcp.getWorldPosition(tipPos);
      j.group.getWorldPosition(jointPos);

      toTip.copy(tipPos).sub(jointPos);
      toTgt.copy(targetPos).sub(jointPos);
      if (toTip.lengthSq() < 1e-6 || toTgt.lengthSq() < 1e-6) continue;
      toTip.normalize(); toTgt.normalize();

      let angle = Math.acos(THREE.MathUtils.clamp(toTip.dot(toTgt), -1, 1));
      if (angle < 1e-5) continue;

      worldAxis.copy(j.axis).applyQuaternion(j.group.getWorldQuaternion(q)).normalize();
      const cross = tmp.crossVectors(toTip, toTgt);
      const sign = Math.sign(cross.dot(worldAxis)) || 1;
      const localAxis = j.axis.clone();
      const step = sign * angle * 0.5;
      j.group.rotateOnAxis(localAxis, step);
      clampJoint(j);
      j.group.updateWorldMatrix(true, true);
    }
  }

  // orientation pass: aim the flange's +Z (the tool axis the scanner views along)
  // at the surface point, using only the wrist joints — never the tool/tcp.
  if (aimPoint) alignToolAxis(chain, aimPoint);
}

// Rotate wrist joints so the flange +Z axis points from the TCP toward aimPoint.
// This keeps the scanner (rigidly bolted along +Z) facing the surface, and only
// ever moves real joints — the tool/tcp local transform is never touched.
function alignToolAxis(chain, aimPoint) {
  const { joints, tcp, flange } = chain;
  const wristJoints = [joints[4], joints[5], joints[3]];  // J5, J6, J4 order

  // Pass A: point the flange +Z at the surface (aim direction).
  for (let pass = 0; pass < 3; pass++) {
    for (const j of wristJoints) {
      flange.updateWorldMatrix(true, true);
      const flangeQ = flange.getWorldQuaternion(new THREE.Quaternion());
      const fwd = new THREE.Vector3(0,0,1).applyQuaternion(flangeQ).normalize(); // flange +Z
      const tcpPos = tcp.getWorldPosition(new THREE.Vector3());
      const want = aimPoint.clone().sub(tcpPos).normalize();
      const dot = THREE.MathUtils.clamp(fwd.dot(want), -1, 1);
      const angle = Math.acos(dot);
      if (angle < 1e-4) continue;
      const worldAxis = new THREE.Vector3().crossVectors(fwd, want);
      if (worldAxis.lengthSq() < 1e-9) continue;
      worldAxis.normalize();
      const jWorldAxis = j.axis.clone().applyQuaternion(j.group.getWorldQuaternion(new THREE.Quaternion())).normalize();
      const sign = Math.sign(worldAxis.dot(jWorldAxis)) || 1;
      const contribute = sign * angle * Math.abs(worldAxis.dot(jWorldAxis)) * 0.6;
      j.group.rotateOnAxis(j.axis, contribute);
      clampJoint(j);
      j.group.updateWorldMatrix(true, true);
    }
  }

  // Pass B: ROLL correction. Rotate the tool about its own +Z (J6) so the
  // scanner's up axis is as vertical as possible — this stops the camera from
  // tilting sideways. J6 rotation about the tool axis does not change where the
  // camera points, only its roll.
  correctToolRoll(chain);
}

// Roll the tool (J6) so the flange's local up (+Y) projects as close to world-up
// as possible, keeping the camera level (no sideways tilt).
function correctToolRoll(chain) {
  const { joints, flange } = chain;
  const j6 = joints[5];
  flange.updateWorldMatrix(true, true);
  const flangeQ = flange.getWorldQuaternion(new THREE.Quaternion());
  const fwd = new THREE.Vector3(0,0,1).applyQuaternion(flangeQ).normalize(); // view axis
  const up  = new THREE.Vector3(0,1,0).applyQuaternion(flangeQ).normalize(); // current up
  // desired up = world up projected into the plane perpendicular to the view axis
  const worldUp = new THREE.Vector3(0,1,0);
  let desiredUp = worldUp.clone().sub(fwd.clone().multiplyScalar(worldUp.dot(fwd)));
  if (desiredUp.lengthSq() < 1e-6) {
    // looking straight up/down: fall back to world +X reference
    desiredUp = new THREE.Vector3(1,0,0).sub(fwd.clone().multiplyScalar(fwd.x));
  }
  desiredUp.normalize();
  // signed roll angle from current up to desired up, about the view axis
  const cross = new THREE.Vector3().crossVectors(up, desiredUp);
  const s = THREE.MathUtils.clamp(cross.dot(fwd), -1, 1);
  const c = THREE.MathUtils.clamp(up.dot(desiredUp), -1, 1);
  const roll = Math.atan2(s, c);
  if (Math.abs(roll) < 1e-4) return;
  // apply about J6's own axis (its axis is +Z, i.e. the tool axis)
  j6.group.rotateOnAxis(j6.axis, roll);
  clampJoint(j6);
  j6.group.updateWorldMatrix(true, true);
}

function clampJoint(j) {
  // signed angle about the joint's own axis
  const q = j.group.quaternion;
  const raw = 2 * Math.acos(THREE.MathUtils.clamp(q.w, -1, 1));
  let a = 0;
  if (raw > 1e-6) {
    const s = Math.sqrt(1 - q.w*q.w);
    const ax = new THREE.Vector3(q.x/s, q.y/s, q.z/s);
    a = ax.dot(j.axis) >= 0 ? raw : -raw;
  }
  const c = THREE.MathUtils.clamp(a, j.min, j.max);
  if (c !== a) j.group.quaternion.setFromAxisAngle(j.axis, c);
}
