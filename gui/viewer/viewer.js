/*
 * viewer.js — bootstrap for the MAIN Run-tab scan-path viewport.
 *
 * All scene/arm/path logic lives in viewer3d.js (the shared module, also used
 * by the debug popup). This file only wires the module to this page's UI:
 * transport (play/reset/scrub), layer toggles, the camera segment, status line.
 */
'use strict';

const $ = (id) => document.getElementById(id);
const setStatus = (t) => { $('statusText').textContent = t; };

(async function main() {
  const config = await QCViewer.fetchConfig();
  const v = QCViewer.create($('canvas'), { config, sizeToWindow: true });
  window.__viewer = v;   // test/inspection handle

  // playback -> transport UI
  v.play.onStep = (t, n) => {
    $('scrub').value = String(t);
    $('wptLabel').textContent = `${Math.round(t) + 1} / ${n}`;
  };
  v.play.onDone = () => { $('playBtn').textContent = '▶ Play'; };

  $('playBtn').onclick = () => {
    v.play.on = !v.play.on;
    $('playBtn').textContent = v.play.on ? '❚❚ Pause' : '▶ Play';
  };
  $('resetBtn').onclick = () => { v.play.on = false; $('playBtn').textContent = '▶ Play'; v.placeAt(0); };
  $('scrub').oninput = (e) => { v.play.on = false; $('playBtn').textContent = '▶ Play'; v.placeAt(parseFloat(e.target.value)); };

  $('viewSeg').querySelectorAll('button').forEach((b) => b.onclick = () => {
    $('viewSeg').querySelectorAll('button').forEach((x) => x.classList.remove('on'));
    b.classList.add('on');
    v.setView(b.dataset.view);
  });
  document.querySelectorAll('.toggle[data-layer]').forEach((el) => el.onclick = () => {
    const on = !el.classList.contains('on');
    el.classList.toggle('on', on);
    v.setLayer(el.dataset.layer, on);
  });

  // content: the arm + the planned bundle
  setStatus('loading…');
  await v.buildArm('assets/arm/');
  try {
    const r = await fetch('/api/viewer_bundle');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const bundle = await r.json();
    v.buildPart(bundle.part);
    v.buildPath(bundle.waypoints || []);
    $('scrub').max = String(Math.max(0, (bundle.waypoints || []).length - 1));
    v.frameView();
    setStatus(`${(bundle.waypoints || []).length} waypoints · part ${bundle.part.vertices.length} verts`);
  } catch (e) {
    v.frameView();
    setStatus('no plan yet — pick a part (' + e.message + ')');
  }
})();
