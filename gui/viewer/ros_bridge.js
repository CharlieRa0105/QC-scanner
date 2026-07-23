/*
 * ros_bridge.js — minimal rosbridge v2 client over a RAW WebSocket.
 *
 * No roslib.js dependency (keeps the viewer offline/self-contained). Connects to
 * a rosbridge_server (started by qc_bringup/rosbridge.launch.py, default
 * ws://<host>:9090), subscribes to the planned trajectory + mission state, and
 * lets the viewer animate the SR5 through MoveIt's planned JOINT trajectory —
 * the "accurate sim" preview (the exact joints the real arm will run, IK done by
 * MoveIt, not the controller).
 *
 * Degrades silently: if rosbridge isn't reachable (e.g. the ROS graph isn't up),
 * it just retries in the background and the viewer keeps working off the HTTP API.
 */
window.QCRos = (function () {
  // Connect and (re)subscribe. `handlers`: { subscribe:[{topic,type}], onMsg(topic,msg),
  // onStatus(state) }. Returns { close(), connected(), callService(service,type,args,timeoutMs) }.
  function connect(url, handlers) {
    let ws = null, closed = false, retry = 0, live = false, seq = 0;
    const pending = {};        // rosbridge service call id -> {resolve, timer}
    const advertised = new Set();   // topics we've advertised on THIS socket

    function open() {
      try { ws = new WebSocket(url); } catch (e) { schedule(); return; }
      ws.onopen = () => {
        live = true; retry = 0;
        advertised.clear();    // new socket: re-advertise before the next publish
        handlers.onStatus && handlers.onStatus('connected');
        for (const s of (handlers.subscribe || [])) {
          ws.send(JSON.stringify({ op: 'subscribe', topic: s.topic, type: s.type }));
        }
      };
      ws.onmessage = (e) => {
        let m; try { m = JSON.parse(e.data); } catch (_) { return; }
        if (m.op === 'publish' && handlers.onMsg) { handlers.onMsg(m.topic, m.msg); return; }
        if (m.op === 'service_response' && pending[m.id]) {
          const p = pending[m.id]; delete pending[m.id]; clearTimeout(p.timer);
          p.resolve({ ok: m.result !== false, values: m.values || {} });
        }
      };
      ws.onclose = () => { live = false; handlers.onStatus && handlers.onStatus('disconnected'); schedule(); };
      ws.onerror = () => { try { ws.close(); } catch (_) {} };
    }
    function schedule() {
      if (closed) return;
      retry = Math.min(retry + 1, 10);
      setTimeout(open, Math.min(5000, 400 * retry));   // backoff, cap 5 s
    }

    // Call a ROS service. Resolves {ok, values} on response, or {ok:false,
    // unavailable:true} if not connected / times out (so callers can fail-open).
    function callService(service, type, args, timeoutMs) {
      return new Promise((resolve) => {
        if (!live || !ws) { resolve({ ok: false, unavailable: true }); return; }
        const id = 'svc_' + (++seq);
        const timer = setTimeout(() => {
          if (pending[id]) { delete pending[id]; resolve({ ok: false, unavailable: true }); }
        }, timeoutMs || 4000);
        pending[id] = { resolve, timer };
        try { ws.send(JSON.stringify({ op: 'call_service', service: service, type: type, args: args || {}, id: id })); }
        catch (_) { clearTimeout(timer); delete pending[id]; resolve({ ok: false, unavailable: true }); }
      });
    }

    // Publish a message to a topic (advertising once per socket first). Returns
    // false if not connected so callers can surface "ROS graph down". Used to drive
    // the arm over /arm/command (jog) when arm I/O is owned by the ROS graph.
    function publish(topic, type, msg) {
      if (!live || !ws) return false;
      try {
        if (!advertised.has(topic)) { ws.send(JSON.stringify({ op: 'advertise', topic: topic, type: type })); advertised.add(topic); }
        ws.send(JSON.stringify({ op: 'publish', topic: topic, msg: msg || {} }));
        return true;
      } catch (_) { return false; }
    }

    open();
    return { close() { closed = true; try { ws.close(); } catch (_) {} }, connected() { return live; }, callService, publish };
  }

  return { connect };
})();
