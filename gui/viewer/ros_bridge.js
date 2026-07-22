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
  // onStatus(state) }. Returns { close(), connected() }.
  function connect(url, handlers) {
    let ws = null, closed = false, retry = 0, live = false;

    function open() {
      try { ws = new WebSocket(url); } catch (e) { schedule(); return; }
      ws.onopen = () => {
        live = true; retry = 0;
        handlers.onStatus && handlers.onStatus('connected');
        for (const s of (handlers.subscribe || [])) {
          ws.send(JSON.stringify({ op: 'subscribe', topic: s.topic, type: s.type }));
        }
      };
      ws.onmessage = (e) => {
        let m; try { m = JSON.parse(e.data); } catch (_) { return; }
        if (m.op === 'publish' && handlers.onMsg) handlers.onMsg(m.topic, m.msg);
      };
      ws.onclose = () => {
        live = false;
        handlers.onStatus && handlers.onStatus('disconnected');
        schedule();
      };
      ws.onerror = () => { try { ws.close(); } catch (_) {} };
    }
    function schedule() {
      if (closed) return;
      retry = Math.min(retry + 1, 10);
      setTimeout(open, Math.min(5000, 400 * retry));   // backoff, cap 5 s
    }

    open();
    return { close() { closed = true; try { ws.close(); } catch (_) {} }, connected() { return live; } };
  }

  return { connect };
})();
