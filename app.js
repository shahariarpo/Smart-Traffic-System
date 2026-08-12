/* Client-side twin of the C traffic system:
   graph + density-based signal timing + Dijkstra (normal / emergency). */

(() => {
  "use strict";

  const EMERGENCY_SPEED_FACTOR = 0.6;
  const BASE_GREEN_SECONDS = 15;
  const SECONDS_PER_VEHICLE = 2;
  const MIN_GREEN_SECONDS = 10;
  const MAX_GREEN_SECONDS = 90;
  const INF = 1e18;
  const MAX_NAME = 63;

  /** @type {{id:number,name:string,density:number,x:number,y:number}[]} */
  let vertices = [];
  /** @type {{src:number,dest:number,weight:number,active:boolean}[]} */
  let edges = [];
  let nextId = 0;
  let selectedId = null;
  let routeHighlight = { path: [], mode: null };

  const $ = (sel) => document.querySelector(sel);
  const svg = $("#network-svg");
  const edgesLayer = $("#edges-layer");
  const routeLayer = $("#route-layer");
  const nodesLayer = $("#nodes-layer");
  const mapStage = $("#map-stage");
  const mapEmpty = $("#map-empty");
  const toastEl = $("#toast");

  function toast(message, type = "ok") {
    toastEl.hidden = false;
    toastEl.className = `toast ${type}`;
    toastEl.textContent = message;
    clearTimeout(toastEl._t);
    toastEl._t = setTimeout(() => {
      toastEl.hidden = true;
    }, 2800);
  }

  function isAllDigits(s) {
    return /^[0-9]+$/.test(s);
  }

  function greenTime(density) {
    const raw = BASE_GREEN_SECONDS + density * SECONDS_PER_VEHICLE;
    return Math.max(MIN_GREEN_SECONDS, Math.min(MAX_GREEN_SECONDS, raw));
  }

  function findVertexByName(name) {
    return vertices.find((v) => v.name === name) || null;
  }

  function getVertex(id) {
    return vertices.find((v) => v.id === id) || null;
  }

  function outgoing(id) {
    return edges.filter((e) => e.src === id);
  }

  function findEdge(src, dest) {
    return edges.find((e) => e.src === src && e.dest === dest) || null;
  }

  function addVertex(name) {
    const trimmed = name.trim();
    if (!trimmed) return { ok: false, msg: "Name cannot be empty." };
    if (trimmed.length > MAX_NAME) return { ok: false, msg: "Name is too long." };
    if (isAllDigits(trimmed)) {
      return { ok: false, msg: "Purely numeric names are not allowed." };
    }
    if (findVertexByName(trimmed)) {
      return { ok: false, msg: `Intersection "${trimmed}" already exists.` };
    }

    const { w, h } = stageSize();
    const angle = (vertices.length * 2.4) % (Math.PI * 2);
    const radius = Math.min(w, h) * 0.28 + (vertices.length % 3) * 18;
    const x = w / 2 + Math.cos(angle) * radius;
    const y = h / 2 + Math.sin(angle) * radius;

    const v = { id: nextId++, name: trimmed, density: 0, x, y };
    vertices.push(v);
    return { ok: true, msg: `Added "${trimmed}" (id ${v.id}).`, id: v.id };
  }

  function addEdge(src, dest, weight, bidirectional) {
    if (src === dest) return { ok: false, msg: "A road cannot be a self-loop." };
    if (!(weight > 0)) return { ok: false, msg: "Travel time must be > 0." };
    if (!getVertex(src) || !getVertex(dest)) {
      return { ok: false, msg: "Invalid intersection." };
    }
    if (findEdge(src, dest)) {
      return { ok: false, msg: "That directed road already exists." };
    }

    edges.push({ src, dest, weight, active: true });
    if (bidirectional) {
      if (!findEdge(dest, src)) {
        edges.push({ src: dest, dest: src, weight, active: true });
      }
    }
    return { ok: true, msg: "Road added." };
  }

  function setDensity(id, density) {
    const v = getVertex(id);
    if (!v) return { ok: false, msg: "Intersection not found." };
    if (density < 0 || !Number.isFinite(density)) {
      return { ok: false, msg: "Density cannot be negative." };
    }
    v.density = Math.floor(density);
    return {
      ok: true,
      msg: `${v.name} density ${v.density}. Green light: ${greenTime(v.density)}s.`,
    };
  }

  function setBlocked(src, dest, block) {
    const e = findEdge(src, dest);
    if (!e) return { ok: false, msg: "No road exists in that direction." };
    e.active = !block;
    const a = getVertex(src).name;
    const b = getVertex(dest).name;
    // Also block/unblock the reverse direction if it exists
    const rev = findEdge(dest, src);
    if (rev) rev.active = !block;
    const bothWays = rev ? ` (and ${b} → ${a})` : "";
    return {
      ok: true,
      msg: `Road ${a} → ${b}${bothWays} is now ${block ? "BLOCKED" : "OPEN"}.`,
    };
  }

  /* ---- Dijkstra (matches dijkstra.c) ---- */

  function dijkstra(source, emergency) {
    const n = vertices.length;
    if (!n) return null;
    const indexOf = new Map(vertices.map((v, i) => [v.id, i]));
    const ids = vertices.map((v) => v.id);
    if (!indexOf.has(source)) return null;

    const dist = Array(n).fill(INF);
    const pred = Array(n).fill(-1);
    const visited = Array(n).fill(false);
    dist[indexOf.get(source)] = 0;

    for (let iter = 0; iter < n; iter++) {
      let u = -1;
      let best = INF;
      for (let i = 0; i < n; i++) {
        if (!visited[i] && dist[i] < best) {
          best = dist[i];
          u = i;
        }
      }
      if (u === -1) break;
      visited[u] = true;
      const uid = ids[u];
      for (const e of outgoing(uid)) {
        if (!e.active) continue;
        const vi = indexOf.get(e.dest);
        if (vi === undefined || visited[vi]) continue;
        const w = emergency ? e.weight * EMERGENCY_SPEED_FACTOR : e.weight;
        const alt = dist[u] + w;
        if (alt < dist[vi]) {
          dist[vi] = alt;
          pred[vi] = u;
        }
      }
    }

    return { dist, pred, ids, indexOf, source };
  }

  function reconstruct(result, target) {
    if (!result) return null;
    const ti = result.indexOf.get(target);
    if (ti === undefined || result.dist[ti] >= INF) return null;
    const pathIdx = [];
    let cur = ti;
    while (cur !== -1) {
      pathIdx.push(cur);
      cur = result.pred[cur];
    }
    pathIdx.reverse();
    return {
      path: pathIdx.map((i) => result.ids[i]),
      total: result.dist[ti],
    };
  }

  function stageSize() {
    const rect = mapStage.getBoundingClientRect();
    return {
      w: Math.max(rect.width || 640, 320),
      h: Math.max(rect.height || 420, 320),
    };
  }

  function syncSelects() {
    const selects = [
      "#road-from",
      "#road-to",
      "#density-node",
      "#block-from",
      "#block-to",
      "#route-from",
      "#route-to",
    ];
    const options =
      vertices.length === 0
        ? `<option value="">No intersections</option>`
        : vertices
          .map((v) => `<option value="${v.id}">${escapeHtml(v.name)} (${v.id})</option>`)
          .join("");

    for (const sel of selects) {
      const el = $(sel);
      const prev = el.value;
      el.innerHTML = options;
      if ([...el.options].some((o) => o.value === prev)) el.value = prev;
    }
  }

  function escapeHtml(s) {
    return String(s)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  function updateStats() {
    $("#stat-vertices").textContent = String(vertices.length);
    $("#stat-edges").textContent = String(edges.length);
    $("#stat-blocked").textContent = String(edges.filter((e) => !e.active).length);
    mapStage.classList.toggle("has-nodes", vertices.length > 0);
    mapEmpty.hidden = vertices.length > 0;
  }

  function renderNetworkList() {
    const box = $("#network-list");
    if (!vertices.length) {
      box.innerHTML = `<p class="hint">Empty network.</p>`;
      return;
    }
    box.innerHTML = vertices
      .map((v) => {
        const outs = outgoing(v.id)
          .map((e) => {
            const dest = getVertex(e.dest);
            const cls = e.active ? "" : "blocked";
            const tag = e.active ? "" : " [blocked]";
            return `<li class="${cls}">→ ${escapeHtml(dest.name)} · ${e.weight.toFixed(2)} min${tag}</li>`;
          })
          .join("");
        return `<div class="net-node">
          <strong>${escapeHtml(v.name)}</strong>
          <div class="meta">id ${v.id} · density ${v.density} · green ${greenTime(v.density)}s</div>
          <ul class="net-edges">${outs || "<li>No outgoing roads</li>"}</ul>
        </div>`;
      })
      .join("");
  }

  function edgeMidpoint(a, b, offset) {
    const mx = (a.x + b.x) / 2;
    const my = (a.y + b.y) / 2;
    if (!offset) return { x: mx, y: my };
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const len = Math.hypot(dx, dy) || 1;
    return { x: mx - (dy / len) * offset, y: my + (dx / len) * offset };
  }

  function shorten(a, b, pad) {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const len = Math.hypot(dx, dy) || 1;
    return {
      x1: a.x + (dx / len) * pad,
      y1: a.y + (dy / len) * pad,
      x2: b.x - (dx / len) * pad,
      y2: b.y - (dy / len) * pad,
    };
  }

  function renderEdgesOnly() {
    const { w, h } = stageSize();
    svg.setAttribute("viewBox", `0 0 ${w} ${h}`);
    const pairSeen = new Set();
    edgesLayer.innerHTML = "";
    for (const e of edges) {
      const a = getVertex(e.src);
      const b = getVertex(e.dest);
      if (!a || !b) continue;
      const key = `${Math.min(e.src, e.dest)}-${Math.max(e.src, e.dest)}`;
      const reverse = findEdge(e.dest, e.src);
      const curved = reverse && e.src < e.dest;
      const offset = reverse ? 10 : 0;
      const mid = edgeMidpoint(a, b, curved ? offset : -offset);
      const line = shorten(a, b, 22);
      const path = reverse
        ? `M ${line.x1} ${line.y1} Q ${mid.x} ${mid.y} ${line.x2} ${line.y2}`
        : `M ${line.x1} ${line.y1} L ${line.x2} ${line.y2}`;
      const g = document.createElementNS("http://www.w3.org/2000/svg", "g");
      const p = document.createElementNS("http://www.w3.org/2000/svg", "path");
      p.setAttribute("d", path);
      p.setAttribute("class", `edge-line${e.active ? "" : " blocked"}`);
      if (!reverse || e.src < e.dest) p.setAttribute("marker-end", e.active ? "url(#arrow)" : "url(#arrow-blocked)");
      g.appendChild(p);
      if (!pairSeen.has(key) || !reverse) {
        pairSeen.add(key);
        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("class", "edge-weight");
        label.setAttribute("x", String(mid.x));
        label.setAttribute("y", String(mid.y - 4));
        label.setAttribute("text-anchor", "middle");
        label.textContent = `${e.weight.toFixed(1)}m`;
        g.appendChild(label);
      }
      edgesLayer.appendChild(g);
    }
  }

  function renderMap() {
    const { w, h } = stageSize();
    svg.setAttribute("viewBox", `0 0 ${w} ${h}`);

    const pairSeen = new Set();
    edgesLayer.innerHTML = "";
    for (const e of edges) {
      const a = getVertex(e.src);
      const b = getVertex(e.dest);
      if (!a || !b) continue;
      const key = `${Math.min(e.src, e.dest)}-${Math.max(e.src, e.dest)}`;
      const reverse = findEdge(e.dest, e.src);
      const curved = reverse && e.src < e.dest;
      const offset = reverse ? 10 : 0;
      const mid = edgeMidpoint(a, b, curved ? offset : -offset);
      const line = shorten(a, b, 22);
      const path = reverse
        ? `M ${line.x1} ${line.y1} Q ${mid.x} ${mid.y} ${line.x2} ${line.y2}`
        : `M ${line.x1} ${line.y1} L ${line.x2} ${line.y2}`;

      const g = document.createElementNS("http://www.w3.org/2000/svg", "g");
      const p = document.createElementNS("http://www.w3.org/2000/svg", "path");
      p.setAttribute("d", path);
      p.setAttribute("class", `edge-line${e.active ? "" : " blocked"}`);
      if (!reverse || e.src < e.dest) p.setAttribute("marker-end", e.active ? "url(#arrow)" : "url(#arrow-blocked)");
      g.appendChild(p);

      if (!pairSeen.has(key) || !reverse) {
        pairSeen.add(key);
        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("class", "edge-weight");
        label.setAttribute("x", String(mid.x));
        label.setAttribute("y", String(mid.y - 4));
        label.setAttribute("text-anchor", "middle");
        label.textContent = `${e.weight.toFixed(1)}m`;
        g.appendChild(label);
      }
      edgesLayer.appendChild(g);
    }

    routeLayer.innerHTML = "";
    if (routeHighlight.path.length > 1) {
      const pts = routeHighlight.path
        .map((id) => getVertex(id))
        .filter(Boolean);
      const d = pts.map((p, i) => `${i ? "L" : "M"} ${p.x} ${p.y}`).join(" ");
      const rp = document.createElementNS("http://www.w3.org/2000/svg", "path");
      rp.setAttribute("d", d);
      rp.setAttribute(
        "class",
        `route-path ${routeHighlight.mode === "emergency" ? "emergency" : "normal"}`
      );
      const len = rp.getTotalLength ? 0 : 400;
      rp.style.strokeDasharray = "8 10";
      routeLayer.appendChild(rp);
      // force reflow for animation length
      void len;
      try {
        const total = rp.getTotalLength();
        rp.style.strokeDasharray = String(total);
        rp.style.strokeDashoffset = String(total);
        requestAnimationFrame(() => {
          rp.style.transition = "stroke-dashoffset 0.9s cubic-bezier(0.22,1,0.36,1)";
          rp.style.strokeDashoffset = "0";
        });
      } catch {
        /* ignore */
      }
    }

    nodesLayer.innerHTML = "";
    const onRoute = new Set(routeHighlight.path);
    for (const v of vertices) {
      const g = document.createElementNS("http://www.w3.org/2000/svg", "g");
      let cls = "node";
      if (v.id === selectedId) cls += " selected";
      if (v.density >= 20) cls += " dense";
      if (onRoute.has(v.id)) {
        cls +=
          routeHighlight.mode === "emergency"
            ? " on-route-emergency"
            : " on-route";
      }
      g.setAttribute("class", cls);
      g.setAttribute("data-id", String(v.id));
      g.setAttribute("transform", `translate(${v.x},${v.y})`);

      const ring = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      ring.setAttribute("class", "node-ring");
      ring.setAttribute("r", "16");
      ring.setAttribute("cx", "0");
      ring.setAttribute("cy", "0");

      const core = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      core.setAttribute("class", "node-core");
      core.setAttribute("r", "5");
      core.setAttribute("cx", "0");
      core.setAttribute("cy", "0");

      const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
      label.setAttribute("class", "node-label");
      label.setAttribute("x", "0");
      label.setAttribute("y", "30");
      label.textContent = v.name.length > 18 ? v.name.slice(0, 16) + "…" : v.name;

      g.append(ring, core, label);
      g.addEventListener("pointerdown", onNodePointerDown);
      g.addEventListener("click", () => selectNode(v.id));
      nodesLayer.appendChild(g);
    }
  }

  function selectNode(id) {
    selectedId = id;
    const v = getVertex(id);
    const box = $("#inspector");
    if (!v) {
      box.hidden = true;
      return;
    }
    box.hidden = false;
    $("#inspector-name").textContent = v.name;
    $("#inspector-id").textContent = String(v.id);
    $("#inspector-density").textContent = String(v.density);
    $("#inspector-green").textContent = `${greenTime(v.density)}s`;
    renderMap();
  }

  /* Drag nodes */
  let drag = null;

  function onNodePointerDown(ev) {
    const id = Number(ev.currentTarget.getAttribute("data-id"));
    const v = getVertex(id);
    if (!v) return;
    ev.preventDefault();
    ev.currentTarget.setPointerCapture(ev.pointerId);
    drag = { id, el: ev.currentTarget };
    ev.currentTarget.addEventListener("pointermove", onNodePointerMove);
    ev.currentTarget.addEventListener("pointerup", onNodePointerUp);
  }

  function clientToSvg(clientX, clientY) {
    const pt = svg.createSVGPoint();
    pt.x = clientX;
    pt.y = clientY;
    const ctm = svg.getScreenCTM();
    if (!ctm) return { x: clientX, y: clientY };
    const p = pt.matrixTransform(ctm.inverse());
    return { x: p.x, y: p.y };
  }

  function onNodePointerMove(ev) {
    if (!drag) return;
    const v = getVertex(drag.id);
    if (!v) return;
    const p = clientToSvg(ev.clientX, ev.clientY);
    const { w, h } = stageSize();
    v.x = Math.max(28, Math.min(w - 28, p.x));
    v.y = Math.max(28, Math.min(h - 28, p.y));
    // Update the node's transform in-place so pointer capture is not lost
    drag.el.setAttribute("transform", `translate(${v.x},${v.y})`);
    // Redraw only edges (they depend on node positions) without rebuilding nodes
    renderEdgesOnly();
  }

  function onNodePointerUp(ev) {
    if (!drag) return;
    ev.currentTarget.releasePointerCapture(ev.pointerId);
    ev.currentTarget.removeEventListener("pointermove", onNodePointerMove);
    ev.currentTarget.removeEventListener("pointerup", onNodePointerUp);
    drag = null;
    renderMap();
  }

  function refresh() {
    syncSelects();
    updateStats();
    renderNetworkList();
    renderMap();
  }

  function renderRouteResult(mode, route, title) {
    if (!route) {
      return `<div class="unreachable">UNREACHABLE — no valid route with the current network and closures.</div>`;
    }
    const names = route.path.map((id) => getVertex(id).name);
    const steps = names.map((n) => `<li>${escapeHtml(n)}</li>`).join("");
    const signals = route.path
      .map((id) => {
        const v = getVertex(id);
        const note =
          mode === "emergency" ? " · preempted green" : "";
        return `<div class="signal-row"><span>${escapeHtml(v.name)}</span><span>${greenTime(v.density)}s${note}</span></div>`;
      })
      .join("");
    const cls = mode === "emergency" ? "emergency" : "normal";
    return `<article class="route-card ${cls}">
      <h3>${title}</h3>
      <div class="time">${route.total.toFixed(2)} <span>minutes</span></div>
      <ol class="path-steps">${steps}</ol>
      <div class="signal-list"><strong>Active signal sequence</strong>${signals}</div>
    </article>`;
  }

  function runRoute(src, dest, mode) {
    const results = $("#results");
    if (mode === "compare") {
      const n = reconstruct(dijkstra(src, false), dest);
      const e = reconstruct(dijkstra(src, true), dest);
      routeHighlight = {
        path: (e || n)?.path || [],
        mode: e ? "emergency" : "normal",
      };
      let html =
        renderRouteResult("normal", n, "Normal route") +
        renderRouteResult("emergency", e, "Emergency (ambulance) route");
      if (n && e) {
        const saved = n.total - e.total;
        const pct = n.total > 0 ? (saved / n.total) * 100 : 0;
        html += `<div class="compare-summary">
          <h3>Comparison summary</h3>
          <p>Normal: ${n.total.toFixed(2)} min</p>
          <p>Emergency: ${e.total.toFixed(2)} min</p>
          <p class="saved">Saved ${saved.toFixed(2)} min (${pct.toFixed(1)}% faster)</p>
        </div>`;
      } else if (!n && !e) {
        html = `<div class="unreachable">Neither route is reachable.</div>`;
      }
      results.innerHTML = html;
    } else {
      const emergency = mode === "emergency";
      const route = reconstruct(dijkstra(src, emergency), dest);
      routeHighlight = {
        path: route?.path || [],
        mode: emergency ? "emergency" : "normal",
      };
      results.innerHTML = renderRouteResult(
        emergency ? "emergency" : "normal",
        route,
        emergency ? "Emergency (ambulance) route" : "Normal route"
      );
    }
    renderMap();
  }

  function loadDemo() {
    vertices = [];
    edges = [];
    nextId = 0;
    routeHighlight = { path: [], mode: null };
    selectedId = null;

    const names = [
      "Hospital",
      "Central Plaza",
      "North Gate",
      "Market Sq",
      "Riverside",
      "West Depot",
      "East Clinic",
      "Stadium",
    ];
    const { w, h } = stageSize();
    const layout = [
      [0.18, 0.28],
      [0.42, 0.22],
      [0.68, 0.2],
      [0.32, 0.48],
      [0.55, 0.45],
      [0.2, 0.72],
      [0.72, 0.55],
      [0.5, 0.75],
    ];
    names.forEach((name, i) => {
      vertices.push({
        id: nextId++,
        name,
        density: [8, 22, 12, 30, 15, 10, 18, 25][i],
        x: layout[i][0] * w,
        y: layout[i][1] * h,
      });
    });

    const roads = [
      [0, 1, 4, true],
      [1, 2, 3.5, true],
      [1, 3, 2.5, true],
      [1, 4, 3, true],
      [2, 6, 4, true],
      [3, 5, 3, true],
      [3, 4, 2, true],
      [4, 6, 3.5, true],
      [4, 7, 2.5, true],
      [5, 7, 4.5, true],
      [6, 7, 3, true],
      [0, 3, 5, true],
      [2, 4, 5.5, false],
    ];
    for (const [s, d, wgt, bidir] of roads) {
      addEdge(s, d, wgt, bidir);
    }

    $("#inspector").hidden = true;
    refresh();
    toast("Demo city loaded. Try Hospital → East Clinic.");
  }

  /* ---- Forms ---- */

  $("#form-intersection").addEventListener("submit", (ev) => {
    ev.preventDefault();
    const name = $("#input-name").value;
    const res = addVertex(name);
    toast(res.msg, res.ok ? "ok" : "error");
    if (res.ok) {
      $("#input-name").value = "";
      refresh();
      selectNode(res.id);
    }
  });

  $("#form-road").addEventListener("submit", (ev) => {
    ev.preventDefault();
    const src = Number($("#road-from").value);
    const dest = Number($("#road-to").value);
    const weight = Number($("#road-weight").value);
    const bidir = $("#road-bidir").checked;
    const res = addEdge(src, dest, weight, bidir);
    toast(res.msg, res.ok ? "ok" : "error");
    if (res.ok) {
      routeHighlight = { path: [], mode: null };
      refresh();
    }
  });

  $("#form-density").addEventListener("submit", (ev) => {
    ev.preventDefault();
    const id = Number($("#density-node").value);
    const density = Number($("#density-value").value);
    const res = setDensity(id, density);
    toast(res.msg, res.ok ? "ok" : "error");
    if (res.ok) {
      refresh();
      selectNode(id);
    }
  });

  $("#form-block").addEventListener("submit", (ev) => {
    ev.preventDefault();
    const action = ev.submitter?.value || "block";
    const src = Number($("#block-from").value);
    const dest = Number($("#block-to").value);
    const res = setBlocked(src, dest, action === "block");
    toast(res.msg, res.ok ? "ok" : "error");
    if (res.ok) {
      routeHighlight = { path: [], mode: null };
      refresh();
    }
  });

  $("#form-route").addEventListener("submit", (ev) => {
    ev.preventDefault();
    if (vertices.length < 2) {
      toast("Add at least two connected intersections first.", "error");
      return;
    }
    const src = Number($("#route-from").value);
    const dest = Number($("#route-to").value);
    const mode = document.querySelector('input[name="route-mode"]:checked').value;
    if (src === dest) {
      toast("Start and destination must differ.", "error");
      return;
    }
    runRoute(src, dest, mode);
  });

  $("#btn-demo").addEventListener("click", loadDemo);
  $("#btn-clear").addEventListener("click", () => {
    vertices = [];
    edges = [];
    nextId = 0;
    selectedId = null;
    routeHighlight = { path: [], mode: null };
    $("#inspector").hidden = true;
    $("#results").innerHTML =
      `<p class="results-placeholder">Routes and signal sequences will appear here.</p>`;
    refresh();
    toast("Network cleared.");
  });

  window.addEventListener("resize", () => {
    renderMap();
  });

  refresh();
})();
