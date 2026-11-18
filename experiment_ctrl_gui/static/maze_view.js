// Live maze view: the arm drawn on the maze it is in, as the maze orchestrator reports it
// (sinthlab_bringup/helpers/maze_view.py). Used by the dashboard's Maze card and by maze.html.
//
// Coordinates are the maze's own (a, b) in metres, as in the trial data (rel_a, rel_b) and the
// plots: a to the right, b up. Feed it the server's "maze" events: view.feed({kind, data}).
"use strict";

class MazeView {
  static TRAIL_SEC = 10;      // how much of the arm's recent path is drawn
  static STALE_SEC = 2;       // no position for this long: say so
  static AWAY_M = 0.05;       // further than this outside the maze: the arm is not in it (parking)

  constructor(canvas, statusEl) {
    this.canvas = canvas;
    this.statusEl = statusEl;
    this.geom = null;
    this.st = null;
    this.rxAt = 0;
    this.trail = [];
    new ResizeObserver(() => this.draw()).observe(canvas);
    setInterval(() => this.draw(), 500);      // keeps the "no position for N s" line current
  }

  feed(msg) {
    if (msg.kind === "geometry") this.setGeometry(msg.data);
    else if (msg.kind === "state") this.setState(msg.data);
  }

  setGeometry(g) {
    if (!g) return;
    this.geom = g;
    const pts = g.pieces.flat();
    for (const [a, b, r] of [...g.checkpoints, g.goal]) pts.push([a - r, b - r], [a + r, b + r]);
    if (g.relative) pts.push([0, 0]);
    const as = pts.map((p) => p[0]), bs = pts.map((p) => p[1]);
    this.box = { a0: Math.min(...as), a1: Math.max(...as), b0: Math.min(...bs), b1: Math.max(...bs) };
    this.draw();
  }

  setState(s) {
    if (!s) return;
    const prev = this.st;
    if (!s.anchored || (prev && prev.trial !== s.trial)) this.trail = [];
    const now = performance.now() / 1000;
    if (s.anchored && s.ee) {
      this.trail.push([s.ee[0], s.ee[1], now]);
      while (this.trail.length && now - this.trail[0][2] > MazeView.TRAIL_SEC) this.trail.shift();
    }
    this.st = s;
    this.rxAt = now;
    this.draw();
  }

  away(ee) {
    const b = this.box, m = MazeView.AWAY_M;
    return ee[0] < b.a0 - m || ee[0] > b.a1 + m || ee[1] < b.b0 - m || ee[1] > b.b1 + m;
  }

  status() {
    const g = this.geom, s = this.st;
    if (!g) return "No maze yet — it appears once the maze experiment is running.";
    const age = performance.now() / 1000 - this.rxAt;
    if (!s || age > MazeView.STALE_SEC)
      return s ? `No live position for ${age.toFixed(0)} s — is the maze experiment still running?`
               : "Waiting for the arm's position…";
    if (!s.anchored) return "Waiting for the maze to anchor — it does once the trial's maze phase starts.";
    if (this.away(s.ee)) return "The arm is away from the maze (moving to park or to the start).";
    const tol = g.on_rail_tol_m, mm = Math.round(s.dist_m * 1000);
    const where = s.dist_m <= tol ? `on the path (${mm} mm)` : `${mm} mm off the path — being pulled back`;
    const held = s.active ? "Held to the path" : "Fixture off — not holding the arm";
    const n = g.checkpoints.length, v = (s.visited || []).length;
    const goal = s.completed ? "goal reached" : s.goal_armed ? "goal open" : "goal opens after every checkpoint";
    return `${held} · ${where} · checkpoints ${v}/${n} · ${goal}`;
  }

  draw() {
    const cv = this.canvas, dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (!w || !h) return;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
      cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
    }
    const ctx = cv.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    if (this.statusEl) this.statusEl.textContent = this.status();
    const g = this.geom;
    if (!g) return;

    const css = getComputedStyle(document.documentElement);
    const C = (n) => css.getPropertyValue(n).trim();
    const col = { line: C("--line"), muted: C("--muted"), ink: C("--ink"), ok: C("--ok"), info: C("--info"),
                  accent: C("--accent"), warn: C("--warn"), live: C("--live"), panel: C("--panel") };

    // Fit the maze, equal scale on both axes, a to the right and b up.
    const bx = this.box, spanA = bx.a1 - bx.a0, spanB = bx.b1 - bx.b0;
    const pad = Math.max(0.02, 0.1 * Math.max(spanA, spanB));
    const k = Math.min(w / (spanA + 2 * pad), h / (spanB + 2 * pad));
    const ca = (bx.a0 + bx.a1) / 2, cb = (bx.b0 + bx.b1) / 2;
    const X = (a) => w / 2 + (a - ca) * k, Y = (b) => h / 2 - (b - cb) * k;

    const stroke = (pieces, width, colour, alpha = 1) => {
      ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = colour; ctx.lineWidth = width;
      ctx.lineCap = "round"; ctx.lineJoin = "round";
      for (const pts of pieces) {
        ctx.beginPath();
        pts.forEach(([a, b], i) => (i ? ctx.lineTo(X(a), Y(b)) : ctx.moveTo(X(a), Y(b))));
        ctx.stroke();
      }
      ctx.restore();
    };
    const circle = (a, b, r, { fill, line, dash, alpha = 1, width = 2 }) => {
      ctx.save(); ctx.globalAlpha = alpha; ctx.beginPath(); ctx.arc(X(a), Y(b), r, 0, 2 * Math.PI);
      if (fill) { ctx.fillStyle = fill; ctx.fill(); }
      if (line) { ctx.setLineDash(dash || []); ctx.strokeStyle = line; ctx.lineWidth = width; ctx.stroke(); }
      ctx.restore();
    };
    const label = (text, a, b, colour, dx = 0, dy = 0) => {
      ctx.save(); ctx.fillStyle = colour; ctx.font = "600 12px system-ui, sans-serif";
      ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(text, X(a) + dx, Y(b) + dy); ctx.restore();
    };

    // The path: the "on the path" band (on_rail_tol either side), then its centre line.
    stroke(g.pieces, Math.max(4, 2 * g.on_rail_tol_m * k), col.line, 0.9);
    stroke(g.pieces, 2, col.muted);

    // Start (the anchor), checkpoints, goal.
    if (g.relative) {
      ctx.save(); ctx.strokeStyle = col.muted; ctx.lineWidth = 1.5; ctx.beginPath();
      ctx.moveTo(X(0) - 6, Y(0)); ctx.lineTo(X(0) + 6, Y(0)); ctx.moveTo(X(0), Y(0) - 6); ctx.lineTo(X(0), Y(0) + 6);
      ctx.stroke(); ctx.restore();
    }
    const s = this.st, live = s && s.anchored && performance.now() / 1000 - this.rxAt <= MazeView.STALE_SEC;
    const visited = new Set(live ? s.visited || [] : []);
    g.checkpoints.forEach(([a, b, r], i) => {
      const done = visited.has(i);
      circle(a, b, r * k, done ? { fill: col.ok, line: col.ok, alpha: 0.35 } : { line: col.info, alpha: 0.9 });
      label(String(i), a, b, done ? col.ok : col.info);
    });
    const [ga, gb, gr] = g.goal;
    const armed = live ? s.goal_armed : !g.goal_after_all;
    circle(ga, gb, gr * k, { line: col.accent, dash: armed ? [] : [5, 4], alpha: armed ? 1 : 0.6, width: armed ? 2.5 : 2 });
    label("goal", ga, gb, col.accent, 0, -gr * k - 9);

    // Scale bar: 5 cm.
    const bar = 0.05 * k;
    ctx.save(); ctx.strokeStyle = col.muted; ctx.fillStyle = col.muted; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(12, h - 14); ctx.lineTo(12 + bar, h - 14); ctx.stroke();
    ctx.font = "11px system-ui, sans-serif"; ctx.fillText("5 cm", 12, h - 20); ctx.restore();

    if (!live || this.away(s.ee)) return;

    // The arm: its last few seconds, the pull back to the path when it is off it, then the arm itself.
    if (this.trail.length > 1) stroke([this.trail.map(([a, b]) => [a, b])], 2, col.accent, 0.35);
    if (s.target && s.dist_m > g.on_rail_tol_m) {
      stroke([[s.ee, s.target]], 2, col.warn);
      circle(s.target[0], s.target[1], 4, { line: col.live, width: 2 });
    }
    circle(s.ee[0], s.ee[1], 7, { fill: s.active ? col.accent : col.muted, line: col.panel, width: 2 });
  }
}
window.MazeView = MazeView;
