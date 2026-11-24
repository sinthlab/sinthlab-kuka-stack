// Live view: the running experiment's bounds (live_view.py builds them from the run's parameters) and the
// arm in them, in two views of the robot's base frame:
//   front  Y to the right, Z up -- as seen from the front of the arm, the subject's side
//   top    Y to the right, X down -- the subject at the bottom
"use strict";

const TRAIL_SEC = 8;          // how much of the arm's recent path is drawn
const STALE_SEC = 2;          // no position for this long: say so
const MARGIN = 0.04;          // [m] around the bounds

const VIEWS = {
  front: { h: 1, v: 2, vSign: -1, name: "Front (Y–Z)" },
  top:   { h: 1, v: 0, vSign:  1, name: "Top (X–Y)" },
};

function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

class LiveView {
  constructor(canvases, statusEl) {
    this.canvases = canvases;          // {front: canvas, top: canvas}
    this.statusEl = statusEl;
    this.geom = null;
    this.live = null;
    this.rxAt = 0;
    this.trail = [];
    this.trialSeen = null;
    this.target = null;                // centre-out: this trial's goal end ("either" or a name)
    this.reached = null;               // the end reached this trial
    this.visited = new Set();          // maze: checkpoints passed this trial
    for (const cv of Object.values(canvases)) new ResizeObserver(() => this.draw()).observe(cv);
    setInterval(() => this.draw(), 500);
  }

  setGeometry(g) { this.geom = g; this.fit = g ? this.computeFit(g) : null; this.draw(); }

  setLive(l) {
    if (!l) return;
    if (l.trial !== this.trialSeen) { this.trialSeen = l.trial; this.trail = []; }
    // the trial so far, as the server followed it (live_view.py)
    this.target = l.target ?? null; this.reached = l.reached ?? null; this.visited = new Set(l.visited || []);
    const now = performance.now() / 1000;
    if (l.ee) {
      this.trail.push([l.ee, now]);
      while (this.trail.length && now - this.trail[0][1] > TRAIL_SEC) this.trail.shift();
    }
    this.live = l; this.rxAt = now;
    this.draw();
  }

  // The region each view shows: the bounds' own extent (not the safety distance or the floor), so it
  // does not jump as the arm moves.
  computeFit(g) {
    const pts = [g.anchor, g.start];
    for (const s of g.shapes) {
      if (s.kind === "seg") pts.push(s.a, s.b);
      else if (s.kind === "poly") pts.push(...s.pts);
      else if (s.kind === "marker") for (const d of [-1, 1]) pts.push(s.p.map((v) => v + d * s.r));
      else if (s.kind === "sphere" && s.role !== "trip") for (const d of [-1, 1]) pts.push(s.c.map((v) => v + d * s.r));
      else if (s.kind === "box") {
        const lo = s.min.map((v, i) => (v === null ? g.anchor[i] : v)), hi = s.max.map((v, i) => (v === null ? g.anchor[i] : v));
        pts.push(lo, hi);
      }
    }
    const lo = [0, 1, 2].map((i) => Math.min(...pts.map((p) => p[i])) - MARGIN);
    const hi = [0, 1, 2].map((i) => Math.max(...pts.map((p) => p[i])) + MARGIN);
    return { lo, hi };
  }

  status() {
    const g = this.geom, l = this.live;
    if (!g) return "No experiment running.";
    const age = performance.now() / 1000 - this.rxAt;
    const parts = [];
    if (!g.model) parts.push("Robot model unavailable. Showing positions relative to a nominal start.");
    if (!l || age > STALE_SEC) parts.push("No recent position updates");
    else if (!l.ee) parts.push("Arm position unavailable (no lbr_state)");
    if (l) parts.push(`trial ${l.trial ?? "–"} · ${String(l.phase ?? "–").replace(/_/g, " ")}`);
    if (this.target) parts.push(`goal: ${this.target}`);
    if (this.reached) parts.push(`reached: ${this.reached}`);
    if (g.experiment === "maze") parts.push(`forks passed: ${this.visited.size}`);
    return parts.join(" · ");
  }

  draw() {
    if (this.statusEl) this.statusEl.textContent = this.status();
    for (const [name, cv] of Object.entries(this.canvases)) this.drawView(cv, VIEWS[name]);
  }

  drawView(cv, view) {
    const dpr = window.devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
    if (!w || !h) return;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
      cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
    }
    const ctx = cv.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const g = this.geom;
    ctx.fillStyle = css("--muted"); ctx.font = "12px system-ui, sans-serif";
    ctx.fillText(view.name, 8, 16);
    if (!g || !this.fit) return;

    // metres -> pixels, the same scale on both axes
    const { lo, hi } = this.fit;
    const spanH = hi[view.h] - lo[view.h], spanV = hi[view.v] - lo[view.v];
    const k = Math.min((w - 24) / spanH, (h - 40) / spanV);
    const cx = w / 2, cy = h / 2 + 8, mh = (lo[view.h] + hi[view.h]) / 2, mv = (lo[view.v] + hi[view.v]) / 2;
    const X = (p) => cx + (p[view.h] - mh) * k;
    const Y = (p) => cy + view.vSign * (p[view.v] - mv) * k;
    const xAt = (v) => cx + (v - mh) * k, yAt = (v) => cy + view.vSign * (v - mv) * k;

    // a 5 cm scale bar, bottom right
    ctx.strokeStyle = css("--muted"); ctx.lineWidth = 2; ctx.setLineDash([]);
    ctx.beginPath(); ctx.moveTo(w - 12 - 0.05 * k, h - 12); ctx.lineTo(w - 12, h - 12); ctx.stroke();
    ctx.fillStyle = css("--muted"); ctx.fillText("5 cm", w - 12 - 0.05 * k, h - 18);

    const colour = { rail: css("--info"), wall: css("--warn"), threshold: css("--accent"), floor: css("--err"),
      trip: css("--err"), start: css("--muted"), option: css("--muted"), perturbed: css("--live"),
      guide: css("--muted"), checkpoint: css("--live"), goal: css("--ok"), end: css("--ok") };
    const stroke = (s, width = 2, dash = []) => { ctx.strokeStyle = colour[s.role] || css("--ink"); ctx.lineWidth = width; ctx.setLineDash(dash); };
    // Labels: skip one that would overlap a label already drawn (points stacked in this view).
    const placed = [];
    const label = (text, x, y) => {
      const bx = x + 6, by = y - 6, bw = ctx.measureText(text).width, bh = 12;
      if (placed.some((r) => bx < r[0] + r[2] && bx + bw > r[0] && by - bh < r[1] && by > r[1] - r[3])) return;
      placed.push([bx, by, bw, bh]);
      ctx.setLineDash([]); ctx.fillStyle = css("--muted"); ctx.fillText(text, bx, by);
    };

    for (const s of g.shapes) {
      ctx.beginPath();
      if (s.kind === "zline") {
        if (view.v !== 2) continue;
        stroke(s, s.role === "floor" ? 2 : 1, s.role === "floor" ? [8, 5] : [2, 4]);
        ctx.moveTo(0, yAt(s.z)); ctx.lineTo(w, yAt(s.z)); ctx.stroke();
        if (s.role === "floor") label(s.label, 2, yAt(s.z));
      } else if (s.kind === "seg") {
        stroke(s, s.role === "rail" ? 3 : 1, s.role === "guide" ? [4, 4] : []);
        ctx.moveTo(X(s.a), Y(s.a)); ctx.lineTo(X(s.b), Y(s.b)); ctx.stroke();
      } else if (s.kind === "poly") {
        stroke(s, 3);
        s.pts.forEach((p, i) => (i ? ctx.lineTo(X(p), Y(p)) : ctx.moveTo(X(p), Y(p)))); ctx.stroke();
      } else if (s.kind === "sphere") {
        stroke(s, s.role === "trip" ? 1 : 2, s.role === "trip" ? [2, 4] : [6, 4]);
        ctx.arc(X(s.c), Y(s.c), s.r * k, 0, 2 * Math.PI); ctx.stroke();
        label(s.label, X(s.c) + s.r * k * 0.7, Y(s.c) - s.r * k * 0.7);
      } else if (s.kind === "vcyl") {
        if (view.v !== 0) continue;
        stroke(s, 2); ctx.arc(xAt(s.c[1]), yAt(s.c[0]), s.r * k, 0, 2 * Math.PI); ctx.stroke();
      } else if (s.kind === "box") {
        // Each finite side is a wall line in this view; an open side is not drawn.
        stroke(s, s.role === "threshold" ? 2 : 2.5, s.role === "threshold" ? [6, 4] : []);
        const h0 = s.min[view.h], h1 = s.max[view.h], v0 = s.min[view.v], v1 = s.max[view.v];
        const L = h0 === null ? 0 : xAt(h0), Rr = h1 === null ? w : xAt(h1);
        const T = v0 === null ? (view.vSign < 0 ? h : 0) : yAt(v0), B = v1 === null ? (view.vSign < 0 ? 0 : h) : yAt(v1);
        if (h0 !== null && h1 !== null && h0 === h1) { ctx.moveTo(L, 0); ctx.lineTo(L, h); }
        else if (v0 !== null && v1 !== null && v0 === v1) { ctx.moveTo(0, T); ctx.lineTo(w, T); }
        else {
          if (h0 !== null) { ctx.moveTo(L, T); ctx.lineTo(L, B); }
          if (h1 !== null) { ctx.moveTo(Rr, T); ctx.lineTo(Rr, B); }
          if (v0 !== null) { ctx.moveTo(L, T); ctx.lineTo(Rr, T); }
          if (v1 !== null) { ctx.moveTo(L, B); ctx.lineTo(Rr, B); }
        }
        ctx.stroke();
      } else if (s.kind === "marker") {
        const r = Math.max(4, s.r * k), x = X(s.p), y = Y(s.p);
        let fill = null;
        if (s.role === "checkpoint" && this.visited.has(Number(s.id.split(":")[1]))) fill = colour.checkpoint;
        if (s.role === "end") {
          const name = s.id.split(":")[1];
          if (this.reached === name) fill = css("--err");                      // as the ring: red at the reach
          else if (!this.reached && (this.target === "either" || this.target === name)) fill = colour.end;   // lit
        }
        stroke(s, 2, s.role === "option" ? [2, 3] : []);
        ctx.arc(x, y, r, 0, 2 * Math.PI);
        if (fill) { ctx.fillStyle = fill; ctx.globalAlpha = 0.45; ctx.fill(); ctx.globalAlpha = 1; }
        ctx.stroke();
        if (s.role !== "start" || view.v === 2) label(s.label, x + r * 0.5, y - r * 0.5);
      }
    }

    // the arm: its trail, the pull point, the end effector
    const l = this.live, fresh = l && performance.now() / 1000 - this.rxAt < STALE_SEC;
    ctx.setLineDash([]);
    if (this.trail.length > 1) {
      ctx.beginPath(); ctx.strokeStyle = css("--ink"); ctx.globalAlpha = 0.35; ctx.lineWidth = 1.5;
      this.trail.forEach(([p], i) => (i ? ctx.lineTo(X(p), Y(p)) : ctx.moveTo(X(p), Y(p))));
      ctx.stroke(); ctx.globalAlpha = 1;
    }
    if (fresh && l.ee) {
      if (l.pull) {
        ctx.beginPath(); ctx.strokeStyle = css("--accent"); ctx.lineWidth = 1.5;
        ctx.moveTo(X(l.ee), Y(l.ee)); ctx.lineTo(X(l.pull), Y(l.pull)); ctx.stroke();
        ctx.beginPath(); ctx.arc(X(l.pull), Y(l.pull), 5, 0, 2 * Math.PI); ctx.stroke();
      }
      const x = Math.min(w - 6, Math.max(6, X(l.ee))), y = Math.min(h - 6, Math.max(6, Y(l.ee)));
      ctx.beginPath(); ctx.fillStyle = css("--ink"); ctx.arc(x, y, 6, 0, 2 * Math.PI); ctx.fill();
      if (x !== X(l.ee) || y !== Y(l.ee)) { ctx.fillStyle = css("--err"); ctx.fillText("arm outside this view", 8, h - 8); }
    }
  }
}
