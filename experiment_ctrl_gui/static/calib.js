"use strict";
// Reward calibration page. The server runs every step (experiment_ctrl_gui/calibration.py, which uses
// analysis/reward_calibration.py): this page asks it to dispense, polls GET /api/calib/state, and sends the
// net mass. The pulses keep their timing whatever the browser does; reloading the page loses nothing.

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const ADDR_KEY = "diag.ttl.addr";                 // shared with the Diagnostics page's TTL tab
const SERIES = ["--s1", "--s2", "--s3", "--s4"];  // by reward size, in a fixed order (style.css)
const DEFAULT_WIDTHS = { 1: "100 150 200 300 400", 2: "30 50 75 100", 3: "20 30 50" };
const C = { st: null, set: null, sweep: null, timer: null, setsKey: "", fitOpen: false, lastKey: null };

async function api(path, body) {
  const opt = body === undefined ? {} : {
    method: "POST", headers: { "Content-Type": "application/json", "X-Experiment-Ctrl": "1" }, body: JSON.stringify(body),
  };
  const r = await fetch(path, opt);
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  return j;
}

const cabinet = () => ({ addr: $("#addr").value.trim(), output: +$("#output").value });

async function act(path, body) {
  try { await api(path, body); } catch (e) { showLast({ ok: false, text: e.message }); }
  refresh();
}

// ---------------------------------------------------------------- volumes, as reward_volume.py
const sizeText = (p) => (p.burst === 1 ? "1 pulse" : `${p.burst} pulses, ${p.gap_ms} ms apart`);
const fmtMl = (ml) => (ml == null ? "–" : `${ml.toFixed(2)} mL`);

function volume(p, ms) {
  const [lo, hi] = p.valid_width_ms, pts = p.points;
  let ul;
  if (ms >= lo && ms <= hi) {
    for (let i = 1; i < pts.length; i++) {
      if (ms <= pts[i][0]) { const [w0, v0] = pts[i - 1], [w1, v1] = pts[i]; ul = v0 + (v1 - v0) * (ms - w0) / (w1 - w0); break; }
    }
    if (ul === undefined) ul = pts[0][1];
    return { ml: Math.max(ul, 0) / 1000, extrapolated: false };
  }
  ul = p.slope_ul_per_ms * ms + p.intercept_ul;
  return { ml: Math.max(ul, 0) / 1000, extrapolated: true };
}

const activeSet = () => (C.st ? C.st.sets.find((s) => s.name === C.st.active) : null);

// ---------------------------------------------------------------- the job
function showLast(last) {
  const el = $("#job-last");
  el.classList.toggle("hidden", !last);
  if (!last) return;
  el.className = "job-last " + (last.ok ? "ok" : "bad");
  el.textContent = last.text;
}

function renderJob(st) {
  const j = st.job, pill = $("#pill-job");
  $("#job").classList.toggle("hidden", !j);
  if (!j) {
    pill.className = "pill"; pill.textContent = "Idle";
  } else {
    $("#job-label").textContent = j.label;
    $("#job-fill").style.width = `${Math.round(100 * j.done / Math.max(j.total, 1))}%`;
    const weigh = j.phase === "weigh";
    $("#btn-job-stop").classList.toggle("hidden", weigh);
    $("#weigh-row").classList.toggle("hidden", !weigh);
    pill.className = "pill " + (weigh ? "warn" : "run");
    pill.textContent = weigh ? "Weigh the cup" : j.phase === "draining" ? "Last drops" : `Dispensing ${j.done} / ${j.total}`;
    $("#job-phase").innerHTML = j.phase === "dispensing"
      ? `Dispensing: ${j.done} of ${j.total} ${j.kind === "prime" ? "openings" : "rewards"}, ${j.spacing_s} s apart.`
      : j.phase === "draining" ? `Waiting ${st.defaults.drain_s} s for the last drops.`
      : `Read the scale: the net mass in grams.${j.expected_g != null ? ` The calibration expects about <strong>${j.expected_g.toFixed(2)} g</strong>.` : ""}`;
    if (weigh && document.activeElement !== $("#mass") && !$("#mass").dataset.armed) {
      $("#mass").value = ""; $("#mass").dataset.armed = "1"; $("#mass").focus();
    }
  }
  if (!j || j.phase !== "weigh") delete $("#mass").dataset.armed;
  // The server's outcome of the last job shows once, when it changes: a message from this page (a fit, an
  // error) stays until something new happens.
  const lastKey = JSON.stringify([j && j.started, st.last]);
  if (lastKey !== C.lastKey) {
    C.lastKey = lastKey;
    if (!j && st.last) showLast(st.last);
    else if (j && j.phase === "dispensing") showLast(null);
  }
}

// ---------------------------------------------------------------- in use: the chart and the table
function cssVar(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

function niceStep(span, n) {
  const raw = span / n, mag = 10 ** Math.floor(Math.log10(raw));
  return [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw);
}

function renderChart(set, force = false) {
  const svg = $("#chart"), wrap = $("#chart-wrap");
  const key = JSON.stringify([set && set.profiles, wrap.clientWidth]);
  if (!force && svg.dataset.key === key) return;
  svg.dataset.key = key;
  const profs = set ? set.profiles : [];
  wrap.classList.toggle("hidden", !profs.length);
  $("#legend").innerHTML = profs.map((p, i) =>
    `<span><i style="background:var(${SERIES[i % 4]})"></i>${esc(sizeText(p))}</span>`).join("");
  if (!profs.length) return;
  const W = Math.max(wrap.clientWidth, 280), H = 260, m = { l: 46, r: 64, t: 12, b: 36 };
  const pts = profs.flatMap((p) => p.per_width);
  const xMax = Math.max(...pts.map((t) => t.width_ms)) * 1.05;
  const yMax = Math.max(...pts.map((t) => (t.mean_ul + (t.sd_ul || 0)) / 1000)) * 1.08;
  const x = (w) => m.l + (W - m.l - m.r) * w / xMax, y = (ml) => H - m.b - (H - m.t - m.b) * ml / yMax;
  const ys = niceStep(yMax, 5), xs = niceStep(xMax, 6);
  let g = "";
  for (let v = 0; v <= yMax + 1e-9; v += ys)
    g += `<line class="grid" x1="${m.l}" x2="${W - m.r}" y1="${y(v)}" y2="${y(v)}"/><text class="tick" x="${m.l - 6}" y="${y(v) + 4}" text-anchor="end">${+v.toFixed(2)}</text>`;
  for (let v = 0; v <= xMax + 1e-9; v += xs)
    g += `<text class="tick" x="${x(v)}" y="${H - m.b + 16}" text-anchor="middle">${+v.toFixed(0)}</text>`;
  g += `<line class="axis" x1="${m.l}" x2="${W - m.r}" y1="${y(0)}" y2="${y(0)}"/>`;
  g += `<text class="tick" x="${(m.l + W - m.r) / 2}" y="${H - 4}" text-anchor="middle">valve time per pulse [ms]</text>`;
  g += `<text class="tick" transform="translate(12 ${(m.t + H - m.b) / 2}) rotate(-90)" text-anchor="middle">mL per reward</text>`;
  const dots = [];
  profs.forEach((p, i) => {
    const col = `var(${SERIES[i % 4]})`;
    const line = p.points.map(([w, v]) => `${x(w)},${y(v / 1000)}`).join(" ");
    g += `<polyline class="ser" points="${line}" style="stroke:${col}"/>`;
    for (const t of p.per_width) {
      const excl = p.excluded_widths_ms.includes(t.width_ms), cx = x(t.width_ms), cy = y(t.mean_ul / 1000);
      if (t.sd_ul) g += `<line class="whisk" x1="${cx}" x2="${cx}" y1="${y((t.mean_ul - t.sd_ul) / 1000)}" y2="${y((t.mean_ul + t.sd_ul) / 1000)}" style="stroke:${col}"/>`;
      g += `<circle class="dot${excl ? " excl" : ""}" cx="${cx}" cy="${cy}" r="4.5" style="${excl ? "stroke" : "fill"}:${col}"/>`;
      dots.push({ cx, cy, p, t, excl });
    }
    const [lw, lv] = p.points[p.points.length - 1];
    g += `<text class="dlab" x="${x(lw) + 8}" y="${y(lv / 1000) + 4}">${p.burst === 1 ? "1 pulse" : `${p.burst} pulses`}</text>`;
  });
  dots.forEach((d, k) => { g += `<circle class="hit" data-k="${k}" cx="${d.cx}" cy="${d.cy}" r="11"/>`; });
  svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
  svg.setAttribute("width", W); svg.setAttribute("height", H);
  svg.innerHTML = g;
  const tip = $("#tip");
  svg.querySelectorAll(".hit").forEach((h) => {
    h.onmouseenter = () => {
      const d = dots[+h.dataset.k], t = d.t;
      tip.innerHTML = `<strong>${esc(sizeText(d.p))}</strong><br>${t.width_ms} ms: ${(t.mean_ul / 1000).toFixed(2)} mL`
        + (t.sd_ul ? ` ± ${(t.sd_ul / 1000).toFixed(3)}` : "") + `<br>${t.n} batch${t.n === 1 ? "" : "es"}`
        + (t.cv_pct != null ? `, CV ${t.cv_pct.toFixed(1)} %` : "") + (d.excl ? "<br>left out of the fit (too variable)" : "");
      tip.classList.remove("hidden");
      tip.style.left = `${Math.min(d.cx + 12, W - 170)}px`; tip.style.top = `${Math.max(d.cy - 50, 0)}px`;
    };
    h.onmouseleave = () => tip.classList.add("hidden");
  });
}

function renderInUse(st) {
  const set = activeSet();
  $("#inuse-note").innerHTML = set
    ? `Calibration <strong>${esc(set.name)}</strong>: the dashboard shows each reward in mL and sets rewards by volume from these. Read them as ±10 %.`
    : "No calibration yet: run a sweep under New calibration.";
  renderChart(set);
  $("#profiles").innerHTML = set ? `<tr><th>Reward</th><th>Calibrated widths</th><th>mL per reward</th><th>Spread (CV)</th><th>Fitted</th></tr>` +
    set.profiles.map((p) => {
      const v = p.per_width.filter((t) => !p.excluded_widths_ms.includes(t.width_ms));
      const cv = Math.max(...v.map((t) => t.cv_pct || 0));
      return `<tr><td>${esc(sizeText(p))}</td><td class="v">${p.valid_width_ms[0]}–${p.valid_width_ms[1]} ms</td>
        <td class="v">${(p.points[0][1] / 1000).toFixed(2)}–${(p.points[p.points.length - 1][1] / 1000).toFixed(2)}</td>
        <td class="v">≤ ${cv.toFixed(1)} %</td><td class="v">${esc((p.created || "").slice(0, 10))}</td></tr>`;
    }).join("") : "";
}

// ---------------------------------------------------------------- check
function renderCheck(st) {
  const set = activeSet(), sel = $("#ck-size");
  const opts = set ? set.profiles.map((p) => `<option value="${p.burst}">${esc(sizeText(p))}</option>`).join("") : "";
  if (sel.dataset.key !== opts) { const keep = sel.value; sel.innerHTML = opts; sel.dataset.key = opts; if (keep) sel.value = keep; }
  $(".tol").textContent = st.defaults.tolerance_pct;
  expectCheck();
  const rows = set ? set.checks : [];
  $("#checks").innerHTML = rows.length ? `<tr><th>When</th><th>Reward</th><th>Rewards</th><th>mL each</th><th>From the calibration</th></tr>` +
    rows.map((r) => {
      const n = +r.pulses_confirmed, b = +(r.burst || 1);
      const ml = n ? +r.net_mass_g / n : null;
      const off = (r.note.match(/([+-]\d+(\.\d+)?)%/) || [])[1];
      const bad = off != null && Math.abs(+off) > st.defaults.tolerance_pct;
      return `<tr><td>${esc(r.started.replace("T", " ").slice(0, 16))}</td><td class="v">${b === 1 ? "" : `${b} × `}${esc(r.width_ms)} ms</td>
        <td class="v">${n}</td><td class="v">${ml == null ? "–" : ml.toFixed(2)}</td><td class="v ${bad ? "warn-text" : ""}">${off != null ? `${off} %` : "–"}</td></tr>`;
    }).join("") : `<tr><td class="muted">No checks in this set yet.</td></tr>`;
}

function expectCheck() {
  const set = activeSet();
  const p = set && set.profiles.find((q) => q.burst === +$("#ck-size").value);
  const w = +$("#ck-width").value, n = +$("#ck-n").value;
  $("#btn-check").disabled = !p || !!(C.st && (C.st.job || C.st.busy));
  if (!p || !w || !n) { $("#ck-expect").textContent = ""; return; }
  const v = volume(p, w);
  $("#ck-expect").innerHTML = `${fmtMl(v.ml)} per reward: the cup should gain about <strong>${(v.ml * n * (p.density || 1)).toFixed(2)} g</strong>.`
    + (v.extrapolated ? ` <span class="warn-text">${w} ms is outside the calibrated ${p.valid_width_ms[0]}–${p.valid_width_ms[1]} ms.</span>` : "");
}

// ---------------------------------------------------------------- new calibration
function renderSets(st) {
  const sel = $("#set");
  const key = st.active + "|" + st.sets.map((s) => s.name).join(",");
  if (C.setsKey !== key) {
    C.setsKey = key;
    sel.innerHTML = st.sets.map((s) => `<option value="${esc(s.name)}">${esc(s.name)}${s.name === st.active ? " (in use)" : ""}</option>`).join("")
      + `<option value="">New set…</option>`;
    if (C.set === null || !st.sets.some((s) => s.name === C.set)) C.set = st.sets.length ? st.sets[0].name : "";
    sel.value = C.set;
  }
  const set = st.sets.find((s) => s.name === C.set);
  $("#setup-label").classList.toggle("hidden", !!set);
  const sweeps = set ? set.sweeps : [];
  if (!sweeps.some((w) => w.name === C.sweep)) C.sweep = (sweeps.find((w) => w.next) || sweeps[0] || {}).name || null;
  $("#sweeps").innerHTML = sweeps.length ? `<tr><th>Sweep</th><th>Reward</th><th>Widths [ms]</th><th>Weighed</th><th></th></tr>` +
    sweeps.map((w) => `<tr class="${w.name === C.sweep ? "sel" : ""}"><td class="v">${esc(w.name)}</td>
      <td>${w.burst === 1 ? "1 pulse" : `${w.burst} pulses, ${w.gap_ms} ms apart`}</td><td class="v">${esc((w.settings.widths || []).join(" "))}</td>
      <td class="v">${w.n_ok} / ${w.total}${w.fitted ? " · fitted" : ""}</td>
      <td><button class="btn small" data-sweep="${esc(w.name)}">Open</button></td></tr>`).join("")
    : `<tr><td class="muted">${set ? "No sweeps in this set." : "A new set is created with its first sweep, below."}</td></tr>`;
  $("#sweeps").querySelectorAll("[data-sweep]").forEach((b) => (b.onclick = () => { C.sweep = b.dataset.sweep; C.fitOpen = false; render(); }));
  renderSweep(st, set && sweeps.find((w) => w.name === C.sweep));
}

function renderSweep(st, w) {
  const el = $("#sweep-detail");
  const blocked = !!(st.job || st.busy);
  // Redraw only when something changed: the poll must not wipe the fit form while it is being filled in.
  const key = JSON.stringify([C.set, w, blocked, C.fitOpen]);
  if (el.dataset.key === key) return;
  el.dataset.key = key;
  if (!w) { el.innerHTML = ""; return; }
  const nx = w.next, s = w.settings;
  const what = nx ? (nx.width_ms === 0 ? "a blank (no pulse)" : `${w.burst === 1 ? "" : `${w.burst} × `}${nx.width_ms} ms × ${s.pulses}`) : "";
  const done = w.rows.filter((r) => r.status === "ok").length;
  const fitForm = `<div class="diag-grid fit-grid">
      <label>Model <select id="fit-model"><option value="interp">Segments</option><option value="linear">One line</option></select></label>
      <label>Density [g/mL] <input type="number" id="fit-density" min="0.5" max="2" step="0.01" value="1.0"></label>
      <label>Leave out widths with CV above [%] <input type="number" id="fit-cv" min="1" max="50" value="10"></label>
    </div><div class="row-actions"><button class="btn primary" id="btn-fit">Fit ${esc(w.name)}</button></div>`;
  el.innerHTML = `<div class="sweep-box">
    ${nx ? `<div class="row-actions"><button class="btn primary" id="btn-batch" ${blocked ? "disabled" : ""}>▶ Dispense batch ${nx.batch}</button>
      <span class="note">Next: ${esc(what)}. Tare the cup first. ${w.n_ok} of ${w.total} weighed.</span></div>`
    : `<p class="note">Every batch is weighed.${w.fitted ? " Fitted: fit again to change the model or the limits." : " Fit it to use it."}</p>`}
    ${!nx || C.fitOpen ? fitForm : `<div class="row-actions"><button class="btn small" id="fit-early">Fit now</button><span class="note">with the batches weighed so far</span></div>`}
    ${done || w.rows.length ? `<table class="hist batches"><tr><th>Batch</th><th>Width</th><th>Result</th><th>Net mass</th><th>mL per reward</th></tr>` +
      w.rows.slice().reverse().map((r) => `<tr><td class="v">${r.batch}</td><td class="v">${r.width_ms ? `${r.width_ms} ms` : "blank"}</td>
        <td class="${r.status === "ok" ? "" : "warn-text"}">${esc(r.status)}</td><td class="v">${r.net_mass_g !== "" ? `${(+r.net_mass_g).toFixed(2)} g` : "–"}</td>
        <td class="v">${r.status === "ok" && r.width_ms && r.units ? (r.net_mass_g / r.units).toFixed(3) : "–"}</td></tr>`).join("") + `</table>` : ""}
  </div>`;
  if ($("#btn-batch")) $("#btn-batch").onclick = () => act("/api/calib/batch", { ...cabinet(), set: C.set, name: w.name });
  if ($("#fit-early")) $("#fit-early").onclick = () => { C.fitOpen = true; render(); };
  if ($("#btn-fit")) $("#btn-fit").onclick = async () => {
    try {
      const r = await api("/api/calib/fit", { set: C.set, name: w.name, model: $("#fit-model").value,
        density: +$("#fit-density").value, max_cv: +$("#fit-cv").value });
      const p = r.profile;
      showLast({ ok: true, text: `Fitted ${C.set}/${w.name}: ${p.valid_width_ms[0]}–${p.valid_width_ms[1]} ms = `
        + `${(p.points[0][1] / 1000).toFixed(2)}–${(p.points[p.points.length - 1][1] / 1000).toFixed(2)} mL per reward`
        + (p.excluded_widths_ms.length ? `; left out ${p.excluded_widths_ms.join(", ")} ms (too variable)` : "") + ". " + r.notes.join(" ") });
      C.fitOpen = false;
    } catch (e) { showLast({ ok: false, text: e.message }); }
    refresh(false);
  };
}

function newNote() {
  const b = Math.max(1, +$("#nw-burst").value || 1), gap = b > 1 ? +$("#nw-gap").value || 300 : 0;
  $("#gap-label").classList.toggle("hidden", b === 1);
  const widths = $("#nw-widths").value.split(/[\s,]+/).map(Number).filter((v) => v > 0);
  const reps = +$("#nw-reps").value || 0, blanks = +$("#nw-blanks").value || 0, n = +$("#nw-n").value || 0;
  if (!widths.length) { $("#nw-note").textContent = ""; return; }
  const need = (b * Math.max(...widths) + (b - 1) * gap) / 1000 + 0.3;
  const spacing = Math.max(1, Math.ceil(need * 2) / 2);
  const batches = widths.length * reps + blanks, perBatch = n * spacing + 3;
  const set = C.st && C.st.sets.find((s) => s.name === C.set);
  const name = b === 1 ? "single" : `burst${b}`;
  const clash = set && set.sweeps.some((w) => w.name === name);
  $("#nw-note").innerHTML = `${batches} batches, rewards ${spacing} s apart: about ${Math.round(perBatch)} s of pulses per batch, `
    + `${Math.round(batches * (perBatch + 30) / 60)} min with weighing. Saved as <code>${esc(set ? set.name : `${C.st ? C.st.defaults.today : "today"}_<setup>`)}/${name}</code>.`
    + (clash ? ` <span class="warn-text">This set already has a ${name} sweep: open it above, or choose New set.</span>` : "")
    + (b > 1 && gap !== 300 ? ` <span class="warn-text">The tasks use a ${300} ms gap by default (reward_pulse.gap_ms): calibrate the gap you reward with.</span>` : "");
  $("#btn-new").disabled = !!clash;
}

// ---------------------------------------------------------------- render, poll
function render() {
  const st = C.st;
  if (!st) return;
  const banner = $("#banner");
  const msg = st.busy ? `${st.busy} is running: the valve is the experiment's. You can look, but not dispense, until it stops.`
    : st.demo ? "Demo: a simulated cabinet, and a copy of the calibrations that is thrown away when the dashboard stops." : "";
  banner.textContent = msg;
  banner.className = "banner" + (msg ? (st.busy ? "" : " demo") : " hidden");
  $("#btn-prime").disabled = !!(st.job || st.busy);
  renderJob(st);
  renderInUse(st);
  renderCheck(st);
  renderSets(st);
  newNote();
}

async function refresh(again = true) {
  clearTimeout(C.timer);
  try { C.st = await api("/api/calib/state"); render(); } catch (e) { showLast({ ok: false, text: `Dashboard not reachable: ${e.message}` }); }
  if (again) C.timer = setTimeout(refresh, C.st && C.st.job ? 400 : 3000);
}

// ---------------------------------------------------------------- wiring
function showTab(name) {
  document.querySelectorAll(".diag-tabs .tab").forEach((t) => {
    const on = t.dataset.tab === name;
    t.classList.toggle("active", on); t.setAttribute("aria-selected", on ? "true" : "false");
  });
  document.querySelectorAll(".diag-panel").forEach((p) => p.classList.toggle("hidden", p.id !== `tab-${name}`));
  if (name === "inuse") renderChart(activeSet(), true);
  try { history.replaceState(null, "", `#${name}`); } catch (e) { /* file: or sandboxed */ }
}
document.querySelectorAll(".diag-tabs .tab").forEach((t) => (t.onclick = () => showTab(t.dataset.tab)));

$("#addr").addEventListener("change", () => { try { localStorage.setItem(ADDR_KEY, $("#addr").value.trim()); } catch (e) { /* blocked */ } });
$("#btn-cabinet").onclick = async () => {
  const pill = $("#cab-pill");
  try {
    const j = await api("/api/calib/cabinet", cabinet());
    pill.className = "pill " + (j.ok ? "ok" : "err");
    pill.textContent = j.ok ? `outputs available · ${j.ms} ms` : "no answer";
    pill.title = j.text;
  } catch (e) { pill.className = "pill err"; pill.textContent = e.message; }
};
$("#btn-prime").onclick = () => {
  const p = C.st.defaults.prime;
  act("/api/calib/prime", { ...cabinet(), width_ms: p.width_ms, count: p.count });
};
$("#btn-job-stop").onclick = () => act("/api/calib/stop", {});
$("#btn-record").onclick = () => {
  const m = $("#mass").value.trim();
  if (m === "") { $("#mass").focus(); return; }
  act("/api/calib/weigh", { mass_g: +m });
};
$("#mass").addEventListener("keydown", (e) => { if (e.key === "Enter") $("#btn-record").click(); });
$("#btn-discard").onclick = () => act("/api/calib/weigh", { discard: true });
$("#btn-check").onclick = () => act("/api/calib/check", { ...cabinet(), burst: +$("#ck-size").value,
  width_ms: +$("#ck-width").value, rewards: +$("#ck-n").value });
for (const id of ["#ck-size", "#ck-width", "#ck-n"]) $(id).addEventListener("input", expectCheck);
$("#set").onchange = () => { C.set = $("#set").value; C.sweep = null; C.fitOpen = false; render(); };
$("#nw-burst").addEventListener("input", () => {
  const b = +$("#nw-burst").value;
  if (DEFAULT_WIDTHS[b]) $("#nw-widths").value = DEFAULT_WIDTHS[b];
  newNote();
});
for (const id of ["#nw-gap", "#nw-widths", "#nw-reps", "#nw-blanks", "#nw-n"]) $(id).addEventListener("input", newNote);
$("#btn-new").onclick = async () => {
  try {
    const b = +$("#nw-burst").value;
    const r = await api("/api/calib/sweep", { set: C.set, setup: $("#setup").value, burst: b, gap_ms: +$("#nw-gap").value,
      widths: $("#nw-widths").value.split(/[\s,]+/).filter(Boolean).map(Number), reps: +$("#nw-reps").value,
      blanks: +$("#nw-blanks").value, rewards: +$("#nw-n").value });
    C.set = r.set; C.sweep = r.name; C.setsKey = "";
    showLast({ ok: true, text: `Created ${r.set}/${r.name}. Tare the cup, then dispense its first batch.` });
  } catch (e) { showLast({ ok: false, text: e.message }); }
  refresh(false);
};
window.addEventListener("resize", () => renderChart(activeSet()));

try { $("#addr").value = localStorage.getItem(ADDR_KEY) || ""; } catch (e) { /* blocked */ }
(async () => {
  await refresh(false);
  if (!$("#addr").value && C.st) $("#addr").value = C.st.defaults.cabinet;
  const tab = location.hash.slice(1);
  showTab(["inuse", "check", "new"].includes(tab) ? tab : "inuse");
  refresh();
})();
