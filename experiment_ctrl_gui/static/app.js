"use strict";
// KUKA Experiment Control -- front end. Talks only to server.py (same origin).

// The trial steps each orchestrator reports (TrialRecorder.mark / ExperimentControl.on_event).
// Cue-delivery tokens (cue_audio_end, cue_visual_ack, cue_snap, freeze) go to the event list only.
const STEPS = {
  apple_pluck: ["trial_start", "at_start", "quiet_end", "cue_go", "armed", "snap", "recover_start", "trial_end"],
  perturb: ["trial_start", "at_start", "quiet_end", "cue_go", "perturb_delay_start", "perturb_applied",
            "armed", "snap", "recover_start", "trial_end"],
  restricted_plane: ["trial_start", "at_start", "cue_go", "armed", "snap", "recover_start", "trial_end"],
  maze: ["trial_start", "at_start", "fixture_active", "cue_go", "maze_armed", "checkpoint", "goal",
         "release_wait", "released", "trial_end"],
};
const STEP_LABEL = {
  trial_start: "start", at_start: "at start", quiet_end: "quiet", cue_go: "go cue", armed: "armed",
  snap: "threshold", recover_start: "recover", trial_end: "end", perturb_delay_start: "delay",
  perturb_applied: "perturbed", fixture_active: "fixture", maze_armed: "maze on", checkpoint: "checkpoint",
  goal: "goal / timeout", release_wait: "release wait", released: "released",
};
const PHASE_TEXT = {
  starting: "starting up", trial_start: "moving to start", at_start: "quiet window", quiet_end: "go cue",
  cue_go: "go cue", armed: "waiting for the pull", snap: "threshold reached — holding", freeze: "holding",
  cue_snap: "threshold reached — holding", recover_start: "recovering to start", trial_end: "trial complete",
  perturb_delay_start: "delay before perturbation", perturb_applied: "perturbed — settling",
  fixture_active: "fixture on — quiet window", maze_armed: "in the maze", checkpoint: "in the maze",
  goal: "goal — waiting for release", timeout: "timed out — waiting for release",
  safety_trip: "SAFETY ABORT — recovering", release_wait: "waiting for release", released: "recovering to start",
  paused: "paused at the start", prestart_done: "moving to start", cue_audio_end: null, cue_visual_ack: null,
};
const TAB_HELP = {
  live: {
    running: "Changes go to the running orchestrator and apply from the NEXT trial — the current trial is never changed mid-way. Purple = accepted, waiting for the next trial.",
    idle: "These can also be changed while the experiment runs. Edits made now set the values it starts with.",
  },
  run: {
    running: "Locked for this run: the orchestrator read these once at start-up. Stop, edit, and Start again.",
    idle: "Everything else in the experiment YAML, poses and maze geometry included. Edits apply to the next Start and are then locked for the run. The package YAML is never changed: the dashboard runs an edited copy (runs/…yaml), and every trial's sidecar records the values used. Amber notes flag parameters that other things depend on.",
  },
  fixed: {
    running: "Shared by every experiment and not in the experiment YAML: the SmartPad (FRI) selections, launch arguments, the controller configuration and the CLIK posture. Change these at the SmartPad or in the files named.",
    idle: "Shared by every experiment and not in the experiment YAML: the SmartPad (FRI) selections, launch arguments, the controller configuration and the CLIK posture. Change these at the SmartPad or in the files named.",
  },
};

const $ = (s, el = document) => el.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const store = {
  get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* private window */ } },
};

const S = {
  experiments: [], selected: new URLSearchParams(location.search).get("exp") || store.get("exp", "apple_pluck"),
  tab: new URLSearchParams(location.search).get("tab") || store.get("tab", "live"),   // ?exp=maze&tab=run
  params: {}, app: null, events: [], lastPhaseT: null, logs: [], lastSeq: 0,
  logLevel: "all", logFilter: "", paramFilter: "", renderSig: "", renderDeferred: false,
};

// ---------------------------------------------------------------- server calls
async function api(path, body) {
  const opt = body === undefined ? {} : {
    method: "POST", headers: { "Content-Type": "application/json", "X-Experiment-Ctrl": "1" },
    body: JSON.stringify(body),
  };
  const r = await fetch(path, opt);
  const data = await r.json().catch(() => ({}));
  if (!r.ok || data.ok === false) throw new Error(data.error || `${r.status} ${r.statusText}`);
  return data;
}
function toast(msg, kind = "") {
  const t = document.createElement("div");
  t.className = `toast ${kind}`;
  t.textContent = msg;
  $("#toasts").appendChild(t);
  setTimeout(() => t.remove(), kind === "err" ? 9000 : 4000);
}
async function act(fn, okMsg) {
  try { const r = await fn(); if (okMsg) toast(r && r.message ? `${okMsg}: ${r.message}` : okMsg, "ok"); return r; }
  catch (e) { toast(e.message, "err"); return null; }
}

// ---------------------------------------------------------------- derived state
function statusLive() {
  const s = S.app && S.app.status;
  return s && !s.stale ? s : null;
}
function runningKey() {
  const a = S.app; if (!a) return null;
  if (a.runner.state !== "idle" && a.runner.experiment) return a.runner.experiment;
  const s = statusLive(); return s ? s.experiment : null;
}
function isRunningSelected() { return runningKey() === S.selected; }
function exp(key) { return S.experiments.find((e) => e.key === key); }

// ---------------------------------------------------------------- experiments list
function renderExperiments() {
  const list = $("#exp-list");
  const rk = runningKey();
  list.innerHTML = "";
  for (const e of S.experiments) {
    const b = document.createElement("button");
    b.className = "exp"; b.setAttribute("role", "radio");
    b.setAttribute("aria-checked", String(e.key === S.selected));
    const edits = S.app ? S.app.edited[e.key] : 0;
    b.innerHTML = `<div class="t">${esc(e.label)}
        ${rk === e.key ? '<span class="tag running">running</span>' : ""}
        ${edits ? `<span class="tag edited">${edits} edited</span>` : ""}</div>
      <div class="d">${esc(e.summary)}</div>`;
    b.onclick = () => select(e.key);
    list.appendChild(b);
  }
}
async function select(key) {
  S.selected = key; store.set("exp", key);
  renderExperiments(); renderChecklist();
  await loadParams(key);
  renderControls();
}
function renderChecklist() {
  const e = exp(S.selected); if (!e) return;
  const sp = Object.entries(e.smartpad).map(([k, v]) => `<li>${esc(k)}: <code>${esc(v)}</code></li>`).join("");
  $("#checklist").innerHTML = `
    <li>Arm powered, cabinet in <strong>T1</strong> for a first run; hand on the E-stop.</li>
    <li>On the SmartPad start the app and choose:<ul>${sp}</ul></li>
    <li>The app waits ~60 s for ROS — press <strong>Start</strong> here within that window.</li>
    <li>Visual cue over Wi-Fi? Join this computer to <code>KUKA_NEOPIXEL</code> first.</li>
    <li>Runs <code>ros2 launch sinthlab_bringup ${esc(e.launch_file)}</code>${e.run_name ? `; data goes to <code>analysis/expt_${esc(e.run_name)}_&lt;time&gt;/</code>` : " (writes its own trajectory CSV)"}.</li>`;
}

// ---------------------------------------------------------------- controls + status
function renderControls() {
  const a = S.app; if (!a) return;
  const r = a.runner, s = statusLive();
  const mine = r.state === "running", stopping = r.state === "stopping";
  const external = a.external;
  const rk = runningKey();

  $("#btn-start").disabled = mine || stopping || external;
  $("#btn-start").textContent = rk && rk !== S.selected ? `▶ Start (stop ${exp(rk)?.label ?? rk} first)` : `▶ Start ${exp(S.selected)?.label ?? ""}`;
  $("#btn-stop").disabled = !mine;
  $("#btn-stop-trial").disabled = !mine || a.stop_after_trial;
  $("#btn-restart").disabled = !mine;
  const canPause = (mine || external) && s;
  const pb = $("#btn-pause");
  pb.disabled = !canPause;
  pb.classList.toggle("on", !!(s && s.paused));
  pb.textContent = s && s.paused ? (s.held ? "▶ Resume" : "⏸ Pausing… (undo)") : "⏸ Pause after trial";
  $("#btn-stop-trial").textContent = a.stop_after_trial ? "■ Stopping after trial…" : "■ Stop after trial";
  $("#btn-validate").disabled = !a.data_folder;

  // pills
  const pr = $("#pill-ros");
  if (a.ros.demo) { pr.className = "pill warn"; pr.textContent = "DEMO"; }
  else if (a.ros.available) { pr.className = "pill ok"; pr.textContent = "ROS ✓"; }
  else { pr.className = "pill err"; pr.textContent = "ROS ✗"; }
  const rb = a.robot, pp = $("#pill-robot");
  if (rb.lbr_state_hz) { pp.className = rb.session_state === "COMMANDING_ACTIVE" ? "pill ok" : "pill warn";
    pp.textContent = `${rb.session_state ?? "FRI ?"} · ${rb.lbr_state_hz} Hz`; }
  else { pp.className = "pill"; pp.textContent = "Robot: no state"; }
  const pl = $("#pill-run");
  if (stopping) { pl.className = "pill warn"; pl.textContent = "Stopping…"; }
  else if (s && s.held) { pl.className = "pill pause"; pl.textContent = `Paused · ${exp(s.experiment)?.label ?? ""}`; }
  else if (s) { pl.className = "pill run"; pl.textContent = `${external ? "External · " : ""}${exp(s.experiment)?.label ?? s.experiment} · trial ${s.trial}`; }
  else if (mine) { pl.className = "pill warn"; pl.textContent = `Starting ${exp(r.experiment)?.label ?? ""}…`; }
  else { pl.className = "pill"; pl.textContent = r.exit_code != null ? `Idle (last exit ${r.exit_code})` : "Idle"; }

  // banner
  const bn = $("#banner");
  let msg = "", cls = "banner";
  if (a.ros.demo) { msg = "Demo mode — a simulated experiment. Nothing is launched and no robot is connected."; cls += " demo"; }
  else if (!a.ros.available) msg = `ROS is not available: ${a.ros.reason} Start and Stop still work; live changes, pause and robot state need ROS.`;
  if (external) msg = `${exp(s.experiment)?.label ?? s.experiment} is running from a terminal (${s.node}). Live changes and pause work from here; stop it with Ctrl-C in its terminal.`;
  if (s && s.phase === "safety_trip") { msg = "SAFETY ABORT — the runaway monitor tripped. The arm is recovering to the start pose."; }
  bn.className = msg ? cls : "banner hidden"; bn.textContent = msg;

  // run info
  const rows = [];
  if (r.experiment) rows.push(["Experiment", exp(r.experiment)?.label ?? r.experiment]);
  if (r.started_at && r.state !== "idle") rows.push(["Elapsed", fmtDur(Date.now() / 1000 - r.started_at)]);
  if (r.params_file) rows.push(["Params", r.params_file.split("/").slice(-2).join("/")]);
  if (a.data_folder) rows.push(["Data", a.data_folder.split("/").slice(-2).join("/")]);
  if (r.log_path) rows.push(["Log file", r.log_path.split("/").slice(-2).join("/")]);
  if (r.state === "idle" && r.exit_code != null) rows.push(["Exit code", r.exit_code]);
  $("#run-info").innerHTML = rows.map(([k, v]) => `<dt>${esc(k)}</dt><dd>${esc(v)}</dd>`).join("");
  $("#log-path").textContent = r.log_path ? `Full log: ${r.log_path}` : "";

  renderStatus();
}
function fmtDur(sec) {
  sec = Math.max(0, Math.floor(sec));
  const h = Math.floor(sec / 3600), m = Math.floor(sec / 60) % 60, s = sec % 60;
  return h ? `${h}:${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}` : `${m}:${String(s).padStart(2, "0")}`;
}
function renderStatus() {
  const s = statusLive();
  $("#st-trial").textContent = s ? s.trial || "–" : "–";
  const phaseName = s ? (s.held ? "paused" : s.phase) : null;
  const text = s ? (PHASE_TEXT[phaseName] ?? phaseName) : "not running";
  $("#st-phase").textContent = s && s.paused && !s.held ? `${text} (pausing after this trial)` : text;
  $("#st-phase-t").textContent = s ? `${Math.max(0, (Date.now() / 1000 - s.phase_t)).toFixed(0)} s` : "–";

  const key = s ? s.experiment : S.selected;
  const steps = STEPS[key] || [];
  let cur = s ? s.phase : null;
  if (cur === "timeout" || cur === "safety_trip") cur = "goal";
  if (cur === "cue_snap" || cur === "freeze") cur = "snap";
  if (cur === "quiet_end" && key === "restricted_plane") cur = "cue_go";
  let idx = steps.indexOf(cur);
  if (idx < 0 && s) {   // a cue/ack token: keep the last real step highlighted
    const last = [...S.events].reverse().find((e) => steps.includes(e.phase));
    idx = last ? steps.indexOf(last.phase) : -1;
  }
  $("#steps").innerHTML = steps.map((st, i) =>
    `<li class="${!s ? "" : i < idx ? "done" : i === idx ? "now" : ""}">${esc(STEP_LABEL[st] || st)}</li>`).join("");

  const pend = s && s.pending ? Object.entries(s.pending) : [];
  const pe = $("#pending");
  if (pend.length) {
    pe.classList.remove("hidden");
    pe.innerHTML = `<strong>Applies from trial ${s.trial + 1}:</strong> ` +
      pend.map(([k, v]) => `<code>${esc(k)}</code> = ${esc(JSON.stringify(v))}`).join(", ");
  } else pe.classList.add("hidden");
}
function noteEvent(s) {
  if (!s || s.phase_t === S.lastPhaseT) return;
  S.lastPhaseT = s.phase_t;
  S.events.push({ t: s.phase_t, trial: s.trial, phase: s.phase, arg: s.phase_arg });
  if (S.events.length > 300) S.events.shift();
  const li = document.createElement("li");
  const d = new Date(s.phase_t * 1000);
  li.innerHTML = `<span>${d.toLocaleTimeString([], { hour12: false })}.${String(d.getMilliseconds()).padStart(3, "0")}</span>
    <span class="muted">#${esc(s.trial)}</span>
    <span><span class="ev-tok">${esc(s.phase)}</span>${s.phase_arg != null ? ` <span class="muted">${esc(JSON.stringify(s.phase_arg))}</span>` : ""}</span>`;
  const list = $("#events");
  list.prepend(li);
  while (list.children.length > 200) list.lastChild.remove();
  $("#ev-count").textContent = `(${S.events.length})`;
}

// ---------------------------------------------------------------- hover help
// Every parameter name, group heading and Fixed row carries data-tip="<id>"; the popup content is
// built from the YAML's own documentation (description on the key's line, notes above it).
const TIPS = new Map();
let tipSeq = 0;
function tipId(html) { const id = `t${++tipSeq}`; TIPS.set(id, html); return id; }
function tipBlock(label, text, cls = "") { return text ? `<div class="tip-sec ${cls}"><div class="tip-label">${esc(label)}</div><div class="tip-text">${esc(text)}</div></div>` : ""; }
// YAML notes: prose paragraphs wrap; indented lines (tables, diagrams) keep their columns.
function notesBlock(text) {
  if (!text) return "";
  const lines = text.split("\n").map((l) => l === "" ? `<div class="gap"></div>`
    : `<div class="${/^\s/.test(l) ? "pre" : "prose"}">${esc(l)}</div>`).join("");
  return `<div class="tip-sec"><div class="tip-label">Notes (from the YAML)</div><div class="tip-notes">${lines}</div></div>`;
}
function paramTip(p, groups) {
  const parent = p.name.includes(".") ? p.name.slice(0, p.name.lastIndexOf(".")) : null;
  const pg = parent && groups[parent] && parent !== p.group ? groups[parent] : null;
  const meta = [p.type, `YAML default ${fmtVal(p.default)}`, p.limits ? `range ${p.limits[0]}–${p.limits[1]}` : null,
    p.choices ? `one of ${p.choices.join(" | ")}` : null, p.tier === "live" ? "LIVE: can change while running" : "per-run: locked once started"]
    .filter(Boolean).join(" · ");
  return `<div class="tip-name">${esc(p.name)}</div>` +
    (p.help ? `<div class="tip-desc">${esc(p.help)}</div>` : `<div class="tip-desc muted">No description in the YAML.</div>`) +
    `<div class="tip-meta">${esc(meta)}</div>` +
    (pg && pg.help ? tipBlock(`Block ${parent}`, pg.help) : "") +
    notesBlock(p.notes) +
    tipBlock("Caution", p.caution, "caution") +
    (p.linked && p.linked.length ? tipBlock("Linked", `Editing this also sets ${p.linked.join(", ")}`, "linked") : "");
}
function groupTip(name, g) {
  if (!g) return null;
  return `<div class="tip-name">${esc(name)}</div>` + (g.help ? `<div class="tip-desc">${esc(g.help)}</div>` : "") + notesBlock(g.notes);
}
function rowTip(r, source) {
  return `<div class="tip-name">${esc(r.name)}</div>` + (r.help ? `<div class="tip-desc">${esc(r.help)}</div>` : "") +
    `<div class="tip-meta">${esc(source)}</div>` + notesBlock(r.notes);
}
function showTip(el) {
  const html = TIPS.get(el.dataset.tip); if (!html) return;
  const tip = $("#tip"); tip.innerHTML = html; tip.classList.add("on");
  const r = el.getBoundingClientRect(), w = tip.offsetWidth, h = tip.offsetHeight;
  let x = Math.min(r.left, innerWidth - w - 12), y = r.bottom + 6;
  if (y + h > innerHeight - 8) y = Math.max(8, r.top - h - 6);
  tip.style.left = `${Math.max(8, x)}px`; tip.style.top = `${y}px`;
}
function hideTip() { $("#tip").classList.remove("on"); }
function wireTips() {
  const find = (e) => e.target.closest && e.target.closest("[data-tip]");
  document.addEventListener("mouseover", (e) => { const el = find(e); if (el) showTip(el); });
  document.addEventListener("mouseout", (e) => { const el = find(e); if (el && !el.contains(e.relatedTarget)) hideTip(); });
  document.addEventListener("focusin", (e) => { const el = find(e); if (el) showTip(el); });
  document.addEventListener("focusout", hideTip);
  document.addEventListener("keydown", (e) => { if (e.key === "Escape") hideTip(); });
  addEventListener("scroll", hideTip, true);
}

// ---------------------------------------------------------------- parameters
async function loadParams(key, force = false) {
  if (!S.params[key] || force) {
    try { S.params[key] = await api(`/api/params/${key}`); }
    catch (e) { toast(`Could not load parameters: ${e.message}`, "err"); return; }
  }
  S.renderSig = ""; renderParams();
}
function paramMode() { return isRunningSelected() ? "running" : "idle"; }

function renderParams(force = false) {
  const data = S.params[S.selected]; if (!data) return;
  const mode = paramMode(), s = statusLive();
  const liveVals = mode === "running" && s ? s.live : null, pending = mode === "running" && s ? s.pending : null;
  const sig = JSON.stringify([S.selected, S.tab, mode, liveVals, pending, S.paramFilter, data.params.map((p) => [p.value, p.edited])]);
  if (!force && sig === S.renderSig) return;
  if (!force && $("#params").contains(document.activeElement) && document.activeElement.tagName !== "BUTTON") {
    S.renderDeferred = true; return;           // do not yank an input out from under the cursor
  }
  S.renderSig = sig; S.renderDeferred = false;

  const counts = { live: 0, run: 0 };
  data.params.forEach((p) => counts[p.tier]++);
  $("#n-live").textContent = counts.live; $("#n-run").textContent = counts.run;
  $("#n-fixed").textContent = data.fixed.reduce((n, sec) => n + sec.rows.length, 0);
  document.querySelectorAll(".tab").forEach((t) => t.classList.toggle("active", t.dataset.tab === S.tab));
  $("#tab-help").textContent = TAB_HELP[S.tab][mode];
  $("#btn-reset-edits").disabled = mode === "running" || !data.params.some((p) => p.edited);

  const f = S.paramFilter.toLowerCase();
  const rows = data.params.filter((p) => p.tier === S.tab && (!f || p.name.toLowerCase().includes(f) || (p.help || "").toLowerCase().includes(f)));
  const root = $("#params"); root.innerHTML = ""; TIPS.clear(); hideTip();
  const groups = data.groups || {};
  const byGroup = new Map();
  for (const p of rows) { if (!byGroup.has(p.group)) byGroup.set(p.group, []); byGroup.get(p.group).push(p); }
  for (const [g, ps] of byGroup) {
    const box = document.createElement("div"); box.className = "group";
    const gt = groupTip(g, groups[g]);
    const gdesc = groups[g] && groups[g].help;
    box.innerHTML = `<h3${gt ? ` data-tip="${tipId(gt)}" tabindex="0"` : ""}>${esc(g)}${gdesc ? `<span class="gdesc"> — ${esc(gdesc)}</span>` : ""}</h3>`;
    for (const p of ps) box.appendChild(paramRow(p, mode, liveVals, pending, groups));
    root.appendChild(box);
  }
  if (S.tab === "fixed") {
    for (const sec of data.fixed) {
      const rowsF = sec.rows.filter((r) => !f || r.name.toLowerCase().includes(f));
      if (!rowsF.length) continue;
      const box = document.createElement("div"); box.className = "group fixed-section";
      box.innerHTML = `<h3>${esc(sec.title)}</h3><div class="src">${esc(sec.source)}</div>` +
        rowsF.map((r) => `<div class="param"><div class="name" tabindex="0" data-tip="${tipId(rowTip(r, sec.source))}">${esc(r.name)}</div><div class="ro">${esc(fmtVal(r.value))}</div>` +
          (r.help ? `<div class="help">${esc(r.help)}</div>` : "") + `</div>`).join("");
      root.appendChild(box);
    }
  }
  if (!root.children.length) root.innerHTML = `<p class="muted">No parameters${f ? " match the filter" : ""}.</p>`;
}
function fmtVal(v) { return Array.isArray(v) ? `[${v.join(", ")}]` : typeof v === "object" && v !== null ? JSON.stringify(v) : String(v); }

function paramRow(p, mode, liveVals, pending, groups = {}) {
  const row = document.createElement("div"); row.className = "param";
  const isLiveRunning = p.tier === "live" && mode === "running";
  const locked = p.tier === "run" && mode === "running";
  if (locked) row.classList.add("locked");
  const value = isLiveRunning && liveVals && p.name in liveVals ? liveVals[p.name] : p.value;
  const pend = isLiveRunning && pending && p.name in pending;
  // The group heading already carries the prefix; show the rest (full name on hover).
  const short = p.group !== "general" && p.name.startsWith(p.group + ".") ? p.name.slice(p.group.length + 1) : p.name;
  row.innerHTML = `<div class="name" tabindex="0" data-tip="${tipId(paramTip(p, groups))}">${esc(short)}${p.edited ? '<span class="badge edited">edited</span>' : ""}${pend ? `<span class="badge pending">next trial → ${esc(fmtVal(pending[p.name]))}</span>` : ""}</div>`;
  const ctl = document.createElement("div"); ctl.className = "ctl";
  {
    const commit = async (v) => {
      if (isLiveRunning) {
        const r = await act(() => api("/api/live", { name: p.name, value: v }), `${p.name}`);
        if (!r) renderParams(true);        // rejected: put the control back to the real value
      } else {
        const r = await act(() => api(`/api/params/${S.selected}`, { name: p.name, value: v }));
        await loadParams(S.selected, true);
        if (r) toast(`${p.name} = ${fmtVal(r.value)} for the next Start` +
                     (p.linked && p.linked.length ? ` (and ${p.linked.length} linked)` : ""), "ok");
      }
    };
    buildControl(ctl, p, value, commit, locked);
    if (!locked && !isLiveRunning && p.edited) {
      const rv = document.createElement("button"); rv.className = "revert"; rv.title = `Back to the YAML value (${fmtVal(p.default)})`;
      rv.textContent = "↺"; rv.onclick = () => commit(p.default); ctl.appendChild(rv);
    }
    if (p.limits) { const d = document.createElement("span"); d.className = "default"; d.textContent = `${p.limits[0]}–${p.limits[1]}`; ctl.appendChild(d); }
  }
  row.appendChild(ctl);
  if (p.help) { const h = document.createElement("div"); h.className = "help"; h.textContent = p.help; row.appendChild(h); }
  if (p.caution) { const h = document.createElement("div"); h.className = "help caution"; h.textContent = `⚠ ${p.caution}`; row.appendChild(h); }
  if (p.linked && p.linked.length) {
    const h = document.createElement("div"); h.className = "help linked";
    h.textContent = `🔗 Editing this also sets: ${p.linked.join(", ")}`; row.appendChild(h);
  }
  return row;
}

function buildControl(ctl, p, value, commit, disabled) {
  if (p.type === "bool") {
    const lab = document.createElement("label"); lab.className = "switch";
    lab.innerHTML = `<input type="checkbox" ${value ? "checked" : ""} ${disabled ? "disabled" : ""} aria-label="${esc(p.name)}"><span></span>`;
    lab.querySelector("input").onchange = (e) => commit(e.target.checked);
    ctl.appendChild(lab);
    const t = document.createElement("span"); t.className = "default"; t.textContent = value ? "on" : "off"; ctl.appendChild(t);
    return;
  }
  if (p.choices) {
    const sel = document.createElement("select"); sel.disabled = disabled; sel.setAttribute("aria-label", p.name);
    for (const c of p.choices) { const o = document.createElement("option"); o.value = o.textContent = c; if (c === String(value)) o.selected = true; sel.appendChild(o); }
    sel.onchange = () => commit(sel.value);
    ctl.appendChild(sel); return;
  }
  if (p.name.startsWith("visual_cue.colours.") && Array.isArray(value)) {
    const [r, g, b, w = 0] = value.map(Number);
    const sw = document.createElement("span"); sw.className = "swatch";
    sw.style.background = `rgb(${Math.min(255, r + w)}, ${Math.min(255, g + w)}, ${Math.min(255, b + w)})`;
    sw.title = "Approximate: the ring's white LED is shown as added white";
    const pick = document.createElement("input"); pick.type = "color"; pick.disabled = disabled;
    pick.value = "#" + [r, g, b].map((x) => x.toString(16).padStart(2, "0")).join("");
    pick.onchange = () => { const h = pick.value; commit([1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16)).concat(value.length > 3 ? [w] : [])); };
    ctl.append(sw, pick);
  }
  const inp = document.createElement("input");
  const numeric = p.type === "int" || p.type === "float";
  inp.type = numeric ? "number" : "text"; inp.disabled = disabled; inp.setAttribute("aria-label", p.name);
  if (numeric) { inp.step = p.type === "int" ? "1" : "any"; if (p.limits) { inp.min = p.limits[0]; inp.max = p.limits[1]; } }
  inp.value = Array.isArray(value) ? `[${value.join(", ")}]` : String(value);
  inp.onkeydown = (e) => { if (e.key === "Escape") { inp.value = Array.isArray(value) ? `[${value.join(", ")}]` : String(value); inp.blur(); } };
  inp.onchange = () => commit(inp.value);
  inp.onblur = () => { if (S.renderDeferred) setTimeout(() => renderParams(), 0); };
  ctl.appendChild(inp);
}

// ---------------------------------------------------------------- log
function levelShown(l) {
  if (S.logLevel === "all") return true;
  if (S.logLevel === "warn") return l === "warn" || l === "error";
  if (S.logLevel === "error") return l === "error";
  if (S.logLevel === "gui") return l === "gui" || l === "cmd";
  return true;
}
function logEl(e) {
  const div = document.createElement("div");
  div.className = `l ${e.level}`;
  const d = new Date(e.t * 1000);
  let text = esc(e.text);
  if (S.logFilter) {
    const re = new RegExp(S.logFilter.replace(/[.*+?^${}()|[\]\\]/g, "\\$&"), "gi");
    text = text.replace(re, (m) => `<mark>${m}</mark>`);
  }
  div.innerHTML = `<span class="ts">${d.toLocaleTimeString([], { hour12: false })}</span>${text}`;
  return div;
}
function logMatches(e) { return levelShown(e.level) && (!S.logFilter || e.text.toLowerCase().includes(S.logFilter.toLowerCase())); }
function appendLog(e) {
  if (e.seq <= S.lastSeq) return;
  S.lastSeq = e.seq;
  S.logs.push(e); if (S.logs.length > 5000) S.logs.shift();
  if (!logMatches(e)) return;
  const box = $("#log");
  box.appendChild(logEl(e));
  while (box.children.length > 3000) box.firstChild.remove();
  if ($("#log-follow").checked) box.scrollTop = box.scrollHeight;
}
function rerenderLog() {
  const box = $("#log"); box.innerHTML = "";
  const frag = document.createDocumentFragment();
  S.logs.filter(logMatches).slice(-3000).forEach((e) => frag.appendChild(logEl(e)));
  box.appendChild(frag);
  if ($("#log-follow").checked) box.scrollTop = box.scrollHeight;
}

// ---------------------------------------------------------------- wiring
function onState(a) {
  const prevKey = runningKey();
  S.app = a;
  noteEvent(statusLive());
  const key = runningKey();
  // When an experiment starts (here or in a terminal), show it: its Live tab is what you need.
  if (key && key !== prevKey && key !== S.selected) { select(key); return; }
  renderControls();
  if (key !== prevKey) renderExperiments();
  renderParams();
}
function connect() {
  // ?nostream: poll instead of streaming -- for a proxy that breaks Server-Sent Events, and for
  // headless screenshots (an open stream never lets a headless browser go idle).
  if (new URLSearchParams(location.search).has("nostream")) {
    setInterval(async () => {
      try { onState(await api("/api/state")); (await api("/api/logs")).forEach(appendLog); } catch { /* retry */ }
    }, 1000);
    return;
  }
  const es = new EventSource("/api/events");
  es.onopen = () => { if (S.app) renderControls(); };
  es.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === "log") appendLog(m.data);
    else if (m.type === "state") onState(m.data);
  };
  es.onerror = () => { $("#pill-run").className = "pill err"; $("#pill-run").textContent = "Dashboard disconnected"; };
}

async function init() {
  S.experiments = await api("/api/experiments");
  if (!exp(S.selected)) S.selected = S.experiments[0].key;
  (await api("/api/logs")).forEach(appendLog);
  S.app = await api("/api/state");
  if (runningKey()) S.selected = runningKey();     // open on whatever is running
  renderExperiments(); renderChecklist();
  await loadParams(S.selected);
  renderControls();
  connect();
  wireTips();
  // ?hover=<parameter name>: open that parameter's help, e.g. to link someone to it.
  const hv = new URLSearchParams(location.search).get("hover");
  if (hv) setTimeout(() => {
    const p = S.params[S.selected]?.params.find((x) => x.name === hv);
    if (p) { S.tab = p.tier; renderParams(true); }
    const el = [...document.querySelectorAll("#params .name[data-tip]")].find((n) => TIPS.get(n.dataset.tip)?.includes(`>${esc(hv)}<`));
    if (el) {
      const r = el.getBoundingClientRect();
      if (r.top < 0 || r.bottom > innerHeight) el.scrollIntoView({ block: "center" });
      showTip(el);
    }
  }, 300);

  $("#btn-start").onclick = () => act(() => api("/api/start", { experiment: S.selected }), "Starting");
  $("#btn-stop").onclick = () => { if (confirm("Stop now? The current trial is cut short and saved as partial.")) act(() => api("/api/stop", { mode: "now" }), "Stopping"); };
  $("#btn-stop-trial").onclick = () => act(() => api("/api/stop", { mode: "after_trial" }), "Will stop after this trial");
  $("#btn-restart").onclick = () => { if (confirm("Restart? This stops the launch (the current trial is saved as partial) and starts it again with the current edits.")) act(() => api("/api/restart", {}), "Restarting"); };
  $("#btn-pause").onclick = () => { const s = statusLive(); act(() => api("/api/pause", { paused: !(s && s.paused) }), s && s.paused ? "Resumed" : "Pause requested"); };
  $("#btn-validate").onclick = () => act(() => api("/api/validate", {}), "Validating — see the log");
  $("#btn-reset-edits").onclick = async () => { await act(() => api(`/api/params/${S.selected}/reset`, {}), "Edits cleared"); loadParams(S.selected, true); };
  document.querySelectorAll(".tab").forEach((t) => t.onclick = () => { S.tab = t.dataset.tab; store.set("tab", S.tab); renderParams(true); });
  $("#param-filter").oninput = (e) => { S.paramFilter = e.target.value; renderParams(true); };
  document.querySelectorAll("#log-levels button").forEach((b) => b.onclick = () => {
    S.logLevel = b.dataset.level;
    document.querySelectorAll("#log-levels button").forEach((x) => x.classList.toggle("on", x === b));
    rerenderLog();
  });
  $("#log-filter").oninput = (e) => { S.logFilter = e.target.value; rerenderLog(); };
  $("#btn-log-clear").onclick = () => { $("#log").innerHTML = ""; };
  setInterval(() => { if (S.app) renderStatus(); }, 1000);
}
init().catch((e) => toast(`Dashboard failed to load: ${e.message}`, "err"));
