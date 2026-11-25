"use strict";
// KUKA Experiment Control -- front end. Talks only to server.py (same origin).

// The trial steps each orchestrator reports (TrialRecorder.mark / ExperimentControl.on_event). A trial
// runs from trial_start (at park, as the wait before the move to start begins) to trial_end (the
// outcome, or a failure); the park move after it is the trial's tail.
// Cue-delivery tokens (cue_audio_end, cue_visual_ack, cue_snap, freeze) go to the event list only.
const STEPS = {
  apple_pluck: ["trial_start", "at_start", "quiet_end", "cue_go", "armed", "snap", "trial_end", "recover_start"],
  // Apple pluck, perturb paradigm (picked by the paradigm, see stepsFor).
  apple_pluck_perturb: ["trial_start", "at_start", "quiet_end", "cue_go", "perturb_delay_start", "perturb_applied",
            "armed", "snap", "trial_end", "recover_start"],
  // Apple pluck, perturb bump paradigm: the perturbation waits for the grasp.
  apple_pluck_perturb_bump: ["trial_start", "at_start", "quiet_end", "cue_go", "grasp_wait", "grasp",
            "perturb_delay_start", "perturb_applied", "armed", "snap", "trial_end", "recover_start"],
  restricted_plane: ["trial_start", "at_start", "cue_go", "armed", "snap", "trial_end", "recover_start"],
  maze: ["trial_start", "at_start", "fixture_active", "cue_go", "maze_armed", "checkpoint", "goal",
         "trial_end", "release_wait", "released"],
};
// Pre-training. Free Move: each trial is one session, ended by session_sec or Stop. Move Vertical /
// Horizontal are Free Move on a rail, one centre-out reach per trial (target, then reach / reach_wrong).
STEPS.restricted_move_horizontal = STEPS.restricted_move_vertical = STEPS.free_move = ["trial_start", "at_start", "fixture_active", "cue_go", "armed", "session_end",
                   "trial_end", "release_wait", "released"];
STEPS.reach_map = STEPS.free_move;
STEPS.n_choice = ["trial_start", "at_start", "fixture_active", "cue_go", "armed", "choice",
                  "trial_end", "release_wait", "released"];
STEPS.move_vertical = STEPS.move_horizontal = STEPS.colour_rule =
  ["trial_start", "at_start", "fixture_active", "cue_go", "armed", "reach",
   "trial_end", "release_wait", "released"];
const STEP_LABEL = {
  trial_start: "start", at_start: "at start", quiet_end: "quiet", cue_go: "go cue", armed: "armed",
  snap: "threshold", recover_start: "park", trial_end: "end", perturb_delay_start: "delay",
  perturb_applied: "perturbed", fixture_active: "fixture", maze_armed: "maze on", checkpoint: "checkpoint",
  goal: "goal / timeout", release_wait: "release wait", released: "released", threshold: "threshold",
  session_end: "session over", grasp_wait: "grasp wait", grasp: "grasped", reach: "reach", choice: "choice",
};
const PHASE_TEXT = {
  starting: "starting up", trial_start: "at park — then to the start pose", at_start: "quiet window", quiet_end: "go cue",
  cue_go: "go cue", armed: "waiting for the pull", snap: "threshold reached — plucked", freeze: "plucked — no pull-back",
  cue_snap: "threshold reached — plucked", release_wait: "waiting for the arm to come to rest", recover_start: "moving to park", trial_end: "trial over",
  parking: "first move: to the park pose (slow)",
  perturb_delay_start: "delay before perturbation", perturb_applied: "perturbed — settling",
  fixture_active: "fixture on — quiet window", maze_armed: "in the maze", checkpoint: "in the maze",
  goal: "goal — waiting for release", timeout: "timed out — waiting for release",
  safety_trip: "SAFETY ABORT — parking", released: "moving to park",
  early_touch: "EARLY TOUCH — no reward, parking, then a penalty wait",
  response_timeout: "NO RESPONSE in time — no reward, parking",
  stopped: "stopped — holding at park until the launch ends", cue_audio_end: null, cue_visual_ack: null,
  threshold: "threshold reached — coming back",
  session_end: "session over — waiting for release",
  grasp_wait: "waiting for the grasp", grasp: "grasped — delay before the bump",
  grasp_timeout: "no grasp — resetting",
  stop_requested: "STOP — ending the trial, parking",
  closed_end: null,      // the one-direction trial's closed half of the rail, marked just before `target`
  cue_colour: null, reversal: null,   // the colour rule's cue and reversal marks, just before `target`
};
// Where a phase means something different per experiment.
const PHASE_TEXT_BY_EXP = {
  free_move: { armed: "free move — the arm follows the hand", session_end: "session over — waiting for release",
               release_wait: "waiting for release",
               // Long moves (long_move.reward each_move): each movement is reported as it starts and ends.
               move_start: "moving", move_end: "free move — the arm follows the hand",
               long_move: "long movement — reward" },
  move_vertical: { session_end: "session over — waiting for release",
                   release_wait: "waiting for release", target: "go — the goal lit", armed: "reach to the lit end of the rail", reach: "reached the goal — reward, waiting for release", reach_wrong: "reached the other end — no reward, waiting for release", },
  move_horizontal: { session_end: "session over — waiting for release",
                     release_wait: "waiting for release", target: "go — the goal lit", armed: "reach to the lit end of the rail", reach: "reached the goal — reward, waiting for release", reach_wrong: "reached the other end — no reward, waiting for release", },
  n_choice: { release_wait: "waiting for release", armed: "reach out along a rail",
              choice: "chose — reward, waiting for release", choice_unrewarded: "chose — no reward, waiting for release" },
  colour_rule: { release_wait: "waiting for release", target: "go — the cue colour lit", armed: "choose an end",
                 reach: "correct — reward, waiting for release", reach_wrong: "wrong end — no reward, waiting for release" },
  reach_map: { armed: "mapping — he moves the arm freely in the plane", session_end: "session over — waiting for release",
               release_wait: "waiting for release", move_start: "moving", move_end: "mapping — he moves the arm freely in the plane",
               long_move: "long movement — reward", new_best: "new best reach — reward" },
  restricted_move_horizontal: { armed: "free move on the rail", session_end: "session over — waiting for release",
                                release_wait: "waiting for release" },
  restricted_move_vertical: { armed: "free move on the rail", session_end: "session over — waiting for release",
                              release_wait: "waiting for release" },
  maze: { release_wait: "waiting for release", response_timeout: "timed out — waiting for release" },
  restricted_plane: { release_wait: "waiting for release" },
};
// The reward: on every task's front page (Task view), first, since it is the setting changed most over
// training (started large, rolled back). Also in the Monkey view, with the rest of the profile.
const REWARD_ROWS = ["reward_pulse.pulse_ms", "reward_pulse.count", "reward_pulse.gap_ms"];
const REWARD_STEP_MS = 10;
// Task view group headings.
const GROUP_LABEL = { general: "Session", centre_out: "Choice", rule: "Colour rule", n_choice: "Choices", long_move: "Movements",
                      reach_map: "Reach map", free_move: "Geometry", reward_pulse: "Reward",
                      apple_pluck_impedance_control_displacement: "Pull", resistance: "Pull", perturb_start: "Perturbation",
                      grasp: "Grasp", virtual_fixtures: "Fixture" };
// mL per reward from the valve calibration (experiment_ctrl_gui/reward_volume.py does the same sums):
// interpolated within a profile's calibrated widths, its straight-line fit outside them (marked).
function rewardVolume(ms, count = 1, gap = 0) {
  const p = ((S.cal && S.cal.profiles) || []).find((x) => x.burst === Number(count));
  if (!p) return { ml: null, status: "none", note: S.cal && S.cal.set ? `no calibration for ${count} pulse(s)` : "no reward calibration" };
  const [lo, hi] = p.valid_width_ms; ms = Number(ms);
  let ul, status = "calibrated", note = "";
  if (ms >= lo && ms <= hi) {
    const pts = p.points; let i = 0;
    while (i < pts.length - 2 && ms > pts[i + 1][0]) i++;
    const [w0, v0] = pts[i], [w1, v1] = pts[i + 1];
    ul = v0 + (v1 - v0) * (ms - w0) / (w1 - w0);
  } else { ul = p.slope_ul_per_ms * ms + p.intercept_ul; status = "extrapolated"; note = `calibrated ${lo}–${hi} ms`; }
  if (Number(count) > 1 && p.gap_ms && Number(gap) !== p.gap_ms) note = [note, `calibrated with a ${p.gap_ms} ms gap`].filter(Boolean).join("; ");
  return { ml: Math.max(ul, 0) / 1000, status, note };
}
function fmtMl(v) { return v == null ? "–" : v === 0 ? "0 mL" : v >= 100 ? `${Math.round(v)} mL` : `${v.toFixed(v < 10 ? 2 : 1)} mL`; }
// Set a parameter of the selected experiment: live while it runs (if live), else for the next Start.
async function setParam(name, value) {
  const p = (S.params[S.selected]?.params || []).find((x) => x.name === name);
  const live = isRunningSelected() && p && p.tier === "live";
  const r = await act(() => live ? api("/api/live", { name, value }) : api(`/api/params/${S.selected}`, { name, value }),
                      `${name} = ${value}${live ? " from the next trial" : " for the next Start"}`);
  if (!live) await loadParams(S.selected, true);
  return r;
}
const TAB_HELP = {
  task: {
    running: "The reward and today's settings for this task. The reward and other live settings change from the next trial; the rest are locked for this run. The monkey's other settings are under Monkey, everything else under All settings.",
    idle: "The reward, then the few settings this task needs today. The monkey's other settings (feel, reach, pacing) are under Monkey; everything else under All settings. Edits apply to the next Start.",
  },
  monkey: {
    running: "The monkey's profile, as this task uses it. Tune the feel live (from the next move or trial); when it feels right, Save to profile and every task gets it from its next Start.",
    idle: "The monkey's profile, as this task uses it: the feel, the reach (as this task's rail ends), the pacing and the reward. A change here is for the next Start only, until you Save to profile.",
  },
  live: {
    running: "Changes go to the running orchestrator and apply from the NEXT trial — the current trial is never changed mid-way. Purple = accepted, waiting for the next trial. Exception: rows marked “applies now” (the move speed) act at once — from the next move, even within this trial — and each change is logged in the trial's sidecar.",
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
  experiments: [], groups: [], group: "experiments", selected: new URLSearchParams(location.search).get("exp") || store.get("exp", "apple_pluck"),
  tab: new URLSearchParams(location.search).get("tab") || store.get("tab", "live"),   // ?exp=maze&tab=run
  // The settings view: task (today's few), monkey (the profile's, as this task uses them) or all (Live / Per-run / Fixed).
  pview: new URLSearchParams(location.search).get("view") || store.get("pview", "task"),
  prof: null, profSig: null,      // /api/profile: the monkey, its profile, the layout, the change log
  cal: null,                      // /api/reward/calibration: the valve calibration, for mL per reward
  rewardPop: false, warnCount: 0, curView: "task",
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
function reportedStiffness() {     // X stiffness the SmartPad app reports [N/m], or null
  const r = (statusLive() || {}).smartpad;
  const k = r && r.received && r.settings && Array.isArray(r.settings.stiffness) ? Number(r.settings.stiffness[0]) : NaN;
  return Number.isFinite(k) && k > 0 ? k : null;
}
function runningKey() {
  const a = S.app; if (!a) return null;
  if (a.runner.state !== "idle" && a.runner.experiment) return a.runner.experiment;
  const s = statusLive(); return s ? s.experiment : null;
}
function isRunningSelected() { return runningKey() === S.selected; }
function exp(key) { return S.experiments.find((e) => e.key === key); }
// Paradigm experiments (apple pluck; Move Vertical / Horizontal): the run's paradigm is the
// experiment's paradigm_param (`paradigm`, or `centre_out.target`).
function paradigmParam(key) { return exp(key)?.paradigm_param || "paradigm"; }
// A live paradigm parameter (the rails' centre_out.target) can be switched during the run, from the next trial.
function paradigmIsLive(key) {
  const d = S.params[key], name = paradigmParam(key); const p = d && d.params.find((x) => x.name === name);
  return !!p && p.tier === "live";
}
// While `key` runs: the paradigm the current trial runs (`now`) and one waiting for the next trial (`next`).
function runningParadigm(key) {
  const s = statusLive(), name = paradigmParam(key);
  if (!s || runningKey() !== key || s.experiment !== key) return { now: null, next: null };
  const now = s.live && name in s.live ? String(s.live[name]) : null;
  const next = s.pending && name in s.pending ? String(s.pending[name]) : null;
  return { now, next: next === now ? null : next };
}
// The paradigm in effect: while running, the current trial's; with upcoming, the next trial's.
function paradigmOf(key, upcoming = false) {
  const run = runningParadigm(key);
  if (run.now !== null) return upcoming && run.next !== null ? run.next : run.now;
  const d = S.params[key], name = paradigmParam(key); const p = d && d.params.find((x) => x.name === name);
  return p ? String(p.value) : null;
}
function expLabel(key) {
  const e = exp(key); if (!e) return key ?? "";
  const pr = (e.paradigms || []).find((x) => x.key === paradigmOf(key));
  return pr ? `${e.label} · ${pr.label}` : e.label;      // e.label already names a card's variant
}
function stepsFor(key) {
  if (key === "apple_pluck" && paradigmOf(key) === "perturb") return STEPS.apple_pluck_perturb;
  if (key === "apple_pluck" && paradigmOf(key) === "perturb_bump") return STEPS.apple_pluck_perturb_bump;
  return STEPS[key] || [];
}

// ---------------------------------------------------------------- experiments list
// Cards: an experiment is a card of its own, unless it shares one (Experiment.card: Restricted Move and Cued
// Reaches each hold a horizontal and a vertical experiment). A card with several shows a variant row (Rail)
// above the selected experiment's paradigm tabs; the variant last used on a card is remembered.
function cardsOf(group) {
  const cards = [];
  for (const e of S.experiments.filter((x) => x.group === group)) {
    const key = e.card || e.key;
    let c = cards.find((x) => x.key === key);
    if (!c) { c = { key, label: e.card_label || e.label, summary: e.card_summary || e.summary, members: [] }; cards.push(c); }
    c.members.push(e);
  }
  return cards;
}
function renderExperiments() {
  // The run bar's task picker: one drop-down of cards, grouped Experiments / Pre-training, then chips for the
  // card's variant (Rail: Horizontal / Vertical) and the selected experiment's paradigm.
  const box = $("#task-picker"), rk = runningKey(), sel = exp(S.selected);
  const selCard = sel ? (sel.card || sel.key) : null;
  const cards = S.groups.map((g) => ({ g, cs: cardsOf(g.key) }));
  box.innerHTML = `<select id="task-select" aria-label="Task"${rk ? ` disabled title="Stop ${esc(expLabel(rk))} to change the task"` : ""}>` + cards.map(({ g, cs }) =>
    `<optgroup label="${esc(g.label)}">` + cs.map((c) => `<option value="${esc(c.key)}"${c.key === selCard ? " selected" : ""}>` +
      `${esc(c.label)}${c.members.some((m) => m.key === rk) ? " ● running" : ""}</option>`).join("") + `</optgroup>`).join("") + `</select>`;
  $("#task-select").onchange = (ev) => {
    const c = cards.flatMap((x) => x.cs).find((x) => x.key === ev.target.value); if (!c) return;
    const target = c.members.find((m) => m.key === rk) || c.members.find((m) => m.key === store.get(`variant.${c.key}`)) || c.members[0];
    select(target.key);
  };
  const card = cards.flatMap((x) => x.cs).find((x) => x.key === selCard);
  if (card && card.members.length > 1) {          // each variant is its own experiment: fixed while one of them runs
    const locked = card.members.some((m) => m.key === rk);
    const row = document.createElement("span"); row.className = "chips"; row.setAttribute("role", "tablist"); row.setAttribute("aria-label", `${card.label} rail`);
    for (const m of card.members) {
      const t = document.createElement("button"); const on = m.key === sel.key;
      t.className = `chip${on ? " on" : ""}`; t.textContent = m.variant || m.label; t.setAttribute("role", "tab"); t.setAttribute("aria-selected", String(on));
      t.disabled = locked && !on; t.title = locked ? "Fixed for this run — stop to change it" : `${card.label} on the ${(m.variant || "").toLowerCase()} rail`;
      t.onclick = () => { if (!t.disabled && !on) select(m.key); };
      row.appendChild(t);
    }
    box.appendChild(row);
  }
  if (sel) renderParadigmTabs(sel, box, rk);
  const edits = S.app && sel ? (S.app.edited[sel.key] || 0) : 0;
  $("#task-sub").textContent = card ? `${card.summary}${edits ? ` · ${edits} edit${edits > 1 ? "s" : ""} for the next Start` : ""}` : "";
  $("#task-sub").title = card ? card.summary : "";
}
function renderParadigmTabs(e, box, rk) {
  // The paradigm chips (apple pluck: normal / perturb / ...; the rails: either / cued / one end only). They set the
  // paradigm parameter for the next Start. While that experiment runs they are locked -- unless the parameter is
  // live (the rails, the Reach Map's reward): then they switch it from the next trial, and the one waiting is marked.
  if (!(e.paradigms || []).length || !S.params[e.key]) return;
  const running = rk === e.key, live = running && paradigmIsLive(e.key);
  const run = running ? runningParadigm(e.key) : { now: null, next: null };
  const cur = paradigmOf(e.key), locked = running && !live;
  if (box.querySelector(".chips")) { const sep = document.createElement("span"); sep.className = "chip-sep"; box.appendChild(sep); }
  const row = document.createElement("span"); row.className = "chips"; row.setAttribute("role", "tablist"); row.setAttribute("aria-label", `${e.label} paradigm`);
  for (const pr of e.paradigms) {
    const t = document.createElement("button");
    const isNext = pr.key === run.next;
    t.className = `chip${pr.key === cur ? " on" : ""}${isNext ? " next" : ""}`; t.setAttribute("role", "tab");
    t.setAttribute("aria-selected", String(pr.key === cur));
    t.textContent = pr.label + (isNext ? " · next trial" : "");
    t.disabled = locked && pr.key !== cur;
    t.title = locked ? "Fixed for this run — stop to change it"
      : live ? (pr.key === cur && !run.next ? "Running now" : `Switch to ${pr.label.toLowerCase()} from the next trial`)
      : `Run ${e.label} in the ${pr.label.toLowerCase()} paradigm`;
    t.onclick = async () => {
      if (locked) return;
      if (live) {          // the status update that follows re-renders the chips
        if (pr.key === (run.next ?? cur)) return;
        await act(() => api("/api/live", { name: paradigmParam(e.key), value: pr.key }), `${e.label}: ${pr.label.toLowerCase()} from the next trial`);
        return;
      }
      if (pr.key === cur) return;
      await act(() => api(`/api/params/${e.key}`, { name: paradigmParam(e.key), value: pr.key }));
      await loadParams(e.key, true); renderExperiments(); renderControls();
    };
    row.appendChild(t);
  }
  box.appendChild(row);
}
async function select(key) {
  const rk = runningKey();
  if (rk && key !== rk) { toast(`Stop ${expLabel(rk)} first: the page stays on the running task`, "err"); key = rk; }
  S.selected = key; store.set("exp", key);
  if (exp(key)?.card) store.set(`variant.${exp(key).card}`, key);   // the card reopens on this rail
  if (!runningKey()) { SV.data = SV.hist = null; setTimeout(() => refreshSession(true), 0); }
  S.group = exp(key)?.group ?? S.group;
  renderExperiments(); renderChecklist();
  await loadParams(key);
  renderExperiments();          // the paradigm tabs need the parameters
  renderControls();
}
function renderChecklist() {
  const e = exp(S.selected); if (!e) return;
  const sp = Object.entries(e.smartpad).map(([k, v]) => `<li>${esc(k)}: <code>${esc(v)}</code></li>`).join("");
  $("#checklist-sum").textContent = `SmartPad: ${e.smartpad["Cartesian stiffness (K diagonal)"]} · damping 0.7 · elbow 30 · ${e.launch_file}`;
  $("#checklist").innerHTML = `
    <li>On the SmartPad start the app and choose:<ul>${sp}</ul></li>
    <li>The app waits ~60 s for ROS — press <strong>Start</strong> here within that window.</li>
    <li>Visual cue over Wi-Fi? Join this computer to <code>KUKA_NEOPIXEL</code> first.</li>
    <li>Runs <code>ros2 launch sinthlab_bringup ${esc(e.launch_file)}</code>; data goes to <code>analysis/expt_${esc(e.run_name)}_&lt;time&gt;/</code>.</li>`;
}

// ---------------------------------------------------------------- controls + status
function renderControls() {
  const a = S.app; if (!a) return;
  const r = a.runner, s = statusLive();
  const mine = r.state === "running", stopping = r.state === "stopping";
  const external = a.external;
  const rk = runningKey();

  $("#btn-start").disabled = mine || stopping || external;
  $("#btn-start").textContent = "▶ Start";
  $("#btn-start").title = rk && rk !== S.selected ? `Stop ${expLabel(rk)} first` : `Start ${expLabel(S.selected)}`;
  $("#btn-start").classList.toggle("hidden", mine || stopping);
  $("#btn-stop").classList.toggle("hidden", !(mine || stopping));
  const showStatus = !!(s || r.state !== "idle");
  if (showStatus === $("#status-card").classList.contains("hidden")) {
    $("#status-card").classList.toggle("hidden", !showStatus);
    liveToggled(false);
  }
  $("#checklist-card").classList.toggle("hidden", !!(s || r.state !== "idle"));
  $("#btn-stop").disabled = !mine;
  $("#btn-stop").textContent = a.stopping_now ? "■ Parking… (press to skip)" : "■ Stop";
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
  else if (s && s.held) { pl.className = "pill pause"; pl.textContent = `Stopped at park · ${expLabel(s.experiment)}`; }
  else if (s) { pl.className = "pill run"; pl.textContent = `${external ? "External · " : ""}${expLabel(s.experiment)} · trial ${s.trial}`; }
  else if (mine) { pl.className = "pill warn"; pl.textContent = `Starting ${exp(r.experiment)?.label ?? ""}…`; }
  else { pl.className = "pill"; pl.textContent = r.exit_code != null ? `Idle (last exit ${r.exit_code})` : "Idle"; }

  // banner
  const bn = $("#banner");
  let msg = "", cls = "banner";
  if (a.ros.demo) { msg = "Demo mode — a simulated experiment. Nothing is launched and no robot is connected."; cls += " demo"; }
  else if (!a.ros.available) msg = `ROS is not available: ${a.ros.reason} Start and Stop still work; live changes and robot state need ROS.`;
  if (external) msg = `${exp(s.experiment)?.label ?? s.experiment} is running from a terminal (${s.node}). Live changes work from here; stop it with Ctrl-C in its terminal.`;
  if (s && s.phase === "safety_trip") { msg = "SAFETY ABORT — the runaway monitor tripped. The arm is moving to the park pose."; }
  if (s && s.phase === "early_touch") { msg = "Early touch: the apple was moved before the go cue. No reward; the arm parks and a penalty wait runs before the next trial."; }
  bn.className = msg ? cls : "banner hidden"; bn.textContent = msg;

  // run info
  const rows = [];
  if (r.experiment) rows.push(["Experiment", expLabel(r.experiment)]);
  if (r.started_at && r.state !== "idle") rows.push(["Elapsed", fmtDur(Date.now() / 1000 - r.started_at)]);
  if (r.params_file) rows.push(["Params", r.params_file.split("/").slice(-2).join("/")]);
  if (a.data_folder) rows.push(["Data", a.data_folder.split("/").slice(-2).join("/")]);
  if (r.log_path) rows.push(["Log file", r.log_path.split("/").slice(-2).join("/")]);
  if (r.state === "idle" && r.exit_code != null) rows.push(["Exit code", r.exit_code]);
  $("#run-info").innerHTML = rows.map(([k, v]) => `<dt>${esc(k)}</dt><dd>${esc(v)}</dd>`).join("");
  $("#run-card").classList.toggle("hidden", !rows.length);
  renderRewardBar();
  $("#log-path").textContent = r.log_path ? `Full log: ${r.log_path}` : "";

  renderStatus();
}
function fmtDur(sec) {
  sec = Math.max(0, Math.floor(sec));
  const h = Math.floor(sec / 3600), m = Math.floor(sec / 60) % 60, s = sec % 60;
  return h ? `${h}:${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}` : `${m}:${String(s).padStart(2, "0")}`;
}
// The run bar while a run goes: how long, and the juice given so far (from the run monitor's trials).
function renderRunStats() {
  const a = S.app, el = $("#run-stats"); if (!a || !el) return;
  const r = a.runner, on = r.state !== "idle" && r.started_at;
  if (!on) { el.innerHTML = ""; return; }
  const j = SV.juice;
  el.innerHTML = `<span class="k">⏱</span>${fmtDur(Date.now() / 1000 - r.started_at)}` +
    (j && j.ml != null ? ` · <span class="k">juice</span>${j.unsure ? "≈ " : ""}${fmtMl(j.ml)}${j.rate ? ` <span class="k">${fmtMl(j.rate)}/h</span>` : ""}` : "");
}
function renderStatus() {
  renderRunStats();
  const s = statusLive();
  $("#st-trial").textContent = s ? s.trial || "–" : "–";
  const phaseName = s ? (s.held ? "stopped" : s.phase) : null;
  const text = s ? ((PHASE_TEXT_BY_EXP[s.experiment] || {})[phaseName] ?? PHASE_TEXT[phaseName] ?? phaseName) : "not running";
  $("#st-phase").textContent = text;
  $("#st-phase-t").textContent = s ? `${Math.max(0, (Date.now() / 1000 - s.phase_t)).toFixed(0)} s` : "–";

  const key = s ? s.experiment : S.selected;
  const steps = stepsFor(key);
  let cur = s ? s.phase : null;
  if (cur === "timeout" || cur === "safety_trip" || (cur === "response_timeout" && key === "maze")) cur = "goal";
  if (cur === "reach_wrong") cur = "reach";
  if (cur === "choice_unrewarded") cur = "choice";
  if (cur === "early_touch" || cur === "response_timeout") cur = "trial_end";
  if (cur === "cue_snap" || cur === "freeze") cur = "trial_end";   // they follow the snap's trial_end
  if (cur === "quiet_end" && key === "restricted_plane") cur = "cue_go";
  let idx = steps.indexOf(cur);
  if (idx < 0 && s) {   // a cue/ack token: keep the last real step highlighted
    const last = [...S.events].reverse().find((e) => steps.includes(e.phase));
    idx = last ? steps.indexOf(last.phase) : -1;
  }
  $("#steps").innerHTML = steps.map((st, i) =>
    `<li class="${!s ? "" : i < idx ? "done" : i === idx ? "now" : ""}">${esc(STEP_LABEL[st] || st)}</li>`).join("");

  renderSmartpadReport(s);

  const pend = s && s.pending ? Object.entries(s.pending) : [];
  const pe = $("#pending");
  if (pend.length) {
    pe.classList.remove("hidden");
    pe.innerHTML = `<strong>Applies from trial ${s.trial + 1}:</strong> ` +
      pend.map(([k, v]) => `<code>${esc(k)}</code> = ${esc(JSON.stringify(v))}`).join(", ");
  } else pe.classList.add("hidden");
}
// What the SmartPad app reports it is running with (helpers/smartpad_link.py), against the checklist.
function renderSmartpadReport(s) {
  const box = $("#smartpad-report"), e = exp(s ? s.experiment : S.selected);
  const r = s && s.smartpad;
  if (!r) { box.className = "sp-report"; box.innerHTML = "SmartPad report: shown once the experiment is running."; return; }
  if (!r.received) {
    box.className = "sp-report warn";
    box.innerHTML = `<strong>No report from the SmartPad app</strong> — ${esc(r.reason)}. Trials record the settings as unavailable.`;
    return;
  }
  const g = r.settings, want = e ? e.smartpad : {};
  const num = (v) => parseFloat(String(v));
  const diffs = [];
  if (want["Cartesian stiffness (K diagonal)"] && g.stiffness_profile !== want["Cartesian stiffness (K diagonal)"])
    diffs.push(`stiffness “${g.stiffness_profile}”, checklist says “${want["Cartesian stiffness (K diagonal)"]}”`);
  if (want["Damping ratio (D0)"] && Math.abs(num(g.damping) - num(want["Damping ratio (D0)"])) > 1e-6)
    diffs.push(`damping ${g.damping}, checklist says ${num(want["Damping ratio (D0)"])}`);
  if (want["Null-space (elbow) stiffness"] && Math.abs(num(g.nullspace_stiffness) - num(want["Null-space (elbow) stiffness"])) > 1e-6)
    diffs.push(`elbow ${g.nullspace_stiffness}, checklist says ${num(want["Null-space (elbow) stiffness"])}`);
  box.className = `sp-report ${diffs.length ? "warn" : "ok"}`;
  box.innerHTML = `<strong>SmartPad reports:</strong> ${esc(g.stiffness_profile)} <code>[${(g.stiffness || []).join(", ")}]</code>, ` +
    `damping ${esc(g.damping)}, elbow ${esc(g.nullspace_stiffness)}` +
    (diffs.length ? `<br><strong>Differs from the checklist:</strong> ${diffs.map(esc).join("; ")}.` : " — matches the checklist.");
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
  const layered = data.params.some((p) => p.layer !== "other");      // has a Task view
  const profiled = data.params.some((p) => p.layer === "profile");   // the monkey's profile applies (not Navin's tasks)
  const view = !layered ? "all" : S.pview === "monkey" && !profiled ? "task" : S.pview;
  const sig = JSON.stringify([S.selected, S.tab, view, S.profSig, mode, liveVals, pending, S.paramFilter, reportedStiffness(), data.params.map((p) => [p.value, p.edited])]);
  if (!force && sig === S.renderSig) return;
  if (!force && $("#params").contains(document.activeElement) && document.activeElement.tagName !== "BUTTON") {
    S.renderDeferred = true; return;           // do not yank an input out from under the cursor
  }
  S.renderSig = sig; S.renderDeferred = false;

  // Hide the paradigm itself (the card's tabs choose it) and parameters of another paradigm -- while a
  // live paradigm switch waits, those of the paradigm coming next, so they can be set before it starts.
  const para = paradigmOf(S.selected, true);
  const shown = data.params.filter((p) => !p.hidden && (!p.paradigm_only || p.paradigm_only.includes(para)));
  const counts = { live: 0, run: 0, task: 0, profile: 0 };
  const isReward = (p) => layered && REWARD_ROWS.includes(p.name);   // the pulses (and gap) on every Task view
  shown.forEach((p) => { counts[p.tier]++; if (p.layer !== "other") counts[p.layer]++; if (isReward(p)) counts.task++; });
  $("#n-live").textContent = counts.live; $("#n-run").textContent = counts.run;
  $("#n-task").textContent = layered ? counts.task : ""; $("#n-monkey").textContent = profiled ? counts.profile : "";
  $("#n-fixed").textContent = data.fixed.reduce((n, sec) => n + sec.rows.length, 0);
  document.querySelectorAll(".tab[data-pview]").forEach((t) => {
    const on = t.dataset.pview === view;
    t.classList.toggle("active", on); t.setAttribute("aria-selected", String(on));
    t.disabled = (!layered && t.dataset.pview !== "all") || (!profiled && t.dataset.pview === "monkey");
    t.title = t.disabled ? (t.dataset.pview === "monkey" ? "The monkey's profile applies to the free-move tasks only: this one keeps its own settings"
                                                          : "See All settings") : "";
  });
  $("#all-tabs").classList.toggle("hidden", view !== "all");
  document.querySelectorAll(".tab[data-tab]").forEach((t) => t.classList.toggle("active", t.dataset.tab === S.tab));
  $("#tab-help").textContent = TAB_HELP[view === "all" ? S.tab : view][mode];
  $("#tab-help").classList.toggle("hidden", view === "task");
  $("#btn-reset-edits").disabled = mode === "running" || !data.params.some((p) => p.edited);
  renderProfileBar(view, layered);

  const f = S.paramFilter.toLowerCase();
  const inView = (p) => view === "all" ? p.tier === S.tab : view === "task" ? p.layer === "task" || isReward(p) : p.layer === "profile";
  const rows = shown.filter((p) => inView(p) && (!f || p.name.toLowerCase().includes(f) || (p.help || "").toLowerCase().includes(f)));
  const root = $("#params"); root.innerHTML = ""; TIPS.clear(); hideTip();
  S.curView = view; root.classList.toggle("compact", view !== "all");
  renderRewardBar();
  const groups = data.groups || {};
  if (view === "monkey") {
    // Grouped by the profile's sections (feel, reach, pacing, reward), in the profile's order.
    const sections = (S.prof && S.prof.layout.sections) || [];
    const order = new Map(sections.flatMap((sec) => sec.fields.map((fl, i) => [fl.name, i])));
    for (const sec of sections) {
      const ps = rows.filter((p) => (p.profile_field || "").startsWith(sec.key + "."))
        .sort((a, b) => (order.get(a.profile_field) ?? 99) - (order.get(b.profile_field) ?? 99));
      if (!ps.length) continue;
      const box = document.createElement("div"); box.className = "group";
      box.innerHTML = `<h3 class="section" title="${esc(sec.help)}">${esc(sec.label)}</h3>`;
      for (const p of ps) box.appendChild(paramRow(p, mode, liveVals, pending, groups));
      root.appendChild(box);
    }
  } else {
    if (view === "task") {          // the valve time is in the run bar; the pulses (and their gap) here
      const ps = REWARD_ROWS.slice(1).map((n) => rows.find((p) => p.name === n)).filter(Boolean);
      const cnt = ps.find((p) => p.name === "reward_pulse.count");
      const count = cnt ? Number(liveVals && cnt.name in liveVals ? liveVals[cnt.name] : cnt.value) : 1;
      if (ps.length) {
        const box = document.createElement("div"); box.className = "group";
        box.innerHTML = `<h3 class="section">Reward</h3>`;
        for (const p of ps) if (p.name !== "reward_pulse.gap_ms" || count > 1) box.appendChild(paramRow(p, mode, liveVals, pending, groups));
        root.appendChild(box);
      }
    }
    const byGroup = new Map();
    // The Task view groups by plain heading, so blocks that share one (two "Pull" blocks) show as one.
    const gkey = (p) => (view === "task" ? GROUP_LABEL[p.group] || p.group : p.group);
    for (const p of rows.filter((x) => !(view === "task" && isReward(x)))) { const k = gkey(p); if (!byGroup.has(k)) byGroup.set(k, []); byGroup.get(k).push(p); }
    for (const [g, ps] of byGroup) {
      const box = document.createElement("div"); box.className = "group";
      const gt = groupTip(g, groups[g]);
      const gdesc = groups[g] && groups[g].help;
      box.innerHTML = view === "task" ? `<h3 class="section">${esc(g)}</h3>`
        : `<h3${gt ? ` data-tip="${tipId(gt)}" tabindex="0"` : ""}>${esc(g)}${gdesc ? `<span class="gdesc"> — ${esc(gdesc)}</span>` : ""}</h3>`;
      for (const p of ps) box.appendChild(paramRow(p, mode, liveVals, pending, groups));
      root.appendChild(box);
    }
  }
  if (view === "all" && S.tab === "fixed") {
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
  if (!root.children.length) root.innerHTML = `<p class="muted">${view === "task" && !f ? "This task has no settings of its own beyond its tabs above." : `No settings${f ? " match the filter" : ""}.`}</p>`;
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
  const who = curSubject();
  const offProfile = p.layer === "profile" && hasProfile() && !p.edited && fmtVal(value) !== fmtVal(p.default);
  const compact = S.curView !== "all";     // Task and Monkey views: plain names, no descriptions (the tooltip has them)
  const friendly = compact && p.profile_field;
  const label = friendly ? profileLabel(p.profile_field) : compact && p.label ? p.label : short;
  row.innerHTML = `<div class="name" tabindex="0" data-tip="${tipId(paramTip(p, groups))}">${esc(label)}${p.edited ? `<span class="badge edited" title="${p.layer === "profile" && hasProfile() ? `Today only: ${esc(who)}'s profile has ${esc(fmtVal(p.default))}` : "Edited for the next Start"}">${p.layer === "profile" && hasProfile() ? "today only" : "edited"}</span>` : ""}${offProfile ? `<span class="badge profile" title="${esc(who)}'s profile has ${esc(fmtVal(p.default))}: Save to profile keeps this value">not in profile</span>` : ""}${p.immediate ? '<span class="badge now" title="Applied the moment you change it, from the next move on — a move already under way finishes at its speed">applies now</span>' : ""}${pend ? `<span class="badge pending">next trial → ${esc(fmtVal(pending[p.name]))}</span>` : ""}</div>`;
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
    // Task and Monkey views: a rail's -Y / down end is shown and typed as a distance (0.15, not -0.15).
    if (compact && p.name === "free_move.rail_min_m") {
      const raw = commit;
      p = { ...p, limits: p.limits ? [-p.limits[1], -p.limits[0]] : null, step: null };
      buildControl(ctl, p, -Number(value), (v) => raw(-Number(v)), locked);
      if (!locked && !isLiveRunning && p.edited) {
        const rv = document.createElement("button"); rv.className = "revert"; rv.textContent = "↺";
        rv.title = `Back to ${hasProfile() ? `${curSubject()}'s profile` : "the default"} (${fmtVal(-Number(p.default))})`;
        rv.onclick = () => raw(p.default); ctl.appendChild(rv);
      }
      if (p.limits) { const d = document.createElement("span"); d.className = "default"; d.textContent = `${p.limits[0]}–${p.limits[1]}`; ctl.appendChild(d); }
      row.appendChild(ctl);
      if (p.profile_note) { const h = document.createElement("div"); h.className = "help pnote"; h.textContent = `ⓘ ${p.profile_note}`; row.appendChild(h); }
      return row;
    }
    // The felt stiffness can go up to the stiffness the SmartPad app reports running, not the YAML base.
    const rep = isLiveRunning && p.name === "resistance.stiffness_n_per_m" && p.limits ? reportedStiffness() : null;
    if (rep) p = { ...p, limits: [p.limits[0], Math.min(rep, 5000)] };
    buildControl(ctl, p, value, commit, locked);
    if (!locked && !isLiveRunning && p.edited) {
      const rv = document.createElement("button"); rv.className = "revert";
      rv.title = p.layer === "profile" && hasProfile() ? `Back to ${curSubject()}'s profile (${fmtVal(p.default)})` : `Back to the default (${fmtVal(p.default)})`;
      rv.textContent = "↺"; rv.onclick = () => commit(p.default); ctl.appendChild(rv);
    }
    if (p.name === "reward_pulse.pulse_ms" && !locked) {        // roll the reward back (or up) in small steps
      for (const dv of [-REWARD_STEP_MS, REWARD_STEP_MS]) {
        const sb = document.createElement("button"); sb.className = "btn small step";
        sb.textContent = dv < 0 ? `−${REWARD_STEP_MS}` : `+${REWARD_STEP_MS}`;
        sb.title = `${dv < 0 ? "Less" : "More"} juice: ${dv < 0 ? "−" : "+"}${REWARD_STEP_MS} ms valve time${isLiveRunning ? ", from the next trial" : ", for the next Start"}`;
        sb.onclick = () => commit(Math.max(p.limits ? p.limits[0] : 10, Number(value) + dv));
        ctl.appendChild(sb);
      }
    }
    if (p.limits) { const d = document.createElement("span"); d.className = "default"; d.textContent = `${p.limits[0]}–${p.limits[1]}`; ctl.appendChild(d); }
  }
  row.appendChild(ctl);
  if (p.help && !compact) { const h = document.createElement("div"); h.className = "help"; h.textContent = p.help; row.appendChild(h); }
  if (p.profile_note) { const h = document.createElement("div"); h.className = "help pnote"; h.textContent = `ⓘ ${p.profile_note}`; row.appendChild(h); }
  if (p.caution && !compact) { const h = document.createElement("div"); h.className = "help caution"; h.textContent = `⚠ ${p.caution}`; row.appendChild(h); }
  if (p.linked && p.linked.length) {
    const h = document.createElement("div"); h.className = "help linked";
    h.textContent = `🔗 Editing this also sets: ${p.linked.join(", ")}`; row.appendChild(h);
  }
  if (p.name === "move_to_start.target_joint_position" && !locked && !isLiveRunning) row.appendChild(startHeightRow());
  return row;
}

// The run bar's reward: the valve time (and pulses) in effect, mL per reward, -10 / +10 ms, and a popover to set
// it by volume or save it to the monkey's profile. Live while the experiment runs (from the next trial).
function renderRewardBar() {
  const box = $("#reward-bar"), d = S.params[S.selected];
  const par = (n) => d && d.params.find((x) => x.name === n);
  if (!par("reward_pulse.pulse_ms")) { box.className = "rb-reward none"; box.innerHTML = ""; return; }
  if (box.contains(document.activeElement) && document.activeElement.tagName === "INPUT") return;   // typing in the popover
  box.className = "rb-reward";
  const running = isRunningSelected(), s = statusLive();
  const val = (n) => { const q = par(n); if (!q) return undefined; return running && s && s.live && n in s.live ? s.live[n] : q.value; };
  const next = (n) => (running && s && s.pending && n in s.pending ? s.pending[n] : undefined);
  const ms = Number(val("reward_pulse.pulse_ms")), n = Number(val("reward_pulse.count") ?? 1), gap = val("reward_pulse.gap_ms");
  const nx = next("reward_pulse.pulse_ms"), vol = rewardVolume(ms, n, gap);
  const fmt = (m, k) => (k > 1 ? `${k} × ${m} ms` : `${m} ms`);
  const prof = (S.prof && S.prof.profile) || null, who = curSubject(), layered = d.params.some((x) => x.layer === "profile");
  const own = prof && (prof.reward_by_task || {})[S.selected];
  const pv = prof && layered ? { ms: (own && own["reward.pulse_ms"]) ?? prof.values["reward.pulse_ms"], n: (own && own["reward.count"]) ?? prof.values["reward.count"] } : null;
  const off = pv && (Number(pv.ms) !== Number(nx ?? ms) || Number(pv.n) !== n);
  const volTxt = vol.ml == null ? `<span class="muted small" title="${esc(vol.note)}">no mL</span>`
    : `<span class="ml ${vol.status}" title="${esc(`${vol.status}${vol.note ? `: ${vol.note}` : ""}${S.cal ? ` · calibration ${S.cal.set}` : ""}`)}">≈ ${fmtMl(vol.ml)}${vol.status === "extrapolated" || vol.note ? " ⚠" : ""}</span>`;
  box.innerHTML = `<div class="rb-lab">Reward${running ? " · live" : ""}</div>
    <div class="rw"><span class="big">${esc(fmt(ms, n))}</span>${volTxt}${nx !== undefined ? `<span class="nxt">→ ${esc(nx)} ms next trial</span>` : ""}
      <button class="btn small step" data-d="-${REWARD_STEP_MS}" title="Less juice: −${REWARD_STEP_MS} ms${running ? ", from the next trial" : ", for the next Start"}">−${REWARD_STEP_MS}</button>
      <button class="btn small step" data-d="${REWARD_STEP_MS}" title="More juice: +${REWARD_STEP_MS} ms${running ? ", from the next trial" : ", for the next Start"}">+${REWARD_STEP_MS}</button>
      <button class="btn small" id="rw-more" title="Set by volume; save to the profile">mL…</button></div>
    <div class="rb-sub">${pv ? `${esc(who)}'s profile${own ? ` (this task)` : ""}: ${esc(fmt(pv.ms, pv.n))}${off ? " · differs" : ""}` : layered && who && who !== "test" ? "no profile" : "&nbsp;"}</div>` +
    (S.rewardPop ? `<div class="pop">
      ${S.cal && S.cal.profiles.length ? `<div class="row"><label>Set by volume <input type="number" id="rw-ml" step="0.05" min="0.3" max="5" value="${vol.ml != null ? vol.ml.toFixed(2) : ""}"> mL per reward</label><button class="btn small" id="rw-set">Set</button></div>
        ${vol.note ? `<div class="row warn-text">⚠ ${esc(vol.note)}</div>` : ""}<div class="row muted">Calibration ${esc(S.cal.set)} (±10 %)</div>` : `<div class="row muted">No reward calibration found.</div>`}
      ${off ? `<div class="row"><button class="btn small" id="rw-save">Save to ${esc(who)}'s profile…</button></div>` : ""}
      <div class="row muted">Pulses and the gap: the Task tab.</div></div>` : "");
  box.querySelectorAll(".step").forEach((b) => b.onclick = () => setParam("reward_pulse.pulse_ms", Math.max(10, Number(nx ?? ms) + Number(b.dataset.d))));
  $("#rw-more").onclick = () => { S.rewardPop = !S.rewardPop; renderRewardBar(); if (S.rewardPop) $("#rw-ml")?.focus(); };
  const set = $("#rw-set");
  if (set) set.onclick = async () => {
    let w; try { w = await api(`/api/reward/width?ml=${encodeURIComponent($("#rw-ml").value)}&count=${n}`); } catch (e) { toast(e.message, "err"); return; }
    const r = await setParam("reward_pulse.pulse_ms", w.ms);
    if (r) { toast(`${fmtMl(w.ml)} per reward = ${n > 1 ? `${n} × ` : ""}${w.ms} ms${w.status === "extrapolated" ? " (extrapolated)" : ""}`, "ok"); S.rewardPop = false; renderRewardBar(); }
  };
  const sv = $("#rw-save");
  if (sv) sv.onclick = () => { S.rewardPop = false; renderRewardBar(); openSaveDialog(true); };
}

// The start pose from a height above the ground: the floor's reference point (apple bottom or base edge)
// is put there by moving the end effector straight up or down from the current start (kinematics.py).
function startHeightRow() {
  const sh = (S.params[S.selected] || {}).start_height || {};
  const box = document.createElement("div"); box.className = "help start-height";
  if (!sh.available) { box.textContent = `Start height: ${sh.reason || "unavailable"}`; return box; }
  const ref = sh.reference.replace("_", " ");
  box.innerHTML = `Set by height: the ${esc(ref)} at <input type="number" step="0.005" min="${sh.floor_m}" ` +
    `value="${sh.height_m}"> m above the ground <button>Solve</button> ` +
    `<span class="default">now ${sh.height_m.toFixed(3)} m · floor ${Number(sh.floor_m).toFixed(3)} m</span>`;
  const inp = box.querySelector("input");
  box.querySelector("button").onclick = async () => {
    const r = await act(() => api(`/api/start_height/${S.selected}`, { height_m: Number(inp.value) }), "Start pose solved");
    if (r) await loadParams(S.selected, true);
  };
  return box;
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
  const numeric = p.type === "int" || p.type === "float";
  // A parameter with a step (live_params.STEPS) gets a slider next to its text box. The slider moves in
  // steps and commits when released; the text box takes any value within the limits.
  if (numeric && p.step && p.limits && !Array.isArray(value)) {
    const sl = document.createElement("input");
    sl.type = "range"; sl.className = "slider"; sl.disabled = disabled; sl.setAttribute("aria-label", `${p.name} slider`);
    sl.min = p.limits[0]; sl.max = p.limits[1]; sl.step = p.step; sl.value = value;
    const box = document.createElement("input");
    box.type = "number"; box.className = "slider-box"; box.disabled = disabled; box.setAttribute("aria-label", p.name);
    box.min = p.limits[0]; box.max = p.limits[1]; box.step = "any"; box.value = String(value);
    sl.oninput = () => { box.value = sl.value; };          // follow while dragging
    sl.onchange = () => commit(sl.value);                   // commit on release
    box.onkeydown = (e) => { if (e.key === "Escape") { box.value = String(value); box.blur(); } };
    box.onchange = () => { sl.value = box.value; commit(box.value); };
    box.onblur = () => { if (S.renderDeferred) setTimeout(() => renderParams(), 0); };
    ctl.append(sl, box);
    return;
  }
  const inp = document.createElement("input");
  inp.type = numeric ? "number" : "text"; inp.disabled = disabled; inp.setAttribute("aria-label", p.name);
  if (numeric) { inp.step = p.type === "int" ? "1" : "any"; if (p.limits) { inp.min = p.limits[0]; inp.max = p.limits[1]; } }
  inp.value = Array.isArray(value) ? `[${value.join(", ")}]` : String(value);
  inp.onkeydown = (e) => { if (e.key === "Escape") { inp.value = Array.isArray(value) ? `[${value.join(", ")}]` : String(value); inp.blur(); } };
  inp.onchange = () => commit(inp.value);
  inp.onblur = () => { if (S.renderDeferred) setTimeout(() => renderParams(), 0); };
  ctl.appendChild(inp);
}

// ---------------------------------------------------------------- the monkey and its profile
// One monkey at a time (the server holds it). Its profile sets the feel, reach, pacing and reward of every
// free-move task; a task's own few settings are its Task view; Save to profile keeps what was tuned today.
function curSubject() { return (S.app && S.app.subject) || ""; }
function hasProfile() { return !!(S.app && S.app.profile); }
function profileLabel(field) {
  for (const sec of (S.prof && S.prof.layout.sections) || []) {
    const f = sec.fields.find((x) => x.name === field);
    if (f) return f.label;
  }
  return field;
}
async function loadProfile() {
  try { S.prof = await api("/api/profile"); } catch (e) { toast(`Could not load the profile: ${e.message}`, "err"); }
  renderMonkey();
}
function renderMonkey() {
  const sel = $("#monkey"), pr = S.prof; if (!pr) return;
  const cur = curSubject();
  const names = new Set(pr.names);
  const opts = [`<option value="">Choose the monkey…</option>`];
  if (pr.names.length) opts.push(`<optgroup label="Profiles">${pr.names.map((n) => `<option value="${esc(n)}">${esc(n)}</option>`).join("")}</optgroup>`);
  const others = pr.recent.filter((n) => !names.has(n) && n !== "test");
  if (others.length) opts.push(`<optgroup label="No profile yet">${others.map((n) => `<option value="${esc(n)}">${esc(n)}</option>`).join("")}</optgroup>`);
  opts.push(`<option value="test">test (no animal)</option>`, `<option value="__new">New monkey…</option>`);
  if (cur && !names.has(cur) && !others.includes(cur) && cur !== "test") opts.splice(1, 0, `<option value="${esc(cur)}">${esc(cur)}</option>`);
  sel.innerHTML = opts.join(""); sel.value = cur;
  sel.disabled = !!(S.app && S.app.runner.state !== "idle");
  sel.title = sel.disabled ? "Fixed while an experiment runs" : "";
  const info = $("#monkey-info");
  const p = pr.profile;
  info.title = "";
  if (!cur) info.textContent = "choose the monkey first";
  else if (cur === "test") info.textContent = "no animal: task defaults";
  else if (p) { info.textContent = `profile v${p.version} · ${p.updated.slice(0, 16)}`;
    info.title = "Applied to the pre-training tasks, the Colour Rule and N-Choice; Navin's experiments keep their own settings."; }
  else {
    info.innerHTML = `no profile <button class="btn small" id="btn-profile-create" title="Create ${esc(cur)}'s profile from ${esc(exp(S.selected)?.label || "this task")}'s settings">Create</button>`;
    $("#btn-profile-create").onclick = () => createProfileFromTask();
  }
  $("#btn-profile").disabled = !cur || cur === "test";
}
// Changing the monkey with edits made today: say which, and keep or clear them (or don't change).
function editsSummary() {
  const names = (S.app && S.app.edited_names) || {};
  return Object.entries(names).map(([k, ns]) => `${exp(k)?.label || k}: ${ns.join(", ")}`);
}
function askAboutEdits(subject) {
  const lines = editsSummary();
  if (!lines.length) return Promise.resolve(false);
  return new Promise((resolve) => {
    const dlg = $("#switch-dlg");
    $("#switch-dlg-subject").textContent = subject;
    $("#switch-dlg-list").innerHTML = lines.map((l) => `<li>${esc(l)}</li>`).join("");
    dlg.onclose = () => resolve(dlg.returnValue === "clear" ? true : dlg.returnValue === "keep" ? false : null);
    dlg.returnValue = ""; dlg.showModal();
  });
}
async function selectSubject(subject) {
  if (subject === "__new") {
    const id = (prompt("New monkey's ID (letters, digits, _ and -):") || "").trim();
    if (!id) { renderMonkey(); return; }
    const clear = await askAboutEdits(id);
    if (clear === null) { renderMonkey(); return; }
    const r = await act(() => api("/api/subject", { subject: id, clear_edits: clear }));
    if (r && !r.profile) await act(() => api("/api/profile/save", { values: {}, note: "created" }), `${id}'s profile created from the defaults`);
    store.set("subject", id);
    return;
  }
  if (subject && subject !== curSubject()) {
    const clear = await askAboutEdits(subject);
    if (clear === null) { renderMonkey(); return false; }           // not changed
    const r = await act(() => api("/api/subject", { subject, clear_edits: clear }), `Monkey: ${subject}${clear ? " (today's edits cleared)" : ""}`);
    if (r) { store.set("subject", subject); SV.histSubject = null; } else renderMonkey();
    return !!r;
  }
  const r = await act(() => api("/api/subject", { subject }), subject ? `Monkey: ${subject}` : "");
  if (r) store.set("subject", subject); else renderMonkey();
}
async function createProfileFromTask() {
  const e = exp(S.selected);
  if (!confirm(`Create ${curSubject()}'s profile from ${e.label}'s settings as they are now (feel, reach, pacing, reward)?`)) return;
  await act(() => api("/api/profile/save", { exp: S.selected, from_task: true, note: `created from ${e.label}` }), `${curSubject()}'s profile created`);
}
// The bar above the Monkey view: whose profile, and Save to profile.
function renderProfileBar(view, layered) {
  const bar = $("#profile-bar");
  if (view !== "monkey" || !layered) { bar.classList.add("hidden"); return; }
  bar.classList.remove("hidden");
  const cur = curSubject(), e = exp(S.selected);
  if (!cur) { bar.innerHTML = `<span class="grow">No monkey chosen: these are ${esc(e.label)}'s defaults. Choose the monkey at the top left.</span>`; return; }
  if (cur === "test") { bar.innerHTML = `<span class="grow">test: ${esc(e.label)}'s defaults, no profile.</span>`; return; }
  if (!hasProfile()) {
    bar.innerHTML = `<span class="grow">${esc(cur)} has no profile yet: ${esc(e.label)}'s defaults.</span><button class="btn small" id="pb-create">Create ${esc(cur)}'s profile from these</button>`;
    $("#pb-create").onclick = () => createProfileFromTask();
    return;
  }
  bar.innerHTML = `<span class="grow"><strong>${esc(cur)}</strong>'s profile v${S.app.profile.version}, as ${esc(e.label)} uses it.</span>` +
    `<button class="btn small" id="pb-save" title="Keep the values tuned here (live or for the next Start) in ${esc(cur)}'s profile">Save to ${esc(cur)}'s profile…</button>`;
  $("#pb-save").onclick = () => openSaveDialog();
}
async function openSaveDialog(rewardOnly = false) {
  const key = runningKey() === S.selected ? runningKey() : S.selected, e = exp(key);
  let d;
  try { d = await api(`/api/profile/diff?exp=${encodeURIComponent(key)}`); } catch (err) { toast(err.message, "err"); return; }
  if (rewardOnly) d.rows = d.rows.filter((r) => r.reward);
  if (!d.rows.length) { toast(`Nothing to save: ${e.label} matches ${d.subject}'s profile`, "ok"); return; }
  $("#save-dlg-subject").textContent = d.subject;
  $("#save-dlg-sub").textContent = `From ${e.label}${runningKey() === key ? " (live values)" : ""}: these differ from the profile. The ticked ones are saved, and every task uses them from its next Start.`;
  $("#save-rows").innerHTML = `<tr><th></th><th>Setting</th><th>Profile</th><th>Now</th><th>For</th></tr>` + d.rows.map((r, i) =>
    `<tr><td><input type="checkbox" id="sv${i}" checked aria-label="Save ${esc(r.label)}"></td><td><label for="sv${i}">${esc(r.label)}</label></td>` +
    `<td class="v">${esc(fmtVal(r.profile))}</td><td class="v">${esc(fmtVal(r.task))}</td><td>` +
    (r.reward ? `<select id="svs${i}" aria-label="Scope"><option value="all"${r.task_only_now ? "" : " selected"}>every task</option><option value="task"${r.task_only_now ? " selected" : ""}>${esc(e.label)} only</option></select>` : "every task") +
    `</td></tr>`).join("");
  $("#save-note").value = "";
  $("#save-form").onsubmit = async (ev) => {
    ev.preventDefault();
    const values = {}, taskOnly = {};
    d.rows.forEach((r, i) => {
      if (!$(`#sv${i}`).checked) return;
      if (r.reward && $(`#svs${i}`).value === "task") taskOnly[r.field] = r.task; else values[r.field] = r.task;
    });
    if (!Object.keys(values).length && !Object.keys(taskOnly).length) { $("#save-dlg").close(); return; }
    $("#save-dlg").close();
    await act(() => api("/api/profile/save", { exp: key, values, task_only: taskOnly, note: $("#save-note").value.trim() }),
              `Saved to ${d.subject}'s profile`);
  };
  $("#save-dlg").showModal();
}
async function openProfileDialog() {
  await loadProfile();
  const pr = S.prof, cur = curSubject(); if (!pr || !cur || cur === "test") return;
  const vals = (pr.profile && pr.profile.values) || {};
  $("#profile-dlg-subject").textContent = cur;
  $("#profile-dlg-sub").textContent = pr.profile
    ? `Version ${pr.profile.version}, saved ${pr.profile.updated}. Saving applies it to every free-move task from its next Start (a running one keeps its values until it ends).`
    : "No profile yet: these are the defaults. Saving creates the profile.";
  $("#profile-fields").innerHTML = pr.layout.sections.map((sec) => `<fieldset><legend>${esc(sec.label)}</legend><p class="pf-help">${esc(sec.help)}</p>` +
    sec.fields.map((f) => {
      const v = vals[f.name], id = `pf-${f.name.replace(".", "-")}`;
      const ctl = typeof v === "boolean"
        ? `<input type="checkbox" id="${id}" data-field="${esc(f.name)}" ${v ? "checked" : ""}>`
        : `<input type="number" id="${id}" data-field="${esc(f.name)}" step="any" value="${esc(v ?? "")}"${f.limits.length ? ` min="${f.limits[0]}" max="${f.limits[1]}"` : ""}>`;
      return `<div class="pf"><label for="${id}">${esc(f.label)}</label>${ctl}<div class="pf-sub">${esc(f.help)}${f.limits.length ? ` (${f.limits[0]}–${f.limits[1]})` : ""}</div></div>`;
    }).join("") + `</fieldset>`).join("");
  document.querySelectorAll("#profile-fields [data-field]").forEach((inp) => inp.oninput = inp.onchange = () => {
    const v = vals[inp.dataset.field];
    const now = inp.type === "checkbox" ? inp.checked : Number(inp.value);
    inp.closest(".pf").classList.toggle("changed", inp.type === "checkbox" ? now !== v : Number(v) !== now);
  });
  const byTask = (pr.profile && pr.profile.reward_by_task) || {};
  const bt = Object.entries(byTask);
  $("#profile-by-task").innerHTML = bt.length ? `<div class="by-task"><strong>Rewards for one task only</strong><ul>` + bt.map(([t, d]) =>
    `<li>${esc(exp(t)?.label || t)}: ${Object.entries(d).map(([n, v]) => `${esc(profileLabel(n))} ${esc(fmtVal(v))}`).join(", ")} ` +
    `<button type="button" class="btn small" data-clear="${esc(t)}">Use the profile's</button></li>`).join("") + `</ul></div>` : "";
  document.querySelectorAll("#profile-by-task [data-clear]").forEach((b) => b.onclick = async () => {
    const t = b.dataset.clear;
    await act(() => api("/api/profile/save", { exp: t, task_only: Object.fromEntries(Object.keys(byTask[t]).map((n) => [n, null])),
                                                note: `${exp(t)?.label || t}: back to the profile's reward` }), "Saved");
    openProfileDialog();
  });
  $("#profile-changes").innerHTML = pr.changes.length ? pr.changes.map((c) => {
    const parts = Object.entries(c.changes || {}).map(([n, [a, b]]) => `${profileLabel(n)} ${fmtVal(a)} → ${fmtVal(b)}`);
    for (const [t, ch] of Object.entries(c.reward_by_task || {}))
      parts.push(...Object.entries(ch).map(([n, [a, b]]) => `${exp(t)?.label || t}: ${profileLabel(n)} ${a == null ? "profile's" : fmtVal(a)} → ${b == null ? "profile's" : fmtVal(b)}`));
    return `<li><strong>v${c.version}</strong> ${esc(c.time)}, from ${esc(c.by)}${c.note ? ` (${esc(c.note)})` : ""}: ${esc(parts.join("; ") || "created")}</li>`;
  }).join("") : `<li class="muted">No changes yet.</li>`;
  $("#profile-note").value = "";
  $("#profile-form").onsubmit = async (ev) => {
    ev.preventDefault();
    const values = {};
    document.querySelectorAll("#profile-fields [data-field]").forEach((inp) => {
      const v = vals[inp.dataset.field], now = inp.type === "checkbox" ? inp.checked : Number(inp.value);
      if (inp.type === "checkbox" ? now !== v : Number(v) !== now) values[inp.dataset.field] = now;
    });
    if (!Object.keys(values).length && pr.profile) { $("#profile-dlg").close(); return; }
    const r = await act(() => api("/api/profile/save", { values, note: $("#profile-note").value.trim() }), `${cur}'s profile saved`);
    if (r) $("#profile-dlg").close();
  };
  $("#profile-dlg").showModal();
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
function logStrip(e) {
  const last = $("#log-last"); if (!last) return;
  last.className = `log-last ${e.level}`;
  last.textContent = `${new Date(e.t * 1000).toLocaleTimeString([], { hour12: false })}  ${e.text}`;
  if ((e.level === "warn" || e.level === "error") && $("#log-drawer").classList.contains("hidden")) {
    S.warnCount++;
  }
  const w = $("#log-warns"); w.textContent = `Warnings ${S.warnCount}`; w.classList.toggle("has", S.warnCount > 0);
}
function openLog(open = true, level = null) {
  const d = $("#log-drawer");
  d.classList.toggle("hidden", !open);
  $("#log-toggle").textContent = open ? "Close log ▾" : "Open log ▴";
  if (level) document.querySelector(`#log-levels button[data-level="${level}"]`)?.click();
  if (open) { S.warnCount = 0; $("#log-warns").textContent = "Warnings 0"; $("#log-warns").classList.remove("has"); const b = $("#log"); b.scrollTop = b.scrollHeight; }
}
function appendLog(e) {
  if (e.seq <= S.lastSeq) return;
  S.lastSeq = e.seq;
  S.logs.push(e); if (S.logs.length > 5000) S.logs.shift();
  logStrip(e);
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

// ---------------------------------------------------------------- the live view card
// The same drawing as /live.html (live_view.js), in the trial card (shown while a run goes), open unless folded
// with its heading (remembered). Made when first shown; fed by the event stream's "live" messages (15 Hz),
// or polled with ?nostream.
const LV = { view: null, version: null, poll: null };
async function liveLoad() {
  if (!LV.view) return;
  try {
    const m = await api("/api/live_view");
    LV.view.setGeometry(m.geometry); LV.version = m.geometry ? m.geometry.version : null;
    LV.view.setLive(m.live);
    $("#live-sub").textContent = m.geometry ? "in the experiment's bounds, live" : "no experiment running";
  } catch { /* the next message retries */ }
}
function liveShown() { return $("#live-card").open && !$("#status-card").classList.contains("hidden"); }
function onLive(l) {
  if (!LV.view || !liveShown()) return;
  if ((l.version ?? null) !== LV.version || (l.experiment && !LV.view.geom) || (!l.experiment && LV.view.geom)) liveLoad();
  else LV.view.setLive(l);
}
function liveToggled(remember = true) {
  if (remember) store.set("liveOpen", $("#live-card").open);
  const open = liveShown();
  if (open && !LV.view) LV.view = new LiveView({ front: $("#cv-front"), top: $("#cv-top") }, $("#live-status"));
  if (open) liveLoad();
  clearInterval(LV.poll); LV.poll = null;
  if (open && new URLSearchParams(location.search).has("nostream")) LV.poll = setInterval(liveLoad, 500);
}

// ---------------------------------------------------------------- wiring
function onState(a) {
  const prevKey = runningKey();
  const wasIdle = !S.app || S.app.runner.state === "idle";
  S.app = a;
  noteEvent(statusLive());
  const psig = `${a.subject}|${a.profile ? a.profile.version : ""}`;
  if (psig !== S.profSig || wasIdle !== (a.runner.state === "idle")) {   // the monkey, its profile, or a run starting / ending
    const changed = psig !== S.profSig;
    S.profSig = psig;
    if (changed) { S.params = {}; loadProfile(); loadParams(S.selected, true).then(() => { renderExperiments(); }); refreshSession(true); }
    else { renderMonkey(); if (a.runner.state === "idle") { S.params = {}; loadParams(S.selected, true); } }
  }
  const key = runningKey();
  // When an experiment starts (here or in a terminal), show it: its Live tab is what you need.
  if (key && key !== prevKey && key !== S.selected) { select(key); return; }
  renderControls();
  sessionOnState();
  // The paradigm tabs follow a live switch (the rails): re-render when the running or waiting one changes.
  const para = key ? JSON.stringify(runningParadigm(key)) : null;
  if (key !== prevKey || para !== S.paradigmSig) renderExperiments();
  S.paradigmSig = para;
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
    else if (m.type === "live") onLive(m.data);
  };
  es.onerror = () => { $("#pill-run").className = "pill err"; $("#pill-run").textContent = "Dashboard disconnected"; };
}

// ---------------------------------------------------------------- the session card
// Run monitor (mock-up D): this run's trials.csv as it grows, or the last run between runs.
// Today vs recent (mock-up E): the selected experiment's key settings across the subject's last sessions
// (sessions.csv), today's values beside them, and the last session at a glance.
const OUTCOME_CLASS = {
  reach: "success", choice: "success", snap: "success", goal: "success", session_end: "success",
  choice_unrewarded: "unrewarded", reach_wrong: "wrong",
  response_timeout: "none", timeout: "none", early_touch: "early",
  safety_trip: "abort", smartpad_mismatch: "abort", stop_requested: "abort",
};
const OUTCOME_LABEL = { success: "Correct / done", unrewarded: "Chose, not paid", wrong: "Wrong end",
  none: "No response", early: "Early touch", abort: "Aborted (safety, Stop)" };
const SV = { view: null, userView: false, data: null, hist: null, lastTrial: null, lastFetch: 0, busy: false };

function sessionKey() { return runningKey() || S.selected; }
function pct(n, d) { return d ? `${Math.round(100 * n / d)}%` : "–"; }
const num = (v) => (v === "" || v == null ? NaN : Number(v));

function chooseSessionView() {
  if (!SV.userView) SV.view = runningKey() || sessionKey() === "reach_map" ? "monitor" : "history";
  document.querySelectorAll(".session-card .tab").forEach((t) => {
    const on = t.dataset.view === SV.view;
    t.classList.toggle("active", on); t.setAttribute("aria-selected", String(on));
  });
}

async function refreshSession(force = false) {
  chooseSessionView();
  const key = sessionKey();
  if (!key || SV.busy) return;
  const now = Date.now();
  if (!force && now - SV.lastFetch < 1500) return;
  SV.busy = true; SV.lastFetch = now;
  try {
    const rk = runningKey();
    if (rk && !(SV.view === "monitor" && key !== "reach_map")) SV.data = await api(`/api/run/trials?exp=${encodeURIComponent(rk)}`);
    if (SV.view === "monitor" && key === "reach_map") SV.map = await api("/api/reach_map");
    else if (SV.view === "monitor") SV.data = await api(`/api/run/trials?exp=${encodeURIComponent(key)}`);
    else SV.hist = await api(`/api/history?exp=${encodeURIComponent(key)}&subject=${encodeURIComponent(histSubject())}`);
  } catch (e) { /* shown as empty */ }
  finally { SV.busy = false; }
  SV.juice = runningKey() && SV.data ? juiceOf(SV.data) : null;
  renderRunStats();
  renderSession();
}

// Called on every status update: refetch the trials when a trial ends.
function sessionOnState() {
  const s = statusLive();
  const t = s ? `${s.experiment}:${s.trial}:${["trial_end", "recover_start"].includes(s.phase)}` : null;
  const viewWas = SV.view;
  chooseSessionView();
  if (t !== SV.lastTrial || SV.view !== viewWas) { SV.lastTrial = t; refreshSession(true); }
  // The Reach Map grows within its one long session: redraw it every few seconds while it runs.
  else if (s && s.experiment === "reach_map" && SV.view === "monitor" && Date.now() - SV.lastFetch > 5000) refreshSession(true);
}

function renderSession() {
  if (SV.view === "monitor" && sessionKey() === "reach_map") renderReachMap();
  else if (SV.view === "monitor") renderMonitor(); else renderHistory();
}

// ---------------------------------------------------------------- the Reach Map view
// Where the hand went (frontal plane, as the monkey sees it: right is +Y), each movement's farthest point,
// the comfortable-reach outline (the percentile per direction) and the farthest one, the box (the arm's
// limit), the horizontal tasks' rail height; then the four profile values, now vs this map, to save.
const REACH_DIRS = [["reach.left_m", "Left", ""], ["reach.right_m", "Right", ""],
                    ["reach.down_m", "Down", "on the vertical rail"], ["reach.up_m", "Up", "on the vertical rail"]];
function cm(v) { return v == null ? "–" : `${(v * 100).toFixed(1)} cm`; }
function reachSvg(d) {
  const box = d.box || [-0.2, 0.2, -0.1, 0.1], pad = 0.025;
  const x0 = Math.min(box[0], -0.05) - pad, x1 = Math.max(box[1], 0.05) + pad;
  const y0 = Math.min(box[2], -0.05) - pad, y1 = Math.max(box[3], d.horizontal_dz || 0, 0.05) + pad;
  const W = 600, sc = W / (x1 - x0), H = Math.round((y1 - y0) * sc);
  const X = (a) => ((a - x0) * sc).toFixed(1), Y = (b) => ((y1 - b) * sc).toFixed(1);
  const poly = (key) => {
    const rows = d.rows; if (rows.some((r) => r[key] == null)) {
      return rows.map((r, i) => { const n = rows[(i + 1) % rows.length]; if (r[key] == null || n[key] == null) return "";
        const p = [r, n].map((q) => [q[key] * Math.cos(q.deg * Math.PI / 180), q[key] * Math.sin(q.deg * Math.PI / 180)]);
        return `M${X(p[0][0])},${Y(p[0][1])}L${X(p[1][0])},${Y(p[1][1])}`; }).join("");
    }
    return rows.map((r, i) => `${i ? "L" : "M"}${X(r[key] * Math.cos(r.deg * Math.PI / 180))},${Y(r[key] * Math.sin(r.deg * Math.PI / 180))}`).join("") + "Z";
  };
  const pr = d.proposal, cur = d.profile_reach, dz = d.horizontal_dz || 0;
  // All four at the map's start: left and right are measured there (5 cm below the horizontal rail).
  const at = { "reach.left_m": (v) => [-v, 0], "reach.right_m": (v) => [v, 0], "reach.down_m": (v) => [0, -v], "reach.up_m": (v) => [0, v] };
  const marks = (vals, cls) => Object.entries(at).map(([k, f]) => vals && vals[k] != null ? (([a, b]) => `<circle class="${cls}" cx="${X(a)}" cy="${Y(b)}" r="5"/>`)(f(vals[k])) : "").join("");
  const propVals = Object.fromEntries(Object.entries(pr).map(([k, v]) => [k, v.value]));
  return `<svg class="reach-svg" viewBox="0 0 ${W} ${H}" role="img" aria-label="Reach map: where the hand went and the reach per direction">
    <rect class="rm-box" x="${X(box[0])}" y="${Y(box[3])}" width="${((box[1] - box[0]) * sc).toFixed(1)}" height="${((box[3] - box[2]) * sc).toFixed(1)}"/>
    <line class="rm-rail" x1="${X(x0)}" x2="${X(x1)}" y1="${Y(dz)}" y2="${Y(dz)}"/><text class="rm-label" x="${X(x0) * 1 + 4}" y="${Y(dz) - 4}">horizontal tasks' rail</text>
    <line class="rm-rail" x1="${X(0)}" x2="${X(0)}" y1="${Y(y1)}" y2="${Y(y0)}"/>
    <g class="rm-pts">${d.points.map(([a, b]) => `<circle cx="${X(a)}" cy="${Y(b)}" r="1.2"/>`).join("")}</g>
    <g class="rm-reach">${d.reaches.map(([a, b]) => `<circle cx="${X(a)}" cy="${Y(b)}" r="2.6"/>`).join("")}</g>
    <path class="rm-far" d="${poly("farthest_m")}"/><path class="rm-comf" d="${poly("comfortable_m")}"/>
    ${d.rows.filter((r) => r.arm_limited).map((r) => `<circle class="rm-limited" cx="${X(r.comfortable_m * Math.cos(r.deg * Math.PI / 180))}" cy="${Y(r.comfortable_m * Math.sin(r.deg * Math.PI / 180))}" r="6"/>`).join("")}
    ${marks(cur, "rm-cur")}${marks(propVals, "rm-prop")}
    <path class="rm-start" d="M${X(-0.006)},${Y(0)}H${X(0.006)}M${X(0)},${Y(-0.006)}V${Y(0.006)}"/>
    <text class="rm-label" x="${W - 4}" y="${H - 6}" text-anchor="end">box = the arm's limit · right = +Y, as he sees it</text>
  </svg>`;
}
function renderReachMap() {
  const box = $("#session-view"), d = SV.map;
  $("#session-sub").textContent = d && d.run ? `${d.running ? "running" : "last run"}: ${d.run}${d.run_subject ? ` · ${d.run_subject}` : ""}` : "";
  if (d && d.running && (!d.run || !d.n_movements)) {
    box.innerHTML = `<p class="empty">Mapping ${esc(d.run_subject || d.subject || "")}: waiting for the first movements. The map fills in as he moves the arm.</p>`;
    return;
  }
  if (!d || !d.run) { box.innerHTML = `<p class="empty">No Reach Map run yet. Start one: the map fills in as he moves the arm.</p>`; return; }
  const enough = d.rows.filter((r) => r.comfortable_m != null).length;
  const tiles = [["Monkey", esc(d.run_subject || "?")], ["Movements", d.n_movements], ["Mapped", `${d.minutes} <small>min</small>`],
                 ["Directions", `${enough}/${d.sectors} <small>with ≥5 movements</small>`]];
  const who = d.subject, mine = !!d.run_subject && d.run_subject === who, canSave = who && who !== "test" && mine;
  const saveWhy = !who || who === "test" ? "Choose the monkey to save" : !d.run_subject ? "This recording has no subject: it cannot be saved"
    : !mine ? `This is ${d.run_subject}'s map: choose ${d.run_subject} to save it` : "";
  const rows = REACH_DIRS.map(([k, label, where], i) => {
    const p = d.proposal[k], now = d.profile_reach ? d.profile_reach[k] : null;
    const usable = p.value != null;
    const note = !usable ? p.why : p.arm_limited ? "arm-limited: against the box, he would go farther" : p.why || where;
    return `<tr><td><input type="checkbox" id="rmv${i}" ${usable && !p.arm_limited ? "checked" : ""} ${usable && canSave ? "" : "disabled"} aria-label="Save ${label}"></td>
      <td><label for="rmv${i}">${label}</label></td><td class="v">${cm(now)}</td><td class="v"><strong>${cm(p.value)}</strong></td>
      <td class="small ${p.arm_limited ? "warn" : "muted"}">${esc(note)}</td></tr>`;
  }).join("");
  box.innerHTML = `<div class="tiles">${tiles.map(([k, v]) => `<div class="tile"><div class="k">${k}</div><div class="v">${v}</div></div>`).join("")}</div>
    ${reachSvg(d)}
    <div class="rm-legend small"><span class="lg comf"></span>comfortable reach (${d.percentile}th percentile of each direction's movements)
      <span class="lg far"></span>farthest <span class="lg reach"></span>a movement's farthest point
      <span class="lg prop"></span>this map's values <span class="lg cur"></span>${esc(who || "the profile")}'s now</div>
    <table class="save-rows rm-table"><tr><th></th><th>Reach</th><th>${esc(who || "Profile")} now</th><th>This map</th><th></th></tr>${rows}</table>
    <div class="row-actions"><button class="btn small" id="rm-save" ${canSave ? "" : "disabled"}>${canSave ? `Save ticked to ${esc(who)}'s reach` : esc(saveWhy)}</button>
      <span class="muted small">The horizontal rail is ${(d.horizontal_dz * 100).toFixed(1)} cm above this map's start (${esc(d.horizontal_dz_how)}).</span></div>
    <details class="rm-sectors"><summary class="small">Every direction</summary><table class="save-rows"><tr><th>Direction</th><th>Movements</th><th>Comfortable</th><th>Farthest</th><th>Box</th></tr>
      ${d.rows.map((r) => `<tr><td>${esc(r.name)}</td><td>${r.n}</td><td class="v">${cm(r.comfortable_m)}${r.arm_limited ? " ⚠" : ""}</td><td class="v">${cm(r.farthest_m)}</td><td class="v">${cm(r.wall_m)}</td></tr>`).join("")}</table></details>`;
  const b = $("#rm-save");
  if (b) b.onclick = async () => {
    const values = {};
    REACH_DIRS.forEach(([k], i) => { if ($(`#rmv${i}`).checked) values[k] = d.proposal[k].value; });
    if (!Object.keys(values).length) { toast("Tick at least one direction", "err"); return; }
    if (!confirm(`Save ${Object.entries(values).map(([k, v]) => `${k.slice(6, -2)} ${cm(v)}`).join(", ")} to ${who}'s reach? The rail tasks use it from their next Start.`)) return;
    await act(() => api("/api/profile/save", { exp: "reach_map", values, reach_map_run: d.run, note: `Reach Map ${d.run}` }), `Saved to ${who}'s reach`);
    refreshSession(true);
  };
}

// Juice: each trial's rewards at their mean width and pulse count, through the valve calibration.
function juiceOf(d) {
  const rows = (d && d.rows) || [];
  if (!(S.cal && S.cal.profiles.length) || !rows.length) return { ml: null };
  let ml = 0, unsure = false;
  for (const r of rows) {
    const nr = num(r.n_rewards) || 0, pulses = num(r.reward_pulses) || nr, ms = num(r.reward_ms) || 0;
    if (!nr) continue;
    const v = rewardVolume(ms / pulses, Math.round(pulses / nr), 100);
    if (v.ml == null) { unsure = true; continue; }
    if (v.status !== "calibrated") unsure = true;
    ml += nr * v.ml;
  }
  const t0 = num(rows[0].t_start), t1 = d.running ? Date.now() / 1000 : num(rows[rows.length - 1].t_end);
  const hours = (t1 - t0) / 3600;
  return { ml, unsure, rate: hours > 0.05 ? ml / hours : null };
}
function renderMonitor() {
  const box = $("#session-view"), d = SV.data;
  const rows = (d && d.rows) || [];
  $("#session-sub").textContent = d && d.run ? `${d.running ? "running" : "last run"}: ${d.run}` : "";
  if (!rows.length) {
    box.innerHTML = `<p class="empty">No trials yet${d && d.run ? " in this run" : ` for ${esc(exp(sessionKey())?.label || "")}`}.</p>`;
    return;
  }
  const last = rows[rows.length - 1];
  const done = rows.filter((r) => r.success === "0" || r.success === "1" || r.success === 0 || r.success === 1);
  const ok = done.filter((r) => String(r.success) === "1").length;
  const rewards = rows.reduce((a, r) => a + (num(r.n_rewards) || 0), 0);
  const valve = rows.reduce((a, r) => a + (num(r.reward_ms) || 0), 0);
  const tiles = [["Trials", rows.length], ["Success", `${pct(ok, done.length)} <small>${ok}/${done.length}</small>`],
                 ["Rewards", `${rewards} <small>${(valve / 1000).toFixed(1)} s valve</small>`]];
  const j = juiceOf(d);
  if (j.ml != null && rewards) tiles.push(["Juice", `${j.unsure ? "≈ " : ""}${fmtMl(j.ml)}${j.rate ? ` <small>${fmtMl(j.rate)}/h</small>` : ""}`]);
  const chosen = rows.filter((r) => String(r.correct) === "0" || String(r.correct) === "1");
  if (chosen.length) {
    const lastN = chosen.slice(-20), c = lastN.filter((r) => String(r.correct) === "1").length;
    tiles.push(["Correct, last 20", `${pct(c, lastN.length)} <small>${c}/${lastN.length}</small>`]);
  }
  if (last.block) tiles.push(["Block · mapping", `${esc(last.block)} <small>${esc(last.mapping || "")}</small>`]);
  if ("choice" in last && !("correct" in last)) {
    const ch = rows.filter((r) => r.choice), lastN = ch.slice(-20);
    tiles.push(["Paid, last 20", pct(lastN.filter((r) => String(r.rewarded) === "1").length, lastN.length)]);
    tiles.push(["Chose best", pct(ch.filter((r) => String(r.chose_best) === "1").length, ch.length)]);
  }
  if ("n_long_moves" in last) {
    const mv = rows.reduce((a, r) => a + (num(r.n_movements) || 0), 0), lm = rows.reduce((a, r) => a + (num(r.n_long_moves) || 0), 0);
    tiles.push(["Long moves", `${lm} <small>of ${mv}</small>`]);
  }
  const cls = (r) => OUTCOME_CLASS[r.outcome] || "none";
  const title = (r) => [`Trial ${r.trial}: ${r.outcome || "no outcome"}`, r.cue_colour && `cue ${r.cue_colour}`,
    r.chosen_end && `chose ${r.chosen_end}`, r.choice && `chose ${r.choice}${r.p_chosen !== "" && r.p_chosen != null ? ` (p ${r.p_chosen})` : ""}`,
    r.reversal == 1 && "first trial after a reversal"].filter(Boolean).join(" · ");
  // The strip: the last 120 trials, grouped by block when the task has blocks (the colour rule).
  const shown = rows.slice(-120);
  const groups = [];
  for (const r of shown) {
    const g = r.block ? `B${r.block} · ${r.mapping || ""}` : "";
    if (!groups.length || groups[groups.length - 1].label !== g) groups.push({ label: g, rows: [] });
    groups[groups.length - 1].rows.push(r);
  }
  const strip = groups.map((g) => `<div class="blk">${g.label ? `<span class="blk-l">${esc(g.label)}</span>` : ""}<div class="cells">` +
    g.rows.map((r) => `<span class="cell o-${cls(r)}" title="${esc(title(r))}"></span>`).join("") + "</div></div>").join("");
  const counts = {};
  rows.forEach((r) => { const k = cls(r); counts[k] = (counts[k] || 0) + 1; });
  const present = Object.keys(OUTCOME_LABEL).filter((k) => counts[k]);
  const legend = present.map((k) => `<span><i class="o-${k}"></i>${OUTCOME_LABEL[k]}</span>`).join("");
  const bars = (items, max) => `<div class="bars">` + items.map(([k, v]) =>
    `<div class="b"><span title="${esc(k)}">${esc(k)}</span><span class="track"><span class="fill" style="display:block;width:${max ? Math.round(100 * v / max) : 0}%"></span></span><span class="n">${v}</span></div>`).join("") + "</div>";
  const panels = [`<div><h3>Outcomes</h3>${bars(present.map((k) => [OUTCOME_LABEL[k], counts[k]]), rows.length)}</div>`];
  if (rows.some((r) => r.block)) {      // the colour rule: choices to criterion, block by block
    const per = {};
    rows.forEach((r) => { if (r.block && (String(r.correct) === "0" || String(r.correct) === "1")) per[r.block] = (per[r.block] || 0) + 1; });
    const blocks = Object.keys(per).map(Number).sort((a, b) => a - b);
    const items = blocks.map((b) => [`B${b}${b === Math.max(...blocks) ? " (now)" : ""}`, per[b]]);
    panels.push(`<div><h3>Choices per block</h3><p class="muted small">Completed blocks ended at criterion: falling counts mean faster reversals.</p>${bars(items, Math.max(...items.map((x) => x[1])))}</div>`);
  }
  if (rows.some((r) => r.choice)) {     // N-choice: how often each direction was chosen
    const per = {};
    rows.forEach((r) => { if (r.choice) per[r.choice] = (per[r.choice] || 0) + 1; });
    const items = Object.entries(per).sort((a, b) => b[1] - a[1]);
    panels.push(`<div><h3>Choices per direction</h3>${bars(items, Math.max(...items.map((x) => x[1])))}</div>`);
  }
  box.innerHTML = `<div class="tiles">${tiles.map(([k, v]) => `<div class="tile"><div class="k">${k}</div><div class="v">${v}</div></div>`).join("")}</div>` +
    `<div class="legend">${legend}</div><div class="strip" aria-label="Trial outcomes, oldest to newest">${strip}</div>` +
    `<div class="mon-grid">${panels.join("")}</div>`;
}

// Values as sessions.csv writes them (Python str / json.dumps) and as the dashboard holds them: compare loosely.
function normVal(v) {
  if (Array.isArray(v)) return JSON.stringify(v).replace(/\s+/g, "");
  const t = String(v ?? "").trim().replace(/\s+/g, "");
  if (/^(true|false)$/i.test(t)) return t.toLowerCase();
  const n = Number(t);
  return t !== "" && Number.isFinite(n) ? String(n) : t.replace(/^"(.*)"$/, "$1");
}
function parseVal(t) {
  const x = String(t).trim();
  if (/^(true|false)$/i.test(x)) return x.toLowerCase() === "true";
  try { return JSON.parse(x); } catch { return x; }
}

// Today vs recent looks up any subject's sessions without changing the monkey being run.
function histSubject() { return SV.histSubject || curSubject() || store.get("subject", ""); }
function renderHistory() {
  const box = $("#session-view"), h = SV.hist;
  const key = sessionKey(), label = exp(key)?.label || key;
  const subject = histSubject(), other = !!curSubject() && subject !== curSubject();
  const running = !!runningKey(), sessions = (h && h.sessions) || [];
  $("#session-sub").textContent = h ? `${label}${subject ? ` · ${subject}` : ""}` : "";
  const sameWhy = running ? "Stop the run to change per-run settings" : !sessions.length ? "No earlier session to copy"
    : other ? `These are ${subject}'s sessions: choose ${subject} as the monkey to copy them` : "";
  const tools = `<div class="hist-tools"><label for="hist-subject" class="small">Sessions of</label>
    <input id="hist-subject" list="hist-subjects" value="${esc(subject)}" size="10" spellcheck="false" title="Look up another subject's sessions (the monkey being run does not change)">
    <datalist id="hist-subjects">${((h && h.subjects) || []).map((x) => `<option value="${esc(x)}"></option>`).join("")}</datalist>
    ${other ? `<button class="btn small" id="hist-back">Back to ${esc(curSubject())}</button>` : ""}
    <button class="btn small" id="hist-same" ${sameWhy ? `disabled title="${esc(sameWhy)}"` : ""}>Same as last session</button></div>`;
  if (!h) { box.innerHTML = tools + `<p class="empty">Loading…</p>`; wireHistory(); return; }
  const sess = h.sessions || [];
  if (!sess.length) {
    box.innerHTML = tools + `<p class="empty">No earlier ${esc(label)} sessions${subject ? ` for ${esc(subject)}` : ""} in sessions.csv. ` +
      `Each run adds one when it ends.</p>`;
    wireHistory(); return;
  }
  const lastS = sess[sess.length - 1];
  const head = sess.map((x) => `<th>${esc(x.date)} ${esc(x.start)}</th>`).join("");
  const nChanged = h.settings.filter((st) => st.history.length && st.history[st.history.length - 1] !== "" &&
    normVal(st.history[st.history.length - 1]) !== normVal(st.today)).length;
  const body = h.settings.map((st, i) => {
    const prev = st.history[st.history.length - 1];
    const changed = prev !== "" && prev != null && normVal(prev) !== normVal(st.today);
    const cells = st.history.map((v, j) => `<td class="v">${v === "" ? "–" :
      running || other ? esc(v) : `<button class="use" data-i="${i}" data-j="${j}" title="Use ${esc(v)} today">${esc(v)}</button>`}</td>`).join("");
    const today = Array.isArray(st.today) ? JSON.stringify(st.today) : String(st.today);
    return `<tr class="${changed ? "changed" : ""}"><td title="${esc(st.help || "")}">${esc(st.name)}</td>${cells}` +
      `<td class="v today">${esc(today)}${changed ? " ●" : ""}</td></tr>`;
  }).join("");
  const glance = [["Trials", lastS.n_trials], ["Success", lastS.success_pct !== "" ? `${lastS.success_pct}%` : "–"],
    ["Rewards", `${lastS.n_rewards} <small>${(num(lastS.reward_ms) / 1000 || 0).toFixed(1)} s valve</small>`],
    ["Duration", lastS.duration_min !== "" ? `${lastS.duration_min} min` : "–"]];
  const common = new Set(["run", "date", "start", "duration_min", "subject", "experiment", "paradigms", "n_trials", "n_success",
    "success_pct", "n_rewards", "reward_ms", "settings", "changed", "git_sha"]);
  Object.entries(lastS).forEach(([k, v]) => { if (!common.has(k) && v !== "" && v != null) glance.push([k.replace(/_/g, " "), esc(v)]); });
  box.innerHTML = tools +
    `<h3>Last session at a glance <span class="muted small">${esc(lastS.date)} ${esc(lastS.start)} · ${esc(lastS.paradigms || "")}</span></h3>` +
    `<div class="tiles">${glance.map(([k, v]) => `<div class="tile"><div class="k">${esc(k)}</div><div class="v">${v}</div></div>`).join("")}</div>` +
    (lastS.changed ? `<p class="muted small">Changed during that session: ${esc(lastS.changed)}</p>` : "") +
    `<h3>Settings <span class="muted small">${nChanged ? `${nChanged} differ from last session (●)` : "same as last session"}` +
    `${running ? "" : " · click an earlier value to use it today"}</span></h3>` +
    `<div class="hist-wrap"><table class="hist"><thead><tr><th>Setting</th>${head}<th>Today</th></tr></thead><tbody>${body}</tbody></table></div>`;
  wireHistory();
}

function wireHistory() {
  const inp = $("#hist-subject");
  if (inp) inp.onchange = () => { SV.histSubject = inp.value.trim() || null; refreshSession(true); };   // look up only
  const back = $("#hist-back");
  if (back) back.onclick = () => { SV.histSubject = null; refreshSession(true); };
  const h = SV.hist;
  const apply = async (pairs) => {
    for (const [name, val] of pairs) await act(() => api(`/api/params/${sessionKey()}`, { name, value: parseVal(val) }));
    await loadParams(sessionKey(), true); renderExperiments(); renderControls();
    toast(pairs.length ? `${pairs.length} setting(s) set for the next Start` : "Already the same as last session", "ok");
    refreshSession(true);
  };
  const same = $("#hist-same");
  if (same && h) same.onclick = () => apply(h.settings.filter((st) => {
    const prev = st.history[st.history.length - 1];
    return prev !== "" && prev != null && normVal(prev) !== normVal(st.today);
  }).map((st) => [st.name, st.history[st.history.length - 1]]));
  document.querySelectorAll("table.hist button.use").forEach((b) => b.onclick = () => {
    const st = h.settings[Number(b.dataset.i)];
    apply([[st.name, st.history[Number(b.dataset.j)]]]);
  });
}

document.querySelectorAll(".session-card .tab").forEach((t) => t.onclick = () => {
  SV.view = t.dataset.view; SV.userView = true; refreshSession(true);
});

// ---------------------------------------------------------------- Start: ask who is being run
async function openStartDialog() {
  const dlg = $("#start-dlg"), inp = $("#start-subject");
  $("#start-dlg-exp").textContent = expLabel(S.selected);
  inp.value = curSubject() || store.get("subject", "");
  const e = exp(S.selected);
  $("#start-dlg-profile").innerHTML = !curSubject() ? "Recorded with every trial. Use <code>test</code> for a run without an animal."
    : hasProfile() && e && e.node === "free_move_orchestrator" ? `With ${esc(curSubject())}'s profile v${S.app.profile.version}, and today's settings. A different ID loads that monkey first, for you to check.`
    : "Recorded with every trial. A different ID loads that monkey first, for you to check.";
  try {
    const subjects = await api("/api/subjects");
    $("#start-subjects").innerHTML = subjects.map((x) => `<option value="${esc(x)}"></option>`).join("");
  } catch { /* no history yet */ }
  dlg.showModal();
  inp.focus(); inp.select();
}
$("#start-cancel").onclick = () => $("#start-dlg").close();
$("#start-form").onsubmit = async (ev) => {
  ev.preventDefault();
  const subject = $("#start-subject").value.trim();
  if (!$("#start-subject").checkValidity()) return $("#start-subject").reportValidity();
  $("#start-dlg").close();
  if (subject !== curSubject()) {           // another monkey: load it (and its profile) first, for a look before Start
    if (await selectSubject(subject)) toast(`Monkey set to ${subject}: check the settings, then press Start again`, "ok");
    return;
  }
  store.set("subject", subject);
  await act(() => api("/api/start", { experiment: S.selected, subject }), `Starting (subject ${subject})`);
};

async function init() {
  S.experiments = await api("/api/experiments");
  S.groups = await api("/api/groups");
  if (!exp(S.selected)) S.selected = S.experiments[0].key;
  S.group = exp(S.selected).group;
  (await api("/api/logs")).forEach(appendLog);
  S.app = await api("/api/state");
  // The monkey this browser last chose, unless the dashboard already has one (another tab, or a run).
  if (!S.app.subject && store.get("subject", "") && S.app.runner.state === "idle") {
    try { await api("/api/subject", { subject: store.get("subject", "") }); S.app = await api("/api/state"); } catch { /* an old ID */ }
  }
  S.profSig = `${S.app.subject}|${S.app.profile ? S.app.profile.version : ""}`;
  await loadProfile();
  try { S.cal = await api("/api/reward/calibration"); } catch { S.cal = null; }
  if (runningKey()) S.selected = runningKey();     // open on whatever is running
  renderExperiments(); renderChecklist();
  await loadParams(S.selected);
  renderExperiments();          // the paradigm tabs need the parameters
  renderControls();
  refreshSession(true);
  connect();
  wireTips();
  // ?hover=<parameter name>: open that parameter's help, e.g. to link someone to it.
  const hv = new URLSearchParams(location.search).get("hover");
  if (hv) setTimeout(() => {
    const p = S.params[S.selected]?.params.find((x) => x.name === hv);
    if (p) { S.tab = p.tier; S.pview = "all"; renderParams(true); }
    const el = [...document.querySelectorAll("#params .name[data-tip]")].find((n) => TIPS.get(n.dataset.tip)?.includes(`>${esc(hv)}<`));
    if (el) {
      const r = el.getBoundingClientRect();
      if (r.top < 0 || r.bottom > innerHeight) el.scrollIntoView({ block: "center" });
      showTip(el);
    }
  }, 300);

  $("#btn-start").onclick = openStartDialog;
  $("#btn-stop").onclick = () => {
    const q = S.app && S.app.stopping_now
      ? "Skip parking and stop the launch now? FRI closes wherever the arm is."
      : "Stop? The current trial is cut short (saved, marked stop_requested), the arm parks, then the launch stops.";
    if (confirm(q)) act(() => api("/api/stop", {}), "Stopping");
  };
  $("#btn-validate").onclick = () => act(() => api("/api/validate", {}), "Validating — see the log");
  $("#btn-reset-edits").onclick = async () => { await act(() => api(`/api/params/${S.selected}/reset`, {}), "Edits cleared"); loadParams(S.selected, true); };
  document.querySelectorAll(".tab[data-tab]").forEach((t) => t.onclick = () => { S.tab = t.dataset.tab; store.set("tab", S.tab); renderParams(true); });
  document.querySelectorAll(".tab[data-pview]").forEach((t) => t.onclick = () => { S.pview = t.dataset.pview; store.set("pview", S.pview); renderParams(true); });
  $("#monkey").onchange = (e) => selectSubject(e.target.value);
  $("#btn-profile").onclick = () => openProfileDialog();
  $("#profile-cancel").onclick = () => $("#profile-dlg").close();
  $("#save-cancel").onclick = () => $("#save-dlg").close();
  $("#param-filter").oninput = (e) => { S.paramFilter = e.target.value; renderParams(true); };
  document.querySelectorAll("#log-levels button").forEach((b) => b.onclick = () => {
    S.logLevel = b.dataset.level;
    document.querySelectorAll("#log-levels button").forEach((x) => x.classList.toggle("on", x === b));
    rerenderLog();
  });
  $("#log-filter").oninput = (e) => { S.logFilter = e.target.value; rerenderLog(); };
  $("#btn-log-clear").onclick = () => { $("#log").innerHTML = ""; };
  $("#log-toggle").onclick = () => openLog($("#log-drawer").classList.contains("hidden"));
  $("#live-card").open = store.get("liveOpen", true);
  $("#live-card").addEventListener("toggle", () => liveToggled());
  $("#log-warns").onclick = () => openLog(true, "warn");
  $("#btn-log-tall").onclick = () => $("#log-drawer").classList.toggle("tall");
  // Close the reward popover with a click elsewhere.
  document.addEventListener("click", (ev) => { if (S.rewardPop && !$("#reward-bar").contains(ev.target)) { S.rewardPop = false; renderRewardBar(); } });
  setInterval(() => { if (S.app) renderStatus(); }, 1000);
}
init().catch((e) => toast(`Dashboard failed to load: ${e.message}`, "err"));
