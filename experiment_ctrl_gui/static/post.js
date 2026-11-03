"use strict";
// Post-processing page: archive a session as NWB (analysis/nwb_export.py) and upload it to the server.
// The server runs one job at a time and reports progress on /api/events as {"type": "post"} --
// see experiment_ctrl_gui/postprocess.py.

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const STORE = "postprocess.meta";
const P = { nspBase: null, robot: null, job: null };

async function api(path, body) {
  const r = await fetch(path, {
    method: "POST", headers: { "Content-Type": "application/json", "X-Experiment-Ctrl": "1" },
    body: JSON.stringify(body || {}),
  });
  const j = await r.json();
  if (j.ok === false || j.error) throw new Error(j.error || "request failed");
  return j;
}

function toast(text, kind = "") {
  const el = document.createElement("div");
  el.className = "toast " + kind;
  el.textContent = text;
  $("#toasts").appendChild(el);
  setTimeout(() => el.remove(), 6000);
}

function kv(el, rows) {
  el.innerHTML = rows.filter(([, v]) => v !== undefined && v !== null && v !== "")
    .map(([k, v]) => `<dt>${esc(k)}</dt><dd>${esc(v)}</dd>`).join("");
}

const mb = (b) => (b >= 1e9 ? `${(b / 1e9).toFixed(2)} GB` : `${(b / 1e6).toFixed(1)} MB`);
const dirname = (p) => p.replace(/\/[^/]*$/, "");

// ---------------------------------------------------------------- metadata (kept on this computer)

function meta() {
  const m = {};
  document.querySelectorAll("[data-meta]").forEach((el) => { m[el.dataset.meta] = el.value.trim(); });
  return m;
}

function loadMeta() {
  let m = {};
  try { m = JSON.parse(localStorage.getItem(STORE) || "{}"); } catch (e) { /* storage blocked */ }
  document.querySelectorAll("[data-meta]").forEach((el) => {
    if (m[el.dataset.meta] !== undefined) el.value = m[el.dataset.meta];
  });
}

function saveMeta() {
  try { localStorage.setItem(STORE, JSON.stringify(meta())); } catch (e) { /* storage blocked */ }
}

// ---------------------------------------------------------------- defaults

function defaultOut() {
  const robot = $("#robot").value.trim().replace(/\/+$/, "");
  if (!robot) return;
  const sub = (meta().subject_id || "SUBJECT").replace(/[^A-Za-z0-9-]/g, "-");
  const ses = (P.robot && P.robot.session_id) || robot.split("/").pop().replace(/^expt_/, "");
  if (!$("#out").dataset.edited) $("#out").value = `${robot}/sub-${sub}_ses-${ses.replace(/[^A-Za-z0-9-]/g, "-")}.nwb`;
}

// ---------------------------------------------------------------- 1. recordings

async function scan() {
  const robot = $("#robot").value.trim(), nsp = $("#nsp").value.trim();
  if (!robot) return toast("Choose the robot run first.", "warn");
  $("#btn-scan").disabled = true;
  kv($("#robot-info"), [["…", "reading"]]);
  if (nsp) kv($("#nsp-info"), [["…", "reading the header"]]);
  try {
    const j = await api("/api/post/scan", { robot, nsp });
    const r = j.robot;
    P.robot = r && !r.error ? r : null;
    kv($("#robot-info"), r && r.error ? [["Problem", r.error]] : [
      ["Experiment", r.experiment], ["Session", r.session_id], ["Trials", r.trials],
      ["Duration", `${r.duration_s} s`], ["Started (UTC)", r.start],
      ["trial_start / trial_end events", `${r.trial_start_events} / ${r.trial_end_events}`]]);
    const n = j.nsp;
    if (!nsp) kv($("#nsp-info"), [["–", "none: the NWB holds the robot data alone"]]);
    else if (n.error) kv($("#nsp-info"), [["Problem", n.error]]);
    else {
      P.nspBase = n.base;
      const rows = [["Recording", n.base], ["Started (UTC)", n.start || "not in the files"], ["Segments", n.segments]];
      for (const s of n.streams)
        rows.push([s.name, `${s.rate_hz} Hz · ${s.neural_channels} neural · ${s.duration_s} s` +
                   (s.analog_inputs.length ? ` · ${s.analog_inputs.join(", ")}` : "")]);
      rows.push(["Spike units (NEV)", n.spike_units], ["Event channels (NEV)", n.event_channels]);
      const ain = n.streams.flatMap((s) => s.analog_inputs.map((a) => a.toLowerCase()));
      for (const id of ["#sync-start", "#sync-end"]) {
        const c = $(id).value.trim().toLowerCase();
        if (!ain.includes(c)) rows.push(["Sync", `${c} is not in this recording`]);
      }
      kv($("#nsp-info"), rows);
      if (!$("#up-dest").dataset.edited) $("#up-dest").value = dirname(n.base);
    }
    defaultOut();
  } catch (e) {
    toast(e.message, "err");
  } finally {
    $("#btn-scan").disabled = false;
  }
}

// ---------------------------------------------------------------- 3. archive

async function convert() {
  const m = meta();
  if (!m.subject_id) return toast("Subject ID is required.", "warn");
  saveMeta();
  defaultOut();
  try {
    await api("/api/post/convert", {
      robot: $("#robot").value.trim(), nsp: $("#nsp").value.trim(), out: $("#out").value.trim(), meta: m,
      sync_channels: [$("#sync-start").value.trim(), $("#sync-end").value.trim()],
      threshold_v: parseFloat($("#threshold").value),
      include_continuous: $("#inc-cont").checked, include_spikes: $("#inc-spikes").checked,
    });
    kv($("#result"), []);
  } catch (e) {
    toast(e.message, "err");
  }
}

async function upload() {
  try {
    await api("/api/post/upload", {
      nwb: $("#up-src").value.trim(), dest: $("#up-dest").value.trim(), overwrite: $("#up-overwrite").checked,
    });
    kv($("#up-result"), []);
  } catch (e) {
    toast(e.message, "err");
  }
}

// ---------------------------------------------------------------- job state

function showJob(job) {
  P.job = job;
  const running = job && job.state === "running";
  const pill = $("#pill-job");
  pill.className = "pill " + (!job ? "" : running ? "run" : job.state === "done" ? "ok" : "err");
  pill.textContent = !job ? "Idle" : running ? (job.kind === "convert" ? "Archiving…" : "Uploading…")
    : `${job.kind === "convert" ? "Archive" : "Upload"}: ${job.state}`;
  for (const id of ["#btn-convert", "#btn-upload", "#btn-scan"]) $(id).disabled = running;
  $("#btn-cancel").classList.toggle("hidden", !running);
  $("#progress").classList.toggle("hidden", !running);
  if (running) {
    const f = job.fraction;
    $("#bar").style.width = f >= 0 ? `${Math.round(f * 100)}%` : "100%";
    $("#bar").classList.toggle("indeterminate", f < 0);
    $("#progress-msg").textContent = job.message + (f >= 0 ? ` · ${Math.round(f * 100)}%` : "");
  }
  if (!job || running) return;
  if (job.kind === "convert") {
    if (job.state === "done") {
      const r = job.result, s = r.sync;
      kv($("#result"), [["File", r.out], ["Size", mb(r.bytes)], ["Trials", r.trials], ["Time base", r.time_base],
        ["Session start (UTC)", r.session_start],
        ...(s ? [["Sync matched", Object.entries(s.matched).map(([k, v]) => `${k} ${v}`).join(" · ")],
                 ["Sync fit", `rms ${s.rms_ms} ms · max ${s.max_ms} ms · drift ${s.drift_ppm} ppm`]] : [])]);
      $("#up-src").value = r.out;
      $("#upload-prompt").innerHTML = "<strong>The archive is ready. Upload it to the server folder below.</strong>";
      $("#upload-card").classList.add("pp-attention");
      $("#upload-card").scrollIntoView({ behavior: "smooth", block: "nearest" });
    } else {
      kv($("#result"), [["Not created", job.error]]);
    }
  } else if (job.state === "done") {
    kv($("#up-result"), [["Uploaded", job.result.dest], ["Size", mb(job.result.bytes)],
      ["SHA-256 (verified)", job.result.sha256]]);
    $("#upload-card").classList.remove("pp-attention");
    $("#upload-prompt").textContent = "Uploaded and verified.";
  } else {
    kv($("#up-result"), [["Not uploaded", job.error]]);
  }
}

// ---------------------------------------------------------------- start

async function init() {
  loadMeta();
  const st = await (await fetch("/api/post/state")).json();
  if (st.deps) {
    $("#banner").textContent = `NWB export unavailable: ${st.deps}`;
    $("#banner").classList.remove("hidden");
  }
  $("#run-pick").innerHTML += st.runs.map((r) => `<option value="${esc(r.path)}">${esc(r.name)} (${r.trials} trials)</option>`).join("");
  showJob(st.job);
  $("#run-pick").addEventListener("change", () => { if ($("#run-pick").value) { $("#robot").value = $("#run-pick").value; P.robot = null; defaultOut(); } });
  $("#robot").addEventListener("change", () => { P.robot = null; defaultOut(); });
  $("#out").addEventListener("input", () => { $("#out").dataset.edited = "1"; });
  $("#up-dest").addEventListener("input", () => { $("#up-dest").dataset.edited = "1"; });
  document.querySelector('[data-meta="subject_id"]').addEventListener("input", defaultOut);
  $("#btn-scan").addEventListener("click", scan);
  $("#btn-convert").addEventListener("click", convert);
  $("#btn-upload").addEventListener("click", upload);
  $("#btn-cancel").addEventListener("click", () => api("/api/post/cancel").catch((e) => toast(e.message, "err")));
  const es = new EventSource("/api/events");
  es.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === "post") showJob(m.data);
  };
}

init();
