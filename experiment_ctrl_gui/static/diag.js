"use strict";
// Diagnostics page. Every board command goes through the dashboard server (POST /api/diag/board),
// which relays it to the cue board on its own Wi-Fi -- see experiment_ctrl_gui/board.py.

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const D = { path: "wifi", watching: null, last: null, fired: null, edges: 0 };

async function board(cmd, params = {}) {
  const r = await fetch("/api/diag/board", {
    method: "POST", headers: { "Content-Type": "application/json", "X-Experiment-Ctrl": "1" },
    body: JSON.stringify({ addr: $("#addr").value.trim(), cmd, params }),
  });
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  $("#reply").textContent = `${cmd} (${j.ms} ms)\n${j.text}`;
  return j;
}

// The board answers in "key=value" lines (comments start with #).
function parse(text) {
  const kv = {};
  for (const line of text.split("\n")) {
    const i = line.indexOf("=");
    if (line && !line.startsWith("#") && i > 0) kv[line.slice(0, i).trim()] = line.slice(i + 1).trim();
  }
  return kv;
}

const truthy = (v) => ["1", "true", "True"].includes(String(v));

function showStatus(kv) {
  const rows = [
    ["Enabled", truthy(kv.enabled) ? "yes" : "NO — the wire will not fire a cue"],
    ["Brightness", kv.brightness], ["Colour (r,g,b,w)", `${kv.r}, ${kv.g}, ${kv.b}, ${kv.w}`],
    ["Pattern", `${kv.pattern} (${kv.duration} s, period ${kv.period} s)`], ["Mode", kv.mode],
    ["LEDs", kv.leds], ["Cue running", truthy(kv.cue_active) ? "yes" : "no"],
    ["Wire cues fired", kv.wire_fired], ["Settings saved", kv.nvm], ["Uptime", `${kv.uptime_s} s`],
  ];
  $("#board-status").innerHTML = rows.map(([k, v]) => `<dt>${esc(k)}</dt><dd>${esc(v ?? "–")}</dd>`).join("");
}

function fillForm(kv) {
  for (const k of ["r", "g", "b", "w", "segments", "duration", "period", "brightness", "debounce_ms"])
    if (kv[k] !== undefined) $("#" + k).value = kv[k];
  if (kv.patterns) $("#pattern").innerHTML = kv.patterns.split(",").map((p) => `<option>${esc(p)}</option>`).join("");
  if (kv.pattern) $("#pattern").value = kv.pattern;
  if (kv.mode) $("#mode").value = kv.mode;
  $("#enabled").checked = $("#enabled2").checked = truthy(kv.enabled);
  $("#active_low").checked = truthy(kv.active_low);
  $("#retrigger").checked = truthy(kv.retrigger);
  brightWarn();
}

async function check() {
  const pill = $("#reach");
  try {
    const j = await board("status");
    pill.className = "pill " + (j.ok ? "ok" : "err");
    pill.textContent = j.ok ? `reachable · ${j.ms} ms` : "not reachable";
    if (!j.ok) return null;
    const kv = parse(j.text);
    showStatus(kv); fillForm(kv);
    return kv;
  } catch (e) { pill.className = "pill err"; pill.textContent = e.message; return null; }
}

async function act(cmd, params) {
  try { const j = await board(cmd, params); if (!j.ok) $("#reach").className = "pill err"; await check(); }
  catch (e) { $("#reply").textContent = e.message; }
}

const cueParams = () => ({
  r: $("#r").value, g: $("#g").value, b: $("#b").value, w: $("#w").value, pattern: $("#pattern").value,
  segments: $("#segments").value, duration: $("#duration").value, period: $("#period").value,
  brightness: $("#brightness").value, enabled: $("#enabled").checked ? 1 : 0,
});

function brightWarn() {
  const b = parseFloat($("#brightness").value) || 0;
  const colour = (+$("#r").value) + (+$("#g").value) + (+$("#b").value) > 0;
  $("#bright-warn").classList.toggle("hidden", !(b > 0.3 && colour) && !(b > 0.6));
}

function setPath(p) {
  D.path = p;
  document.querySelectorAll(".diag-path .tab").forEach((t) => t.classList.toggle("active", t.dataset.path === p));
  $("#path-wifi").classList.toggle("hidden", p !== "wifi");
  $("#path-wire").classList.toggle("hidden", p !== "wire");
  $("#path-note").textContent = p === "wifi"
    ? "Fires the board directly over its Wi-Fi — the experiments' test trigger (visual_cue.remote_test_trigger: true). If the ring lights here, the board, the Pixel Shifter and the ring are fine."
    : "The experiment trigger: a contact closure at X76 1–2 pulls Metro D2 low through the media flange. Watching it needs Wi-Fi too (to read the board), but the cue itself comes from the wire.";
  if (p !== "wire") stopWatch();
}

// ---------------- wire monitor: poll /status, log every change of the D2 input
async function poll() {
  let j;
  try { j = await board("status"); } catch (e) { return; }
  if (!j.ok) { $("#w-raw").textContent = "board not reachable"; $("#w-raw").className = "pill err"; return; }
  const kv = parse(j.text);
  const asserted = truthy(kv.trigger_debounced);
  $("#w-raw").textContent = `pin ${kv.trigger_pin_raw}`; $("#w-raw").className = "pill";
  $("#w-asserted").textContent = asserted ? "ASSERTED" : "idle";
  $("#w-asserted").className = "pill " + (asserted ? "ok" : "");
  $("#w-fired").textContent = `fired ${kv.wire_fired}`;
  $("#w-cue").textContent = truthy(kv.cue_active) ? "cue running" : "no cue";
  const now = new Date().toLocaleTimeString();
  if (D.last !== null && asserted !== D.last) {
    D.edges++;
    const li = document.createElement("li");
    li.textContent = `${now}  ${asserted ? "↓ asserted (contacts closed)" : "↑ released"}`;
    $("#w-log").prepend(li);
  }
  if (D.fired !== null && kv.wire_fired !== D.fired) {
    const li = document.createElement("li");
    li.textContent = `${now}  cue fired by the wire (count ${kv.wire_fired})`;
    $("#w-log").prepend(li);
  }
  // Nothing connected and the line already reads asserted: the polarity is inverted somewhere.
  const pol = $("#w-polarity");
  if (D.edges === 0 && asserted) {
    pol.textContent = "The line reads ASSERTED with nothing shorted. If X76 1–2 are open right now, the polarity is inverted: untick “Active low” and apply — no rewiring needed.";
    pol.classList.remove("hidden");
  } else pol.classList.add("hidden");
  if (!truthy(kv.enabled)) {
    pol.textContent = "The board is not enabled: the wire will be seen above but fire no cue. Tick “Enabled” and apply.";
    pol.classList.remove("hidden");
  }
  D.last = asserted; D.fired = kv.wire_fired;
}

function startWatch() {
  D.last = null; D.fired = null; D.edges = 0; $("#w-log").innerHTML = "";
  D.watching = setInterval(poll, 250); poll();
  $("#btn-watch").textContent = "■ Stop watching";
}
function stopWatch() {
  if (D.watching) clearInterval(D.watching);
  D.watching = null; $("#btn-watch").textContent = "Start watching";
}

// ---------------- LED test: show which pixel is lit
let ltTimer = null;
async function ltPoll() {
  let j; try { j = await board("status"); } catch (e) { return; }
  const kv = parse(j.text);
  const px = parseInt(kv.led_test_pixel, 10);
  $("#lt-pixel").textContent = px >= 0 ? `pixel ${px} · ${kv.led_test_colour}` : "pixel – (idle)";
  if (!(px >= 0)) { clearInterval(ltTimer); ltTimer = null; }
}

// ---------------- wiring
$("#btn-check").onclick = check;
document.querySelectorAll(".diag-path .tab").forEach((t) => (t.onclick = () => setPath(t.dataset.path)));
$("#btn-apply").onclick = () => act("config", cueParams());
$("#btn-fire").onclick = () => act("cue");
$("#btn-solid").onclick = () => act("set_color", { r: $("#r").value, g: $("#g").value, b: $("#b").value, w: $("#w").value });
$("#btn-off").onclick = () => act("off");
$("#btn-save").onclick = () => act("config", { save: 1 });
$("#btn-reset").onclick = () => { if (confirm("Reset every board setting to the firmware defaults?")) act("config", { reset: 1 }); };
$("#btn-lt").onclick = async () => {
  await act("led_test", { from: $("#lt-from").value, to: $("#lt-to").value, step: $("#lt-step").value, loop: $("#lt-loop").checked ? 1 : 0 });
  if (!ltTimer) ltTimer = setInterval(ltPoll, 300);
};
$("#btn-lt-off").onclick = () => act("off");
$("#btn-watch").onclick = () => (D.watching ? stopWatch() : startWatch());
$("#btn-apply-trig").onclick = () => act("config", {
  active_low: $("#active_low").checked ? 1 : 0, debounce_ms: $("#debounce_ms").value,
  retrigger: $("#retrigger").checked ? 1 : 0, mode: $("#mode").value, enabled: $("#enabled2").checked ? 1 : 0,
});
for (const id of ["brightness", "r", "g", "b"]) $("#" + id).addEventListener("input", brightWarn);
setPath(location.hash === "#wire" ? "wire" : "wifi");
check();
