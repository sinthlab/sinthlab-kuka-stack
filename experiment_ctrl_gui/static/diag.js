"use strict";
// Diagnostics page. Every board command goes through the dashboard server (POST /api/diag/board),
// which relays it to the cue board on its own Wi-Fi -- see experiment_ctrl_gui/board.py.

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const D = { path: "wifi" };

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
    ["Enabled", truthy(kv.enabled) ? "yes" : "no"],
    ["Brightness", kv.brightness], ["Colour (r,g,b,w)", `${kv.r}, ${kv.g}, ${kv.b}, ${kv.w}`],
    ["Pattern", `${kv.pattern} (${kv.duration} s, period ${kv.period} s)`], ["Mode", kv.mode],
    ["LEDs", kv.leds], ["Cue running", truthy(kv.cue_active) ? "yes" : "no"],
    ["Serial link", kv.serial ?? "–"], ["Serial cues fired", kv.serial_fired], ["Settings saved", kv.nvm], ["Uptime", `${kv.uptime_s} s`],
  ];
  $("#board-status").innerHTML = rows.map(([k, v]) => `<dt>${esc(k)}</dt><dd>${esc(v ?? "–")}</dd>`).join("");
}

function fillForm(kv) {
  for (const k of ["r", "g", "b", "w", "segments", "duration", "period", "brightness"])
    if (kv[k] !== undefined) $("#" + k).value = kv[k];
  if (kv.patterns) $("#pattern").innerHTML = kv.patterns.split(",").map((p) => `<option>${esc(p)}</option>`).join("");
  if (kv.pattern) $("#pattern").value = kv.pattern;
  $("#enabled").checked = truthy(kv.enabled);
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
  $("#path-serial").classList.toggle("hidden", p !== "serial");
  $("#path-note").textContent = p === "wifi"
    ? "Fires the board over its own Wi-Fi (visual_cue.trigger: wifi). If the ring lights here, the board, the Pixel Shifter and the ring are fine."
    : "Fires the board over the RS-422 serial link (visual_cue.trigger: serial). The board answers only once the ring is lit, so the round trip below is the cue's delivery time.";
}

// ---------------- serial path: one command per click, via POST /api/diag/serial
async function serial(cmd) {
  const li = document.createElement("li");
  const now = new Date().toLocaleTimeString();
  try {
    const r = await fetch("/api/diag/serial", {
      method: "POST", headers: { "Content-Type": "application/json", "X-Experiment-Ctrl": "1" },
      body: JSON.stringify({ port: $("#ser-port").value.trim(), cmd }),
    });
    const j = await r.json();
    if (j.error) throw new Error(j.error);
    li.textContent = `${now}  ${cmd}  →  ${j.text}${j.ok ? `  (${j.ms} ms)` : ""}`;
    $("#ser-state").className = "pill " + (j.ok ? "ok" : "err");
    $("#ser-state").textContent = j.ok ? `answered · ${j.ms} ms` : "no answer";
  } catch (e) {
    li.textContent = `${now}  ${cmd}  →  ${e.message}`;
    $("#ser-state").className = "pill err"; $("#ser-state").textContent = "error";
  }
  $("#ser-log").prepend(li);
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
$("#btn-ser-ping").onclick = () => serial("PING");
$("#btn-ser-cue").onclick = () => serial("CUE");
$("#btn-ser-colour").onclick = () => serial(`CUE ${+$("#r").value} ${+$("#g").value} ${+$("#b").value} ${+$("#w").value}`);
$("#btn-ser-quarter").onclick = () => serial("SEG 4 0,255,0,0");
$("#btn-ser-off").onclick = () => serial("OFF");
for (const id of ["brightness", "r", "g", "b"]) $("#" + id).addEventListener("input", brightWarn);
setPath(location.hash === "#serial" ? "serial" : "wifi");
check();
