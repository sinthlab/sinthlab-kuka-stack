"use strict";
// Diagnostics page. Every board command goes through the dashboard server (POST /api/diag/board),
// which relays it to the cue board over its own Wi-Fi or over the serial link (`GET /<cmd>` runs the
// same board function) -- see experiment_ctrl_gui/board.py. The controls are the same on both paths.

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const D = { path: "wifi" };

// show = false keeps the reply panel on the previous command (the status refresh after a change).
async function board(cmd, params = {}, show = true) {
  const r = await fetch("/api/diag/board", {
    method: "POST", headers: { "Content-Type": "application/json", "X-Experiment-Ctrl": "1" },
    body: JSON.stringify({ addr: $("#addr").value.trim(), port: $("#port").value.trim(), path: D.path, cmd, params }),
  });
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  if (show) $("#reply").textContent = `${cmd} over ${D.path === "serial" ? "serial" : "Wi-Fi"} (${j.ms} ms)\n${j.text}`;
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

async function check(show = true) {
  const pill = $("#reach");
  try {
    const j = await board("status", {}, show);
    pill.className = "pill " + (j.ok ? "ok" : "err");
    pill.textContent = j.ok ? `reachable · ${j.ms} ms` : "not reachable";
    if (!j.ok) return null;
    const kv = parse(j.text);
    showStatus(kv); fillForm(kv);
    return kv;
  } catch (e) { pill.className = "pill err"; pill.textContent = e.message; return null; }
}

// A command, then the status refresh -- which keeps the command's own reply in view, says plainly when the
// board refused it, and checks that a brightness change actually took (the form is refilled from the board).
async function act(cmd, params) {
  try {
    const j = await board(cmd, params);
    if (!j.ok) {
      $("#reach").className = "pill err";
      $("#reply").textContent = `${cmd} NOT applied over ${D.path === "serial" ? "serial" : "Wi-Fi"}: ${j.text}`;
      return;
    }
    const kv = await check(false);
    const want = params && params.brightness !== undefined ? parseFloat(params.brightness) : NaN;
    if (kv && Number.isFinite(want) && Math.abs(parseFloat(kv.brightness) - want) > 0.01)
      $("#reply").textContent += `\nThe board reports brightness ${kv.brightness}, not ${want}: the change did not take.`;
  } catch (e) { $("#reply").textContent = e.message; }
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
  $("#addr-label").classList.toggle("hidden", p !== "wifi");
  $("#port-label").classList.toggle("hidden", p !== "serial");
  $("#path-note").textContent = p === "wifi"
    ? "Connect this computer to the KUKA_NEOPIXEL network (visual_cue.trigger: wifi). The board address is 192.168.4.1."
    : "Serial uses RS-422 through X76 and the media flange (visual_cue.trigger: serial). Attach the StarTech adapter to WSL with usbipd attach --wsl --busid <id>. The port is reserved during experiments.";
  $("#reach").className = "pill"; $("#reach").textContent = "not checked";
  check();
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
// One ring quarter green (pixels 15q..15q+14), the rest dark: the quarters the pre-training ring states use.
document.querySelectorAll("[data-quarter]").forEach((b) => (b.onclick = () => {
  const q = +b.dataset.quarter;
  const colors = [0, 1, 2, 3].map((i) => (i === q ? "0,255,0,0" : "0,0,0,0")).join(".");
  act("segments", { factor: 4, colors });
}));
$("#btn-save").onclick = () => act("config", { save: 1 });
$("#btn-reset").onclick = () => { if (confirm("Restore the firmware defaults for all cue board settings?")) act("config", { reset: 1 }); };
$("#btn-lt").onclick = async () => {
  await act("led_test", { from: $("#lt-from").value, to: $("#lt-to").value, step: $("#lt-step").value, loop: $("#lt-loop").checked ? 1 : 0 });
  if (!ltTimer) ltTimer = setInterval(ltPoll, 300);
};
$("#btn-lt-off").onclick = () => act("off");
for (const id of ["brightness", "r", "g", "b"]) $("#" + id).addEventListener("input", brightWarn);
setPath(location.hash === "#serial" ? "serial" : "wifi");
