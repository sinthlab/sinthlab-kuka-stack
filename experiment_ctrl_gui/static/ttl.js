"use strict";
// Diagnostics page, TTL sync tab, and the page's device tabs. Every command goes through the dashboard
// server (POST /api/diag/ttl), which sends it to the cabinet's UDP 30002 -- see experiment_ctrl_gui/ttl.py.
// Uses $ and esc from diag.js.

const T = { run: 0, busy: false };
const ADDR_KEY = "diag.ttl.addr";
const GAP_MS = 250;              // between the pulses of one output in "All", as PulseTestServer
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function tlog(text, kind = "") {
  const li = document.createElement("li");
  li.textContent = `${new Date().toLocaleTimeString()}  ${text}`;
  if (kind) li.className = kind === "err" ? "warn-text" : "";
  $("#ttl-log").prepend(li);
  while ($("#ttl-log").children.length > 200) $("#ttl-log").lastChild.remove();
}

async function ttl(cmd) {
  const addr = $("#ttl-addr").value.trim();
  try { localStorage.setItem(ADDR_KEY, addr); } catch (e) { /* storage blocked */ }
  const r = await fetch("/api/diag/ttl", {
    method: "POST", headers: { "Content-Type": "application/json", "X-Experiment-Ctrl": "1" },
    body: JSON.stringify({ addr, cmd }),
  });
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  const good = j.ok && j.text.startsWith("OK");
  tlog(`${cmd}  →  ${j.text}  (${j.ms} ms)`, good ? "" : "err");
  return { ...j, good };
}

async function ttlCheck() {
  const pill = $("#ttl-reach");
  try {
    const j = await ttl("STATUS");
    pill.className = "pill " + (j.good ? "ok" : "err");
    pill.textContent = j.good ? `outputs available · ${j.ms} ms` : j.ok ? "outputs unavailable" : "no answer";
    $("#ttl-status").textContent = j.text;
  } catch (e) {
    pill.className = "pill err"; pill.textContent = "refused";
    tlog(e.message, "err");
  }
}

// One shot: one output, or "all" = output N as N pulses, outputs one after another.
async function shot(out, width, run) {
  if (out !== "all") return ttl(`PULSE ${out} ${width}`);
  for (let ch = 1; ch <= 4; ch++) {
    for (let k = 0; k < ch; k++) {
      if (run !== T.run) return null;
      const j = await ttl(`PULSE ${ch} ${width}`);
      if (!j.good) return j;
      await sleep(width + GAP_MS);
    }
    await sleep(2 * GAP_MS);
  }
  return { good: true };
}

async function fire() {
  if (T.busy) return;
  const out = $("#ttl-out").value;
  const width = parseInt($("#ttl-width").value, 10);
  const times = Math.max(1, parseInt($("#ttl-times").value, 10) || 1);
  const every = Math.max(100, (parseFloat($("#ttl-every").value) || 1) * 1000);
  const run = ++T.run;
  T.busy = true;
  $("#btn-ttl-fire").disabled = true;
  try {
    for (let n = 1; n <= times && run === T.run; n++) {
      $("#ttl-progress").textContent = `${n} / ${times}`;
      const t0 = performance.now();
      const j = await shot(out, width, run);
      if (j && !j.good) break;           // no answer or refused: the log says why
      if (n < times) await sleep(Math.max(0, every - (performance.now() - t0)));
    }
  } catch (e) {
    tlog(e.message, "err");
  } finally {
    T.busy = false;
    $("#btn-ttl-fire").disabled = false;
    $("#ttl-progress").textContent = run === T.run ? "done" : "stopped";
  }
}

async function stop() {
  T.run++;
  try { await ttl("ALLOFF"); } catch (e) { tlog(e.message, "err"); }
}

// ---------------- the page's device tabs
function showTab(name) {
  document.querySelectorAll(".diag-tabs .tab").forEach((t) => {
    const on = t.dataset.tab === name;
    t.classList.toggle("active", on);
    t.setAttribute("aria-selected", on ? "true" : "false");
  });
  document.querySelectorAll(".diag-panel").forEach((p) => p.classList.toggle("hidden", p.id !== `tab-${name}`));
}

document.querySelectorAll(".diag-tabs .tab").forEach((t) => (t.onclick = () => {
  showTab(t.dataset.tab);
  if (t.dataset.tab !== "visual") stopWatch();
}));
try { const a = localStorage.getItem(ADDR_KEY); if (a) $("#ttl-addr").value = a; } catch (e) { /* storage blocked */ }
$("#btn-ttl-check").onclick = ttlCheck;
$("#btn-ttl-fire").onclick = fire;
$("#btn-ttl-stop").onclick = stop;
$("#btn-ttl-alloff").onclick = () => ttl("ALLOFF").catch((e) => tlog(e.message, "err"));
if (location.hash === "#ttl") showTab("ttl");
