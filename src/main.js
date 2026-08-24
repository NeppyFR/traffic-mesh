// Bootstraps the app: sizes/fits the canvas, runs the loop, wires the controls
// and the city importer.
import { CONFIG as C } from "./config.js";
import { Simulation } from "./simulation.js";
import { Renderer } from "./renderer.js";
import { gridNetwork } from "./network.js";
import { loadCity } from "./osm.js";

const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d");
const el = (id) => document.getElementById(id);

let net = gridNetwork();
let sim = new Simulation(net);
let renderer = new Renderer(ctx, net);
let running = true;
const opts = { showMsgs: true };

function rebuild(newNet, label) {
  const keep = { cars: sim.target, v0: sim.v0, lightMode: sim.lightMode };
  net = newNet;
  sim = new Simulation(net, keep);
  renderer = new Renderer(ctx, net);
  el("mode-label").textContent = (sim.lightMode ? "Traffic lights · " : "V2V mesh · ") + label;
  resize();
}

// ---- Fit the whole world into the frame (contain + centre). ----
function resize() {
  const b = net.bounds;
  const o = net.origin;
  const cssW = canvas.clientWidth || 900;
  const maxH = Math.min(window.innerHeight * 0.74, 760);
  const scale = Math.min(cssW / b.w, maxH / b.h);
  const cssH = b.h * scale;
  const offX = (cssW - b.w * scale) / 2;
  canvas.style.height = cssH + "px";
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(cssW * dpr);
  canvas.height = Math.round(cssH * dpr);
  ctx.setTransform(dpr * scale, 0, 0, dpr * scale, dpr * (offX - o.x * scale), dpr * (0 - o.y * scale));
}
window.addEventListener("resize", resize);

// ---- Fixed-timestep loop. ----
let acc = 0;
let last = performance.now();
function frame(now) {
  const real = Math.min(0.05, (now - last) / 1000);
  last = now;
  if (running) {
    acc += real;
    let guard = 0;
    while (acc >= C.dt && guard < 8) {
      sim.tick(C.dt);
      renderer.ingest(sim.messages, now / 1000);
      acc -= C.dt;
      guard++;
    }
  }
  renderer.draw(sim.cars, opts, now / 1000);
  updateStats();
  requestAnimationFrame(frame);
}

function updateStats() {
  const s = sim.stats();
  el("stat-active").textContent = s.active;
  el("stat-flow").textContent = s.throughput.toFixed(0);
  el("stat-speed").textContent = s.avgSpeed.toFixed(0);
  el("stat-wait").textContent = s.waiting;
}

// ---- Controls. ----
el("btn-play").addEventListener("click", () => {
  running = !running;
  el("btn-play").textContent = running ? "Pause" : "Play";
});
el("btn-reset").addEventListener("click", () => rebuild(net, net.label));

const cars = el("ctl-cars");
cars.addEventListener("input", () => {
  sim.target = parseInt(cars.value, 10);
  el("val-cars").textContent = sim.target;
});
const speed = el("ctl-speed");
speed.addEventListener("input", () => {
  const kmh = parseInt(speed.value, 10);
  sim.v0 = kmh / 3.6;
  el("val-speed").textContent = kmh;
});
el("ctl-msgs").addEventListener("change", (e) => (opts.showMsgs = e.target.checked));
el("ctl-lights").addEventListener("change", (e) => {
  sim.lightMode = e.target.checked;
  el("mode-label").textContent = (sim.lightMode ? "Traffic lights · " : "V2V mesh · ") + net.label;
});

// ---- City importer. ----
let loading = false;
async function doLoad(query) {
  if (loading || !query.trim()) return;
  loading = true;
  const status = el("city-status");
  el("btn-load").disabled = true;
  status.textContent = "";
  status.className = "city-status working";
  try {
    const newNet = await loadCity(query, { onStatus: (m) => (status.textContent = m) });
    rebuild(newNet, newNet.label);
    status.textContent = newNet.label + " · " + newNet.junctions.length + " intersections";
    status.className = "city-status ok";
  } catch (err) {
    status.textContent = err.message || "Load failed.";
    status.className = "city-status err";
  } finally {
    loading = false;
    el("btn-load").disabled = false;
  }
}
el("btn-load").addEventListener("click", () => doLoad(el("city-input").value));
el("city-input").addEventListener("keydown", (e) => {
  if (e.key === "Enter") doLoad(el("city-input").value);
});
el("btn-grid").addEventListener("click", () => {
  rebuild(gridNetwork(), "demo grid");
  el("city-status").textContent = "";
  el("city-status").className = "city-status";
});
document.querySelectorAll("[data-city]").forEach((chip) =>
  chip.addEventListener("click", () => {
    el("city-input").value = chip.dataset.city;
    doLoad(chip.dataset.city);
  }),
);

// ---- Init control values. ----
cars.value = C.cars;
el("val-cars").textContent = C.cars;
speed.value = Math.round(C.idm.v0 * 3.6);
el("val-speed").textContent = Math.round(C.idm.v0 * 3.6);
el("mode-label").textContent = "V2V mesh · demo grid";

resize();
requestAnimationFrame(frame);
