// Renders a PNG snapshot of the demo grid through the real Renderer.
// Usage: node scripts/preview.mjs
import { createCanvas } from "canvas";
import { Simulation } from "../src/simulation.js";
import { Renderer } from "../src/renderer.js";
import { gridNetwork } from "../src/network.js";
import fs from "fs";

const net = gridNetwork();
const sim = new Simulation(net, { cars: 48 });
const b = net.bounds;
const o = net.origin;

const dpr = 2;
const W = 1160;
const scale = W / b.w;
const H = Math.round(b.h * scale);

const canvas = createCanvas(W * dpr, H * dpr);
const ctx = canvas.getContext("2d");
ctx.setTransform(dpr * scale, 0, 0, dpr * scale, dpr * (0 - o.x * scale), dpr * (0 - o.y * scale));

const renderer = new Renderer(ctx, net);

// Warm up so traffic spreads out and intersections are busy.
let msgs = [];
for (let i = 0; i < 900; i++) {
  sim.tick();
  if (i >= 897) msgs = msgs.concat(sim.messages.map((m) => ({ ...m })));
}
// Seed pulses slightly in the past so packets sit mid-flight in the still.
renderer.ingest(msgs, 99.9);
renderer.draw(sim.cars, { showMsgs: true }, 100.0);

fs.writeFileSync(new URL("../docs/preview.png", import.meta.url), canvas.toBuffer("image/png"));
console.log(`wrote docs/preview.png (${W}x${H}) cars=${sim.cars.length} pulses=${renderer.pulses.length}`);
