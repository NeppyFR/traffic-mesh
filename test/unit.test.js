// Pure unit tests — no network access. Run: node test/unit.test.js
import { buildGraphFromOverpass } from "../src/osm.js";
import { Network } from "../src/network.js";
import { Polyline, offsetRight, V } from "../src/util.js";
import { Simulation } from "../src/simulation.js";

let failures = 0;
const check = (cond, msg) => {
  if (!cond) {
    console.error("  FAIL:", msg);
    failures++;
  } else {
    console.log("  ok:", msg);
  }
};

// --- Polyline arc-length sanity ---
console.log("\n[polyline]");
{
  const pl = new Polyline([
    { x: 0, y: 0 },
    { x: 3, y: 0 },
    { x: 3, y: 4 },
  ]);
  check(Math.abs(pl.length - 7) < 1e-6, "length is sum of segments (7)");
  const mid = pl.posAt(3);
  check(Math.abs(mid.x - 3) < 1e-6 && Math.abs(mid.y) < 1e-6, "posAt(3) hits the corner");
  const q = pl.posAt(5);
  check(Math.abs(q.x - 3) < 1e-6 && Math.abs(q.y - 2) < 1e-6, "posAt(5) is 2 up the second leg");
  const off = offsetRight(
    [
      { x: 0, y: 0 },
      { x: 10, y: 0 },
    ],
    2,
  );
  check(Math.abs(off[0].y - 2) < 1e-6, "east-bound lane offsets to +y (right-hand)");
}

// --- OSM parse: a tiny synthetic 3-way junction ---
console.log("\n[osm parse]");
{
  // A "dumbbell": two 3-way junctions (100, 200) joined by a middle street
  // that carries a shape point (9). Each junction also has two dead-end spurs.
  const data = {
    elements: [
      { type: "node", id: 1, lat: 40.001, lon: -73.0 },
      { type: "node", id: 2, lat: 39.999, lon: -73.0 },
      { type: "node", id: 100, lat: 40.0, lon: -73.0 },
      { type: "node", id: 9, lat: 40.0, lon: -72.999, tags: {} }, // shape point
      { type: "node", id: 200, lat: 40.0, lon: -72.998 },
      { type: "node", id: 3, lat: 40.001, lon: -72.998 },
      { type: "node", id: 4, lat: 39.999, lon: -72.998 },
      { type: "way", id: 1000, nodes: [1, 100], tags: { highway: "residential" } },
      { type: "way", id: 1001, nodes: [2, 100], tags: { highway: "residential" } },
      { type: "way", id: 1002, nodes: [100, 9, 200], tags: { highway: "tertiary" } },
      { type: "way", id: 1003, nodes: [200, 3], tags: { highway: "residential" } },
      { type: "way", id: 1004, nodes: [200, 4], tags: { highway: "residential" } },
      { type: "way", id: 1005, nodes: [1, 2], tags: { building: "yes" } }, // ignored
    ],
  };
  const g = buildGraphFromOverpass(data, { lat: 40.0, lon: -72.999 });
  check(g.ways.length === 5, "only the five highway ways are kept");
  check(g.nodes.has("9"), "mid-way shape point is retained for geometry");
  check(!g.nodes.has("999"), "unknown nodes are absent");

  const net = new Network(g.nodes, g.ways, "test");
  check(net.junctions.length === 2, "two 3-way junctions detected");
  // 5 roads (2 spurs + mid + 2 spurs) x 2 directions = 10 lanes.
  check(net.lanes.size === 10, "produces 10 directed lanes (5 roads x 2)");
  const box = net.boxStep("100", net.laneBetween("1", "100"), net.laneBetween("100", "200"));
  check(Number.isFinite(box.len) && box.len > 0, "box step across the junction has positive length");

  // The whole thing should simulate without NaN for a bit.
  const sim = new Simulation(net, { cars: 10 });
  let nan = false;
  for (let i = 0; i < 1500; i++) {
    sim.tick();
    for (const c of sim.cars) {
      const p = c.pose().pos;
      if (!Number.isFinite(p.x) || !Number.isFinite(c.v)) nan = true;
    }
  }
  check(!nan, "synthetic OSM network simulates cleanly");
  check(sim.arrivals > 0, "cars complete trips on the synthetic network");
}

console.log(failures === 0 ? "\nAll unit checks passed." : `\n${failures} unit check(s) failed.`);
process.exit(failures === 0 ? 0 : 1);
