// Headless behaviour checks. Run: node test/smoke.test.js
import { Simulation } from "../src/simulation.js";
import { gridNetwork } from "../src/network.js";
import { CONFIG as C } from "../src/config.js";
import { segSegDist } from "../src/util.js";

const CONFLICT_DIST = C.carWidth * 1.4;
let failures = 0;
const check = (cond, msg) => {
  if (!cond) {
    console.error("  FAIL:", msg);
    failures++;
  }
};

function run(label, opts) {
  const net = gridNetwork();
  const sim = new Simulation(net, opts);
  let worstBox = Infinity;
  let worstLane = Infinity;
  let nan = false;
  let popLow = Infinity;

  for (let i = 0; i < 6000; i++) {
    sim.tick();
    if (i % 4 !== 0) continue;
    popLow = Math.min(popLow, sim.cars.length);

    for (const car of sim.cars) {
      const p = car.pose().pos;
      if (!Number.isFinite(p.x) || !Number.isFinite(p.y) || !Number.isFinite(car.v)) nan = true;
    }

    const inside = sim.cars.filter((c) => c.step().type === "box");
    for (let a = 0; a < inside.length; a++) {
      for (let b = a + 1; b < inside.length; b++) {
        const A = inside[a].step();
        const B = inside[b].step();
        if (A.nodeId !== B.nodeId) continue;
        const d = segSegDist(A.chord.a, A.chord.b, B.chord.a, B.chord.b);
        if (d < CONFLICT_DIST) worstBox = Math.min(worstBox, d);
      }
    }

    const byLane = new Map();
    for (const car of sim.cars) {
      const st = car.step();
      if (st.type !== "lane") continue;
      if (!byLane.has(st.laneId)) byLane.set(st.laneId, []);
      byLane.get(st.laneId).push(car.s);
    }
    for (const arr of byLane.values()) {
      arr.sort((x, y) => x - y);
      for (let k = 1; k < arr.length; k++) worstLane = Math.min(worstLane, arr[k] - arr[k - 1]);
    }
  }

  const st = sim.stats();
  console.log(`\n[${label}]`);
  console.log(`  arrivals=${st.totalArrived} cars=${st.active} throughput=${st.throughput.toFixed(1)}/min avgSpeed=${st.avgSpeed.toFixed(1)}km/h popLow=${popLow}`);
  console.log(`  min in-box conflict dist=${worstBox === Infinity ? "n/a" : worstBox.toFixed(2)} (want none < ${CONFLICT_DIST.toFixed(2)})`);
  console.log(`  min same-lane gap=${worstLane === Infinity ? "n/a" : worstLane.toFixed(2)} (want > ${(C.carLength * 0.6).toFixed(2)})`);

  check(!nan, "no NaN positions/speeds");
  check(st.totalArrived > 30, "cars keep reaching destinations");
  check(worstBox === Infinity, "no conflicting cars share an intersection box");
  check(worstLane === Infinity || worstLane > C.carLength * 0.6, "no overlapping cars on a lane");
  check(popLow >= Math.min(opts.cars ?? C.cars, 55) * 0.5, "grid stays busy (population holds up)");
}

run("V2V mesh (default)", { cars: 45 });
run("V2V mesh (dense)", { cars: 120 });
run("Traffic-light mode", { cars: 60, lightMode: true });

console.log(failures === 0 ? "\nAll checks passed." : `\n${failures} check(s) failed.`);
process.exit(failures === 0 ? 0 : 1);
