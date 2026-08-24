// Cross-language determinism probe (JavaScript side).
// Its pair is java/src/trafficmesh/Determinism.java, which prints the identical
// format. Run both and diff:
//
//   node java/determinism.js            > /tmp/js.txt
//   java -cp java/out trafficmesh.Determinism > /tmp/java.txt
//   diff /tmp/js.txt /tmp/java.txt
//
// Positions and speeds are dumped as raw IEEE-754 bit patterns rather than
// rounded decimals, so the diff proves bit-for-bit agreement rather than
// "close enough".

import { Simulation } from "../src/simulation.js";
import { gridNetwork } from "../src/network.js";

const buf = new DataView(new ArrayBuffer(8));
const bits = (x) => {
  buf.setFloat64(0, x);
  return buf.getBigUint64(0).toString(16).padStart(16, "0");
};

function run(label, opts) {
  const net = gridNetwork();
  const sim = new Simulation(net, opts);
  console.log(
    `# ${label} nodes=${net.nodes.size} lanes=${net.lanes.size} junctions=${net.junctions.length}`,
  );
  for (let i = 1; i <= 6000; i++) {
    sim.tick();
    if (i % 250 !== 0) continue;
    let sx = 0;
    let sy = 0;
    let sv = 0;
    for (const c of sim.cars) {
      const p = c.pose().pos;
      sx += p.x;
      sy += p.y;
      sv += c.v;
    }
    console.log(
      `${label} t=${i} cars=${sim.cars.length} arr=${sim.arrivals} x=${bits(sx)} y=${bits(sy)} v=${bits(sv)}`,
    );
  }
}

// Same scenarios in the same order as test/smoke.test.js, so the shared RNG
// stream is consumed identically on both sides.
run("default", { cars: 45 });
run("dense", { cars: 120 });
run("lights", { cars: 60, lightMode: true });
