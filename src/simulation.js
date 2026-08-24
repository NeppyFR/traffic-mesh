// The simulation engine. Pure logic, no DOM — runs headless in Node for tests.
//
// Decentralized intersection rule (no lights, no server). Every car broadcasts
// its position and intended path through the next box; every car runs the SAME
// rule on what it hears:
//   1. Priority = distance to the box (closer wins; ties by id). A car inside
//      outranks approachers. Strict total order => no deadlock, no starvation.
//   2. Two movements conflict only if their chords across the box cross
//      (tested geometrically) — opposing straights and most turns don't.
//   3. Enter iff no higher-priority CONFLICTING car contends, and the far lane
//      has room ("don't block the box").
// Rear-end safety is handled separately by the Intelligent Driver Model.

import { CONFIG as C } from "./config.js";
import { Vehicle } from "./vehicle.js";
import { V, segSegDist } from "./util.js";
import { random } from "./rng.js";

const CONFLICT_DIST = C.carWidth * 1.4;

export class Simulation {
  constructor(net, opts = {}) {
    this.net = net;
    this.cars = [];
    this.t = 0;
    this.arrivals = 0;
    this.arrivalTimes = [];
    this.messages = []; // transient V2V events for this frame's rendering

    this.target = opts.cars ?? C.cars;
    this.v0 = opts.v0 ?? C.idm.v0;
    this.lightMode = opts.lightMode ?? false;

    for (let i = 0; i < this.target; i++) this.trySpawn();
  }

  trySpawn() {
    const route = this.net.spawnRoute();
    if (!route) return false;
    const st = route.steps[0];
    // Only spawn if the head of the entry lane is clear.
    for (const car of this.cars) {
      const cs = car.step();
      if (cs.type === "lane" && cs.laneId === st.laneId && car.s < C.carLength + C.idm.s0 + 2) return false;
    }
    this.cars.push(new Vehicle(route));
    return true;
  }

  movementsConflict(a, b) {
    return segSegDist(a.a, a.b, b.a, b.b) < CONFLICT_DIST;
  }
  betterPriority(o, c) {
    if (o.inside && !c.inside) return true;
    if (!o.inside && c.inside) return false;
    if (Math.abs(o.dist - c.dist) > 0.4) return o.dist < c.dist;
    return o.id < c.id;
  }

  lightAllows(axis) {
    if (axis === null) return true;
    const { cycle, allRed } = C.light;
    const p = this.t % cycle;
    const half = cycle / 2;
    if (p < half - allRed) return axis === "NS";
    if (p >= half && p < cycle - allRed) return axis === "EW";
    return false;
  }

  leaderFor(car) {
    const st = car.step();
    if (st.type !== "lane") return null;
    let best = null;
    let bestGap = Infinity;
    for (const o of this.cars) {
      if (o === car) continue;
      const os = o.step();
      let gap = Infinity;
      if (os.type === "lane" && os.laneId === st.laneId && o.s > car.s) {
        gap = o.s - car.s - C.carLength;
      } else if (os.type === "box" && o.prevLaneId === st.laneId) {
        gap = st.len - car.s + o.s - C.carLength;
      }
      if (gap < bestGap) {
        bestGap = gap;
        best = o;
      }
    }
    return best ? { gap: bestGap, v: best.v } : null;
  }

  mustHold(car, claim, contenders) {
    if (this.lightMode && !this.lightAllows(car.approachAxis())) return true;
    for (const o of contenders) {
      if (o.car === car || o.claim.nodeId !== claim.nodeId) continue;
      if (!this.movementsConflict(claim.chord, o.claim.chord)) continue;
      const oPri = { inside: o.claim.inside, dist: o.claim.dist, id: o.car.id };
      const cPri = { inside: claim.inside, dist: claim.dist, id: car.id };
      if (this.betterPriority(oPri, cPri)) return true;
    }
    const exit = car.exitLaneAfterBox();
    if (exit) {
      let minAhead = Infinity;
      for (const o of this.cars) {
        if (o === car) continue;
        const os = o.step();
        if (os.type === "lane" && os.laneId === exit.laneId) minAhead = Math.min(minAhead, o.s);
      }
      if (minAhead < C.carLength + C.idm.s0) return true;
    }
    return false;
  }

  tick(dt = C.dt) {
    this.t += dt;
    this.messages.length = 0;

    // --- Broadcast phase: collect intersection claims within range. ---
    const contenders = [];
    for (const car of this.cars) {
      const claim = car.claim();
      if (!claim) continue;
      if (!claim.inside && claim.dist > C.radioRange) continue;
      contenders.push({ car, claim });
    }

    // --- Emit V2V message events (for the packet-hop visuals). ---
    this.emitMessages(contenders);

    // --- Decision + kinematics. ---
    const v0 = this.v0;
    for (const car of this.cars) {
      const claim = car.claim();
      let hold = false;
      if (claim && !claim.inside) hold = this.mustHold(car, claim, contenders);
      car.holding = hold;

      let accel = car.idmAccel(Infinity, 0, v0);
      const lead = this.leaderFor(car);
      if (lead) accel = Math.min(accel, car.idmAccel(lead.gap, car.v - lead.v, v0));
      if (hold && claim) {
        const stopGap = claim.dist - (C.carLength / 2 + 1);
        accel = Math.min(accel, car.idmAccel(stopGap, car.v, v0));
      }
      accel = Math.max(accel, -C.maxDecel);
      car.advance(accel, dt, v0);
    }

    // --- Keep every car supplied with road ahead (endless destinations). ---
    for (const car of this.cars) this.topUp(car);

    // --- Maintain population. ---
    if (this.cars.length < this.target) this.trySpawn();
    else if (this.cars.length > this.target) this.cars.splice(this.target);

    while (this.arrivalTimes.length && this.arrivalTimes[0] < this.t - 60) this.arrivalTimes.shift();
  }

  // Ensure at least two steps remain after the current one; when we append a
  // leg, the car has "arrived" at its previous destination.
  topUp(car) {
    if (car.steps.length - car.si > 2) {
      this.compact(car);
      return;
    }
    const lastLane = [...car.steps].reverse().find((s) => s.type === "lane");
    const prevLane = this.net.lanes.get(lastLane.laneId);
    if (!prevLane) return;
    const ext = this.net.extendRoute(prevLane);
    if (ext) {
      car.steps = car.steps.concat(ext.steps);
      car.destNode = ext.destNode;
      car.legsDone++;
      this.arrivals++;
      this.arrivalTimes.push(this.t);
    }
    this.compact(car);
  }

  // Trim consumed steps so the list can't grow without bound.
  compact(car) {
    if (car.si > 80) {
      const drop = car.si - 8;
      car.steps = car.steps.slice(drop);
      car.si -= drop;
    }
  }

  emitMessages(contenders) {
    // Real chatter: every pair contending for the same box within range.
    for (let i = 0; i < contenders.length; i++) {
      for (let j = i + 1; j < contenders.length; j++) {
        if (contenders[i].claim.nodeId !== contenders[j].claim.nodeId) continue;
        this.messages.push({ a: contenders[i].car.id, b: contenders[j].car.id, kind: "coord" });
      }
    }
    // Ambient beacons: a few random in-range links so the mesh feels alive.
    const n = this.cars.length;
    if (n < 2) return;
    const beacons = Math.min(4, n);
    for (let k = 0; k < beacons; k++) {
      const a = this.cars[(random() * n) | 0];
      const b = this.cars[(random() * n) | 0];
      if (a === b) continue;
      if (V.dist(a.pose().pos, b.pose().pos) <= C.radioRange) {
        this.messages.push({ a: a.id, b: b.id, kind: "beacon" });
      }
    }
  }

  stats() {
    let sumV = 0;
    let held = 0;
    for (const car of this.cars) {
      sumV += car.v;
      if (car.holding) held++;
    }
    const n = this.cars.length;
    const window = Math.min(60, this.t) || 1;
    return {
      active: n,
      avgSpeed: n ? (sumV / n) * 3.6 : 0, // km/h
      throughput: (this.arrivalTimes.length / window) * 60, // trips/min
      waiting: held,
      totalArrived: this.arrivals,
    };
  }
}
