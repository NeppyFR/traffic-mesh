// One vehicle. It walks its step list (lane, box, lane, box, ...), broadcasts
// its intersection intent each tick, and moves with the Intelligent Driver
// Model. It never despawns: the simulation keeps appending new legs so the car
// perpetually drives to a fresh (hidden) destination.

import { CONFIG as C } from "./config.js";
import { V, clamp, bezier, bezierTangent } from "./util.js";

let NEXT_ID = 1;

export class Vehicle {
  constructor(route) {
    this.id = NEXT_ID++;
    this.steps = route.steps;
    this.destNode = route.destNode;
    this.si = 0;
    this.s = 0;
    this.v = 0;
    this.hue = (this.id * 47) % 360;
    this.holding = false;
    this.legsDone = 0; // destinations reached (for trip counting)
    this.prevLaneId = this.steps[0].type === "lane" ? this.steps[0].laneId : null;
  }

  step() {
    return this.steps[this.si];
  }

  pose() {
    const st = this.step();
    if (st.type === "lane") {
      return { pos: st.poly.posAt(this.s), dir: st.poly.dirAt(this.s) };
    }
    const t = clamp(this.s / st.len, 0, 1);
    return { pos: bezier(st.p0, st.ctrl, st.p1, t), dir: bezierTangent(st.p0, st.ctrl, st.p1, t) };
  }

  // What the car is contending for at the next/current intersection box.
  claim() {
    const st = this.step();
    if (st.type === "box") {
      return { nodeId: st.nodeId, chord: st.chord, dist: -(st.len - this.s), inside: true };
    }
    const nxt = this.steps[this.si + 1];
    if (nxt && nxt.type === "box") {
      return { nodeId: nxt.nodeId, chord: nxt.chord, dist: st.len - this.s, inside: false };
    }
    return null;
  }

  approachAxis() {
    const st = this.step();
    let d = null;
    if (st.type === "box") d = st.inDir;
    else if (this.steps[this.si + 1]?.type === "box") d = st.poly.dirAt(this.s);
    if (!d) return null;
    return Math.abs(d.y) > Math.abs(d.x) ? "NS" : "EW";
  }

  // Lane the car will emerge onto after its next box (for "don't block the box").
  exitLaneAfterBox() {
    const nxt = this.steps[this.si + 1];
    if (nxt && nxt.type === "box") {
      const after = this.steps[this.si + 2];
      if (after && after.type === "lane") return after;
    }
    return null;
  }

  idmAccel(gap, dv, v0) {
    const { a, b, T, s0, delta } = C.idm;
    const sStar = s0 + Math.max(0, this.v * T + (this.v * dv) / (2 * Math.sqrt(a * b)));
    const g = Math.max(gap, 0.1);
    return a * (1 - Math.pow(this.v / v0, delta) - Math.pow(sStar / g, 2));
  }

  advance(accel, dt, v0) {
    this.v = clamp(this.v + accel * dt, 0, v0 * 1.05);
    this.s += this.v * dt;
    while (this.s > this.step().len) {
      this.s -= this.step().len;
      this.si++;
      if (this.si >= this.steps.length) {
        // Ran out of plan — clamp at the end; the simulation will top us up.
        this.si = this.steps.length - 1;
        this.s = this.step().len;
        this.v = Math.min(this.v, 0.5);
        return;
      }
      if (this.step().type === "lane") this.prevLaneId = this.step().laneId;
    }
  }
}
