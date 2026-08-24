// The road network. Built from a generic graph { nodes, ways } so the SAME
// class backs both the demo grid and a real city imported from OpenStreetMap.
//
// Pipeline:
//   1. Contract the graph: nodes touched by !=2 road-segments are "real" nodes
//      (junctions if degree>=3, endpoints if degree 1). Runs of degree-2 nodes
//      are mid-road shape points and get merged into a single polyline "chain"
//      between two real nodes — so curves survive but no spurious intersections
//      are created.
//   2. Each chain becomes two directed, offset "lanes" (one per travel
//      direction), trimmed back from junctions so cars sit on the road, not in
//      the conflict box.
//   3. Boxes exist only at junctions (degree>=3). Routing is Dijkstra over the
//      directed lanes; a route is turned into an alternating lane/box step list.

import { CONFIG as C } from "./config.js";
import { V, choice, offsetRight, Polyline, trimStart, trimEnd } from "./util.js";

let LANE_SEQ = 1;

function segKey(a, b) {
  return a < b ? a + "|" + b : b + "|" + a;
}

// Reduce a raw graph to chains between real nodes.
function contract(nodes, ways) {
  const adj = new Map(); // id -> [neighborId,...]  (multi-edges allowed)
  const add = (a, b) => {
    if (!adj.has(a)) adj.set(a, []);
    adj.get(a).push(b);
  };
  for (const way of ways) {
    for (let i = 0; i + 1 < way.length; i++) {
      const a = way[i];
      const b = way[i + 1];
      if (a === b || !nodes.has(a) || !nodes.has(b)) continue;
      add(a, b);
      add(b, a);
    }
  }
  const deg = (id) => (adj.get(id) ? adj.get(id).length : 0);
  const real = (id) => deg(id) !== 2; // junction (>=3) or dead-end (1)

  const chains = [];
  const walked = new Set();
  const walk = (start, first) => {
    let prev = start;
    let cur = first;
    const ids = [start, first];
    walked.add(segKey(start, first));
    let guard = 0;
    while (!real(cur) && guard++ < 100000) {
      const ns = adj.get(cur);
      const next = ns[0] === prev ? ns[1] : ns[0];
      if (next === undefined) break;
      walked.add(segKey(cur, next));
      ids.push(next);
      prev = cur;
      cur = next;
      if (cur === start) break; // closed loop back to origin
    }
    return { ids, a: start, b: cur };
  };

  for (const id of adj.keys()) {
    if (!real(id)) continue;
    for (const nb of adj.get(id)) {
      if (walked.has(segKey(id, nb))) continue;
      chains.push(walk(id, nb));
    }
  }
  // Any leftover segments belong to loops with no junction — break arbitrarily.
  for (const way of ways) {
    for (let i = 0; i + 1 < way.length; i++) {
      const a = way[i];
      const b = way[i + 1];
      if (!nodes.has(a) || !nodes.has(b) || a === b) continue;
      if (walked.has(segKey(a, b))) continue;
      chains.push(walk(a, b));
    }
  }
  return { adj, chains, deg };
}

export class Network {
  constructor(nodes, ways, label = "grid") {
    this.label = label;
    this.nodes = new Map(); // id -> { id, pos, deg, junction, box }
    this.lanes = new Map(); // laneId -> Lane
    this.laneOut = new Map(); // fromId -> [{ to, laneId, len }]
    this.laneKey = new Map(); // "from>to" -> laneId
    this.roads = []; // centreline point lists, for drawing
    this.junctions = []; // node ids with a conflict box

    const { adj, chains, deg } = contract(nodes, ways);

    for (const [id, nd] of nodes) {
      if (!adj.has(id)) continue; // drop isolated nodes
      const d = deg(id);
      const junction = d >= 3;
      this.nodes.set(id, {
        id,
        pos: { x: nd.x, y: nd.y },
        deg: d,
        junction,
        box: junction ? C.boxBase + Math.min(6, (d - 3) * 2.5) : 0,
      });
    }

    for (const ch of chains) {
      const coords = ch.ids.map((id) => this.nodes.get(id).pos);
      if (coords.length < 2) continue;
      this.roads.push(coords);
      this.addLane(ch.a, ch.b, coords);
      this.addLane(ch.b, ch.a, [...coords].reverse());
    }

    for (const [id, node] of this.nodes) if (node.junction) this.junctions.push(id);

    this.keepLargestComponent();
    this.bounds = this.computeBounds();
  }

  addLane(fromId, toId, coords) {
    const fromNode = this.nodes.get(fromId);
    const toNode = this.nodes.get(toId);
    let pts = offsetRight(coords, C.laneOffset);
    let a = fromNode.junction ? fromNode.box / 2 : 0;
    let b = toNode.junction ? toNode.box / 2 : 0;
    const raw = new Polyline(pts).length;
    if (a + b > raw * 0.8) {
      const k = (raw * 0.8) / (a + b);
      a *= k;
      b *= k;
    }
    if (a > 0.01) pts = trimStart(pts, a);
    if (b > 0.01) pts = trimEnd(pts, b);
    const poly = new Polyline(pts);
    const id = "L" + LANE_SEQ++;
    const lane = { id, from: fromId, to: toId, poly, length: poly.length };
    this.lanes.set(id, lane);
    if (!this.laneOut.has(fromId)) this.laneOut.set(fromId, []);
    this.laneOut.get(fromId).push({ to: toId, laneId: id, len: poly.length });
    this.laneKey.set(fromId + ">" + toId, id);
    return lane;
  }

  // Drop everything outside the largest strongly-ish connected blob so routing
  // never dead-ends (good hygiene for messy OSM extracts).
  keepLargestComponent() {
    const seen = new Set();
    let best = new Set();
    for (const start of this.nodes.keys()) {
      if (seen.has(start)) continue;
      const comp = new Set();
      const q = [start];
      seen.add(start);
      while (q.length) {
        const u = q.shift();
        comp.add(u);
        for (const e of this.laneOut.get(u) || []) {
          if (!seen.has(e.to)) {
            seen.add(e.to);
            q.push(e.to);
          }
        }
        // also traverse inbound so we get an undirected component
        for (const [, lane] of this.lanes) {
          if (lane.to === u && !seen.has(lane.from)) {
            seen.add(lane.from);
            q.push(lane.from);
          }
        }
      }
      if (comp.size > best.size) best = comp;
    }
    if (best.size === this.nodes.size) return;
    for (const id of [...this.nodes.keys()]) if (!best.has(id)) this.nodes.delete(id);
    for (const [id, lane] of [...this.lanes]) {
      if (!best.has(lane.from) || !best.has(lane.to)) this.lanes.delete(id);
    }
    for (const [k, arr] of this.laneOut) {
      this.laneOut.set(
        k,
        arr.filter((e) => best.has(e.to) && this.lanes.has(e.laneId)),
      );
    }
    this.junctions = this.junctions.filter((id) => best.has(id));
    this.roads = this.roads.filter((c) => c.every((p) => true)); // keep all drawn roads
  }

  computeBounds() {
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const n of this.nodes.values()) {
      minX = Math.min(minX, n.pos.x);
      minY = Math.min(minY, n.pos.y);
      maxX = Math.max(maxX, n.pos.x);
      maxY = Math.max(maxY, n.pos.y);
    }
    const pad = 14;
    // Shift everything so the world starts at (0,0) with a little padding.
    this.origin = { x: minX - pad, y: minY - pad };
    return { w: maxX - minX + pad * 2, h: maxY - minY + pad * 2 };
  }

  laneBetween(a, b) {
    const id = this.laneKey.get(a + ">" + b);
    return id ? this.lanes.get(id) : null;
  }

  randomLane() {
    const arr = [...this.lanes.values()];
    return choice(arr);
  }
  randomJunction() {
    return this.junctions.length ? choice(this.junctions) : choice([...this.nodes.keys()]);
  }

  // Dijkstra over directed lanes. `firstAvoid` bans the very first hop to that
  // node (used to forbid an immediate U-turn when re-routing).
  route(from, to, firstAvoid = null) {
    if (from === to) return [from];
    const dist = new Map([[from, 0]]);
    const prev = new Map([[from, null]]);
    const pq = [[0, from]];
    while (pq.length) {
      let bi = 0;
      for (let i = 1; i < pq.length; i++) if (pq[i][0] < pq[bi][0]) bi = i;
      const [d, u] = pq.splice(bi, 1)[0];
      if (u === to) break;
      if (d > (dist.get(u) ?? Infinity)) continue;
      for (const e of this.laneOut.get(u) || []) {
        if (u === from && firstAvoid !== null && e.to === firstAvoid) continue;
        const nd = d + e.len;
        if (nd < (dist.get(e.to) ?? Infinity)) {
          dist.set(e.to, nd);
          prev.set(e.to, u);
          pq.push([nd, e.to]);
        }
      }
    }
    if (!prev.has(to)) return null;
    const path = [];
    for (let n = to; n !== null; n = prev.get(n)) path.unshift(n);
    return path;
  }

  laneSeq(nodePath) {
    const seq = [];
    for (let i = 0; i + 1 < nodePath.length; i++) {
      const lane = this.laneBetween(nodePath[i], nodePath[i + 1]);
      if (!lane) return null;
      seq.push(lane);
    }
    return seq;
  }

  laneStep(lane) {
    return { type: "lane", laneId: lane.id, to: lane.to, from: lane.from, poly: lane.poly, len: lane.length };
  }
  boxStep(nodeId, prevLane, nextLane) {
    const node = this.nodes.get(nodeId);
    const p0 = prevLane.poly.last;
    const p1 = nextLane.poly.first;
    const len = Math.max(1, V.dist(p0, node.pos) + V.dist(node.pos, p1));
    return { type: "box", nodeId, p0, p1, ctrl: node.pos, chord: { a: p0, b: p1 }, len, inDir: prevLane.poly.dirAt(prevLane.length) };
  }

  // Weave a lane sequence into an alternating lane/box step list.
  weave(seq) {
    const steps = [this.laneStep(seq[0])];
    for (let i = 1; i < seq.length; i++) {
      steps.push(this.boxStep(seq[i - 1].to, seq[i - 1], seq[i]));
      steps.push(this.laneStep(seq[i]));
    }
    return steps;
  }

  // A fresh route: start mid-network on a random lane, drive to a random node.
  spawnRoute() {
    for (let tries = 0; tries < 40; tries++) {
      const first = this.randomLane();
      const dest = this.randomJunction();
      if (dest === first.to) continue;
      const nodePath = this.route(first.to, dest, first.from) || this.route(first.to, dest);
      if (!nodePath || nodePath.length < 2) continue;
      const rest = this.laneSeq(nodePath);
      if (!rest) continue;
      return { steps: this.weave([first, ...rest]), destNode: dest };
    }
    return null;
  }

  // Extend a car already driving: append a new leg from `prevLane` onward to a
  // new random destination. Returns { steps: [...appended], destNode }.
  extendRoute(prevLane) {
    for (let tries = 0; tries < 40; tries++) {
      const dest = this.randomJunction();
      if (dest === prevLane.to) continue;
      const nodePath = this.route(prevLane.to, dest, prevLane.from) || this.route(prevLane.to, dest);
      if (!nodePath || nodePath.length < 2) continue;
      const seq = this.laneSeq(nodePath);
      if (!seq) continue;
      const steps = [];
      let prev = prevLane;
      for (const lane of seq) {
        steps.push(this.boxStep(prev.to, prev, lane));
        steps.push(this.laneStep(lane));
        prev = lane;
      }
      return { steps, destNode: dest };
    }
    return null;
  }
}

// ---- Map sources -----------------------------------------------------------

// Demo grid, emitted as a { nodes, ways } graph and run through the same
// contraction pipeline as real map data.
export function gridGraph() {
  const { rows, cols, spacing } = C.grid;
  const nodes = new Map();
  const id = (r, c) => `${r},${c}`;
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      nodes.set(id(r, c), { x: c * spacing, y: r * spacing });
    }
  }
  const ways = [];
  for (let r = 0; r < rows; r++) for (let c = 0; c + 1 < cols; c++) ways.push([id(r, c), id(r, c + 1)]);
  for (let c = 0; c < cols; c++) for (let r = 0; r + 1 < rows; r++) ways.push([id(r, c), id(r + 1, c)]);
  return { nodes, ways };
}

export function gridNetwork() {
  const g = gridGraph();
  return new Network(g.nodes, g.ways, "demo grid");
}
