// Canvas renderer. Draws in world (metre) coordinates; main.js sets up the
// transform (scale + origin offset). Vehicle destinations are intentionally
// never drawn — they're private to each car.
import { CONFIG as C } from "./config.js";
import { V } from "./util.js";

const COLORS = {
  bg: "#0f141b",
  asphalt: "#212a36",
  center: "#3a4658",
  box: "#28313f",
  boxActive: "#33414f",
  pulse: "#37c6b4",
  coord: "#8be3f2",
  hold: "#e0a44a",
};

const PULSE_LIFE = 0.26; // seconds

export class Renderer {
  constructor(ctx, net) {
    this.ctx = ctx;
    this.net = net;
    this.pulses = []; // { a, b, born } — a,b are car ids
    this.pairSeen = new Map(); // "a-b" -> born, to avoid stacking duplicates
  }

  ingest(messages, now) {
    for (const m of messages) {
      const key = m.a < m.b ? m.a + "-" + m.b : m.b + "-" + m.a;
      const prev = this.pairSeen.get(key);
      if (prev !== undefined && now - prev < PULSE_LIFE * 0.6) continue;
      this.pairSeen.set(key, now);
      this.pulses.push({ a: m.a, b: m.b, born: now, kind: m.kind });
      if (this.pulses.length > 260) this.pulses.shift();
    }
    for (let i = this.pulses.length - 1; i >= 0; i--) {
      if (now - this.pulses[i].born > PULSE_LIFE) this.pulses.splice(i, 1);
    }
  }

  draw(cars, opts, now) {
    const { ctx } = this;
    const b = this.net.bounds;
    const o = this.net.origin;
    ctx.fillStyle = COLORS.bg;
    ctx.fillRect(o.x - 40, o.y - 40, b.w + 80, b.h + 80);

    // Asphalt.
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.strokeStyle = COLORS.asphalt;
    ctx.lineWidth = C.roadDrawWidth;
    for (const road of this.net.roads) this.poly(road);

    // Dashed centreline.
    ctx.strokeStyle = COLORS.center;
    ctx.lineWidth = 0.35;
    ctx.setLineDash([3, 5]);
    for (const road of this.net.roads) this.poly(road);
    ctx.setLineDash([]);

    // Intersection boxes.
    const active = new Set();
    for (const car of cars) if (car.step().type === "box") active.add(car.step().nodeId);
    for (const id of this.net.junctions) {
      const n = this.net.nodes.get(id);
      if (!n) continue;
      const s = n.box;
      ctx.fillStyle = active.has(id) ? COLORS.boxActive : COLORS.box;
      this.roundRect(n.pos.x - s / 2, n.pos.y - s / 2, s, s, 2);
      ctx.fill();
    }

    // V2V packet hops.
    if (opts.showMsgs) this.drawPulses(cars, now);

    // Vehicles.
    for (const car of cars) this.drawCar(car);
  }

  drawPulses(cars, now) {
    const { ctx } = this;
    const byId = new Map();
    for (const car of cars) byId.set(car.id, car.pose().pos);
    for (const p of this.pulses) {
      const A = byId.get(p.a);
      const B = byId.get(p.b);
      if (!A || !B) continue;
      const age = (now - p.born) / PULSE_LIFE;
      const col = p.kind === "coord" ? COLORS.coord : COLORS.pulse;
      // faint base line
      ctx.strokeStyle = this.rgba(col, (1 - age) * (p.kind === "coord" ? 0.5 : 0.22));
      ctx.lineWidth = p.kind === "coord" ? 0.7 : 0.4;
      ctx.beginPath();
      ctx.moveTo(A.x, A.y);
      ctx.lineTo(B.x, B.y);
      ctx.stroke();
      // travelling packet
      const t = age;
      const pk = V.lerp(A, B, t);
      ctx.fillStyle = this.rgba(col, 1 - age);
      ctx.beginPath();
      ctx.arc(pk.x, pk.y, p.kind === "coord" ? 1.6 : 1.1, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  drawCar(car) {
    const { ctx } = this;
    const { pos, dir } = car.pose();
    ctx.save();
    ctx.translate(pos.x, pos.y);
    ctx.rotate(Math.atan2(dir.y, dir.x));
    const L = C.carLength;
    const W = C.carWidth;
    if (car.holding) {
      ctx.fillStyle = `hsl(${car.hue},22%,44%)`;
      ctx.strokeStyle = COLORS.hold;
      ctx.lineWidth = 0.5;
    } else {
      ctx.fillStyle = `hsl(${car.hue},60%,58%)`;
      ctx.strokeStyle = "rgba(0,0,0,0.35)";
      ctx.lineWidth = 0.3;
    }
    this.roundRect(-L / 2, -W / 2, L, W, 1);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "rgba(255,255,255,0.4)";
    this.roundRect(L / 2 - 1.1, -W / 2 + 0.4, 0.7, W - 0.8, 0.3);
    ctx.fill();
    ctx.restore();
  }

  poly(pts) {
    const { ctx } = this;
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
    ctx.stroke();
  }

  roundRect(x, y, w, h, r) {
    const { ctx } = this;
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  rgba(hex, a) {
    const n = parseInt(hex.slice(1), 16);
    return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${a.toFixed(3)})`;
  }
}
