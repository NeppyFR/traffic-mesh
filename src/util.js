// Vector, geometry, polyline and geo-projection helpers.
// Points are plain {x, y}. Screen space: +x right, +y down (north = up = -y).
//
// Note on portability: lengths use sqrt(x*x + y*y) rather than Math.hypot, and
// integer powers go through ipow() rather than Math.pow. Neither Math.hypot nor
// Math.pow is required to be correctly rounded, so their last bit can differ
// between JS engines and the JVM; sqrt and multiplication are exact in both.
// That keeps the Java port in java/ bit-identical to this one.

import { random } from "./rng.js";

export const V = {
  add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y }),
  sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y }),
  mul: (a, s) => ({ x: a.x * s, y: a.y * s }),
  len: (a) => Math.sqrt(a.x * a.x + a.y * a.y),
  dist: (a, b) => {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return Math.sqrt(dx * dx + dy * dy);
  },
  dot: (a, b) => a.x * b.x + a.y * b.y,
  norm: (a) => {
    const m = Math.sqrt(a.x * a.x + a.y * a.y) || 1;
    return { x: a.x / m, y: a.y / m };
  },
  perpR: (d) => ({ x: -d.y, y: d.x }), // right-hand perpendicular (y-down)
  lerp: (a, b, t) => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t }),
};

export const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
export const choice = (arr) => arr[(random() * arr.length) | 0];

// x**n for non-negative integer n, by squaring. Only multiplications, so the
// result is identical in JS and Java (unlike Math.pow).
export function ipow(x, n) {
  let r = 1;
  let b = x;
  let e = n;
  while (e > 0) {
    if (e & 1) r *= b;
    b *= b;
    e >>= 1;
  }
  return r;
}

// Minimum distance between segment p1p2 and segment p3p4 (Ericson, RTCD).
export function segSegDist(p1, p2, p3, p4) {
  const d1 = V.sub(p2, p1);
  const d2 = V.sub(p4, p3);
  const r = V.sub(p1, p3);
  const a = V.dot(d1, d1);
  const e = V.dot(d2, d2);
  const f = V.dot(d2, r);
  const EPS = 1e-9;
  let s, t;
  if (a <= EPS && e <= EPS) return V.len(r);
  if (a <= EPS) {
    s = 0;
    t = clamp(f / e, 0, 1);
  } else {
    const c = V.dot(d1, r);
    if (e <= EPS) {
      t = 0;
      s = clamp(-c / a, 0, 1);
    } else {
      const b = V.dot(d1, d2);
      const denom = a * e - b * b;
      s = denom !== 0 ? clamp((b * f - c * e) / denom, 0, 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = clamp(-c / a, 0, 1);
      } else if (t > 1) {
        t = 1;
        s = clamp((b - c) / a, 0, 1);
      }
    }
  }
  const cp1 = V.add(p1, V.mul(d1, s));
  const cp2 = V.add(p3, V.mul(d2, t));
  return V.len(V.sub(cp1, cp2));
}

// Quadratic bezier point + tangent (used for smooth turns through a box).
export function bezier(p0, cp, p1, t) {
  const u = 1 - t;
  return {
    x: u * u * p0.x + 2 * u * t * cp.x + t * t * p1.x,
    y: u * u * p0.y + 2 * u * t * cp.y + t * t * p1.y,
  };
}
export function bezierTangent(p0, cp, p1, t) {
  const u = 1 - t;
  return V.norm({
    x: 2 * u * (cp.x - p0.x) + 2 * t * (p1.x - cp.x),
    y: 2 * u * (cp.y - p0.y) + 2 * t * (p1.y - cp.y),
  });
}

// Offset a centreline polyline to the right of travel by distance d, averaging
// segment normals at interior vertices so lanes stay parallel around bends.
export function offsetRight(coords, d) {
  const n = coords.length;
  if (n === 1) return [{ x: coords[0].x, y: coords[0].y }];
  const dir = [];
  for (let i = 0; i < n - 1; i++) dir.push(V.norm(V.sub(coords[i + 1], coords[i])));
  const out = [];
  for (let i = 0; i < n; i++) {
    let nrm;
    if (i === 0) nrm = V.perpR(dir[0]);
    else if (i === n - 1) nrm = V.perpR(dir[n - 2]);
    else {
      const a = V.perpR(dir[i - 1]);
      const b = V.perpR(dir[i]);
      const avg = V.norm(V.add(a, b));
      const cos = Math.max(0.35, V.dot(avg, a)); // miter length, capped
      nrm = V.mul(avg, 1 / cos);
    }
    out.push(V.add(coords[i], V.mul(nrm, d)));
  }
  return out;
}

// A drivable polyline with arc-length lookup.
export class Polyline {
  constructor(pts) {
    this.pts = pts;
    this.cum = [0];
    for (let i = 1; i < pts.length; i++) {
      this.cum.push(this.cum[i - 1] + V.dist(pts[i - 1], pts[i]));
    }
    this.length = this.cum[this.cum.length - 1] || 0.0001;
  }
  seg(s) {
    const c = this.cum;
    let lo = 0;
    let hi = c.length - 1;
    s = clamp(s, 0, this.length);
    while (lo < hi - 1) {
      const mid = (lo + hi) >> 1;
      if (c[mid] <= s) lo = mid;
      else hi = mid;
    }
    const segLen = c[lo + 1] - c[lo] || 1;
    return { i: lo, t: (s - c[lo]) / segLen };
  }
  posAt(s) {
    const { i, t } = this.seg(s);
    return V.lerp(this.pts[i], this.pts[i + 1], t);
  }
  dirAt(s) {
    const { i } = this.seg(s);
    return V.norm(V.sub(this.pts[i + 1], this.pts[i]));
  }
  get first() {
    return this.pts[0];
  }
  get last() {
    return this.pts[this.pts.length - 1];
  }
}

// Trim `len` metres off the start / end of a polyline's point list.
export function trimStart(pts, len) {
  const out = pts.map((p) => ({ x: p.x, y: p.y }));
  let rem = len;
  while (out.length > 2) {
    const segLen = V.dist(out[0], out[1]);
    if (segLen > rem) {
      out[0] = V.lerp(out[0], out[1], rem / segLen);
      return out;
    }
    rem -= segLen;
    out.shift();
  }
  const segLen = V.dist(out[0], out[1]) || 1;
  out[0] = V.lerp(out[0], out[1], Math.min(0.85, rem / segLen));
  return out;
}
export function trimEnd(pts, len) {
  const r = trimStart([...pts].reverse(), len);
  return r.reverse();
}

// Equirectangular projection of lon/lat to local metres about (lat0, lon0).
export function projector(lat0, lon0) {
  const mPerDegLat = 110574;
  const mPerDegLon = 111320 * Math.cos((lat0 * Math.PI) / 180);
  return (lat, lon) => ({ x: (lon - lon0) * mPerDegLon, y: -(lat - lat0) * mPerDegLat });
}
