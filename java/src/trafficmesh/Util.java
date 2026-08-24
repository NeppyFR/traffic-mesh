package trafficmesh;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Port of src/util.js (minus {@link Polyline} and {@link Vec2}, which get their own files). */
public final class Util {
  private Util() {}

  public static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  public static <T> T choice(List<T> arr) {
    return arr.get((int) (Rng.random() * arr.size()));
  }

  /**
   * JS coerces 0 and NaN to false, so {@code x || fallback} silently replaces
   * both. Java has no such coercion; several ports below rely on it, so it gets
   * a name.
   */
  public static double orDefault(double v, double fallback) {
    return (v == 0 || Double.isNaN(v)) ? fallback : v;
  }

  /**
   * x**n for non-negative integer n, by squaring. Only multiplications, so the
   * result is identical in JS and Java (unlike Math.pow, which is not required
   * to be correctly rounded in either language).
   */
  public static double ipow(double x, int n) {
    double r = 1;
    double b = x;
    int e = n;
    while (e > 0) {
      if ((e & 1) != 0) r *= b;
      b *= b;
      e >>= 1;
    }
    return r;
  }

  /** Minimum distance between segment p1p2 and segment p3p4 (Ericson, RTCD). */
  public static double segSegDist(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4) {
    Vec2 d1 = Vec2.sub(p2, p1);
    Vec2 d2 = Vec2.sub(p4, p3);
    Vec2 r = Vec2.sub(p1, p3);
    double a = Vec2.dot(d1, d1);
    double e = Vec2.dot(d2, d2);
    double f = Vec2.dot(d2, r);
    final double EPS = 1e-9;
    double s;
    double t;
    if (a <= EPS && e <= EPS) return Vec2.len(r);
    if (a <= EPS) {
      s = 0;
      t = clamp(f / e, 0, 1);
    } else {
      double c = Vec2.dot(d1, r);
      if (e <= EPS) {
        t = 0;
        s = clamp(-c / a, 0, 1);
      } else {
        double b = Vec2.dot(d1, d2);
        double denom = a * e - b * b;
        s = denom != 0 ? clamp((b * f - c * e) / denom, 0, 1) : 0;
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
    Vec2 cp1 = Vec2.add(p1, Vec2.mul(d1, s));
    Vec2 cp2 = Vec2.add(p3, Vec2.mul(d2, t));
    return Vec2.len(Vec2.sub(cp1, cp2));
  }

  /** Quadratic bezier point (used for smooth turns through a box). */
  public static Vec2 bezier(Vec2 p0, Vec2 cp, Vec2 p1, double t) {
    double u = 1 - t;
    return new Vec2(
        u * u * p0.x + 2 * u * t * cp.x + t * t * p1.x,
        u * u * p0.y + 2 * u * t * cp.y + t * t * p1.y);
  }

  public static Vec2 bezierTangent(Vec2 p0, Vec2 cp, Vec2 p1, double t) {
    double u = 1 - t;
    return Vec2.norm(
        new Vec2(
            2 * u * (cp.x - p0.x) + 2 * t * (p1.x - cp.x),
            2 * u * (cp.y - p0.y) + 2 * t * (p1.y - cp.y)));
  }

  /**
   * Offset a centreline polyline to the right of travel by distance d, averaging
   * segment normals at interior vertices so lanes stay parallel around bends.
   */
  public static List<Vec2> offsetRight(List<Vec2> coords, double d) {
    int n = coords.size();
    List<Vec2> out = new ArrayList<>();
    if (n == 1) {
      out.add(new Vec2(coords.get(0).x, coords.get(0).y));
      return out;
    }
    List<Vec2> dir = new ArrayList<>();
    for (int i = 0; i < n - 1; i++) dir.add(Vec2.norm(Vec2.sub(coords.get(i + 1), coords.get(i))));
    for (int i = 0; i < n; i++) {
      Vec2 nrm;
      if (i == 0) {
        nrm = Vec2.perpR(dir.get(0));
      } else if (i == n - 1) {
        nrm = Vec2.perpR(dir.get(n - 2));
      } else {
        Vec2 a = Vec2.perpR(dir.get(i - 1));
        Vec2 b = Vec2.perpR(dir.get(i));
        Vec2 avg = Vec2.norm(Vec2.add(a, b));
        double cos = Math.max(0.35, Vec2.dot(avg, a)); // miter length, capped
        nrm = Vec2.mul(avg, 1 / cos);
      }
      out.add(Vec2.add(coords.get(i), Vec2.mul(nrm, d)));
    }
    return out;
  }

  /** Trim {@code len} metres off the start of a polyline's point list. */
  public static List<Vec2> trimStart(List<Vec2> pts, double len) {
    List<Vec2> out = new ArrayList<>();
    for (Vec2 p : pts) out.add(new Vec2(p.x, p.y));
    double rem = len;
    while (out.size() > 2) {
      double segLen = Vec2.dist(out.get(0), out.get(1));
      if (segLen > rem) {
        out.set(0, Vec2.lerp(out.get(0), out.get(1), rem / segLen));
        return out;
      }
      rem -= segLen;
      out.remove(0);
    }
    double segLen = orDefault(Vec2.dist(out.get(0), out.get(1)), 1);
    out.set(0, Vec2.lerp(out.get(0), out.get(1), Math.min(0.85, rem / segLen)));
    return out;
  }

  /** Trim {@code len} metres off the end of a polyline's point list. */
  public static List<Vec2> trimEnd(List<Vec2> pts, double len) {
    List<Vec2> rev = new ArrayList<>(pts);
    Collections.reverse(rev);
    List<Vec2> r = trimStart(rev, len);
    Collections.reverse(r);
    return r;
  }

  /** Equirectangular projection of lon/lat to local metres about (lat0, lon0). */
  public interface Projector {
    Vec2 project(double lat, double lon);
  }

  public static Projector projector(double lat0, double lon0) {
    final double mPerDegLat = 110574;
    final double mPerDegLon = 111320 * Math.cos((lat0 * Math.PI) / 180);
    return (lat, lon) -> new Vec2((lon - lon0) * mPerDegLon, -(lat - lat0) * mPerDegLat);
  }
}
