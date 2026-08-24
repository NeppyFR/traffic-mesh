package trafficmesh;

import java.util.ArrayList;
import java.util.List;

/** A drivable polyline with arc-length lookup. Port of the class in src/util.js. */
public final class Polyline {
  public final List<Vec2> pts;
  public final double[] cum;
  public final double length;

  public Polyline(List<Vec2> pts) {
    this.pts = new ArrayList<>(pts);
    this.cum = new double[pts.size()];
    cum[0] = 0;
    for (int i = 1; i < pts.size(); i++) {
      cum[i] = cum[i - 1] + Vec2.dist(pts.get(i - 1), pts.get(i));
    }
    this.length = Util.orDefault(cum[cum.length - 1], 0.0001);
  }

  /** Segment index plus the 0..1 fraction along it, for arc-length {@code s}. */
  public static final class Seg {
    public final int i;
    public final double t;

    Seg(int i, double t) {
      this.i = i;
      this.t = t;
    }
  }

  public Seg seg(double s) {
    double[] c = cum;
    int lo = 0;
    int hi = c.length - 1;
    s = Util.clamp(s, 0, length);
    while (lo < hi - 1) {
      int mid = (lo + hi) >> 1;
      if (c[mid] <= s) lo = mid;
      else hi = mid;
    }
    double segLen = Util.orDefault(c[lo + 1] - c[lo], 1);
    return new Seg(lo, (s - c[lo]) / segLen);
  }

  public Vec2 posAt(double s) {
    Seg g = seg(s);
    return Vec2.lerp(pts.get(g.i), pts.get(g.i + 1), g.t);
  }

  public Vec2 dirAt(double s) {
    Seg g = seg(s);
    return Vec2.norm(Vec2.sub(pts.get(g.i + 1), pts.get(g.i)));
  }

  public Vec2 first() {
    return pts.get(0);
  }

  public Vec2 last() {
    return pts.get(pts.size() - 1);
  }
}
