package trafficmesh;

import java.util.ArrayList;
import java.util.List;

/**
 * Port of src/vehicle.js. One vehicle: it walks its step list (lane, box, lane,
 * box, ...), broadcasts its intersection intent each tick, and moves with the
 * Intelligent Driver Model. It never despawns — the simulation keeps appending
 * new legs so the car perpetually drives to a fresh (hidden) destination.
 */
public final class Vehicle {

  private static int NEXT_ID = 1;

  /** Reset the vehicle-id counter. Only used by tests needing a fresh process-like state. */
  public static void resetIds() {
    NEXT_ID = 1;
  }

  /** Position plus heading, as returned by {@link #pose()}. */
  public static final class Pose {
    public final Vec2 pos;
    public final Vec2 dir;

    Pose(Vec2 pos, Vec2 dir) {
      this.pos = pos;
      this.dir = dir;
    }
  }

  /** What the car is contending for at the next/current intersection box. */
  public static final class Claim {
    public final String nodeId;
    public final Network.Chord chord;
    public final double dist;
    public final boolean inside;

    Claim(String nodeId, Network.Chord chord, double dist, boolean inside) {
      this.nodeId = nodeId;
      this.chord = chord;
      this.dist = dist;
      this.inside = inside;
    }
  }

  public final int id;
  public List<Network.Step> steps;
  public String destNode;
  public int si;
  public double s;
  public double v;
  public final int hue;
  public boolean holding;
  public int legsDone; // destinations reached (for trip counting)
  public String prevLaneId;

  public Vehicle(Network.Route route) {
    this.id = NEXT_ID++;
    this.steps = new ArrayList<>(route.steps);
    this.destNode = route.destNode;
    this.si = 0;
    this.s = 0;
    this.v = 0;
    this.hue = (this.id * 47) % 360;
    this.holding = false;
    this.legsDone = 0;
    this.prevLaneId = this.steps.get(0).isLane() ? this.steps.get(0).laneId : null;
  }

  public Network.Step step() {
    return this.steps.get(this.si);
  }

  private Network.Step stepAt(int i) {
    return (i >= 0 && i < this.steps.size()) ? this.steps.get(i) : null;
  }

  public Pose pose() {
    Network.Step st = step();
    if (st.isLane()) {
      return new Pose(st.poly.posAt(this.s), st.poly.dirAt(this.s));
    }
    double t = Util.clamp(this.s / st.len, 0, 1);
    return new Pose(
        Util.bezier(st.p0, st.ctrl, st.p1, t), Util.bezierTangent(st.p0, st.ctrl, st.p1, t));
  }

  public Claim claim() {
    Network.Step st = step();
    if (st.isBox()) {
      return new Claim(st.nodeId, st.chord, -(st.len - this.s), true);
    }
    Network.Step nxt = stepAt(this.si + 1);
    if (nxt != null && nxt.isBox()) {
      return new Claim(nxt.nodeId, nxt.chord, st.len - this.s, false);
    }
    return null;
  }

  /** "NS", "EW", or null when the car is not approaching a box. */
  public String approachAxis() {
    Network.Step st = step();
    Vec2 d = null;
    if (st.isBox()) {
      d = st.inDir;
    } else {
      Network.Step nxt = stepAt(this.si + 1);
      if (nxt != null && nxt.isBox()) d = st.poly.dirAt(this.s);
    }
    if (d == null) return null;
    return Math.abs(d.y) > Math.abs(d.x) ? "NS" : "EW";
  }

  /** Lane the car will emerge onto after its next box (for "don't block the box"). */
  public Network.Step exitLaneAfterBox() {
    Network.Step nxt = stepAt(this.si + 1);
    if (nxt != null && nxt.isBox()) {
      Network.Step after = stepAt(this.si + 2);
      if (after != null && after.isLane()) return after;
    }
    return null;
  }

  public double idmAccel(double gap, double dv, double v0) {
    double a = Config.Idm.a;
    double b = Config.Idm.b;
    double T = Config.Idm.T;
    double s0 = Config.Idm.s0;
    int delta = Config.Idm.delta;
    double sStar = s0 + Math.max(0, this.v * T + (this.v * dv) / (2 * Math.sqrt(a * b)));
    double g = Math.max(gap, 0.1);
    return a * (1 - Util.ipow(this.v / v0, delta) - Util.ipow(sStar / g, 2));
  }

  public void advance(double accel, double dt, double v0) {
    this.v = Util.clamp(this.v + accel * dt, 0, v0 * 1.05);
    this.s += this.v * dt;
    while (this.s > step().len) {
      this.s -= step().len;
      this.si++;
      if (this.si >= this.steps.size()) {
        // Ran out of plan — clamp at the end; the simulation will top us up.
        this.si = this.steps.size() - 1;
        this.s = step().len;
        this.v = Math.min(this.v, 0.5);
        return;
      }
      if (step().isLane()) this.prevLaneId = step().laneId;
    }
  }
}
