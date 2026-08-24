package trafficmesh;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

/**
 * Port of src/simulation.js. Pure logic, no rendering.
 *
 * <p>Decentralized intersection rule (no lights, no server). Every car broadcasts
 * its position and intended path through the next box; every car runs the SAME
 * rule on what it hears:
 *
 * <ol>
 *   <li>Priority = distance to the box (closer wins; ties by id). A car inside
 *       outranks approachers. Strict total order =&gt; no deadlock, no starvation.
 *   <li>Two movements conflict only if their chords across the box cross (tested
 *       geometrically) — opposing straights and most turns don't.
 *   <li>Enter iff no higher-priority CONFLICTING car contends, and the far lane
 *       has room ("don't block the box").
 * </ol>
 *
 * Rear-end safety is handled separately by the Intelligent Driver Model.
 */
public final class Simulation {

  private static final double CONFLICT_DIST = Config.carWidth * 1.4;

  /** Mirrors the JS {@code opts} object: unset fields fall back to {@link Config}. */
  public static final class Options {
    public Integer cars;
    public Double v0;
    public Boolean lightMode;

    public Options cars(int n) {
      this.cars = n;
      return this;
    }

    public Options v0(double v) {
      this.v0 = v;
      return this;
    }

    public Options lightMode(boolean on) {
      this.lightMode = on;
      return this;
    }
  }

  public static final class Stats {
    public final int active;
    public final double avgSpeed; // km/h
    public final double throughput; // trips/min
    public final int waiting;
    public final int totalArrived;

    Stats(int active, double avgSpeed, double throughput, int waiting, int totalArrived) {
      this.active = active;
      this.avgSpeed = avgSpeed;
      this.throughput = throughput;
      this.waiting = waiting;
      this.totalArrived = totalArrived;
    }
  }

  /** A transient V2V event for this frame's rendering. */
  public static final class Msg {
    public final int a;
    public final int b;
    public final String kind;

    Msg(int a, int b, String kind) {
      this.a = a;
      this.b = b;
      this.kind = kind;
    }
  }

  private static final class Contender {
    final Vehicle car;
    final Vehicle.Claim claim;

    Contender(Vehicle car, Vehicle.Claim claim) {
      this.car = car;
      this.claim = claim;
    }
  }

  /** Priority tuple compared by {@link #betterPriority}. */
  private static final class Pri {
    final boolean inside;
    final double dist;
    final int id;

    Pri(boolean inside, double dist, int id) {
      this.inside = inside;
      this.dist = dist;
      this.id = id;
    }
  }

  private static final class Lead {
    final double gap;
    final double v;

    Lead(double gap, double v) {
      this.gap = gap;
      this.v = v;
    }
  }

  public final Network net;
  public final List<Vehicle> cars = new ArrayList<>();
  public double t;
  public int arrivals;
  public final ArrayDeque<Double> arrivalTimes = new ArrayDeque<>();
  public final List<Msg> messages = new ArrayList<>();

  public final int target;
  public final double v0;
  public final boolean lightMode;

  public Simulation(Network net) {
    this(net, new Options());
  }

  public Simulation(Network net, Options opts) {
    this.net = net;
    this.t = 0;
    this.arrivals = 0;

    this.target = opts.cars != null ? opts.cars : Config.cars;
    this.v0 = opts.v0 != null ? opts.v0 : Config.Idm.v0;
    this.lightMode = opts.lightMode != null ? opts.lightMode : false;

    for (int i = 0; i < this.target; i++) trySpawn();
  }

  public boolean trySpawn() {
    Network.Route route = this.net.spawnRoute();
    if (route == null) return false;
    Network.Step st = route.steps.get(0);
    // Only spawn if the head of the entry lane is clear.
    for (Vehicle car : this.cars) {
      Network.Step cs = car.step();
      if (cs.isLane()
          && cs.laneId.equals(st.laneId)
          && car.s < Config.carLength + Config.Idm.s0 + 2) return false;
    }
    this.cars.add(new Vehicle(route));
    return true;
  }

  public boolean movementsConflict(Network.Chord a, Network.Chord b) {
    return Util.segSegDist(a.a, a.b, b.a, b.b) < CONFLICT_DIST;
  }

  private boolean betterPriority(Pri o, Pri c) {
    if (o.inside && !c.inside) return true;
    if (!o.inside && c.inside) return false;
    if (Math.abs(o.dist - c.dist) > 0.4) return o.dist < c.dist;
    return o.id < c.id;
  }

  public boolean lightAllows(String axis) {
    if (axis == null) return true;
    double cycle = Config.Light.cycle;
    double allRed = Config.Light.allRed;
    double p = this.t % cycle;
    double half = cycle / 2;
    if (p < half - allRed) return "NS".equals(axis);
    if (p >= half && p < cycle - allRed) return "EW".equals(axis);
    return false;
  }

  private Lead leaderFor(Vehicle car) {
    Network.Step st = car.step();
    if (!st.isLane()) return null;
    Vehicle best = null;
    double bestGap = Double.POSITIVE_INFINITY;
    for (Vehicle o : this.cars) {
      if (o == car) continue;
      Network.Step os = o.step();
      double gap = Double.POSITIVE_INFINITY;
      if (os.isLane() && os.laneId.equals(st.laneId) && o.s > car.s) {
        gap = o.s - car.s - Config.carLength;
      } else if (os.isBox() && st.laneId.equals(o.prevLaneId)) {
        gap = st.len - car.s + o.s - Config.carLength;
      }
      if (gap < bestGap) {
        bestGap = gap;
        best = o;
      }
    }
    return best != null ? new Lead(bestGap, best.v) : null;
  }

  private boolean mustHold(Vehicle car, Vehicle.Claim claim, List<Contender> contenders) {
    if (this.lightMode && !lightAllows(car.approachAxis())) return true;
    for (Contender o : contenders) {
      if (o.car == car || !o.claim.nodeId.equals(claim.nodeId)) continue;
      if (!movementsConflict(claim.chord, o.claim.chord)) continue;
      Pri oPri = new Pri(o.claim.inside, o.claim.dist, o.car.id);
      Pri cPri = new Pri(claim.inside, claim.dist, car.id);
      if (betterPriority(oPri, cPri)) return true;
    }
    Network.Step exit = car.exitLaneAfterBox();
    if (exit != null) {
      double minAhead = Double.POSITIVE_INFINITY;
      for (Vehicle o : this.cars) {
        if (o == car) continue;
        Network.Step os = o.step();
        if (os.isLane() && os.laneId.equals(exit.laneId)) minAhead = Math.min(minAhead, o.s);
      }
      if (minAhead < Config.carLength + Config.Idm.s0) return true;
    }
    return false;
  }

  public void tick() {
    tick(Config.dt);
  }

  public void tick(double dt) {
    this.t += dt;
    this.messages.clear();

    // --- Broadcast phase: collect intersection claims within range. ---
    List<Contender> contenders = new ArrayList<>();
    for (Vehicle car : this.cars) {
      Vehicle.Claim claim = car.claim();
      if (claim == null) continue;
      if (!claim.inside && claim.dist > Config.radioRange) continue;
      contenders.add(new Contender(car, claim));
    }

    // --- Emit V2V message events (for the packet-hop visuals). ---
    emitMessages(contenders);

    // --- Decision + kinematics. ---
    double v0 = this.v0;
    for (Vehicle car : this.cars) {
      Vehicle.Claim claim = car.claim();
      boolean hold = false;
      if (claim != null && !claim.inside) hold = mustHold(car, claim, contenders);
      car.holding = hold;

      double accel = car.idmAccel(Double.POSITIVE_INFINITY, 0, v0);
      Lead lead = leaderFor(car);
      if (lead != null) accel = Math.min(accel, car.idmAccel(lead.gap, car.v - lead.v, v0));
      if (hold && claim != null) {
        double stopGap = claim.dist - (Config.carLength / 2 + 1);
        accel = Math.min(accel, car.idmAccel(stopGap, car.v, v0));
      }
      accel = Math.max(accel, -Config.maxDecel);
      car.advance(accel, dt, v0);
    }

    // --- Keep every car supplied with road ahead (endless destinations). ---
    for (Vehicle car : this.cars) topUp(car);

    // --- Maintain population. ---
    if (this.cars.size() < this.target) trySpawn();
    else if (this.cars.size() > this.target) this.cars.subList(this.target, this.cars.size()).clear();

    while (!this.arrivalTimes.isEmpty() && this.arrivalTimes.peekFirst() < this.t - 60) {
      this.arrivalTimes.pollFirst();
    }
  }

  /**
   * Ensure at least two steps remain after the current one; when we append a leg,
   * the car has "arrived" at its previous destination.
   */
  public void topUp(Vehicle car) {
    if (car.steps.size() - car.si > 2) {
      compact(car);
      return;
    }
    Network.Step lastLane = null;
    for (int i = car.steps.size() - 1; i >= 0; i--) {
      if (car.steps.get(i).isLane()) {
        lastLane = car.steps.get(i);
        break;
      }
    }
    if (lastLane == null) return;
    Network.Lane prevLane = this.net.lanes.get(lastLane.laneId);
    if (prevLane == null) return;
    Network.Route ext = this.net.extendRoute(prevLane);
    if (ext != null) {
      car.steps = new ArrayList<>(car.steps);
      car.steps.addAll(ext.steps);
      car.destNode = ext.destNode;
      car.legsDone++;
      this.arrivals++;
      this.arrivalTimes.addLast(this.t);
    }
    compact(car);
  }

  /** Trim consumed steps so the list can't grow without bound. */
  public void compact(Vehicle car) {
    if (car.si > 80) {
      int drop = car.si - 8;
      car.steps = new ArrayList<>(car.steps.subList(drop, car.steps.size()));
      car.si -= drop;
    }
  }

  private void emitMessages(List<Contender> contenders) {
    // Real chatter: every pair contending for the same box within range.
    for (int i = 0; i < contenders.size(); i++) {
      for (int j = i + 1; j < contenders.size(); j++) {
        if (!contenders.get(i).claim.nodeId.equals(contenders.get(j).claim.nodeId)) continue;
        this.messages.add(
            new Msg(contenders.get(i).car.id, contenders.get(j).car.id, "coord"));
      }
    }
    // Ambient beacons: a few random in-range links so the mesh feels alive.
    int n = this.cars.size();
    if (n < 2) return;
    int beacons = Math.min(4, n);
    for (int k = 0; k < beacons; k++) {
      Vehicle a = this.cars.get((int) (Rng.random() * n));
      Vehicle b = this.cars.get((int) (Rng.random() * n));
      if (a == b) continue;
      if (Vec2.dist(a.pose().pos, b.pose().pos) <= Config.radioRange) {
        this.messages.add(new Msg(a.id, b.id, "beacon"));
      }
    }
  }

  public Stats stats() {
    double sumV = 0;
    int held = 0;
    for (Vehicle car : this.cars) {
      sumV += car.v;
      if (car.holding) held++;
    }
    int n = this.cars.size();
    double window = Util.orDefault(Math.min(60, this.t), 1);
    return new Stats(
        n,
        n != 0 ? (sumV / n) * 3.6 : 0, // km/h
        (this.arrivalTimes.size() / window) * 60, // trips/min
        held,
        this.arrivals);
  }
}
