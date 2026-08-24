package trafficmesh;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Headless demo: runs the demo grid for ~6000 ticks (100 simulated seconds at
 * 60 Hz), prints the same figures the browser HUD shows, and re-checks the two
 * safety invariants the JS smoke test asserts — no two conflicting cars sharing
 * an intersection box, and no two cars overlapping on a lane.
 *
 * <p>Run: {@code java -cp out trafficmesh.Main [cars] [--lights]}
 */
public final class Main {

  private static final double CONFLICT_DIST = Config.carWidth * 1.4;
  private static final int TICKS = 6000;

  public static void main(String[] args) {
    int cars = Config.cars;
    boolean lights = false;
    for (String arg : args) {
      if (arg.equals("--lights")) lights = true;
      else cars = Integer.parseInt(arg);
    }

    Network net = Network.gridNetwork();
    Simulation.Options opts = new Simulation.Options().cars(cars);
    if (lights) opts.lightMode(true);
    Simulation sim = new Simulation(net, opts);

    // Printed output stays ASCII: the Windows console codepage mangles non-ASCII.
    System.out.println("traffic-mesh - headless Java run");
    System.out.printf(
        Locale.ROOT,
        "  map=%s  nodes=%d  lanes=%d  junctions=%d%n",
        net.label,
        net.nodes.size(),
        net.lanes.size(),
        net.junctions.size());
    System.out.printf(
        Locale.ROOT,
        "  cars=%d  mode=%s  ticks=%d (%.0fs simulated)%n",
        cars,
        lights ? "traffic lights" : "V2V mesh",
        TICKS,
        TICKS * Config.dt);

    double worstBox = Double.POSITIVE_INFINITY;
    double worstLane = Double.POSITIVE_INFINITY;
    boolean nan = false;
    int popLow = Integer.MAX_VALUE;

    for (int i = 0; i < TICKS; i++) {
      sim.tick();
      if (i % 4 != 0) continue;
      popLow = Math.min(popLow, sim.cars.size());

      for (Vehicle car : sim.cars) {
        Vec2 p = car.pose().pos;
        if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(car.v)) nan = true;
      }

      List<Vehicle> inside = new ArrayList<>();
      for (Vehicle c : sim.cars) if (c.step().isBox()) inside.add(c);
      for (int a = 0; a < inside.size(); a++) {
        for (int b = a + 1; b < inside.size(); b++) {
          Network.Step A = inside.get(a).step();
          Network.Step B = inside.get(b).step();
          if (!A.nodeId.equals(B.nodeId)) continue;
          double d = Util.segSegDist(A.chord.a, A.chord.b, B.chord.a, B.chord.b);
          if (d < CONFLICT_DIST) worstBox = Math.min(worstBox, d);
        }
      }

      Map<String, List<Double>> byLane = new LinkedHashMap<>();
      for (Vehicle car : sim.cars) {
        Network.Step st = car.step();
        if (!st.isLane()) continue;
        byLane.computeIfAbsent(st.laneId, k -> new ArrayList<>()).add(car.s);
      }
      for (List<Double> arr : byLane.values()) {
        arr.sort(Double::compare);
        for (int k = 1; k < arr.size(); k++) {
          worstLane = Math.min(worstLane, arr.get(k) - arr.get(k - 1));
        }
      }
    }

    Simulation.Stats st = sim.stats();
    System.out.println();
    System.out.printf(Locale.ROOT, "  arrivals   %d%n", st.totalArrived);
    System.out.printf(Locale.ROOT, "  active     %d cars (low-water mark %d)%n", st.active, popLow);
    System.out.printf(Locale.ROOT, "  avg speed  %.1f km/h%n", st.avgSpeed);
    System.out.printf(Locale.ROOT, "  throughput %.1f trips/min%n", st.throughput);
    System.out.printf(Locale.ROOT, "  waiting    %d cars holding at a box%n", st.waiting);

    System.out.println();
    int failures = 0;
    failures += report(!nan, "no NaN positions/speeds");
    failures +=
        report(
            worstBox == Double.POSITIVE_INFINITY,
            String.format(
                Locale.ROOT,
                "no conflicting cars share an intersection box (min dist %s, want none < %.2f)",
                worstBox == Double.POSITIVE_INFINITY
                    ? "n/a"
                    : String.format(Locale.ROOT, "%.2f", worstBox),
                CONFLICT_DIST));
    failures +=
        report(
            worstLane == Double.POSITIVE_INFINITY || worstLane > Config.carLength * 0.6,
            String.format(
                Locale.ROOT,
                "no overlapping cars on a lane (min gap %s, want > %.2f)",
                worstLane == Double.POSITIVE_INFINITY
                    ? "n/a"
                    : String.format(Locale.ROOT, "%.2f", worstLane),
                Config.carLength * 0.6));
    failures += report(st.totalArrived > 30, "cars keep reaching destinations");

    System.out.println(failures == 0 ? "\nAll assertions passed." : "\n" + failures + " assertion(s) failed.");
    System.exit(failures == 0 ? 0 : 1);
  }

  private static int report(boolean ok, String msg) {
    System.out.println((ok ? "  ok:   " : "  FAIL: ") + msg);
    return ok ? 0 : 1;
  }

  private static boolean isFinite(double d) {
    return !Double.isNaN(d) && !Double.isInfinite(d);
  }
}
