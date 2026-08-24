package trafficmesh;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Port of test/smoke.test.js and test/unit.test.js. Plain assertions, no JUnit —
 * the JS side has no test framework either.
 *
 * <p>Run: {@code java -cp out trafficmesh.JavaTest}
 *
 * <p>Node runs each .js test file in its own process, so the module-level
 * counters (RNG state, lane ids, vehicle ids) start fresh for each file. Both
 * suites live in one JVM here, so {@link #resetProcessState()} stands in for
 * that — without it the second suite would inherit the first suite's RNG stream
 * and diverge from its JS counterpart.
 */
public final class JavaTest {

  private static final double CONFLICT_DIST = Config.carWidth * 1.4;

  private static int failures = 0;

  private static void check(boolean cond, String msg) {
    if (!cond) {
      System.out.println("  FAIL: " + msg);
      failures++;
    } else {
      System.out.println("  ok: " + msg);
    }
  }

  private static void resetProcessState() {
    Rng.setSeed(Rng.DEFAULT_SEED);
    Network.resetLaneSeq();
    Vehicle.resetIds();
  }

  public static void main(String[] args) {
    smokeSuite();
    unitSuite();

    System.out.println(
        failures == 0 ? "\nAll checks passed." : "\n" + failures + " check(s) failed.");
    System.exit(failures == 0 ? 0 : 1);
  }

  // ---- test/smoke.test.js ----------------------------------------------------

  private static void smokeSuite() {
    resetProcessState();
    System.out.println("=== smoke ===");
    run("V2V mesh (default)", new Simulation.Options().cars(45));
    run("V2V mesh (dense)", new Simulation.Options().cars(120));
    run("Traffic-light mode", new Simulation.Options().cars(60).lightMode(true));
  }

  private static void run(String label, Simulation.Options opts) {
    Network net = Network.gridNetwork();
    Simulation sim = new Simulation(net, opts);
    double worstBox = Double.POSITIVE_INFINITY;
    double worstLane = Double.POSITIVE_INFINITY;
    boolean nan = false;
    int popLow = Integer.MAX_VALUE;

    for (int i = 0; i < 6000; i++) {
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
    System.out.println("\n[" + label + "]");
    System.out.printf(
        Locale.ROOT,
        "  arrivals=%d cars=%d throughput=%.1f/min avgSpeed=%.1fkm/h popLow=%d%n",
        st.totalArrived,
        st.active,
        st.throughput,
        st.avgSpeed,
        popLow);
    System.out.printf(
        Locale.ROOT,
        "  min in-box conflict dist=%s (want none < %.2f)%n",
        fmt(worstBox),
        CONFLICT_DIST);
    System.out.printf(
        Locale.ROOT,
        "  min same-lane gap=%s (want > %.2f)%n",
        fmt(worstLane),
        Config.carLength * 0.6);

    int targetCars = opts.cars != null ? opts.cars : Config.cars;
    check(!nan, "no NaN positions/speeds");
    check(st.totalArrived > 30, "cars keep reaching destinations");
    check(worstBox == Double.POSITIVE_INFINITY, "no conflicting cars share an intersection box");
    check(
        worstLane == Double.POSITIVE_INFINITY || worstLane > Config.carLength * 0.6,
        "no overlapping cars on a lane");
    check(
        popLow >= Math.min(targetCars, 55) * 0.5,
        "grid stays busy (population holds up)");
  }

  // ---- test/unit.test.js -----------------------------------------------------

  private static void unitSuite() {
    resetProcessState();
    System.out.println("\n=== unit ===");

    // --- Polyline arc-length sanity ---
    System.out.println("\n[polyline]");
    {
      Polyline pl =
          new Polyline(List.of(new Vec2(0, 0), new Vec2(3, 0), new Vec2(3, 4)));
      check(Math.abs(pl.length - 7) < 1e-6, "length is sum of segments (7)");
      Vec2 mid = pl.posAt(3);
      check(Math.abs(mid.x - 3) < 1e-6 && Math.abs(mid.y) < 1e-6, "posAt(3) hits the corner");
      Vec2 q = pl.posAt(5);
      check(
          Math.abs(q.x - 3) < 1e-6 && Math.abs(q.y - 2) < 1e-6,
          "posAt(5) is 2 up the second leg");
      List<Vec2> off = Util.offsetRight(List.of(new Vec2(0, 0), new Vec2(10, 0)), 2);
      check(Math.abs(off.get(0).y - 2) < 1e-6, "east-bound lane offsets to +y (right-hand)");
    }

    // --- OSM parse: a tiny synthetic 3-way junction ---
    System.out.println("\n[osm parse]");
    {
      // A "dumbbell": two 3-way junctions (100, 200) joined by a middle street
      // that carries a shape point (9). Each junction also has two dead-end spurs.
      String data =
          """
          { "elements": [
            { "type": "node", "id": 1,   "lat": 40.001, "lon": -73.0 },
            { "type": "node", "id": 2,   "lat": 39.999, "lon": -73.0 },
            { "type": "node", "id": 100, "lat": 40.0,   "lon": -73.0 },
            { "type": "node", "id": 9,   "lat": 40.0,   "lon": -72.999, "tags": {} },
            { "type": "node", "id": 200, "lat": 40.0,   "lon": -72.998 },
            { "type": "node", "id": 3,   "lat": 40.001, "lon": -72.998 },
            { "type": "node", "id": 4,   "lat": 39.999, "lon": -72.998 },
            { "type": "way", "id": 1000, "nodes": [1, 100],      "tags": { "highway": "residential" } },
            { "type": "way", "id": 1001, "nodes": [2, 100],      "tags": { "highway": "residential" } },
            { "type": "way", "id": 1002, "nodes": [100, 9, 200], "tags": { "highway": "tertiary" } },
            { "type": "way", "id": 1003, "nodes": [200, 3],      "tags": { "highway": "residential" } },
            { "type": "way", "id": 1004, "nodes": [200, 4],      "tags": { "highway": "residential" } },
            { "type": "way", "id": 1005, "nodes": [1, 2],        "tags": { "building": "yes" } }
          ] }
          """;
      Network.Graph g =
          Osm.buildGraphFromOverpass(data, new Osm.LatLon(40.0, -72.999));
      check(g.ways.size() == 5, "only the five highway ways are kept");
      check(g.nodes.containsKey("9"), "mid-way shape point is retained for geometry");
      check(!g.nodes.containsKey("999"), "unknown nodes are absent");

      Network net = new Network(g.nodes, g.ways, "test");
      check(net.junctions.size() == 2, "two 3-way junctions detected");
      // 5 roads (2 spurs + mid + 2 spurs) x 2 directions = 10 lanes.
      check(net.lanes.size() == 10, "produces 10 directed lanes (5 roads x 2)");
      Network.Step box =
          net.boxStep("100", net.laneBetween("1", "100"), net.laneBetween("100", "200"));
      check(isFinite(box.len) && box.len > 0, "box step across the junction has positive length");

      // The whole thing should simulate without NaN for a bit.
      Simulation sim = new Simulation(net, new Simulation.Options().cars(10));
      boolean nan = false;
      for (int i = 0; i < 1500; i++) {
        sim.tick();
        for (Vehicle c : sim.cars) {
          Vec2 p = c.pose().pos;
          if (!isFinite(p.x) || !isFinite(c.v)) nan = true;
        }
      }
      check(!nan, "synthetic OSM network simulates cleanly");
      check(sim.arrivals > 0, "cars complete trips on the synthetic network");
    }
  }

  private static String fmt(double v) {
    return v == Double.POSITIVE_INFINITY ? "n/a" : String.format(Locale.ROOT, "%.2f", v);
  }

  private static boolean isFinite(double d) {
    return !Double.isNaN(d) && !Double.isInfinite(d);
  }
}
