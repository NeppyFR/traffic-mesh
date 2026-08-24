package trafficmesh;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Port of src/network.js. The road network, built from a generic graph
 * { nodes, ways } so the SAME class backs both the demo grid and a real city
 * imported from OpenStreetMap.
 *
 * <p>Pipeline:
 *
 * <ol>
 *   <li>Contract the graph: nodes touched by !=2 road-segments are "real" nodes
 *       (junctions if degree>=3, endpoints if degree 1). Runs of degree-2 nodes
 *       are mid-road shape points and get merged into a single polyline "chain"
 *       between two real nodes — so curves survive but no spurious intersections
 *       are created.
 *   <li>Each chain becomes two directed, offset "lanes" (one per travel
 *       direction), trimmed back from junctions so cars sit on the road, not in
 *       the conflict box.
 *   <li>Boxes exist only at junctions (degree>=3). Routing is Dijkstra over the
 *       directed lanes; a route is turned into an alternating lane/box step list.
 * </ol>
 *
 * <p>Every {@code Map} that the JS relied on for insertion order is a
 * {@link LinkedHashMap} here — lane creation order feeds {@code randomLane()},
 * so it is load-bearing for determinism, not just tidiness.
 */
public final class Network {

  private static int LANE_SEQ = 1;

  /** Reset the lane-id counter. Only used by tests that need a fresh process-like state. */
  public static void resetLaneSeq() {
    LANE_SEQ = 1;
  }

  // ---- Value types -----------------------------------------------------------

  public static final class Node {
    public final String id;
    public final Vec2 pos;
    public final int deg;
    public final boolean junction;
    public final double box;

    Node(String id, Vec2 pos, int deg, boolean junction, double box) {
      this.id = id;
      this.pos = pos;
      this.deg = deg;
      this.junction = junction;
      this.box = box;
    }
  }

  public static final class Lane {
    public final String id;
    public final String from;
    public final String to;
    public final Polyline poly;
    public final double length;

    Lane(String id, String from, String to, Polyline poly, double length) {
      this.id = id;
      this.from = from;
      this.to = to;
      this.poly = poly;
      this.length = length;
    }
  }

  /** An entry of {@code laneOut}: one outbound lane from a node. */
  public static final class Edge {
    public final String to;
    public final String laneId;
    public final double len;

    Edge(String to, String laneId, double len) {
      this.to = to;
      this.laneId = laneId;
      this.len = len;
    }
  }

  /** The straight-line chord a car sweeps across an intersection box. */
  public static final class Chord {
    public final Vec2 a;
    public final Vec2 b;

    Chord(Vec2 a, Vec2 b) {
      this.a = a;
      this.b = b;
    }
  }

  /**
   * One element of a car's plan. Mirrors the two JS object shapes (a "lane" step
   * and a "box" step) in a single class so the two sources stay easy to diff;
   * {@link #type} says which fields are meaningful.
   */
  public static final class Step {
    public final String type; // "lane" | "box"
    public final double len;

    // type == "lane"
    public final String laneId;
    public final String from;
    public final String to;
    public final Polyline poly;

    // type == "box"
    public final String nodeId;
    public final Vec2 p0;
    public final Vec2 p1;
    public final Vec2 ctrl;
    public final Chord chord;
    public final Vec2 inDir;

    private Step(
        String type,
        double len,
        String laneId,
        String from,
        String to,
        Polyline poly,
        String nodeId,
        Vec2 p0,
        Vec2 p1,
        Vec2 ctrl,
        Chord chord,
        Vec2 inDir) {
      this.type = type;
      this.len = len;
      this.laneId = laneId;
      this.from = from;
      this.to = to;
      this.poly = poly;
      this.nodeId = nodeId;
      this.p0 = p0;
      this.p1 = p1;
      this.ctrl = ctrl;
      this.chord = chord;
      this.inDir = inDir;
    }

    static Step lane(Lane lane) {
      return new Step(
          "lane", lane.length, lane.id, lane.from, lane.to, lane.poly, null, null, null, null,
          null, null);
    }

    static Step box(String nodeId, Vec2 p0, Vec2 p1, Vec2 ctrl, double len, Vec2 inDir) {
      return new Step(
          "box", len, null, null, null, null, nodeId, p0, p1, ctrl, new Chord(p0, p1), inDir);
    }

    public boolean isLane() {
      return "lane".equals(type);
    }

    public boolean isBox() {
      return "box".equals(type);
    }
  }

  /** A plan fragment plus the destination it ends at. */
  public static final class Route {
    public final List<Step> steps;
    public final String destNode;

    Route(List<Step> steps, String destNode) {
      this.steps = steps;
      this.destNode = destNode;
    }
  }

  /** The generic input graph: node id -> position, plus ordered node-id ways. */
  public static final class Graph {
    public final LinkedHashMap<String, Vec2> nodes;
    public final List<List<String>> ways;

    public Graph(LinkedHashMap<String, Vec2> nodes, List<List<String>> ways) {
      this.nodes = nodes;
      this.ways = ways;
    }
  }

  public static final class Bounds {
    public final double w;
    public final double h;

    Bounds(double w, double h) {
      this.w = w;
      this.h = h;
    }
  }

  private static final class Chain {
    final List<String> ids;
    final String a;
    final String b;

    Chain(List<String> ids, String a, String b) {
      this.ids = ids;
      this.a = a;
      this.b = b;
    }
  }

  private static final class Contracted {
    final LinkedHashMap<String, List<String>> adj;
    final List<Chain> chains;

    Contracted(LinkedHashMap<String, List<String>> adj, List<Chain> chains) {
      this.adj = adj;
      this.chains = chains;
    }
  }

  // ---- Graph contraction -----------------------------------------------------

  private static String segKey(String a, String b) {
    return a.compareTo(b) < 0 ? a + "|" + b : b + "|" + a;
  }

  private static int deg(Map<String, List<String>> adj, String id) {
    List<String> ns = adj.get(id);
    return ns != null ? ns.size() : 0;
  }

  /** A "real" node: a junction (deg>=3) or a dead-end (deg 1). */
  private static boolean real(Map<String, List<String>> adj, String id) {
    return deg(adj, id) != 2;
  }

  private static Chain walk(
      Map<String, List<String>> adj, Set<String> walked, String start, String first) {
    String prev = start;
    String cur = first;
    List<String> ids = new ArrayList<>();
    ids.add(start);
    ids.add(first);
    walked.add(segKey(start, first));
    int guard = 0;
    while (!real(adj, cur) && guard++ < 100000) {
      List<String> ns = adj.get(cur);
      String next = ns.get(0).equals(prev) ? (ns.size() > 1 ? ns.get(1) : null) : ns.get(0);
      if (next == null) break;
      walked.add(segKey(cur, next));
      ids.add(next);
      prev = cur;
      cur = next;
      if (cur.equals(start)) break; // closed loop back to origin
    }
    return new Chain(ids, start, cur);
  }

  /** Reduce a raw graph to chains between real nodes. */
  private static Contracted contract(Map<String, Vec2> nodes, List<List<String>> ways) {
    LinkedHashMap<String, List<String>> adj = new LinkedHashMap<>(); // multi-edges allowed
    for (List<String> way : ways) {
      for (int i = 0; i + 1 < way.size(); i++) {
        String a = way.get(i);
        String b = way.get(i + 1);
        if (a.equals(b) || !nodes.containsKey(a) || !nodes.containsKey(b)) continue;
        adj.computeIfAbsent(a, k -> new ArrayList<>()).add(b);
        adj.computeIfAbsent(b, k -> new ArrayList<>()).add(a);
      }
    }

    List<Chain> chains = new ArrayList<>();
    Set<String> walked = new LinkedHashSet<>();

    for (String id : new ArrayList<>(adj.keySet())) {
      if (!real(adj, id)) continue;
      for (String nb : new ArrayList<>(adj.get(id))) {
        if (walked.contains(segKey(id, nb))) continue;
        chains.add(walk(adj, walked, id, nb));
      }
    }
    // Any leftover segments belong to loops with no junction — break arbitrarily.
    for (List<String> way : ways) {
      for (int i = 0; i + 1 < way.size(); i++) {
        String a = way.get(i);
        String b = way.get(i + 1);
        if (!nodes.containsKey(a) || !nodes.containsKey(b) || a.equals(b)) continue;
        if (walked.contains(segKey(a, b))) continue;
        chains.add(walk(adj, walked, a, b));
      }
    }
    return new Contracted(adj, chains);
  }

  // ---- Instance state --------------------------------------------------------

  public final String label;
  public final LinkedHashMap<String, Node> nodes = new LinkedHashMap<>();
  public final LinkedHashMap<String, Lane> lanes = new LinkedHashMap<>();
  public final LinkedHashMap<String, List<Edge>> laneOut = new LinkedHashMap<>();
  public final LinkedHashMap<String, String> laneKey = new LinkedHashMap<>(); // "from>to" -> laneId
  public final List<List<Vec2>> roads = new ArrayList<>(); // centrelines, for drawing
  public List<String> junctions = new ArrayList<>(); // node ids with a conflict box
  public Vec2 origin;
  public final Bounds bounds;

  public Network(Map<String, Vec2> nodes, List<List<String>> ways) {
    this(nodes, ways, "grid");
  }

  public Network(Map<String, Vec2> nodes, List<List<String>> ways, String label) {
    this.label = label;

    Contracted ct = contract(nodes, ways);
    Map<String, List<String>> adj = ct.adj;

    for (Map.Entry<String, Vec2> e : nodes.entrySet()) {
      String id = e.getKey();
      Vec2 nd = e.getValue();
      if (!adj.containsKey(id)) continue; // drop isolated nodes
      int d = deg(adj, id);
      boolean junction = d >= 3;
      this.nodes.put(
          id,
          new Node(
              id,
              new Vec2(nd.x, nd.y),
              d,
              junction,
              junction ? Config.boxBase + Math.min(6, (d - 3) * 2.5) : 0));
    }

    for (Chain ch : ct.chains) {
      List<Vec2> coords = new ArrayList<>();
      for (String id : ch.ids) coords.add(this.nodes.get(id).pos);
      if (coords.size() < 2) continue;
      this.roads.add(coords);
      addLane(ch.a, ch.b, coords);
      List<Vec2> rev = new ArrayList<>(coords);
      Collections.reverse(rev);
      addLane(ch.b, ch.a, rev);
    }

    for (Node node : this.nodes.values()) if (node.junction) this.junctions.add(node.id);

    keepLargestComponent();
    this.bounds = computeBounds();
  }

  public Lane addLane(String fromId, String toId, List<Vec2> coords) {
    Node fromNode = this.nodes.get(fromId);
    Node toNode = this.nodes.get(toId);
    List<Vec2> pts = Util.offsetRight(coords, Config.laneOffset);
    double a = fromNode.junction ? fromNode.box / 2 : 0;
    double b = toNode.junction ? toNode.box / 2 : 0;
    double raw = new Polyline(pts).length;
    if (a + b > raw * 0.8) {
      double k = (raw * 0.8) / (a + b);
      a *= k;
      b *= k;
    }
    if (a > 0.01) pts = Util.trimStart(pts, a);
    if (b > 0.01) pts = Util.trimEnd(pts, b);
    Polyline poly = new Polyline(pts);
    String id = "L" + LANE_SEQ++;
    Lane lane = new Lane(id, fromId, toId, poly, poly.length);
    this.lanes.put(id, lane);
    this.laneOut.computeIfAbsent(fromId, k -> new ArrayList<>()).add(new Edge(toId, id, poly.length));
    this.laneKey.put(fromId + ">" + toId, id);
    return lane;
  }

  /**
   * Drop everything outside the largest strongly-ish connected blob so routing
   * never dead-ends (good hygiene for messy OSM extracts).
   */
  public void keepLargestComponent() {
    Set<String> seen = new LinkedHashSet<>();
    Set<String> best = new LinkedHashSet<>();
    for (String start : new ArrayList<>(this.nodes.keySet())) {
      if (seen.contains(start)) continue;
      Set<String> comp = new LinkedHashSet<>();
      ArrayDeque<String> q = new ArrayDeque<>();
      q.add(start);
      seen.add(start);
      while (!q.isEmpty()) {
        String u = q.poll();
        comp.add(u);
        for (Edge e : this.laneOut.getOrDefault(u, Collections.emptyList())) {
          if (!seen.contains(e.to)) {
            seen.add(e.to);
            q.add(e.to);
          }
        }
        // also traverse inbound so we get an undirected component
        for (Lane lane : this.lanes.values()) {
          if (lane.to.equals(u) && !seen.contains(lane.from)) {
            seen.add(lane.from);
            q.add(lane.from);
          }
        }
      }
      if (comp.size() > best.size()) best = comp;
    }
    if (best.size() == this.nodes.size()) return;
    final Set<String> keep = best;
    for (String id : new ArrayList<>(this.nodes.keySet())) if (!keep.contains(id)) this.nodes.remove(id);
    for (Map.Entry<String, Lane> e : new ArrayList<>(this.lanes.entrySet())) {
      Lane lane = e.getValue();
      if (!keep.contains(lane.from) || !keep.contains(lane.to)) this.lanes.remove(e.getKey());
    }
    for (Map.Entry<String, List<Edge>> e : this.laneOut.entrySet()) {
      List<Edge> filtered = new ArrayList<>();
      for (Edge edge : e.getValue()) {
        if (keep.contains(edge.to) && this.lanes.containsKey(edge.laneId)) filtered.add(edge);
      }
      e.setValue(filtered);
    }
    List<String> js = new ArrayList<>();
    for (String id : this.junctions) if (keep.contains(id)) js.add(id);
    this.junctions = js;
    // (the JS also re-filters this.roads with a predicate that is always true —
    // a deliberate no-op that keeps every drawn road, so there is nothing to do)
  }

  public Bounds computeBounds() {
    double minX = Double.POSITIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    for (Node n : this.nodes.values()) {
      minX = Math.min(minX, n.pos.x);
      minY = Math.min(minY, n.pos.y);
      maxX = Math.max(maxX, n.pos.x);
      maxY = Math.max(maxY, n.pos.y);
    }
    double pad = 14;
    // Shift everything so the world starts at (0,0) with a little padding.
    this.origin = new Vec2(minX - pad, minY - pad);
    return new Bounds(maxX - minX + pad * 2, maxY - minY + pad * 2);
  }

  public Lane laneBetween(String a, String b) {
    String id = this.laneKey.get(a + ">" + b);
    return id != null ? this.lanes.get(id) : null;
  }

  public Lane randomLane() {
    return Util.choice(new ArrayList<>(this.lanes.values()));
  }

  public String randomJunction() {
    return !this.junctions.isEmpty()
        ? Util.choice(this.junctions)
        : Util.choice(new ArrayList<>(this.nodes.keySet()));
  }

  private static final class PQEntry {
    final double d;
    final String u;

    PQEntry(double d, String u) {
      this.d = d;
      this.u = u;
    }
  }

  public List<String> route(String from, String to) {
    return route(from, to, null);
  }

  /**
   * Dijkstra over directed lanes. {@code firstAvoid} bans the very first hop to
   * that node (used to forbid an immediate U-turn when re-routing).
   *
   * <p>The priority queue is a linear-scan list, exactly as in the JS: ties go to
   * the earliest-inserted entry, which is part of the behaviour being matched.
   */
  public List<String> route(String from, String to, String firstAvoid) {
    if (from.equals(to)) {
      List<String> only = new ArrayList<>();
      only.add(from);
      return only;
    }
    Map<String, Double> dist = new LinkedHashMap<>();
    dist.put(from, 0.0);
    Map<String, String> prev = new LinkedHashMap<>();
    prev.put(from, null);
    List<PQEntry> pq = new ArrayList<>();
    pq.add(new PQEntry(0, from));
    while (!pq.isEmpty()) {
      int bi = 0;
      for (int i = 1; i < pq.size(); i++) if (pq.get(i).d < pq.get(bi).d) bi = i;
      PQEntry cur = pq.remove(bi);
      double d = cur.d;
      String u = cur.u;
      if (u.equals(to)) break;
      if (d > dist.getOrDefault(u, Double.POSITIVE_INFINITY)) continue;
      for (Edge e : this.laneOut.getOrDefault(u, Collections.emptyList())) {
        if (u.equals(from) && firstAvoid != null && e.to.equals(firstAvoid)) continue;
        double nd = d + e.len;
        if (nd < dist.getOrDefault(e.to, Double.POSITIVE_INFINITY)) {
          dist.put(e.to, nd);
          prev.put(e.to, u);
          pq.add(new PQEntry(nd, e.to));
        }
      }
    }
    if (!prev.containsKey(to)) return null;
    List<String> path = new ArrayList<>();
    for (String n = to; n != null; n = prev.get(n)) path.add(0, n);
    return path;
  }

  public List<Lane> laneSeq(List<String> nodePath) {
    List<Lane> seq = new ArrayList<>();
    for (int i = 0; i + 1 < nodePath.size(); i++) {
      Lane lane = laneBetween(nodePath.get(i), nodePath.get(i + 1));
      if (lane == null) return null;
      seq.add(lane);
    }
    return seq;
  }

  public Step laneStep(Lane lane) {
    return Step.lane(lane);
  }

  public Step boxStep(String nodeId, Lane prevLane, Lane nextLane) {
    Node node = this.nodes.get(nodeId);
    Vec2 p0 = prevLane.poly.last();
    Vec2 p1 = nextLane.poly.first();
    double len = Math.max(1, Vec2.dist(p0, node.pos) + Vec2.dist(node.pos, p1));
    return Step.box(nodeId, p0, p1, node.pos, len, prevLane.poly.dirAt(prevLane.length));
  }

  /** Weave a lane sequence into an alternating lane/box step list. */
  public List<Step> weave(List<Lane> seq) {
    List<Step> steps = new ArrayList<>();
    steps.add(laneStep(seq.get(0)));
    for (int i = 1; i < seq.size(); i++) {
      steps.add(boxStep(seq.get(i - 1).to, seq.get(i - 1), seq.get(i)));
      steps.add(laneStep(seq.get(i)));
    }
    return steps;
  }

  /** A fresh route: start mid-network on a random lane, drive to a random node. */
  public Route spawnRoute() {
    for (int tries = 0; tries < 40; tries++) {
      Lane first = randomLane();
      String dest = randomJunction();
      if (dest.equals(first.to)) continue;
      List<String> nodePath = route(first.to, dest, first.from);
      if (nodePath == null) nodePath = route(first.to, dest);
      if (nodePath == null || nodePath.size() < 2) continue;
      List<Lane> rest = laneSeq(nodePath);
      if (rest == null) continue;
      List<Lane> all = new ArrayList<>();
      all.add(first);
      all.addAll(rest);
      return new Route(weave(all), dest);
    }
    return null;
  }

  /**
   * Extend a car already driving: append a new leg from {@code prevLane} onward
   * to a new random destination. Returns the appended steps plus the new dest.
   */
  public Route extendRoute(Lane prevLane) {
    for (int tries = 0; tries < 40; tries++) {
      String dest = randomJunction();
      if (dest.equals(prevLane.to)) continue;
      List<String> nodePath = route(prevLane.to, dest, prevLane.from);
      if (nodePath == null) nodePath = route(prevLane.to, dest);
      if (nodePath == null || nodePath.size() < 2) continue;
      List<Lane> seq = laneSeq(nodePath);
      if (seq == null) continue;
      List<Step> steps = new ArrayList<>();
      Lane prev = prevLane;
      for (Lane lane : seq) {
        steps.add(boxStep(prev.to, prev, lane));
        steps.add(laneStep(lane));
        prev = lane;
      }
      return new Route(steps, dest);
    }
    return null;
  }

  // ---- Map sources -----------------------------------------------------------

  /**
   * Demo grid, emitted as a { nodes, ways } graph and run through the same
   * contraction pipeline as real map data.
   */
  public static Graph gridGraph() {
    int rows = Config.Grid.rows;
    int cols = Config.Grid.cols;
    double spacing = Config.Grid.spacing;
    LinkedHashMap<String, Vec2> nodes = new LinkedHashMap<>();
    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < cols; c++) {
        nodes.put(gridId(r, c), new Vec2(c * spacing, r * spacing));
      }
    }
    List<List<String>> ways = new ArrayList<>();
    for (int r = 0; r < rows; r++) {
      for (int c = 0; c + 1 < cols; c++) ways.add(List.of(gridId(r, c), gridId(r, c + 1)));
    }
    for (int c = 0; c < cols; c++) {
      for (int r = 0; r + 1 < rows; r++) ways.add(List.of(gridId(r, c), gridId(r + 1, c)));
    }
    return new Graph(nodes, ways);
  }

  private static String gridId(int r, int c) {
    return r + "," + c;
  }

  public static Network gridNetwork() {
    Graph g = gridGraph();
    return new Network(g.nodes, g.ways, "demo grid");
  }
}
