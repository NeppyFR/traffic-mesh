package trafficmesh;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Port of the pure half of src/osm.js: Overpass JSON -&gt; the { nodes, ways }
 * graph the {@link Network} understands, projected to local metres.
 *
 * <p>The live {@code geocode} / {@code fetchRoads} / {@code loadCity} calls are
 * deliberately not ported — they are browser-only in the JS. A Java caller
 * supplies its own Overpass response (from a file, an HTTP client, whatever) and
 * hands the parsed JSON to {@link #buildGraphFromOverpass}.
 */
public final class Osm {

  private Osm() {}

  /** Highway classes we treat as drivable, coarse-to-fine. */
  private static final Set<String> DRIVABLE =
      Set.of(
          "motorway",
          "trunk",
          "primary",
          "secondary",
          "tertiary",
          "unclassified",
          "residential",
          "living_street",
          "road",
          "motorway_link",
          "trunk_link",
          "primary_link",
          "secondary_link",
          "tertiary_link");

  public static final class LatLon {
    public final double lat;
    public final double lon;

    public LatLon(double lat, double lon) {
      this.lat = lat;
      this.lon = lon;
    }
  }

  /** Parse raw Overpass JSON text and build the graph in one step. */
  public static Network.Graph buildGraphFromOverpass(String json, LatLon center) {
    return buildGraphFromOverpass(Json.parseObject(json), center);
  }

  /**
   * Pure: parsed Overpass JSON -&gt; { nodes, ways } projected to local metres.
   * A null {@code center} means "use the mean of every node", matching the JS
   * {@code center?.lat === undefined} branch.
   */
  public static Network.Graph buildGraphFromOverpass(Map<String, Object> data, LatLon center) {
    LinkedHashMap<String, double[]> rawNodes = new LinkedHashMap<>(); // id -> { lat, lon }
    List<List<String>> ways = new ArrayList<>();

    List<Object> elements = asList(data.get("elements"));
    for (Object o : elements) {
      Map<String, Object> el = asMap(o);
      if ("node".equals(el.get("type"))) {
        rawNodes.put(idStr(el.get("id")), new double[] {num(el.get("lat")), num(el.get("lon"))});
      }
    }

    double lat0;
    double lon0;
    if (center != null) {
      lat0 = center.lat;
      lon0 = center.lon;
    } else {
      double sLat = 0;
      double sLon = 0;
      int k = 0;
      for (double[] n : rawNodes.values()) {
        sLat += n[0];
        sLon += n[1];
        k++;
      }
      lat0 = k != 0 ? sLat / k : 0;
      lon0 = k != 0 ? sLon / k : 0;
    }

    Util.Projector project = Util.projector(lat0, lon0);
    LinkedHashMap<String, Vec2> nodes = new LinkedHashMap<>();
    Set<String> need = new LinkedHashSet<>();

    for (Object o : elements) {
      Map<String, Object> el = asMap(o);
      if (!"way".equals(el.get("type"))) continue;
      Object rawWayNodes = el.get("nodes");
      if (rawWayNodes == null) continue;
      List<Object> wayNodes = asList(rawWayNodes);
      if (wayNodes.size() < 2) continue;
      Map<String, Object> tags = el.get("tags") == null ? Map.of() : asMap(el.get("tags"));
      Object highway = tags.get("highway");
      if (!(highway instanceof String) || !DRIVABLE.contains(highway)) continue;
      List<String> seq = new ArrayList<>();
      for (Object nid : wayNodes) {
        String id = idStr(nid);
        if (rawNodes.containsKey(id)) seq.add(id);
      }
      if (seq.size() < 2) continue;
      ways.add(seq);
      need.addAll(seq);
    }

    for (String id : need) {
      double[] r = rawNodes.get(id);
      Vec2 p = project.project(r[0], r[1]);
      nodes.put(id, new Vec2(p.x, p.y));
    }
    return new Network.Graph(nodes, ways);
  }

  // ---- JSON shape helpers ----------------------------------------------------

  @SuppressWarnings("unchecked")
  private static List<Object> asList(Object o) {
    if (o == null) return List.of(); // mirrors `data.elements || []`
    if (!(o instanceof List)) throw new IllegalArgumentException("expected a JSON array");
    return (List<Object>) o;
  }

  @SuppressWarnings("unchecked")
  private static Map<String, Object> asMap(Object o) {
    if (!(o instanceof Map)) throw new IllegalArgumentException("expected a JSON object");
    return (Map<String, Object>) o;
  }

  private static double num(Object o) {
    if (o instanceof Number nn) return nn.doubleValue();
    throw new IllegalArgumentException("expected a JSON number, got " + o);
  }

  /** JS {@code String(id)}: an integral JSON number must render as "1", not "1.0". */
  private static String idStr(Object o) {
    if (o instanceof Long l) return l.toString();
    if (o instanceof Double d && d == Math.floor(d) && !d.isInfinite()) {
      return Long.toString(d.longValue());
    }
    return String.valueOf(o);
  }
}
