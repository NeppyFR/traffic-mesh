package trafficmesh;

/**
 * Port of src/config.js. All tunable parameters. World units are METRES, time
 * is seconds — the same whether the map is the demo grid or a real city pulled
 * from OpenStreetMap, so speeds and gaps stay physically meaningful either way.
 */
public final class Config {
  private Config() {}

  /** Demo grid (used as the default map and the offline fallback). */
  public static final class Grid {
    public static final int rows = 5;
    public static final int cols = 8;
    public static final double spacing = 92; // metres between intersections
  }

  // Road / lane geometry.
  public static final double laneOffset = 1.9; // lane centre offset right of the centreline
  public static final double roadDrawWidth = 7.6; // drawn asphalt width (~two lanes)
  public static final double boxBase = 12; // side length of an intersection conflict box

  public static final double carLength = 4.6;
  public static final double carWidth = 2.0;

  /** Intelligent Driver Model (car-following), realistic-ish values. */
  public static final class Idm {
    public static final double a = 1.7; // max acceleration  (m/s^2)
    public static final double b = 2.4; // comfortable brake (m/s^2)
    public static final double v0 = 13.0; // desired speed   (m/s ~= 47 km/h)
    public static final double T = 1.3; // time headway      (s)
    public static final double s0 = 2.2; // standstill gap   (m)
    public static final int delta = 4;
  }

  public static final double maxDecel = 9; // hard emergency braking cap (m/s^2)

  public static final double radioRange = 130; // V2V range (m)
  public static final int cars = 45; // target number of vehicles alive at once
  public static final double dt = 1.0 / 60.0;

  /** Traffic-light comparison mode (a simple synchronized fixed-time plan). */
  public static final class Light {
    public static final double cycle = 24;
    public static final double allRed = 2.0;
  }
}
