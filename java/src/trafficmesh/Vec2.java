package trafficmesh;

/**
 * A 2-D point/vector, and the vector helpers that live on {@code V} in
 * src/util.js. Points are immutable; every JS {x, y} object literal becomes a
 * {@code new Vec2(x, y)}.
 *
 * <p>Screen space: +x right, +y down (north = up = -y).
 */
public final class Vec2 {
  public final double x;
  public final double y;

  public Vec2(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public static Vec2 add(Vec2 a, Vec2 b) {
    return new Vec2(a.x + b.x, a.y + b.y);
  }

  public static Vec2 sub(Vec2 a, Vec2 b) {
    return new Vec2(a.x - b.x, a.y - b.y);
  }

  public static Vec2 mul(Vec2 a, double s) {
    return new Vec2(a.x * s, a.y * s);
  }

  /** sqrt(x*x + y*y) rather than Math.hypot — see the note in src/util.js. */
  public static double len(Vec2 a) {
    return Math.sqrt(a.x * a.x + a.y * a.y);
  }

  public static double dist(Vec2 a, Vec2 b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  public static double dot(Vec2 a, Vec2 b) {
    return a.x * b.x + a.y * b.y;
  }

  public static Vec2 norm(Vec2 a) {
    double m = Util.orDefault(Math.sqrt(a.x * a.x + a.y * a.y), 1);
    return new Vec2(a.x / m, a.y / m);
  }

  /** Right-hand perpendicular (y-down). */
  public static Vec2 perpR(Vec2 d) {
    return new Vec2(-d.y, d.x);
  }

  public static Vec2 lerp(Vec2 a, Vec2 b, double t) {
    return new Vec2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
  }

  @Override
  public String toString() {
    return "(" + x + ", " + y + ")";
  }
}
