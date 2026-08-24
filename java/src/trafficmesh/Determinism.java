package trafficmesh;

import java.util.Locale;

/**
 * Cross-language determinism probe (Java side). Its pair is
 * java/determinism.js, which prints the identical format. See that file for the
 * diff recipe.
 *
 * <p>Positions and speeds are dumped as raw IEEE-754 bit patterns rather than
 * rounded decimals, so the diff proves bit-for-bit agreement rather than "close
 * enough".
 */
public final class Determinism {

  private Determinism() {}

  private static String bits(double x) {
    return String.format(Locale.ROOT, "%016x", Double.doubleToRawLongBits(x));
  }

  public static void main(String[] args) {
    // Same scenarios in the same order as test/smoke.test.js, so the shared RNG
    // stream is consumed identically on both sides.
    run("default", new Simulation.Options().cars(45));
    run("dense", new Simulation.Options().cars(120));
    run("lights", new Simulation.Options().cars(60).lightMode(true));
  }

  private static void run(String label, Simulation.Options opts) {
    Network net = Network.gridNetwork();
    Simulation sim = new Simulation(net, opts);
    System.out.printf(
        Locale.ROOT,
        // "\n" rather than "%n": Node emits LF even on Windows, and these two
        // traces are meant to be byte-identical.
        "# %s nodes=%d lanes=%d junctions=%d\n",
        label,
        net.nodes.size(),
        net.lanes.size(),
        net.junctions.size());
    for (int i = 1; i <= 6000; i++) {
      sim.tick();
      if (i % 250 != 0) continue;
      double sx = 0;
      double sy = 0;
      double sv = 0;
      for (Vehicle c : sim.cars) {
        Vec2 p = c.pose().pos;
        sx += p.x;
        sy += p.y;
        sv += c.v;
      }
      System.out.printf(
          Locale.ROOT,
          "%s t=%d cars=%d arr=%d x=%s y=%s v=%s\n",
          label,
          i,
          sim.cars.size(),
          sim.arrivals,
          bits(sx),
          bits(sy),
          bits(sv));
    }
  }
}
