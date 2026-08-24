package trafficmesh;

/**
 * Port of src/rng.js — mulberry32, bit-for-bit.
 *
 * <p>Java's {@code int} is 32-bit two's complement with wrapping arithmetic and
 * a {@code >>>} that matches JS exactly, so {@code Math.imul(a, b)} is just
 * {@code a * b} here and the two generators emit the same doubles forever.
 */
public final class Rng {
  public static final int DEFAULT_SEED = 1337;

  private static int state = DEFAULT_SEED;

  private Rng() {}

  public static void setSeed(int seed) {
    state = seed;
  }

  /** Uniform double in [0, 1). */
  public static double random() {
    state = state + 0x6d2b79f5;
    int t = (state ^ (state >>> 15)) * (1 | state);
    t = (t + ((t ^ (t >>> 7)) * (61 | t))) ^ t;
    return ((t ^ (t >>> 14)) & 0xFFFFFFFFL) / 4294967296.0;
  }
}
