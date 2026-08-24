// Seeded pseudo-random source (mulberry32) used everywhere the engine needs
// randomness. Math.random() is deliberately avoided: the Java port in java/
// implements the identical generator, so both languages walk the same stream
// and produce bit-identical runs. See java/README.md.

export const DEFAULT_SEED = 1337;

let state = DEFAULT_SEED | 0;

export function setSeed(seed) {
  state = seed | 0;
}

// mulberry32: 32-bit state, uniform double in [0, 1).
export function random() {
  state = (state + 0x6d2b79f5) | 0;
  let t = Math.imul(state ^ (state >>> 15), 1 | state);
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}
