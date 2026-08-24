// All tunable parameters. World units are METRES, time is seconds. The same
// units are used whether the map is the demo grid or a real city pulled from
// OpenStreetMap, so speeds and gaps stay physically meaningful either way.
export const CONFIG = {
  // Demo grid (used as the default map and the offline fallback).
  grid: { rows: 5, cols: 8, spacing: 92 }, // metres between intersections

  // Road / lane geometry.
  laneOffset: 1.9, // lane centre offset right of the road centreline
  roadDrawWidth: 7.6, // drawn asphalt width (~two lanes)
  boxBase: 12, // side length of an intersection conflict box

  carLength: 4.6,
  carWidth: 2.0,

  // Intelligent Driver Model (car-following), realistic-ish values.
  idm: {
    a: 1.7, // max acceleration  (m/s^2)
    b: 2.4, // comfortable brake (m/s^2)
    v0: 13.0, // desired speed    (m/s ≈ 47 km/h) — overridden by UI
    T: 1.3, // time headway      (s)
    s0: 2.2, // standstill gap    (m)
    delta: 4,
  },
  maxDecel: 9, // hard emergency braking cap (m/s^2)

  radioRange: 130, // V2V range (m)
  cars: 45, // target number of vehicles alive at once (overridden by UI)
  dt: 1 / 60,

  // Traffic-light comparison mode (a simple synchronized fixed-time plan).
  light: { cycle: 24, allRed: 2.0 },
};
