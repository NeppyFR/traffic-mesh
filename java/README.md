# Traffic Mesh — Java port

A standalone Java 17 port of the simulation engine. **Zero dependencies**: no
Maven, no Gradle, no JUnit — just `javac` and `java`, mirroring the JS project's
no-build-step ethos.

Only the headless engine is ported. `renderer.js` and `main.js` are browser-only
and have no Java counterpart; `osm.js` is ported *except* for its live
`fetch` calls — the pure parser comes across so a Java caller can reuse it.

## Compile and run

From the repository root:

```bash
# compile (output lands in java/out/)
javac --release 17 -d java/out java/src/trafficmesh/*.java

# the headless demo: 6000 ticks on the demo grid, then the safety assertions
java -cp java/out trafficmesh.Main

# optional args: car count, and traffic-light mode
java -cp java/out trafficmesh.Main 120
java -cp java/out trafficmesh.Main 60 --lights

# the ported test suites (smoke + unit), 27 checks
java -cp java/out trafficmesh.JavaTest
```

On Windows `cmd`/PowerShell the glob does not expand; use
`javac --release 17 -d java\out java\src\trafficmesh\*.java` (javac expands it
itself) or a `@argfile`.

`Main` exits non-zero if any assertion fails, so it works as a CI smoke check.

## Class map

| Java | JavaScript | Notes |
| --- | --- | --- |
| `Vec2` | `V` in `src/util.js` | `{x, y}` literals become an immutable class; the `V.*` helpers are statics on it |
| `Util` | `src/util.js` | `clamp`, `choice`, `segSegDist`, `bezier`, `offsetRight`, `trimStart/End`, `projector` |
| `Polyline` | `Polyline` in `src/util.js` | arc-length lookup |
| `Config` | `src/config.js` | nested `Grid` / `Idm` / `Light` classes for the nested JS objects |
| `Rng` | `src/rng.js` | mulberry32, bit-for-bit identical |
| `Network` | `src/network.js` | graph contraction, Dijkstra, lanes, boxes, `gridNetwork()` |
| `Vehicle` | `src/vehicle.js` | IDM motion, intersection claims |
| `Simulation` | `src/simulation.js` | the V2V rule, spawning, stats |
| `Osm` | `src/osm.js` | **only** `buildGraphFromOverpass`; `geocode`/`fetchRoads`/`loadCity` are browser-only and deliberately absent |
| `Json` | *(none — `JSON.parse`)* | ~200-line reader so `Osm` can take raw Overpass text without a dependency |
| `Main` | *(none)* | headless demo entry point |
| `JavaTest` | `test/smoke.test.js` + `test/unit.test.js` | plain assertions, no framework |
| `Determinism` | `java/determinism.js` | the cross-language probe below |

Nested types (`Network.Node`, `Network.Lane`, `Network.Step`, `Network.Route`,
`Vehicle.Claim`, `Vehicle.Pose`, `Simulation.Options`, `Simulation.Stats`) stand
in for the anonymous JS object literals. `Network.Step` keeps a single class with
a `type` field — rather than a `LaneStep`/`BoxStep` hierarchy — so it lines up
with the JS object shape when diffing the two sources.

## Determinism: the two engines agree bit-for-bit

`Math.random()` was replaced on **both** sides with the same seeded generator
(mulberry32, default seed `1337`) — `src/rng.js` and `Rng.java`. Java's `int` is
32-bit two's-complement with wrapping arithmetic and a `>>>` that matches JS
exactly, so `Math.imul(a, b)` is simply `a * b` and the two streams never part.

Two further changes were needed on **both** sides, because neither
`Math.hypot` nor `Math.pow` is required to be correctly rounded — V8 and the JVM
are each free to return a different last bit:

- vector length uses `sqrt(x*x + y*y)` instead of `Math.hypot`;
- the IDM's integer powers go through `ipow()` (exponentiation by squaring)
  instead of `Math.pow`.

`sqrt` *is* correctly rounded in both languages, and multiplication is exact, so
after these swaps every arithmetic operation in the engine is bit-reproducible.

### The result

`java/determinism.js` and `Determinism.java` run the same three scenarios
(45 cars, 120 cars, 60 cars with lights) for 6000 ticks each, dumping car count,
arrivals, and the summed positions/speeds every 250 ticks. Positions are printed
as **raw IEEE-754 bit patterns**, so the comparison proves bit equality rather
than "close enough":

```bash
node java/determinism.js                  > js.txt
java -cp java/out trafficmesh.Determinism > java.txt
diff js.txt java.txt
```

The two 75-line traces are **byte-identical** — same SHA-256
(`40c4b19d3bc83cb9bf3fbb4e0b073fd78ea4dfc466a2819806e51625c1c0c55b`) — across all
18,000 ticks. Not merely matching arrival counts: every car's x, y and speed
agrees to the last bit for the whole run.

The summary statistics line up as you'd expect from that:

| Scenario | Arrivals | Avg speed | Throughput |
| --- | --- | --- | --- |
| V2V mesh, 45 cars | 87 | 33.4 km/h | 61.0 /min |
| V2V mesh, 120 cars | 96 | 14.2 km/h | 66.0 /min |
| Traffic lights, 60 cars | 42 | 15.0 km/h | 31.0 /min |

Identical in `node test/smoke.test.js` and `java -cp java/out trafficmesh.JavaTest`.

## JS → Java notes

Traps worth knowing if you edit either side and want them to stay in step:

- **Integer division.** JS `/` is always floating point; Java truncates on ints.
  Anything derived from a count is kept in `double`.
- **Map ordering is load-bearing.** `randomLane()` indexes into
  `[...lanes.values()]`, so lane insertion order changes which car goes where.
  Every JS `Map` that is iterated became a `LinkedHashMap`, not a `HashMap`.
- **`arr[(Math.random() * n) | 0]`** becomes `(int)(Rng.random() * n)` — `| 0`
  and a cast to `int` both truncate toward zero.
- **`??` / `?.`** become explicit null checks; `Simulation.Options` uses boxed
  `Integer`/`Double`/`Boolean` so "unset" is distinguishable from `0`/`false`.
- **`x || fallback`** on a number silently swallows `0` *and* `NaN`. Java has no
  such coercion, so `Util.orDefault(v, fallback)` spells it out. This matters in
  `Polyline` (zero-length segments) and `Simulation.stats()`.
- **Spread/`.reverse()`** copy in the JS idiom used here; the Java equivalents
  copy into a new `ArrayList` before `Collections.reverse` to avoid mutating the
  caller's list.
- **Process-level counters.** Node runs each test file in a fresh process, so
  the RNG state and the lane/vehicle id counters restart per file. Both suites
  share one JVM, so `JavaTest.resetProcessState()` stands in for that.

## License

MIT — see [../LICENSE](../LICENSE).
