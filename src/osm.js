// Pulls a real city's road layout from OpenStreetMap and turns it into the
// { nodes, ways } graph the Network understands. The fetch runs in the browser
// (OSM's servers aren't reachable from the build sandbox), so this module keeps
// the network calls thin and the parsing pure + testable.

import { CONFIG as C } from "./config.js";
import { Network } from "./network.js";
import { projector } from "./util.js";

// Highway classes we treat as drivable, coarse-to-fine.
const DRIVABLE = new Set([
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
  "tertiary_link",
]);

// Pure: Overpass JSON -> { nodes, ways } projected to local metres.
export function buildGraphFromOverpass(data, center) {
  const rawNodes = new Map(); // id (string) -> { lat, lon }
  const ways = [];
  for (const el of data.elements || []) {
    if (el.type === "node") rawNodes.set(String(el.id), { lat: el.lat, lon: el.lon });
  }
  let lat0 = center?.lat;
  let lon0 = center?.lon;
  if (lat0 === undefined) {
    let sLat = 0;
    let sLon = 0;
    let k = 0;
    for (const n of rawNodes.values()) {
      sLat += n.lat;
      sLon += n.lon;
      k++;
    }
    lat0 = k ? sLat / k : 0;
    lon0 = k ? sLon / k : 0;
  }
  const project = projector(lat0, lon0);
  const nodes = new Map();
  const need = new Set();
  for (const el of data.elements || []) {
    if (el.type !== "way" || !el.nodes || el.nodes.length < 2) continue;
    const tags = el.tags || {};
    if (!tags.highway || !DRIVABLE.has(tags.highway)) continue;
    const seq = el.nodes.map(String).filter((id) => rawNodes.has(id));
    if (seq.length < 2) continue;
    ways.push(seq);
    for (const id of seq) need.add(id);
  }
  for (const id of need) {
    const r = rawNodes.get(id);
    const p = project(r.lat, r.lon);
    nodes.set(id, { x: p.x, y: p.y });
  }
  return { nodes, ways };
}

async function fetchJSON(url, opts) {
  const res = await fetch(url, opts);
  if (!res.ok) throw new Error("HTTP " + res.status);
  return res.json();
}

// Resolve a place name (or "lat,lon") to a coordinate.
export async function geocode(query) {
  const m = query.trim().match(/^(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)$/);
  if (m) return { lat: parseFloat(m[1]), lon: parseFloat(m[2]), label: query.trim() };
  const url =
    "https://nominatim.openstreetmap.org/search?format=jsonv2&limit=1&q=" +
    encodeURIComponent(query);
  const data = await fetchJSON(url, { headers: { Accept: "application/json" } });
  if (!data.length) throw new Error("Couldn't find “" + query + "”.");
  return { lat: parseFloat(data[0].lat), lon: parseFloat(data[0].lon), label: data[0].display_name };
}

const OVERPASS = "https://overpass-api.de/api/interpreter";

export async function fetchRoads(lat, lon, radius) {
  const filter = "motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|road";
  const q = `[out:json][timeout:30];way["highway"~"^(${filter})(_link)?$"](around:${radius},${lat},${lon});(._;>;);out body;`;
  return fetchJSON(OVERPASS, {
    method: "POST",
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
    body: "data=" + encodeURIComponent(q),
  });
}

// Full flow: name -> Network. `onStatus` reports progress to the UI.
export async function loadCity(query, { radius = 520, onStatus = () => {} } = {}) {
  onStatus("Locating “" + query + "”…");
  const place = await geocode(query);
  onStatus("Downloading roads…");
  const data = await fetchRoads(place.lat, place.lon, radius);
  onStatus("Building the network…");
  const graph = buildGraphFromOverpass(data, { lat: place.lat, lon: place.lon });
  if (graph.nodes.size < 4) throw new Error("No drivable roads found there — try a denser area.");
  const net = new Network(graph.nodes, graph.ways, place.label.split(",")[0]);
  if (net.junctions.length < 2) throw new Error("That area is too sparse to simulate.");
  void C;
  return net;
}
