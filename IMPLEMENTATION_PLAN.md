# Traffic sandbox — implementation plan

Status: initial plan, 3 November 2026. Working title: tiny.detour. No implementation or shared service exists yet.

## Direction and constraints

A browser-based, top-down road-building and traffic-management sandbox, visually informed by Trafficity and mechanically informed by Traffic Manager: President Edition (TM:PE). Design road networks, place destinations, generate traffic, observe congestion, and adjust lanes and junctions. All available tools and building types are usable immediately: no money, upgrades, unlocks, progression, scoring requirements, or forced growth. Traffic demand is configurable. A starter scenario makes the sandbox immediately understandable; an empty map remains available.

Single-player and local-first for initial releases. Multiplayer is explicitly deferred. Preserve the option of neighboring square maps without building a backend, imposing simulated shared ownership, or claiming the data model alone solves synchronization.

## Research and visual reference

Trafficity's [official Steam description](https://store.steampowered.com/app/5185750/Trafficity/) describes gridless roads, road grids, individual added lanes, roundabouts, overpasses, customizable signal phases, lane connections and individual vehicles responding to traffic. These are publisher descriptions of an upcoming game, not verified implementation details. The [developer site](https://onramp.games/) includes six screenshots and a trailer. [TM:PE documentation](https://doc.tmpe.me/home.html) describes control over lane usage and junctions; its [feature list](https://github.com/dylanPowers/Cities-Skylines-Traffic-Manager-President-Edition/blob/master/README.md) includes lane connections, arrows, priority signs and timed signals. This plan makes independent design choices, not claims about internal code.

User-provided reference images (research only; do not ship them as game assets):

- [Elevated ramp](references/01-elevated-ramp.png): road preview, elevation, snapping, visible geometry feedback.
- [Roundabout and interchange](references/02-roundabout-interchange.png): scale, hierarchy, road markings, buildings and vehicles.
- [Ramp editor](references/03-ramp-editor.png): curved ramps and grade validation. Ignore the price label.
- [Stacked interchange](references/04-stacked-interchange.png): multi-level crossings, shadows and queues.
- [Cloverleaf](references/05-cloverleaf.png): loops, merging lanes and layered roads. Ignore the price label.

Additional reference galleries: [official screenshot gallery](https://onramp.games/), [Steam screenshots and gameplay video](https://store.steampowered.com/app/5185750/Trafficity/), and [roundabout/interchange image](https://kotaku.com/igdb/image/upload/t_720p/sc15bzc.jpg) ([source page](https://kotaku.com/games/trafficity)). Images are external reference material, not licensed game assets. Use original artwork and interface in the implementation.

Visual principles: orthographic top-down view; soft pale terrain and water; white roads with thin outlines and lane markings; small readable colored cars; green residential and blue commercial buildings as useful reference cues; visually distinct bridges using shadows and layering; uncluttered tool overlays. Confirm colors and sizes through actual playtesting rather than tracing existing assets.

## Sandbox interactions

- Pan, zoom, pause, resume and change simulation speed; mouse/keyboard first, touch support as a later milestone.
- Draw straight and curved free-angle roads. Snap to endpoints and intersections; preview geometry and reject impossible placements with a reason. Add/delete segments and undo/redo.
- Choose road profiles, directions and lane counts. Add ramps and extra lanes; edit turns and lane connections. Select and inspect roads, junctions, buildings and vehicles.
- Set priority/yield and stop rules, lane arrows and signal phases. Offer sensible defaults and editable settings. Visualize the permitted movements and actual queues.
- Place/remove homes, workplaces, shops and industrial destinations freely. Configure trip rates and traffic mix, including external edge traffic, without an economy. Disconnected buildings clearly show a warning.
- Place and edit roundabouts, grade-separated crossings and bridges after the base graph is stable. All features are free and available as they are implemented; milestones describe development order, never player unlocks.
- Inspect queue, throughput, travel times and a congestion overlay; these are diagnostic measurements, not objectives.

## Square maps and future generation

Use one square coordinate space, initial extent 1024 × 1024 world units (tunable). Viewport can be any aspect ratio. First deliver a handcrafted map in the same versioned format later emitted by the generator, plus a blank variant.

Define canonical edge coordinates: N and S run west to east; E and W run north to south. Each edge has fixed normalized slots (initial proposal: eight interior slots at (i + 0.5)/8). A road port records slot, width/profile, lane directions, elevation and external demand; a water port records slot and opening width. The exact count and width rules need testing. Water may occupy adjacent slots if both edges agree. Match neighboring edges using a canonical boundary ID and invert inbound/outbound road direction as seen from the other tile. Port geometry must end at the boundary with a compatible tangent and width. A tile can use only a subset of slots.

For now, edge roads connect to configurable off-map trip sources/sinks; there is no adjacent-map simulation. An authored scenario supplies water shapes, terrain, road graph, buildings, connection metadata, and initial demand. Validate ports against boundary geometry and check that key origins can reach destinations. Avoid implying that every road crossing water is valid: ground roads must stay on land until a bridge exists.

Later generator: seed -> pick or receive boundary contract -> generate water shapes connecting ports -> place suitable land/features -> create roads and buildings around them -> validate geometry, connectivity and available building space -> retry deterministically with a bounded attempt count or present a clear failure. Generator version and seed are recorded. A pairwise seam test should generate neighbors independently using the same shared boundary specification and verify matching openings, directions and no gaps. Multiplayer authority, conflicts, and cross-map vehicle handoff remain separate future design problems.

## Technical architecture

Static website subpage; TypeScript + Canvas 2D initially, with a DOM-based accessible toolbar and panels. No framework is required. Use fixed-step simulation independent of rendering; pause and speed control alter simulation time, not frame-rate-dependent vehicle physics. Profile on target browsers before introducing workers, WebGL or complex optimization. IndexedDB stores local saves; import/export JSON for backups and debugging. Avoid any cloud dependency for the first build.

Separate modules:

1. World model: square map, terrain, stable object IDs, road centerline curves and profiles, junctions, ports, buildings and editable rules.
2. Geometry and editing: snapping, intersections, curve sampling, clearance, elevation checks, hit testing, validation, undoable commands.
3. Derived lane graph: directed lanes and junction movement connectors generated from editable road data; separate topological connectivity from rendered surface. Support lane-specific rules and elevation from the start in the data model.
4. Simulation: fixed-step demand, pathfinding, vehicles, following, intersection conflict resolution, signals and metrics. Keep an explicit seeded RNG if stochastic behavior is introduced.
5. Presentation: camera, layers, rendering, selection, overlays and accessible controls.
6. Persistence: versioned world and sandbox configuration, migration strategy and local save/export. On load, regenerate derived lane graph and validate references.

Road edits are atomic commands with validation; split segments at same-level crossings and preserve or remap junction settings where possible. Different elevation crossings never connect automatically. When edits invalidate a route, reroute affected vehicles from valid current positions; if impossible, show an explicit stranded state and recovery action instead of silently losing vehicles. Collision/conflict rules must avoid granting incompatible movements simultaneously. Render geometry and simulation topology derive from the same source to avoid visually connected but disconnected roads.

## Milestones and acceptance checks

### 0. Data contract and test fixtures

Define versioned world/edge-port schema and a handcrafted square starter scenario. Implement validators and fixture maps for straight/crossing roads, curves, layered crossings and disconnected destinations. **Accept:** map round-trips through serialization, invalid ports are explained, and same-level versus elevated crossings are unambiguous.

### 1. Buildable road sandbox

Camera, map rendering, straight and curved roads, snap/preview, at-grade crossings, road deletion, undo/redo and local save. Begin with a two-way single-lane-per-direction profile. **Accept:** roads can be drawn at free angles and connected; splits/deletions leave valid topology; saved layouts restore; invalid operations do not corrupt the map.

### 2. Traffic and first usable sandbox

Directed lanes, routing, basic car-following, intersection conflict/priority logic, buildings and edge sources/sinks, configurable trip demand and diagnostics. Fixed scenario includes a deliberately congested junction, alongside a blank sandbox. **Accept:** cars reach valid destinations, queue without overlapping, yield correctly, and a bypass demonstrably alters queue/throughput; changes invalidate or recalculate routes safely. Run deterministic junction and congestion fixtures, and test in an actual browser.

### 3. TM:PE-inspired controls

Lane arrows/connections, priority signs, simple timed lights and visual editing. Design movement conflict validation and guard against conflicting green phases; warn when rules remove every route to a destination. **Accept:** lane restrictions change chosen routes, signals alternate safely, and a player can diagnose and improve the starter junction without rebuilding it. Verify save/reload and road edits preserve or explicitly reset affected settings.

### 4. Flexible infrastructure

Multiple lane profiles, one-way roads, merging, curved ramps, roundabout construction and elevation/bridges. Individual-lane drawing and lane-changing require dedicated design and tests; do not claim multilane realism before they exist. **Accept:** build a small roundabout and a two-level interchange, cars traverse intended paths, ramps obey grade limits, and stacked crossings never create accidental junctions. Roads and bridge layers render correctly at multiple zoom levels.

### 5. Procedural maps and polish

Seeded square-map generation under edge contracts, accessibility, mobile exploration/editing if practical, performance profiling and an onboarding scenario. **Accept:** a documented seed set generates valid maps within bounded attempts; matching neighbors pass seam checks; map saves reproduce their state; performance target is measured on representative desktop and mobile devices rather than assumed.

## Testing and limits

Add automated geometry and graph tests (split, merge, overlap, elevation, seam compatibility), deterministic traffic fixtures (following, conflict, signals, rerouting, destination reachability), and persistence tests (round-trip, malformed data and version changes). Browser checks should cover rendering, input, pause/speed, save/reload and responsive layout. Track vehicle count, tick time, queue length and travel time in reproducible scenarios. Set practical performance budgets after measuring early builds. Visual reference images inform design only; no copying game assets, logos or UI.

## Open decisions to resolve through a prototype

- Exact map size, edge slot count and minimum road curvature/grade.
- Which controls are intuitive enough for beginners and which belong behind an advanced overlay.
- Whether buildings are placed manually, generated, or offered both ways; initial proposal: both authored and freely placeable.
- Desired traffic density and device performance targets.
- How future multi-map sharing and live simulation should work; intentionally not decided here.

## Current implementation note — 3 November 2026

The local prototype has straight roads, sampled curved roads, one-way circular roundabouts, basic lane counts, straight elevation ramps, terrain checks, partial road deletion, and seeded map names/terrain variation. Same-level crossing and grade-separated routing have automated tests. The generator still varies fixed terrain regions; it is not the planned complete procedural map generator. Advanced lane connectors, realistic lane changing, roundabout priority, flexible signal phases, robust vehicle preservation after edits, bridges as detailed structures and the remaining milestone acceptance criteria remain outstanding. See README for controls and current limitations.
