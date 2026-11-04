# Intersection marking tool plan

The target is an editable marking system inspired by the Cities: Skylines [Intersection Marking Tool](https://steamcommunity.com/sharedfiles/filedetails/?id=2140418403), whose published features include crosswalks, line grouping, and templates. The first version should be cosmetic: markings must not silently change lane links, signals, pedestrian behavior, or vehicle routing.

## Interaction

1. Add a Markings tool to the build palette. Select a junction at its visible elevation; show the junction surface, approach edges, lane boundaries, and selectable anchor points. Keep the current lane-connection editor separate.
2. Create a line by selecting two anchors, with a live preview; select, adjust, or delete existing markings. Offer solid and dashed lines, color, width, and dash/gap length. Escape cancels, and each completed edit participates in undo.
3. Add stop lines and zebra crosswalks as approach-based objects with adjustable offset, width, stripe length, spacing, and color. Allow the existing automatic junction marking for each approach to be hidden independently to avoid duplicates.
4. Add filled polygons using ordered anchors, solid and striped fills, and a preview that rejects self-intersections. Add optional curved lines and intermediate points after straight editing works reliably.
5. Add named local templates and copy/paste only when geometry compatibility can be shown and the user can preview the result. Preserve unsupported or unmatched anchors with a clear warning rather than silently discarding them.

## Model and geometry

- Store versioned, user-authored marking records in the world save, separate from the generated graph and automatic paint. Use stable marking IDs and a junction reference plus approach references and normalized positions along approach boundaries; do not rely only on transient graph node IDs or screen coordinates. Define a deterministic junction signature and remap it through graph rebuilds, splits, and lane edits. Keep enough geometry to present unresolved markings for repair if remapping is ambiguous.
- Derive approach boundaries and anchor positions from actual road width, lane layout and junction geometry at the selected elevation. Clip paint to the junction surface; prevent markings from bleeding onto a road at another elevation. Geometry is in world units, with screen-space hit targets so editing remains usable at different zoom levels.
- Render generated defaults and authored paint in a predictable order on the same elevation layer as the road surface, below vehicles. Invalidate the static road cache when marking data changes. Preview and selection handles render dynamically above the map.
- Migrate older saves to an empty marking collection without changing their existing appearance. Include marking edits in existing save and undo flows, and handle deleted junctions explicitly. Cosmetic paint is distinct from traffic rules and lane connections.

## Delivery and checks

1. Stabilize junction/approach identity and define surface boundaries, including curves, road splits, ramps and elevation crossings. Test remapping and ambiguous edits first.
2. Ship junction selection and straight solid/dashed line editing, save/reload, undo and deletion. Test anchors, hit testing, clipping, save migration and cache invalidation.
3. Add approach crosswalks and stop lines with default-paint suppression, followed by polygon fills and curves. Test geometry and styles at multiple scales, overlapping layers and degenerate shapes.
4. Add templates and copy/paste with compatibility checks. Test matching, partial failures, recovery and round-trip persistence.
5. Manually exercise the starter, lane lab and elevation crossing lab: edit, undo, reload, modify adjoining roads, delete the junction and inspect overlapping elevated roads. Confirm simulation routing and lane connections remain unchanged, and check rendering performance with many markings.
