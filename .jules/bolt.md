## 2025-05-18 - Precomputed Cumulative Length Lookup for Bezier Arc-Distance & Grouped Snapshot Queries

**Learning:** Numerical integration loops (`arc_distance` on Bezier curves) called inside nested conflict evaluation loops dominate frame time (~70% of tick overhead). Precalculating a 65-entry cumulative arc-length lookup table (`cum_len: [f32; 65]`) upon `BezierCurvePath` construction reduces `arc_distance` queries to O(1) linear interpolation. Combining this with lane- and path-grouped snapshots (`[Vec<ConflictSnap>; 4]` and `[Vec<ConflictSnap>; 8]`) avoids flat O(N) iteration over irrelevant vehicles during leader, oncoming, and crossing evaluations.

**Action:** Whenever path geometry or spatial queries are evaluated repeatedly in simulation loops, precalculate LUTs during path construction and partition spatial/path snapshots upfront before executing interaction loops.
