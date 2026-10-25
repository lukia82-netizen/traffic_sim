## 2025-05-18 - Precalculated Bezier Arc-Length and Dual Bucketing Snapshots

**Learning:** Repeated numerical integration for Bezier `arc_distance` during conflict evaluations creates a heavy bottleneck in the simulation tick loop. Precalculating a 65-sample cumulative arc-length lookup table on Bezier path construction reduces `arc_distance` and `path_length` to $O(1)$ linear interpolation (yielding a ~5.5x–6.4x speedup). Additionally, when bucketing vehicle snapshots, bucketing by both `lane_id` (`[Vec<ConflictSnap>; 4]`) and `path_id` (`[Vec<ConflictSnap>; 8]`) ensures turning vehicles are not lost from lane leader detection while maintaining $O(N)$ conflict point queries.

**Action:** Always precalculate monotonic curve lookup tables at path instantiation, and partition spatial/state snapshots by both lane and path indices for exact filtering without linear scans.
