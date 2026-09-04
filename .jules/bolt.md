## 2025-03-08 - Precalculated Bezier Cumulative Arc Lengths

**Learning:** Arc distance calculations (`arc_distance` and `path_length`) on Bezier paths are evaluated frequently during conflict resolution and movement updates. Repeating numerical sampling loops (10–20 iterations with square roots) on every call causes significant computational overhead in simulation ticks.

**Action:** Precalculate cumulative arc-length lookup tables (`cum_len: [f32; 65]`) upon curve construction to convert $O(N)$ numerical integration per call into $O(1)$ linear interpolation lookups.
