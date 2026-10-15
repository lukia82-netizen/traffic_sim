## 2025-05-18 - Precalculated Bezier Arc-Length Lookup Tables

**Learning:** In simulation loops querying curve distances (`arc_distance` and `path_length`), numerical integration sampling loops (10–20 Bezier point evaluations per call) generate significant CPU overhead when executed across vehicle pairs. Storing a precalculated 64-segment cumulative length lookup table (`cum_len: [f32; 65]`) upon path construction reduces distance queries to $O(1)$ linear interpolation lookups while increasing sampling precision.

**Action:** For static trajectory curves, precalculate cumulative arc-length tables during path instantiation to keep hot-path spatial and distance evaluations $O(1)$.
