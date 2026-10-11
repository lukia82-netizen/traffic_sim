## 2025-05-18 - Precalculated Cumulative Arc-Length Table for Bézier Trajectories

**Learning:** Numerical integration loops (e.g. 10–20 sub-segment sampling per call) in trajectory distance calculations (`arc_distance` and `path_length`) create significant computational overhead when queried frequently during simulation frames (e.g. for every vehicle evaluating conflict points). Precalculating a fixed 64-sample cumulative arc-length lookup table during path initialization allows distance queries to run in O(1) via linear interpolation, achieving a ~6x speedup.

**Action:** Whenever parametric curves (quadratic or cubic Bézier) are constructed once and queried repeatedly for arc lengths or progress projections in hot simulation loops, precalculate a cumulative length array `[f32; N + 1]` upon construction and perform O(1) table lookups with linear interpolation.
