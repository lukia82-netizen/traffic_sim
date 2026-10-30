## 2025-05-20 - Precalculated Bezier Arc-Length Lookup Table

**Learning:** `BezierCurvePath` and `CubicBezierCurvePath` numerical arc length calculations (`arc_distance` and `path_length`) were re-evaluating 10 to 20 polynomial samples every time collision and conflict TTC checks evaluated trajectory distances between vehicle positions. By precalculating a 64-sample cumulative length lookup table (`cum_len: [f32; 65]`) upon curve construction, `path_length()` becomes an O(1) lookup and `arc_distance(t0, t1)` becomes O(1) linear interpolation.

**Action:** Precalculate static geometry arc lengths in lookup tables during path construction rather than performing numerical integration loops inside hot simulation steps.
