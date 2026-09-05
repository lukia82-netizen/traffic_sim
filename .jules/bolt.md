## 2026-03-31 - Precalculated Bezier Arc-Length Lookup Tables

**Learning:** `arc_distance` on Bezier curves was performing numerical integration loops with 10 to 20 samples (`Self::pos` evaluations and vector distance `sqrt()` calls) on every call. Because `evaluate_conflict_point` evaluates distance along paths for every conflicting vehicle during simulation ticks, this numerical integration loop executed thousands of times per frame, accounting for over 65% of tick CPU time.

**Action:** Precalculate a cumulative arc-length lookup table (`cum_len: [f32; 65]`) on Bezier curve construction (`BezierCurvePath::new` and `CubicBezierCurvePath::new`). Replace loop-based numerical integration in `arc_distance` and `path_length` with O(1) table lookup and linear interpolation, yielding a ~3.1x overall simulation tick speedup.
