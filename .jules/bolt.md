# Bolt's Performance Journal - Critical Learnings

## 2026-10-02 - O(1) Precalculated Arc Length Lookup Tables for Bezier Curves
**Learning:** `arc_distance` and `path_length` calculations on Bezier curves are invoked repeatedly in simulation steps (such as conflict point calculations). Executing numerical sampling loops (10 to 20 steps) on every call causes measurable overhead. Precalculating cumulative arc length lookup tables during curve construction reduces `arc_distance` and `path_length` to O(1) linear interpolation, yielding a ~5.65x benchmark speedup.
**Action:** Always prefer precalculated lookup tables for static parametric curves when distance or arc length queries are performed repeatedly during ticks.
