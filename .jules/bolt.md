## 2026-03-02 - Precomputed Bezier Cumulative Arc Length Table
**Learning:** Evaluating Bezier curve arc lengths in simulation loops via numerical integration/sampling (e.g., 10-20 chord steps per query) causes heavy CPU overhead in conflict-checking functions (`evaluate_conflict_point`).
**Action:** Precalculate a fixed-size cumulative length table (`[f32; 65]`) on curve instantiation and evaluate `arc_distance` and `path_length` queries in O(1) time using linear interpolation.
